// Package flight connects every stage into one loop that reads sensors, estimates, decides, controls, mixes and drives motors.
package flight

import (
	"time"

	"github.com/hybridgroup/tinyflight/pkg/commander"
	"github.com/hybridgroup/tinyflight/pkg/control/attitude"
	"github.com/hybridgroup/tinyflight/pkg/control/position"
	"github.com/hybridgroup/tinyflight/pkg/control/rate"
	altest "github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	attest "github.com/hybridgroup/tinyflight/pkg/estimator/attitude"
	horest "github.com/hybridgroup/tinyflight/pkg/estimator/horizontal"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/mixer"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"github.com/hybridgroup/tinyflight/pkg/params"
	"github.com/hybridgroup/tinyflight/pkg/sensors"
)

// Hardware is what a board provides. IMU, Motors and Input are required, the rest may be nil.
type Hardware struct {
	IMU    hal.IMU
	Motors hal.Motors
	Input  hal.Input
	Mag    hal.Mag
	Baro   hal.Baro
	Range  hal.Rangefinder
	Flow   hal.Flow
	Power  hal.Power
	LED    hal.LED
}

// Stats shows how well the loop keeps time.
type Stats struct {
	Ticks        uint32
	Overruns     uint32
	MaxStep      time.Duration
	MaxJitter    time.Duration
	IMUErrors    uint32
	SensorErrors uint32
}

const maxIMUErrors = 20

// Flight runs the whole flight stack from sensor reads to motor outputs.
type Flight struct {
	cfg params.Config
	hw  Hardware

	imu     *sensors.IMU
	attEst  *attest.Mahony
	altEst  *altest.Filter
	horEst  *horest.Filter
	rateCtl *rate.Controller
	attCtl  *attitude.Controller
	vertCtl *position.Vertical
	horCtl  *position.Horizontal
	mix     *mixer.Mixer
	cmd     *commander.Commander

	raw      msg.IMUSample
	imuS     msg.IMUSample
	magRaw   msg.MagSample
	magS     msg.MagSample
	baro     msg.BaroSample
	rng      msg.RangeSample
	flow     msg.FlowSample
	batt     msg.BatteryStatus
	input    msg.ManualInput
	status   msg.VehicleStatus
	att      msg.Attitude
	pos      msg.LocalPosition
	attSp    msg.AttitudeSetpoint
	rateSp   msg.RateSetpoint
	tt       msg.ThrustTorque
	motors   msg.MotorOutputs
	yawSp    float32
	zSp      float32
	xySp     fmath.Vec3
	xyHold   bool
	landFrom float32
	baroRef  float32 // Pa, first baro reading, heights are measured from it
	landing  bool
	airborne bool // the pilot has asked for thrust since arming
	// What the rate integrators held while flying, averaged, and how long it has been averaged.
	trimAvg   fmath.Vec3
	trimTime  float32
	landedFor float32
	hover     float32
	imuFails  int

	// Gyro sum since the last flow read, so rotation can be removed from flow over the same time.
	flowGyro  fmath.Vec3
	flowGyroN int
	flowMean  fmath.Vec3
	flowNew   bool
	flowUsed  bool // the last flow reading passed the sensor and height checks

	trace       HeightStep
	traceHeight func(*HeightStep)

	slow  slowSensors
	start time.Time
	last  msg.Time
	stats Stats
	led   [3]uint8
}

// New sets up the flight stack on hw and starts the gyro calibration.
func New(hw Hardware, cfg params.Config) *Flight {
	f := &Flight{
		cfg:     cfg,
		hw:      hw,
		imu:     sensors.NewIMU(cfg.Sensors),
		attEst:  attest.New(cfg.AttEst),
		altEst:  altest.New(cfg.AltEst),
		horEst:  horest.New(cfg.HorEst),
		rateCtl: rate.New(cfg.Rate),
		attCtl:  attitude.New(cfg.Attitude),
		vertCtl: position.NewVertical(cfg.Vertical),
		horCtl:  position.NewHorizontal(cfg.Horizontal),
		mix:     mixer.New(cfg.Mixer),
		cmd:     commander.New(cfg.Commander),
	}
	f.imu.Calibrate()
	f.att.Q = fmath.IdentityQuat()
	f.start = time.Now()
	f.setupSlowSensors()
	return f
}

// Status returns the latest vehicle status.
func (f *Flight) Status() msg.VehicleStatus { return f.status }

// Attitude returns the latest attitude estimate.
func (f *Flight) Attitude() msg.Attitude { return f.att }

// Position returns the latest local position estimate.
func (f *Flight) Position() msg.LocalPosition { return f.pos }

// Motors returns the last motor commands.
func (f *Flight) Motors() msg.MotorOutputs { return f.motors }

// Input returns the latest pilot input.
func (f *Flight) Input() msg.ManualInput { return f.input }

// IMU returns the latest filtered IMU sample in the body frame.
func (f *Flight) IMU() msg.IMUSample { return f.imuS }

// Battery returns the latest battery reading.
func (f *Flight) Battery() msg.BatteryStatus { return f.batt }

// Range returns the latest downward range reading.
func (f *Flight) Range() msg.RangeSample { return f.rng }

// Baro returns the latest baro reading.
func (f *Flight) Baro() msg.BaroSample { return f.baro }

// Mag returns the latest magnetometer reading.
func (f *Flight) Mag() msg.MagSample { return f.magS }

// Flow returns the latest optical flow reading.
func (f *Flight) Flow() msg.FlowSample { return f.flow }

// RateIntegral returns the torque the rate integrators hold.
func (f *Flight) RateIntegral() fmath.Vec3 { return f.rateCtl.Integral() }

// HeightSetpoint returns the height setpoint in m, NED so up is negative, used in AltHold and PosHold.
func (f *Flight) HeightSetpoint() float32 { return f.zSp }

// FlowDebug returns the mean gyro rate over the last flow reading, whether it passed the sensor and height checks,
// the speed the horizontal estimator made of it and whether each axis accepted it.
func (f *Flight) FlowDebug() (gyro fmath.Vec3, used bool, speed fmath.Vec3, acceptN, acceptE bool) {
	speed, acceptN, acceptE = f.horEst.LastFlow()
	return f.flowMean, f.flowUsed, speed, acceptN, acceptE
}

// Setpoint returns the latest body rate setpoint.
func (f *Flight) Setpoint() msg.RateSetpoint { return f.rateSp }

// ThrustTorque returns the latest thrust and torque request sent to the mixer.
func (f *Flight) ThrustTorque() msg.ThrustTorque { return f.tt }

// Calibrated reports whether the gyro calibration is done.
func (f *Flight) Calibrated() bool { return f.imu.Calibrated() }

// Stats returns loop timing and error counts.
func (f *Flight) Stats() Stats {
	s := f.stats
	s.SensorErrors = f.slow.errors.Load()
	return s
}

// now is the time since New in microseconds.
func (f *Flight) now() msg.Time { return msg.Time(time.Since(f.start) / time.Microsecond) }

// Run steps the loop at cfg.LoopHz forever, sleeping between passes.
func (f *Flight) Run() { f.run(false) }

// RunDedicated is Run for a loop that has a core to itself. It waits by watching the clock,
// so it never depends on the scheduler waking it on time.
func (f *Flight) RunDedicated() { f.run(true) }

func (f *Flight) run(spin bool) {
	period := time.Duration(float32(time.Second) / f.cfg.LoopHz)
	next := time.Now()
	for {
		t := time.Now()
		if j := t.Sub(next); j > f.stats.MaxJitter {
			f.stats.MaxJitter = j
		}
		f.Step(msg.Time(t.Sub(f.start) / time.Microsecond))
		if d := time.Since(t); d > f.stats.MaxStep {
			f.stats.MaxStep = d
		}
		next = next.Add(period)
		if time.Until(next) <= 0 {
			f.stats.Overruns++
			next = time.Now()
			continue
		}
		if spin {
			for time.Now().Before(next) {
			}
		} else {
			time.Sleep(time.Until(next))
		}
	}
}

var Prof [8]time.Duration

func mark(i int, t *time.Time) {
	n := time.Now()
	if d := n.Sub(*t); d > Prof[i] {
		Prof[i] = d
	}
	*t = n
}

// Step runs one pass of the loop at time now in microseconds.
func (f *Flight) Step(now msg.Time) {
	pt := time.Now()
	f.stats.Ticks++
	nominal := 1 / f.cfg.LoopHz
	dt := nominal
	if f.last != 0 && now > f.last {
		dt = fmath.Clamp(float32(now-f.last)*1e-6, nominal/4, nominal*4)
	}
	f.last = now

	if !f.readIMU(now) {
		return
	}
	mark(0, &pt)
	if f.hw.Input.ReadInput(&f.input) {
		f.input.Time = now
	}
	mark(1, &pt)
	f.runLoopSensors(now)
	mark(2, &pt)
	f.applySlowSensors()
	mark(3, &pt)
	f.estimate(dt)
	mark(4, &pt)

	wasArmed := f.status.Armed
	f.status = f.cmd.Update(commander.State{
		Now:     now,
		Input:   &f.input,
		Tilt:    f.att.Q.Tilt(),
		Battery: &f.batt,
		Ready:   f.imu.Calibrated(),
	})

	switch {
	case !f.status.Armed:
		f.idle()
	case !wasArmed:
		f.takeoffReset()
		fallthrough
	default:
		f.control(now, dt)
	}
	mark(5, &pt)
	f.hw.Motors.SetMotors(&f.motors)
	mark(6, &pt)
	f.updateLED()
	if f.traceHeight != nil {
		f.trace.Time = now
		f.trace.Z, f.trace.VZ, f.trace.Bias = f.pos.Z, f.pos.VZ, f.pos.AccBias
		f.trace.Thrust = f.tt.Thrust
		f.traceHeight(&f.trace)
	}
	f.trace = HeightStep{}
}

// HeightStep is what the height estimator got and gave on one loop pass, so it can be replayed off board.
// Range and baro readings were passed before the accel prediction.
type HeightStep struct {
	Time              msg.Time
	AccelZ, Dt        float32 // vertical accel with gravity removed and the step time passed to Predict
	OnGround          bool    // passed to SetOnGround before Predict
	Range, CosTilt    float32 // range reading passed this pass
	RangeNew, RangeOK bool
	Baro              float32 // baro height passed this pass
	BaroNew, BaroOK   bool
	Z, VZ, Bias       float32 // estimate after this pass
	Thrust            float32
}

// TraceHeight calls fn at the end of every loop pass with the height estimator's inputs and outputs.
// fn runs in the flight loop, so it must be quick and must not allocate.
func (f *Flight) TraceHeight(fn func(*HeightStep)) { f.traceHeight = fn }

func (f *Flight) readIMU(now msg.Time) bool {
	if err := f.hw.IMU.ReadIMU(&f.raw); err != nil {
		f.stats.IMUErrors++
		f.imuFails++
		if f.imuFails > maxIMUErrors {
			// Flying blind is worse than stopping.
			f.mix.Stop(&f.motors)
			f.hw.Motors.SetMotors(&f.motors)
		}
		return false
	}
	f.imuFails = 0
	f.raw.Time = now
	f.imu.Update(&f.raw, &f.imuS)
	f.flowGyro = f.flowGyro.Add(f.imuS.Gyro)
	f.flowGyroN++
	return true
}

func (f *Flight) estimate(dt float32) {
	if !f.imu.Calibrated() {
		f.attEst.Reset(f.imuS.Accel)
	} else {
		f.attEst.Update(f.imuS.Gyro, f.imuS.Accel, dt)
	}
	f.att.Time = f.imuS.Time
	f.att.Q = f.attEst.Attitude()
	f.att.Rate = f.imuS.Gyro

	accel := f.att.Q.Rotate(f.imuS.Accel)
	ground := f.hw.Range != nil && f.rng.Valid && f.rng.Distance < f.cfg.GroundRange
	f.altEst.SetOnGround(ground)
	f.altEst.Predict(accel.Z+fmath.Gravity, dt)
	f.trace.AccelZ, f.trace.Dt, f.trace.OnGround = accel.Z+fmath.Gravity, dt, ground
	f.pos.Time = f.imuS.Time
	f.pos.Z = f.altEst.Z()
	f.pos.VZ = f.altEst.VZ()
	f.pos.AccBias = f.altEst.Bias()
	f.pos.GValid = f.altEst.RangeActive()

	f.horEst.Predict(fmath.Vec3{X: accel.X, Y: accel.Y}, dt)
	if f.flowNew {
		f.flowNew = false
		// The height filter can reject a fast drop, so also check the raw range is not too close for flow.
		close := !f.rng.Valid || f.rng.Distance < f.cfg.HorEst.MinHeight
		cosTilt := f.att.Q.Down().Z
		f.flowUsed = f.flow.Valid && f.pos.GValid && !close && cosTilt > 0
		if f.flowUsed {
			dist := f.altEst.Ground() / cosTilt
			f.horEst.UpdateFlow(f.flow.Rate, f.flowMean, dist, f.pos.VZ, f.att.Q, f.flow.Quality)
		}
	}
	f.pos.X = f.horEst.X()
	f.pos.Y = f.horEst.Y()
	f.pos.VX = f.horEst.VX()
	f.pos.VY = f.horEst.VY()
	f.pos.XYValid = f.horEst.Valid()
}

// idle keeps everything ready for takeoff while disarmed.
func (f *Flight) idle() {
	f.mix.Stop(&f.motors)
	f.tt = msg.ThrustTorque{}
	f.rateSp = msg.RateSetpoint{}
}

// takeoffReset makes the current spot the zero height and current heading the yaw setpoint.
func (f *Flight) takeoffReset() {
	if f.trimTime >= trimLearn {
		// Battery seating and motor warmth change the trim, so start from what the last flight needed.
		f.rateCtl.SetTrim(f.trimAvg)
	}
	f.trimAvg, f.trimTime = f.rateCtl.Trim(), 0
	f.rateCtl.Reset()
	f.vertCtl.Reset()
	f.horCtl.Reset()
	f.altEst.Reset()
	f.horEst.Reset()
	f.xyHold = false
	f.pos.ZValid = false
	f.pos.Z, f.pos.VZ = 0, 0
	f.yawSp = f.att.Q.Yaw()
	f.zSp = 0
	f.airborne = false
	f.landedFor = 0
}

func (f *Flight) control(now msg.Time, dt float32) {
	in := &f.input
	m := &f.cfg.Manual
	q := f.att.Q
	mode := f.status.Mode
	f.updateHover(dt)
	thrust := f.throttleCurve(in.Throttle)
	ground := f.onGround(thrust)
	if f.landed(ground, dt) {
		f.cmd.Disarm()
		f.status.Armed = false
		f.idle()
		return
	}
	if ground {
		// The heading cannot change on the ground, so do not let gyro drift build a yaw error.
		f.yawSp = q.Yaw()
	}

	if mode == msg.ModeAcro && !f.status.Landing {
		f.rateSp.Rate = fmath.Vec3{X: in.Roll, Y: in.Pitch, Z: in.Yaw}.Mul(m.AcroRate)
		f.yawSp = q.Yaw()
		f.xyHold = false
	} else {
		roll, pitch, yawRate := in.Roll*m.MaxTilt, in.Pitch*m.MaxTilt, in.Yaw*m.MaxYawRate
		if f.status.Landing {
			roll, pitch, yawRate = 0, 0, 0
		}
		if mode == msg.ModePosHold && f.pos.XYValid {
			roll, pitch = f.holdPosition(roll, pitch, dt)
		} else {
			f.xyHold = false
		}
		f.yawSp = fmath.WrapPi(f.yawSp + yawRate*dt)
		f.attSp.Q = fmath.QuatFromEuler(roll, pitch, f.yawSp)
		f.attSp.YawRate = yawRate
		f.rateSp.Rate = f.attCtl.Update(q, f.attSp.Q, yawRate)
	}

	holdHeight := mode == msg.ModeAltHold || mode == msg.ModePosHold
	if !f.airborne && !f.status.Landing && ((holdHeight && in.Climb > 0) || (!holdHeight && in.Throttle > 0)) {
		f.airborne = true
	}

	switch {
	case holdHeight && !f.airborne:
		// Stay at idle after arming until the pilot asks to climb, so arming never lifts off.
		thrust = 0
		f.zSp = f.pos.Z
		f.vertCtl.Reset()
	case holdHeight && f.pos.ZValid:
		climb := in.Climb * m.MaxClimb
		if f.status.Landing {
			climb = -f.cfg.Vertical.MaxDescend
		}
		if h := m.MaxHeight; h > 0 {
			// Slow down near the limit the same way the height loop closes on a setpoint.
			climb = fmath.Min(climb, f.cfg.Vertical.ZP*fmath.Max(f.pos.Z+h, 0))
		}
		// Keep the setpoint close so letting go of the stick stops quickly.
		f.zSp = fmath.Clamp(f.zSp-climb*dt, f.pos.Z-0.5, f.pos.Z+0.5)
		if h := m.MaxHeight; h > 0 {
			f.zSp = fmath.Max(f.zSp, -h)
		}
		thrust = f.vertCtl.Update(f.zSp, f.pos.Z, f.pos.VZ, climb, q.Down().Z, dt)
	case f.status.Landing:
		if !f.landing {
			f.landFrom = f.tt.Thrust
		}
		thrust = f.landFrom * (1 - 0.5*f.cmd.LandingElapsed(now)/f.cfg.Commander.LandTime)
	default:
		f.zSp = f.pos.Z
	}

	f.landing = f.status.Landing

	if ground || (thrust < f.cfg.Vertical.MinThrust && !f.cfg.Mixer.AirMode) {
		// On the ground the controllers cannot fix any error, so keep the integrators at the trim.
		f.rateCtl.Reset()
	}
	f.attSp.Thrust = thrust
	f.rateSp.Thrust = thrust
	f.tt.Thrust = thrust
	f.tt.Torque = f.rateCtl.Update(f.rateSp.Rate, f.imuS.Gyro, dt)
	if f.airborne && !ground {
		f.trimAvg = f.trimAvg.Add(f.rateCtl.Integral().Sub(f.trimAvg).Scale(fmath.Min(dt/trimAverage, 1)))
		f.trimTime += dt
	}
	f.mix.Mix(&f.tt, &f.motors)
}

const (
	trimAverage = 2 // s the integrators are averaged over while flying, to find the trim
	trimLearn   = 5 // s of flying before the next flight starts from that trim
)

// hoverTrack is how many seconds the hover thrust takes to follow the battery voltage. Voltage sags
// under load, and following that quickly fed thrust swings back into thrust.
const hoverTrack = 10

// updateHover works out the hover thrust from the battery voltage when the board gives a hover model.
// It follows the voltage slowly, so it tracks the charge left rather than the load.
func (f *Flight) updateHover(dt float32) {
	h := f.cfg.Hover
	want := f.cfg.Vertical.HoverThrust
	if v := f.batt.Voltage; h.Slope != 0 && v > 2.5 {
		want = f.mix.Thrust(h.Offset + h.Slope*v)
	}
	if f.hover == 0 {
		f.hover = want
	}
	f.hover += (want - f.hover) * fmath.Min(dt/hoverTrack, 1)
	f.vertCtl.SetHover(f.hover)
}

// landed reports whether the vehicle has sat on the ground long enough after flying to disarm.
// Only AltHold and PosHold count, since in Stabilize the throttle is the pilot's to cut.
func (f *Flight) landed(ground bool, dt float32) bool {
	mode := f.status.Mode
	holdHeight := mode == msg.ModeAltHold || mode == msg.ModePosHold
	if f.cfg.LandedDisarm <= 0 || !holdHeight || !f.airborne || f.hw.Range == nil || !ground || f.input.Climb > 0 {
		f.landedFor = 0
		return false
	}
	f.landedFor += dt
	return f.landedFor >= f.cfg.LandedDisarm
}

// onGround reports whether the vehicle is still on the ground, from the range when there is one.
func (f *Flight) onGround(thrust float32) bool {
	if f.hw.Range != nil && f.rng.Valid {
		return f.rng.Distance < f.cfg.GroundRange
	}
	return thrust < 0.8*f.hover
}

// throttleCurve puts hover thrust at mid stick, so there is as much room below hover as above it.
func (f *Flight) throttleCurve(s float32) float32 {
	hover := f.hover
	if s < 0.5 {
		return 2 * s * hover
	}
	full := fmath.Min(f.cfg.Manual.FullStick*hover, 1)
	return hover + 2*(s-0.5)*(full-hover)
}

// holdPosition turns the roll and pitch stick angles into a speed and returns the tilt that holds that speed.
// With the sticks centred it holds the current spot.
func (f *Flight) holdPosition(roll, pitch, dt float32) (float32, float32) {
	pos := fmath.Vec3{X: f.pos.X, Y: f.pos.Y}
	if !f.xyHold {
		f.xyHold = true
		f.xySp = pos
		f.horCtl.Reset()
	}
	yaw := f.att.Q.Yaw()
	s, c := fmath.SinCos(yaw)
	scale := f.cfg.Horizontal.MaxSpeed / f.cfg.Manual.MaxTilt
	fwd, right := -pitch*scale, roll*scale
	vel := fmath.Vec3{X: c*fwd - s*right, Y: s*fwd + c*right}
	// Keep the setpoint close so letting go of the sticks stops quickly.
	f.xySp = f.xySp.Add(vel.Scale(dt))
	f.xySp.X = fmath.Clamp(f.xySp.X, pos.X-0.5, pos.X+0.5)
	f.xySp.Y = fmath.Clamp(f.xySp.Y, pos.Y-0.5, pos.Y+0.5)
	return f.horCtl.Update(f.xySp, pos, fmath.Vec3{X: f.pos.VX, Y: f.pos.VY}, vel, yaw, dt)
}

func (f *Flight) updateLED() {
	if f.hw.LED == nil {
		return
	}
	c := [3]uint8{0, 32, 0}
	switch {
	case !f.imu.Calibrated():
		c = [3]uint8{0, 0, 32}
	case f.status.Failsafe != msg.FailsafeNone:
		c = [3]uint8{32, 32, 0}
	case f.status.Armed:
		c = [3]uint8{32, 0, 0}
	}
	if c != f.led {
		f.led = c
		f.hw.LED.SetColor(c[0], c[1], c[2])
	}
}
