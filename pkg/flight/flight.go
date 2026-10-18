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
	rateCtl *rate.Controller
	attCtl  *attitude.Controller
	vertCtl *position.Vertical
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
	landFrom float32
	baroRef  float32 // Pa, first baro reading, heights are measured from it
	landing  bool
	imuFails int

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
		rateCtl: rate.New(cfg.Rate),
		attCtl:  attitude.New(cfg.Attitude),
		vertCtl: position.NewVertical(cfg.Vertical),
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
}

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

	accelZ := f.att.Q.Rotate(f.imuS.Accel).Z + fmath.Gravity
	f.altEst.Predict(accelZ, dt)
	f.pos.Time = f.imuS.Time
	f.pos.Z = f.altEst.Z()
	f.pos.VZ = f.altEst.VZ()
	f.pos.AccBias = f.altEst.Bias()
	f.pos.GValid = f.altEst.RangeActive()
}

// idle keeps everything ready for takeoff while disarmed.
func (f *Flight) idle() {
	f.mix.Stop(&f.motors)
	f.tt = msg.ThrustTorque{}
	f.rateSp = msg.RateSetpoint{}
}

// takeoffReset makes the current spot the zero height and current heading the yaw setpoint.
func (f *Flight) takeoffReset() {
	f.rateCtl.Reset()
	f.vertCtl.Reset()
	f.altEst.Reset()
	f.pos.ZValid = false
	f.pos.Z, f.pos.VZ = 0, 0
	f.yawSp = f.att.Q.Yaw()
	f.zSp = 0
}

func (f *Flight) control(now msg.Time, dt float32) {
	in := &f.input
	m := &f.cfg.Manual
	q := f.att.Q
	mode := f.status.Mode
	thrust := in.Throttle

	if mode == msg.ModeAcro && !f.status.Landing {
		f.rateSp.Rate = fmath.Vec3{X: in.Roll, Y: in.Pitch, Z: in.Yaw}.Mul(m.AcroRate)
		f.yawSp = q.Yaw()
	} else {
		roll, pitch, yawRate := in.Roll*m.MaxTilt, in.Pitch*m.MaxTilt, in.Yaw*m.MaxYawRate
		if f.status.Landing {
			roll, pitch, yawRate = 0, 0, 0
		}
		f.yawSp = fmath.WrapPi(f.yawSp + yawRate*dt)
		f.attSp.Q = fmath.QuatFromEuler(roll, pitch, f.yawSp)
		f.attSp.YawRate = yawRate
		f.rateSp.Rate = f.attCtl.Update(q, f.attSp.Q, yawRate)
	}

	switch {
	case mode == msg.ModeAltHold && f.pos.ZValid:
		climb := in.Climb * m.MaxClimb
		if f.status.Landing {
			climb = -f.cfg.Vertical.MaxDescend
		}
		// Keep the setpoint close so letting go of the stick stops quickly.
		f.zSp = fmath.Clamp(f.zSp-climb*dt, f.pos.Z-0.5, f.pos.Z+0.5)
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

	if thrust < f.cfg.Vertical.MinThrust && !f.cfg.Mixer.AirMode {
		// Still on the ground, keep the integrators empty.
		f.rateCtl.Reset()
	}
	f.attSp.Thrust = thrust
	f.rateSp.Thrust = thrust
	f.tt.Thrust = thrust
	f.tt.Torque = f.rateCtl.Update(f.rateSp.Rate, f.imuS.Gyro, dt)
	f.mix.Mix(&f.tt, &f.motors)
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
