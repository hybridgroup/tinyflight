// Package flight connects every stage into one loop: read sensors, estimate, decide, control, mix, drive motors.
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
	landing  bool
	imuFails int

	tasks [5]task
	next  int
	last  msg.Time
	stats Stats
	led   [3]uint8
}

// task is one slow sensor read on its own schedule.
type task struct {
	period msg.Time
	due    msg.Time
	run    func(f *Flight, now msg.Time) error
}

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
	s := cfg.Schedule
	f.tasks = [5]task{
		newTask(s.BaroHz, hw.Baro != nil, (*Flight).readBaro),
		newTask(s.RangeHz, hw.Range != nil, (*Flight).readRange),
		newTask(s.FlowHz, hw.Flow != nil, (*Flight).readFlow),
		newTask(s.MagHz, hw.Mag != nil, (*Flight).readMag),
		newTask(s.BatteryHz, hw.Power != nil, (*Flight).readBattery),
	}
	return f
}

func newTask(hz float32, present bool, run func(*Flight, msg.Time) error) task {
	if hz <= 0 || !present {
		return task{}
	}
	return task{period: msg.Time(1e6 / hz), run: run}
}

func (f *Flight) Status() msg.VehicleStatus      { return f.status }
func (f *Flight) Attitude() msg.Attitude         { return f.att }
func (f *Flight) Position() msg.LocalPosition    { return f.pos }
func (f *Flight) Motors() msg.MotorOutputs       { return f.motors }
func (f *Flight) Input() msg.ManualInput         { return f.input }
func (f *Flight) IMU() msg.IMUSample             { return f.imuS }
func (f *Flight) Battery() msg.BatteryStatus     { return f.batt }
func (f *Flight) Range() msg.RangeSample         { return f.rng }
func (f *Flight) Baro() msg.BaroSample           { return f.baro }
func (f *Flight) Mag() msg.MagSample             { return f.magS }
func (f *Flight) Flow() msg.FlowSample           { return f.flow }
func (f *Flight) Setpoint() msg.RateSetpoint     { return f.rateSp }
func (f *Flight) ThrustTorque() msg.ThrustTorque { return f.tt }
func (f *Flight) Stats() Stats                   { return f.stats }
func (f *Flight) Calibrated() bool               { return f.imu.Calibrated() }

// Run steps the loop at cfg.LoopHz forever.
func (f *Flight) Run() {
	period := time.Duration(float32(time.Second) / f.cfg.LoopHz)
	start := time.Now()
	next := start
	for {
		t := time.Now()
		if j := t.Sub(next); j > f.stats.MaxJitter {
			f.stats.MaxJitter = j
		}
		f.Step(msg.Time(t.Sub(start) / time.Microsecond))
		if d := time.Since(t); d > f.stats.MaxStep {
			f.stats.MaxStep = d
		}
		next = next.Add(period)
		if wait := time.Until(next); wait > 0 {
			time.Sleep(wait)
		} else {
			f.stats.Overruns++
			next = time.Now()
		}
	}
}

// Step runs one pass of the loop at time now in microseconds.
func (f *Flight) Step(now msg.Time) {
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
	if f.hw.Input.ReadInput(&f.input) {
		f.input.Time = now
	}
	f.runSlowTask(now)
	f.estimate(dt)

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
	f.hw.Motors.SetMotors(&f.motors)
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

// runSlowTask reads at most one due slow sensor so no single pass takes too long.
func (f *Flight) runSlowTask(now msg.Time) {
	for i := range f.tasks {
		k := (f.next + i) % len(f.tasks)
		t := &f.tasks[k]
		if t.run == nil || now < t.due {
			continue
		}
		t.due += t.period
		if t.due < now {
			t.due = now + t.period
		}
		if err := t.run(f, now); err != nil && err != hal.ErrNotReady {
			f.stats.SensorErrors++
		}
		f.next = k + 1
		return
	}
}

func (f *Flight) readBaro(now msg.Time) error {
	if err := f.hw.Baro.ReadBaro(&f.baro); err != nil {
		return err
	}
	f.baro.Time = now
	if f.altEst.UpdateBaro(altest.PressureToAltitude(f.baro.Pressure)) {
		f.pos.ZValid = true
	}
	return nil
}

func (f *Flight) readRange(now msg.Time) error {
	if err := f.hw.Range.ReadRange(&f.rng); err != nil {
		return err
	}
	f.rng.Time = now
	if !f.rng.Valid {
		return nil
	}
	cosTilt := f.att.Q.Down().Z
	f.pos.Ground = f.rng.Distance * cosTilt
	if f.altEst.UpdateRange(f.rng.Distance, cosTilt) {
		f.pos.ZValid = true
	}
	return nil
}

func (f *Flight) readFlow(now msg.Time) error {
	if err := f.hw.Flow.ReadFlow(&f.flow); err != nil {
		return err
	}
	f.flow.Time = now
	return nil
}

func (f *Flight) readMag(now msg.Time) error {
	if err := f.hw.Mag.ReadMag(&f.magRaw); err != nil {
		return err
	}
	f.magRaw.Time = now
	f.imu.Mag(&f.magRaw, &f.magS)
	f.attEst.SetMag(f.magS.Field)
	return nil
}

func (f *Flight) readBattery(now msg.Time) error {
	if err := f.hw.Power.ReadBattery(&f.batt); err != nil {
		return err
	}
	f.batt.Time = now
	return nil
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
