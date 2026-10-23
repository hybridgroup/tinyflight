package sim

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// flowCounts holds the quad at 1 m with the given speed and body rate for 0.1 s and reads the flow.
func flowCounts(vel, omega fmath.Vec3) msg.FlowSample {
	p := StampFly()
	p.FlowNoise = 0
	q := New(p)
	q.Pos.Z = -0.99
	q.Vel, q.Omega = vel, omega
	q.flow = q.flowRate().Scale(0.1 / p.FlowScale)
	var s msg.FlowSample
	q.ReadFlow(&s)
	return s
}

func TestFlowSigns(t *testing.T) {
	// 0.5 m/s at 1 m is 0.5 rad/s, so 0.05 rad in 0.1 s, about 22.5 counts.
	if s := flowCounts(fmath.Vec3{X: 0.5}, fmath.Vec3{}); s.DeltaX < 22 || s.DeltaX > 23 || s.DeltaY != 0 || !s.Valid {
		t.Errorf("forward %+v", s)
	}
	if s := flowCounts(fmath.Vec3{Y: 0.5}, fmath.Vec3{}); s.DeltaY < 22 || s.DeltaY > 23 || s.DeltaX != 0 {
		t.Errorf("right %+v", s)
	}
	// Pitching nose up sweeps the view forward, like moving forward.
	if s := flowCounts(fmath.Vec3{}, fmath.Vec3{Y: 0.5}); s.DeltaX < 22 || s.DeltaX > 23 {
		t.Errorf("pitch up %+v", s)
	}
	// Rolling right sweeps the view left, like moving left.
	if s := flowCounts(fmath.Vec3{}, fmath.Vec3{X: 0.5}); s.DeltaY > -22 || s.DeltaY < -23 {
		t.Errorf("roll right %+v", s)
	}
}

func TestFlowOnGround(t *testing.T) {
	q := New(StampFly())
	var s msg.FlowSample
	q.ReadFlow(&s)
	if s.Valid {
		t.Error("flow valid while sitting on the ground")
	}
}

func TestHoverThrustHolds(t *testing.T) {
	q := New(StampFly())
	if !q.OnGround() {
		t.Fatal("not on the ground at start")
	}
	// Each motor gives MaxThrust·cmd², so this command lifts slightly more than the weight.
	cmd := fmath.Sqrt(q.P.Mass*fmath.Gravity/4/q.P.MaxThrust) * 1.05
	m := msg.MotorOutputs{Count: 4, Value: [msg.MaxMotors]float32{cmd, cmd, cmd, cmd}}
	q.SetMotors(&m)
	for range 2000 {
		q.Step(0.001)
	}
	if q.OnGround() || q.Pos.Z > -0.1 {
		t.Errorf("did not climb, z %v", q.Pos.Z)
	}
	if q.Q.Down().Z < 0.99 {
		t.Errorf("tilted with even thrust, down %+v", q.Q.Down())
	}
}

func TestThrustExp(t *testing.T) {
	p := StampFly()
	p.ThrustExp = 1
	q := New(p)
	q.Cmd = [msg.MaxMotors]float32{0.5, 0.5, 0.5, 0.5}
	for range 200 {
		q.Step(0.001)
	}
	if fmath.Abs(q.Thrust[0]-0.5*p.MaxThrust) > 0.001 {
		t.Errorf("thrust %v, want linear %v", q.Thrust[0], 0.5*p.MaxThrust)
	}
}

func TestSensors(t *testing.T) {
	p := StampFly()
	p.AccelNoise, p.GyroNoise, p.BaroNoise, p.RangeNoise = 0, 0, 0, 0
	q := New(p)

	var imu msg.IMUSample
	if err := q.ReadIMU(&imu); err != nil || fmath.Abs(imu.Accel.Z+fmath.Gravity) > 1e-4 || imu.Gyro != p.GyroBias {
		t.Errorf("imu %+v %v", imu, err)
	}
	q.IMUFail = true
	if err := q.ReadIMU(&imu); err == nil || err.Error() == "" {
		t.Error("no error from a failed imu")
	}

	var b0, b1 msg.BaroSample
	q.ReadBaro(&b0)
	q.Pos.Z = -1
	q.ReadBaro(&b1)
	if d := b0.Pressure - b1.Pressure; d < 10 || d > 13 {
		t.Errorf("pressure drop over 1 m %v Pa", d)
	}

	var r msg.RangeSample
	q.ReadRange(&r)
	if !r.Valid || fmath.Abs(r.Distance-1.01) > 1e-4 {
		t.Errorf("range %+v", r)
	}
	q.Q = fmath.QuatFromEuler(1.2, 0, 0)
	q.ReadRange(&r)
	if r.Valid {
		t.Error("range valid while tilted too far")
	}

	var bat msg.BatteryStatus
	q.ReadBattery(&bat)
	if bat.Voltage != p.Voltage {
		t.Errorf("battery %v", bat.Voltage)
	}
}

func TestRangeDelay(t *testing.T) {
	p := StampFly()
	p.RangeNoise = 0
	p.RangeDelay = 0.05
	q := New(p)
	q.Pos.Z = -1
	for range 100 {
		q.Step(0.001)
		q.Pos.Z = -1
		q.Vel = fmath.Vec3{}
	}
	q.Pos.Z = -2
	var r msg.RangeSample
	q.ReadRange(&r)
	if fmath.Abs(r.Distance-1.01) > 1e-3 {
		t.Errorf("range %v, want the delayed 1.01", r.Distance)
	}
}

func TestPilot(t *testing.T) {
	p := &Pilot{In: msg.ManualInput{Roll: 0.3}}
	var in msg.ManualInput
	if !p.ReadInput(&in) || !in.Valid || in.Roll != 0.3 {
		t.Errorf("input %+v", in)
	}
	p.Silent = true
	if p.ReadInput(&in) {
		t.Error("silent pilot gave input")
	}
}

func TestRound(t *testing.T) {
	if round(1.6) != 2 || round(-1.6) != -2 || round(0.4) != 0 {
		t.Error("round")
	}
}
