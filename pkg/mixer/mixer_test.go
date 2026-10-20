package mixer

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

const (
	fl = iota
	fr
	rl
	rr
)

func mix(cfg Config, thrust float32, torque fmath.Vec3) msg.MotorOutputs {
	var out msg.MotorOutputs
	New(cfg).Mix(&msg.ThrustTorque{Thrust: thrust, Torque: torque}, &out)
	return out
}

func TestDirections(t *testing.T) {
	cfg := Config{Geometry: QuadX}
	if o := mix(cfg, 0.5, fmath.Vec3{}); o.Value[fl] != 0.5 || o.Value[rr] != 0.5 || o.Count != 4 {
		t.Errorf("hover %v", o.Value)
	}
	// Roll right needs more push on the left.
	if o := mix(cfg, 0.5, fmath.Vec3{X: 0.1}); o.Value[fl] <= o.Value[fr] || o.Value[rl] <= o.Value[rr] {
		t.Errorf("roll right %v", o.Value)
	}
	// Nose up needs more push at the front.
	if o := mix(cfg, 0.5, fmath.Vec3{Y: 0.1}); o.Value[fl] <= o.Value[rl] || o.Value[fr] <= o.Value[rr] {
		t.Errorf("pitch up %v", o.Value)
	}
	// Turning right needs the counter clockwise motors to work harder.
	if o := mix(cfg, 0.5, fmath.Vec3{Z: 0.1}); o.Value[fr] <= o.Value[fl] || o.Value[rl] <= o.Value[rr] {
		t.Errorf("yaw right %v", o.Value)
	}
}

func TestRollWinsOverYaw(t *testing.T) {
	o := mix(Config{Geometry: QuadX}, 0.5, fmath.Vec3{X: 0.4, Z: 0.4})
	for i := range 4 {
		if o.Value[i] < 0 || o.Value[i] > 1 {
			t.Fatalf("motor %d out of range %v", i, o.Value[i])
		}
	}
	if diff := (o.Value[fl] + o.Value[rl] - o.Value[fr] - o.Value[rr]) / 2; fmath.Abs(diff-0.8) > 1e-5 {
		t.Errorf("roll torque %v, want full 0.8", diff)
	}
}

func TestFullThrustKeepsRoll(t *testing.T) {
	o := mix(Config{Geometry: QuadX}, 1, fmath.Vec3{X: 0.2})
	if o.Value[fl] != 1 || fmath.Abs(o.Value[fr]-0.6) > 1e-5 {
		t.Errorf("got %v", o.Value)
	}
}

func TestZeroThrustNoAirMode(t *testing.T) {
	o := mix(Config{Geometry: QuadX}, 0, fmath.Vec3{X: 0.3, Z: 0.3})
	for i := range 4 {
		if o.Value[i] != 0 {
			t.Errorf("motor %d spun at %v with zero thrust", i, o.Value[i])
		}
	}
	o = mix(Config{Geometry: QuadX, AirMode: true}, 0, fmath.Vec3{X: 0.3})
	if o.Value[fl] <= o.Value[fr] {
		t.Errorf("air mode lost roll %v", o.Value)
	}
}

func TestThrustModel(t *testing.T) {
	o := mix(Config{Geometry: QuadX, ThrustModel: 1}, 0.25, fmath.Vec3{})
	if fmath.Abs(o.Value[0]-0.5) > 1e-5 {
		t.Errorf("quadratic model gave %v, want 0.5", o.Value[0])
	}
	o = mix(Config{Geometry: QuadX, ThrustModel: 0.5, Idle: 0.1}, 1, fmath.Vec3{})
	if fmath.Abs(o.Value[0]-1) > 1e-5 {
		t.Errorf("full thrust gave %v", o.Value[0])
	}
}

func TestThrustInvertsMix(t *testing.T) {
	m := New(Config{Geometry: QuadX, ThrustModel: 1, Idle: 0.05})
	var out msg.MotorOutputs
	for _, thrust := range []float32{0, 0.2, 0.45, 0.8, 1} {
		m.Mix(&msg.ThrustTorque{Thrust: thrust}, &out)
		if got := m.Thrust(out.Value[0]); fmath.Abs(got-thrust) > 1e-4 {
			t.Errorf("thrust %v mixed to %v and back to %v", thrust, out.Value[0], got)
		}
	}
}
