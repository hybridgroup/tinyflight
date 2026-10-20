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
