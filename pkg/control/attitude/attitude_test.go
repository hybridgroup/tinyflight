package attitude

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

func TestErrorSigns(t *testing.T) {
	level := fmath.IdentityQuat()
	if e := Error(level, fmath.QuatFromEuler(0.1, 0, 0)); e.X < 0.09 || e.X > 0.11 {
		t.Errorf("roll right error %v", e)
	}
	if e := Error(level, fmath.QuatFromEuler(0, -0.1, 0)); e.Y > -0.09 || e.Y < -0.11 {
		t.Errorf("pitch down error %v", e)
	}
	if e := Error(fmath.QuatFromEuler(0, 0, 3), fmath.QuatFromEuler(0, 0, -3)); e.Z < 0.27 || e.Z > 0.29 {
		t.Errorf("yaw across ±pi error %v, want short way", e)
	}
}

func TestConvergesAndLimits(t *testing.T) {
	c := New(Config{P: fmath.Vec3{X: 6, Y: 6, Z: 3}, MaxRate: fmath.Vec3{X: 3, Y: 3, Z: 2}})
	q := fmath.QuatFromEuler(0.5, -0.3, 1)
	sp := fmath.QuatFromEuler(0, 0, 0)
	first := c.Update(q, sp, 0)
	if fmath.Abs(first.X) > 3 || fmath.Abs(first.Z) > 2 {
		t.Errorf("rate not limited %v", first)
	}
	for range 2000 {
		q = q.Integrate(c.Update(q, sp, 0), 0.0025)
	}
	if e := Error(q, sp); e.Norm() > 1e-3 {
		t.Errorf("remaining error %v", e)
	}
}
