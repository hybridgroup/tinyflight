package pid

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

func TestDrivesPlantToSetpoint(t *testing.T) {
	// Plant is a mass with a constant push against it, so only the I term can remove the error.
	c := New(Config{P: 4, I: 4, D: 0.5, IntegralLimit: 10, OutputLimit: 20, SampleHz: 400})
	var x, v float32
	const dt = 0.0025
	for range 4000 {
		u := c.Update(1, x, dt)
		v += (u - 2 - 2*v) * dt
		x += v * dt
	}
	if fmath.Abs(x-1) > 0.01 {
		t.Errorf("settled at %v", x)
	}
	if fmath.Abs(c.Integral()-2) > 0.05 {
		t.Errorf("integral %v, want 2", c.Integral())
	}
}

func TestAntiWindup(t *testing.T) {
	c := New(Config{P: 1, I: 10, IntegralLimit: 100, OutputLimit: 1, SampleHz: 400})
	for range 1000 {
		c.Update(10, 0, 0.0025)
	}
	if c.Integral() > 1 {
		t.Errorf("integral wound up to %v while saturated", c.Integral())
	}
}

func TestIntegralLimit(t *testing.T) {
	c := New(Config{I: 10, IntegralLimit: 0.3, SampleHz: 400})
	for range 1000 {
		c.Update(1, 0, 0.0025)
	}
	if c.Integral() != 0.3 {
		t.Errorf("integral %v", c.Integral())
	}
}

func TestNoDerivativeKick(t *testing.T) {
	c := New(Config{D: 1, SampleHz: 400})
	c.Update(0, 0, 0.0025)
	if out := c.Update(100, 0, 0.0025); out != 0 {
		t.Errorf("setpoint step gave D output %v", out)
	}
}

func TestResetAndSetIntegral(t *testing.T) {
	c := New(Config{P: 1, I: 1, D: 1, IntegralLimit: 0.5, SampleHz: 400})
	c.Update(1, 0, 0.1)
	c.SetIntegral(2)
	if c.Integral() != 0.5 {
		t.Errorf("integral %v, want the 0.5 limit", c.Integral())
	}
	c.Reset()
	if c.Integral() != 0 {
		t.Errorf("integral %v after reset", c.Integral())
	}
	// No D kick on the first update after a reset.
	if out := c.Update(0, 10, 0.1); out != -10 {
		t.Errorf("output %v, want -10", out)
	}
}

func TestVec3(t *testing.T) {
	cfg := Config{P: 2, SampleHz: 400}
	v := NewVec3(cfg, cfg, Config{P: 1, SampleHz: 400})
	out := v.Update(fmath.Vec3{X: 1, Y: -1, Z: 1}, fmath.Vec3{}, 0.0025)
	if out != (fmath.Vec3{X: 2, Y: -2, Z: 1}) {
		t.Errorf("output %+v", out)
	}
	v[0].SetIntegral(1)
	v.Reset()
	if v[0].Integral() != 0 {
		t.Error("reset kept the integral")
	}
}
