package rate

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

func testConfig() Config {
	p := pid.Config{P: 1, I: 1, IntegralLimit: 0.5, SampleHz: 400}
	return Config{Roll: p, Pitch: p, Yaw: p, Trim: fmath.Vec3{X: 0.1, Y: -0.1, Z: 0.05}}
}

func TestStartsFromTrim(t *testing.T) {
	c := New(testConfig())
	if got := c.Integral(); got != c.Trim() {
		t.Errorf("integral %+v, want trim %+v", got, c.Trim())
	}
	out := c.Update(fmath.Vec3{}, fmath.Vec3{}, 0.0025)
	if out != c.Trim() {
		t.Errorf("output %+v with no error, want trim", out)
	}
}

func TestUpdateAndReset(t *testing.T) {
	c := New(testConfig())
	out := c.Update(fmath.Vec3{X: 1, Y: -1, Z: 1}, fmath.Vec3{}, 0.0025)
	if out.X <= 0.1 || out.Y >= -0.1 || out.Z <= 0.05 {
		t.Errorf("output %+v does not push toward the setpoint", out)
	}
	trim := fmath.Vec3{X: 0.2}
	c.SetTrim(trim)
	c.Reset()
	if c.Integral() != trim {
		t.Errorf("integral %+v after reset, want %+v", c.Integral(), trim)
	}
}
