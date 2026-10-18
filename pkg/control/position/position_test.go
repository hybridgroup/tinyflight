package position

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

func TestHoldsHeight(t *testing.T) {
	c := NewVertical(Config{
		ZP:          2,
		Climb:       pid.Config{P: 0.3, I: 0.2, IntegralLimit: 0.3, SampleHz: 400},
		HoverThrust: 0.4, // actual hover is 0.5, the I term has to find the rest
		MaxClimb:    1,
		MaxDescend:  0.5,
		MinThrust:   0.1,
		MaxThrust:   0.9,
	})
	var z, vz float32
	const dt = 0.0025
	for range 8000 {
		thrust := c.Update(-1, z, vz, 0, 1, dt)
		accDown := fmath.Gravity * (1 - thrust/0.5)
		vz += accDown * dt
		z += vz * dt
	}
	if fmath.Abs(z+1) > 0.01 {
		t.Errorf("height %v m, want 1 m up", -z)
	}
}
