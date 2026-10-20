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

func horizontalConfig() HorizontalConfig {
	return HorizontalConfig{
		XYP:      1,
		Vel:      pid.Config{P: 2, I: 0.5, IntegralLimit: 1, OutputLimit: 3, SampleHz: 400},
		MaxSpeed: 0.5,
		MaxTilt:  0.3,
	}
}

func TestHorizontalSigns(t *testing.T) {
	for _, tc := range []struct {
		name        string
		sp          fmath.Vec3
		yaw         float32
		roll, pitch float32 // sign wanted, 0 means near zero
	}{
		{"north ahead", fmath.Vec3{X: 1}, 0, 0, -1},
		{"east to the right", fmath.Vec3{Y: 1}, 0, 1, 0},
		{"north to the left when facing east", fmath.Vec3{X: 1}, fmath.Pi / 2, -1, 0},
		{"east behind when facing west", fmath.Vec3{Y: 1}, -fmath.Pi / 2, 0, 1},
	} {
		c := NewHorizontal(horizontalConfig())
		roll, pitch := c.Update(tc.sp, fmath.Vec3{}, fmath.Vec3{}, fmath.Vec3{}, tc.yaw, 0.0025)
		if !sameSign(roll, tc.roll) || !sameSign(pitch, tc.pitch) {
			t.Errorf("%s: roll %v pitch %v", tc.name, roll, pitch)
		}
	}
}

func sameSign(v, want float32) bool {
	if want == 0 {
		return fmath.Abs(v) < 1e-3
	}
	return v*want > 0
}

func TestHorizontalLimits(t *testing.T) {
	c := NewHorizontal(horizontalConfig())
	roll, pitch := c.Update(fmath.Vec3{X: 100, Y: -100}, fmath.Vec3{}, fmath.Vec3{X: -5}, fmath.Vec3{}, 0, 0.0025)
	if fmath.Abs(roll) > 0.3 || fmath.Abs(pitch) > 0.3 {
		t.Errorf("tilt past the limit, roll %v pitch %v", roll, pitch)
	}
}

func TestHoldsPositionInWind(t *testing.T) {
	c := NewHorizontal(horizontalConfig())
	var pos, vel fmath.Vec3
	const dt = 0.0025
	wind := fmath.Vec3{X: 0.3, Y: -0.2}
	for range 8000 {
		roll, pitch := c.Update(fmath.Vec3{}, pos, vel, fmath.Vec3{}, 0, dt)
		acc := fmath.Vec3{X: -fmath.Gravity * pitch, Y: fmath.Gravity * roll}.Add(wind).Sub(vel.Scale(0.3))
		vel = vel.Add(acc.Scale(dt))
		pos = pos.Add(vel.Scale(dt))
	}
	if pos.Norm() > 0.01 {
		t.Errorf("pushed %v m off the setpoint by wind", pos)
	}
}

func TestMaxHover(t *testing.T) {
	c := NewVertical(Config{
		ZP:          2,
		Climb:       pid.Config{P: 0.3, I: 0.2, IntegralLimit: 0.3, SampleHz: 400},
		HoverThrust: 0.3,
		MaxClimb:    1,
		MaxDescend:  0.5,
		MinThrust:   0.1,
		MaxThrust:   0.9,
		MaxHover:    1.7,
	})
	var thrust float32
	for range 400 {
		// Far below the setpoint and not climbing, as when the height estimate is wrong.
		thrust = c.Update(-5, 0, 0, 0, 1, 0.0025)
	}
	if fmath.Abs(thrust-0.51) > 1e-4 {
		t.Errorf("thrust %v, want the cap of 1.7 times 0.3 hover", thrust)
	}
}
