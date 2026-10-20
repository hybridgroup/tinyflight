package horizontal

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/sim"
)

func testConfig() Config {
	return Config{
		AccelNoise: 0.5,
		BiasNoise:  0.05,
		FlowNoise:  0.05,
		Gate:       5,
		MinQuality: 30,
		MinHeight:  0.08,
		MaxHeight:  2,
		FlowHold:   0.3,
		MaxRate:    5,
	}
}

// flowRate is what the sensor sees at dist m with earth speed v, attitude q and body rate omega.
func flowRate(v fmath.Vec3, q fmath.Quat, omega fmath.Vec3, dist float32) fmath.Vec3 {
	vb := q.RotateInv(v)
	return fmath.Vec3{X: vb.X/dist + omega.Y, Y: vb.Y/dist - omega.X}
}

// fly runs the filter for d seconds with flow at 100 Hz and returns it.
func fly(d float32, v, accel fmath.Vec3, q fmath.Quat, omega fmath.Vec3, quality uint8) *Filter {
	f := New(testConfig())
	r := sim.NewRand(1)
	const dt = 0.0025
	for i := range int(d / dt) {
		f.Predict(accel.Add(fmath.Vec3{X: r.Normal() * 0.1, Y: r.Normal() * 0.1}), dt)
		if i%4 == 0 {
			rate := flowRate(v, q, omega, 1).Add(fmath.Vec3{X: r.Normal() * 0.05, Y: r.Normal() * 0.05})
			f.UpdateFlow(rate, omega, 1, 0, q, quality)
		}
	}
	return f
}

func TestSpeed(t *testing.T) {
	for _, tc := range []struct {
		name   string
		yaw    float32
		v      fmath.Vec3
		vx, vy float32
	}{
		{"north", 0, fmath.Vec3{X: 0.3}, 0.3, 0},
		{"east", 0, fmath.Vec3{Y: -0.2}, 0, -0.2},
		{"facing east", fmath.Pi / 2, fmath.Vec3{X: 0.3, Y: 0.1}, 0.3, 0.1},
	} {
		f := fly(3, tc.v, fmath.Vec3{}, fmath.QuatFromEuler(0, 0, tc.yaw), fmath.Vec3{}, 100)
		if !f.Valid() || fmath.Abs(f.VX()-tc.vx) > 0.03 || fmath.Abs(f.VY()-tc.vy) > 0.03 {
			t.Errorf("%s: speed %v %v valid %v, want %v %v", tc.name, f.VX(), f.VY(), f.Valid(), tc.vx, tc.vy)
		}
		if fmath.Abs(f.X()-3*tc.vx) > 0.15 || fmath.Abs(f.Y()-3*tc.vy) > 0.15 {
			t.Errorf("%s: position %v %v", tc.name, f.X(), f.Y())
		}
	}
}

func TestRotationIsNotSpeed(t *testing.T) {
	f := fly(2, fmath.Vec3{}, fmath.Vec3{}, fmath.IdentityQuat(), fmath.Vec3{X: 1, Y: -0.8}, 100)
	if fmath.Abs(f.VX()) > 0.03 || fmath.Abs(f.VY()) > 0.03 {
		t.Errorf("speed %v %v while only turning", f.VX(), f.VY())
	}
}

func TestTiltedSpeed(t *testing.T) {
	f := fly(3, fmath.Vec3{X: 0.3}, fmath.Vec3{}, fmath.QuatFromEuler(0.1, -0.2, 0.5), fmath.Vec3{}, 100)
	if fmath.Abs(f.VX()-0.3) > 0.03 || fmath.Abs(f.VY()) > 0.03 {
		t.Errorf("speed %v %v while tilted", f.VX(), f.VY())
	}
}

func TestAccelBias(t *testing.T) {
	f := fly(10, fmath.Vec3{}, fmath.Vec3{X: 0.2}, fmath.IdentityQuat(), fmath.Vec3{}, 100)
	if b := f.n.X[2]; fmath.Abs(b-0.2) > 0.05 {
		t.Errorf("bias %v, want 0.2", b)
	}
	if fmath.Abs(f.X()) > 0.1 {
		t.Errorf("drifted %v m from a biased accel", f.X())
	}
}

func TestLowQuality(t *testing.T) {
	f := fly(2, fmath.Vec3{X: 0.3}, fmath.Vec3{X: 0.5}, fmath.IdentityQuat(), fmath.Vec3{}, 10)
	if f.Valid() {
		t.Error("valid with poor flow quality")
	}
	if f.VX() != 0 {
		t.Errorf("speed %v drifted while waiting for flow", f.VX())
	}
}

// TestRecoversOneAxis knocks only the east axis, as a tether yank did in flight.
// The north axis keeps accepting flow, and the east axis must not drift on accel alone.
func TestRecoversOneAxis(t *testing.T) {
	f := New(testConfig())
	r := sim.NewRand(1)
	const dt = 0.0025
	for i := range 4000 {
		a := fmath.Vec3{X: r.Normal() * 0.1, Y: r.Normal() * 0.1}
		if i >= 800 && i < 1000 {
			a.Y += 6
		}
		f.Predict(a, dt)
		if i%4 == 0 {
			rate := fmath.Vec3{X: r.Normal() * 0.05, Y: r.Normal() * 0.05}
			f.UpdateFlow(rate, fmath.Vec3{}, 0.2, 0, fmath.IdentityQuat(), 100)
		}
	}
	t.Logf("speed %.3f %.3f m/s, valid %v", f.VX(), f.VY(), f.Valid())
	if fmath.Abs(f.VY()) > 0.05 || !f.Valid() {
		t.Error("east axis did not recover from the knock")
	}
}

// TestSaturatedFlow wobbles faster than the PMW3901 can follow, as on tethered test 6.
// The flow reading tops out while the gyro does not, and that must not turn into speed.
// It starts without flow, as when the drone has just left the ground.
func TestSaturatedFlow(t *testing.T) {
	f := New(testConfig())
	var worst float32
	for i := range 800 {
		f.Predict(fmath.Vec3{}, 0.0025)
		if i%4 == 0 {
			w := 10 * fmath.Cos(fmath.WrapPi(float32(i)*0.05))
			gyro := fmath.Vec3{X: w}
			rate := fmath.Vec3{Y: -fmath.Clamp(w, -7.4, 7.4)}
			f.UpdateFlow(rate, gyro, 0.2, 0, fmath.IdentityQuat(), 100)
		}
		worst = fmath.Max(worst, fmath.Abs(f.VY()))
	}
	t.Logf("worst speed %.3f m/s", worst)
	if worst > 0.1 {
		t.Errorf("saturated flow gave up to %v m/s", worst)
	}
}
