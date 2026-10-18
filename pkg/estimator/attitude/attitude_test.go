package attitude

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

var earthMag = fmath.Vec3{X: 20, Y: 0, Z: 40}

func stillAccel(q fmath.Quat) fmath.Vec3 {
	return q.RotateInv(fmath.Vec3{Z: -fmath.Gravity})
}

func TestResetFromAccel(t *testing.T) {
	truth := fmath.QuatFromEuler(0.3, -0.2, 0)
	m := New(Config{Kp: 1, AccelGate: 0.2})
	m.Reset(stillAccel(truth))
	r, p, _ := m.Attitude().Euler()
	if fmath.Abs(r-0.3) > 1e-4 || fmath.Abs(p+0.2) > 1e-4 {
		t.Errorf("reset gave roll %v pitch %v", r, p)
	}
}

func TestConvergesToTilt(t *testing.T) {
	truth := fmath.QuatFromEuler(0.4, -0.3, 0)
	m := New(Config{Kp: 2, AccelGate: 0.2})
	m.Reset(fmath.Vec3{Z: -fmath.Gravity})
	for range 4000 {
		m.Update(fmath.Vec3{}, stillAccel(truth), 0.0025)
	}
	r, p, _ := m.Attitude().Euler()
	if fmath.Abs(r-0.4) > 0.01 || fmath.Abs(p+0.3) > 0.01 {
		t.Errorf("converged to roll %v pitch %v", r, p)
	}
}

func TestLearnsGyroBias(t *testing.T) {
	bias := fmath.Vec3{X: 0.02, Y: -0.03}
	m := New(Config{Kp: 1, Ki: 0.1, AccelGate: 0.2})
	m.Reset(fmath.Vec3{Z: -fmath.Gravity})
	for range 40000 {
		m.Update(bias, fmath.Vec3{Z: -fmath.Gravity}, 0.0025)
	}
	r, p, _ := m.Attitude().Euler()
	if fmath.Abs(r) > 0.005 || fmath.Abs(p) > 0.005 {
		t.Errorf("drifted to roll %v pitch %v", r, p)
	}
}

func TestFollowsRotation(t *testing.T) {
	m := New(Config{Kp: 1, AccelGate: 0.2})
	truth := fmath.IdentityQuat()
	m.Reset(stillAccel(truth))
	rate := fmath.Vec3{X: 0.5, Y: 0.2, Z: 1}
	for range 400 {
		truth = truth.Integrate(rate, 0.0025)
		m.Update(rate, stillAccel(truth), 0.0025)
	}
	tr, tp, ty := truth.Euler()
	r, p, y := m.Attitude().Euler()
	if fmath.Abs(r-tr) > 0.01 || fmath.Abs(p-tp) > 0.01 || fmath.Abs(y-ty) > 0.01 {
		t.Errorf("estimate %v %v %v, truth %v %v %v", r, p, y, tr, tp, ty)
	}
}

func TestMagCorrectsYawOnly(t *testing.T) {
	truth := fmath.QuatFromEuler(0.2, 0.1, 1.0)
	m := New(Config{Kp: 2, AccelGate: 0.2, UseMag: true})
	m.Reset(stillAccel(truth))
	for range 8000 {
		m.SetMag(truth.RotateInv(earthMag))
		m.Update(fmath.Vec3{}, stillAccel(truth), 0.0025)
	}
	r, p, y := m.Attitude().Euler()
	if fmath.Abs(r-0.2) > 0.01 || fmath.Abs(p-0.1) > 0.01 || fmath.Abs(y-1.0) > 0.01 {
		t.Errorf("estimate %v %v %v", r, p, y)
	}
}
