package fmath

import "testing"

func near(a, b, tol float32) bool { return Abs(a-b) <= tol }

func nearVec(a, b Vec3, tol float32) bool {
	return near(a.X, b.X, tol) && near(a.Y, b.Y, tol) && near(a.Z, b.Z, tol)
}

func TestEulerRoundTrip(t *testing.T) {
	for _, e := range [][3]float32{{0, 0, 0}, {0.3, -0.2, 1.0}, {-1.0, 0.5, -2.5}, {0.1, 1.2, 3.0}} {
		r, p, y := QuatFromEuler(e[0], e[1], e[2]).Euler()
		if !near(r, e[0], 1e-5) || !near(p, e[1], 1e-5) || !near(y, e[2], 1e-5) {
			t.Errorf("euler %v gave %v %v %v", e, r, p, y)
		}
	}
}

func TestRotateFRD(t *testing.T) {
	// Yaw 90 degrees right turns body forward into earth east.
	q := QuatFromEuler(0, 0, Pi/2)
	if got := q.Rotate(Vec3{1, 0, 0}); !nearVec(got, Vec3{0, 1, 0}, 1e-6) {
		t.Errorf("yaw rotate got %v", got)
	}
	// Pitch nose up 90 degrees turns body forward into earth up.
	q = QuatFromEuler(0, Pi/2, 0)
	if got := q.Rotate(Vec3{1, 0, 0}); !nearVec(got, Vec3{0, 0, -1}, 1e-6) {
		t.Errorf("pitch rotate got %v", got)
	}
	// Roll right 90 degrees turns body right into earth down.
	q = QuatFromEuler(Pi/2, 0, 0)
	if got := q.Rotate(Vec3{0, 1, 0}); !nearVec(got, Vec3{0, 0, 1}, 1e-6) {
		t.Errorf("roll rotate got %v", got)
	}
	if got := q.RotateInv(q.Rotate(Vec3{1, 2, 3})); !nearVec(got, Vec3{1, 2, 3}, 1e-5) {
		t.Errorf("inverse rotate got %v", got)
	}
	if got := q.Down(); !nearVec(got, q.RotateInv(Vec3{0, 0, 1}), 1e-6) {
		t.Errorf("down got %v", got)
	}
}

func TestIntegrate(t *testing.T) {
	q := IdentityQuat()
	for range 1000 {
		q = q.Integrate(Vec3{0, 0, 1}, 0.001)
	}
	if y := q.Yaw(); !near(y, 1, 1e-3) {
		t.Errorf("yaw after 1 rad/s for 1 s is %v", y)
	}
	q = QuatFromEuler(0.5, 0, 0)
	if tilt := q.Tilt(); !near(tilt, 0.5, 1e-4) {
		t.Errorf("tilt %v", tilt)
	}
}

func TestWrapAndDeadband(t *testing.T) {
	if !near(WrapPi(3*Pi/2), -Pi/2, 1e-5) {
		t.Error("wrap")
	}
	if Deadband(0.1, 0.2) != 0 || !near(Deadband(1, 0.2), 1, 1e-6) || !near(Deadband(-0.6, 0.2), -0.5, 1e-6) {
		t.Error("deadband")
	}
}
