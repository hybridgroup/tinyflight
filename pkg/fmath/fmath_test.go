package fmath

import "testing"

func near(a, b, tol float32) bool { return Abs(a-b) <= tol }

func nearVec(a, b Vec3, tol float32) bool {
	return near(a.X, b.X, tol) && near(a.Y, b.Y, tol) && near(a.Z, b.Z, tol)
}

// trigTol covers the tinymath approximations, about 0.003 rad each.
const trigTol = 0.006

func TestEulerRoundTrip(t *testing.T) {
	for _, e := range [][3]float32{{0, 0, 0}, {0.3, -0.2, 1.0}, {-1.0, 0.5, -2.5}, {0.1, 1.2, 3.0}} {
		r, p, y := QuatFromEuler(e[0], e[1], e[2]).Euler()
		if !near(r, e[0], trigTol) || !near(p, e[1], trigTol) || !near(y, e[2], trigTol) {
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
	if y := q.Yaw(); !near(y, 1, trigTol) {
		t.Errorf("yaw after 1 rad/s for 1 s is %v", y)
	}
	q = QuatFromEuler(0.5, 0, 0)
	if tilt := q.Tilt(); !near(tilt, 0.5, trigTol) {
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

func TestAtan2Quadrants(t *testing.T) {
	for _, c := range []struct{ y, x, want float32 }{
		{1, 1, Pi / 4},
		{1, -1, 3 * Pi / 4},
		{-1, -1, -3 * Pi / 4},
		{-1, 1, -Pi / 4},
		{0, -1, Pi},
		{-0.5, -0.866, -5 * Pi / 6},
		{-0.3, 0.9, -0.32175055},
	} {
		if got := Atan2(c.y, c.x); !near(WrapPi(got-c.want), 0, trigTol) {
			t.Errorf("Atan2(%v, %v) = %v, want %v", c.y, c.x, got, c.want)
		}
	}
}

func TestAsinAndSqrt(t *testing.T) {
	for _, c := range [][2]float32{{0, 0}, {0.5, Pi / 6}, {-0.5, -Pi / 6}, {0.7071068, Pi / 4}, {1, Pi / 2}, {-1, -Pi / 2}} {
		if got := Asin(c[0]); !near(got, c[1], trigTol) {
			t.Errorf("Asin(%v) = %v, want %v", c[0], got, c[1])
		}
	}
	for _, c := range [][2]float32{{4, 2}, {2, 1.4142135}, {96.17, 9.806631}, {0.25, 0.5}, {1e-4, 0.01}, {0, 0}} {
		if got := Sqrt(c[0]); !near(got, c[1], c[1]*1e-6+1e-7) {
			t.Errorf("Sqrt(%v) = %v, want %v", c[0], got, c[1])
		}
	}
}
