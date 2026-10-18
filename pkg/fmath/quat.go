package fmath

// Quat is a unit quaternion that rotates vectors from the body frame to the earth frame (NED).
type Quat struct {
	W, X, Y, Z float32
}

func IdentityQuat() Quat { return Quat{W: 1} }

// QuatFromEuler builds a rotation from roll, pitch and yaw in radians, applied in yaw, pitch, roll order.
func QuatFromEuler(roll, pitch, yaw float32) Quat {
	sr, cr := SinCos(roll / 2)
	sp, cp := SinCos(pitch / 2)
	sy, cy := SinCos(yaw / 2)
	// Normalized because approximate sin and cos are not exactly on the unit circle.
	return Quat{
		W: cr*cp*cy + sr*sp*sy,
		X: sr*cp*cy - cr*sp*sy,
		Y: cr*sp*cy + sr*cp*sy,
		Z: cr*cp*sy - sr*sp*cy,
	}.Normalized()
}

// Euler returns roll, pitch and yaw in radians.
func (q Quat) Euler() (roll, pitch, yaw float32) {
	roll = Atan2(2*(q.W*q.X+q.Y*q.Z), 1-2*(q.X*q.X+q.Y*q.Y))
	pitch = Asin(2 * (q.W*q.Y - q.Z*q.X))
	yaw = Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))
	return
}

// Yaw returns only the heading part of Euler.
func (q Quat) Yaw() float32 {
	return Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))
}

func (q Quat) Mul(p Quat) Quat {
	return Quat{
		W: q.W*p.W - q.X*p.X - q.Y*p.Y - q.Z*p.Z,
		X: q.W*p.X + q.X*p.W + q.Y*p.Z - q.Z*p.Y,
		Y: q.W*p.Y - q.X*p.Z + q.Y*p.W + q.Z*p.X,
		Z: q.W*p.Z + q.X*p.Y - q.Y*p.X + q.Z*p.W,
	}
}

func (q Quat) Conj() Quat { return Quat{q.W, -q.X, -q.Y, -q.Z} }

func (q Quat) Normalized() Quat {
	n := Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	if n == 0 {
		return IdentityQuat()
	}
	return Quat{q.W / n, q.X / n, q.Y / n, q.Z / n}
}

// Rotate turns a body frame vector into the earth frame.
func (q Quat) Rotate(v Vec3) Vec3 { return q.Matrix().MulVec(v) }

// RotateInv turns an earth frame vector into the body frame.
func (q Quat) RotateInv(v Vec3) Vec3 { return q.Matrix().Transpose().MulVec(v) }

// Matrix returns the body to earth rotation matrix.
func (q Quat) Matrix() Mat3 {
	w, x, y, z := q.W, q.X, q.Y, q.Z
	return Mat3{
		{1 - 2*(y*y+z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)},
		{2 * (x*y + w*z), 1 - 2*(x*x+z*z), 2 * (y*z - w*x)},
		{2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2*(x*x+y*y)},
	}
}

// Down returns the earth Z axis seen from the body frame. Level flight gives (0, 0, 1).
func (q Quat) Down() Vec3 {
	w, x, y, z := q.W, q.X, q.Y, q.Z
	return Vec3{2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2*(x*x+y*y)}
}

// Integrate advances the rotation by the body rate omega in rad/s over dt seconds.
func (q Quat) Integrate(omega Vec3, dt float32) Quat {
	h := dt / 2
	d := q.Mul(Quat{0, omega.X, omega.Y, omega.Z})
	return Quat{q.W + d.W*h, q.X + d.X*h, q.Y + d.Y*h, q.Z + d.Z*h}.Normalized()
}

// Tilt returns the angle between the body Z axis and straight down, in radians.
func (q Quat) Tilt() float32 {
	return acosClamped(q.Down().Z)
}

func acosClamped(x float32) float32 {
	return Pi/2 - Asin(x)
}
