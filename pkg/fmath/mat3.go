package fmath

// Mat3 is a 3x3 matrix stored row by row.
type Mat3 [3][3]float32

// Identity3 returns the identity matrix.
func Identity3() Mat3 {
	return Mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
}

// MulVec returns m times v.
func (m Mat3) MulVec(v Vec3) Vec3 {
	return Vec3{
		m[0][0]*v.X + m[0][1]*v.Y + m[0][2]*v.Z,
		m[1][0]*v.X + m[1][1]*v.Y + m[1][2]*v.Z,
		m[2][0]*v.X + m[2][1]*v.Y + m[2][2]*v.Z,
	}
}

// Transpose returns m with rows and columns swapped.
func (m Mat3) Transpose() Mat3 {
	return Mat3{
		{m[0][0], m[1][0], m[2][0]},
		{m[0][1], m[1][1], m[2][1]},
		{m[0][2], m[1][2], m[2][2]},
	}
}
