package fmath

// Vec3 is a 3D vector. In the body frame X is forward, Y is right and Z is down.
type Vec3 struct {
	X, Y, Z float32
}

func (a Vec3) Add(b Vec3) Vec3        { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }
func (a Vec3) Sub(b Vec3) Vec3        { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }
func (a Vec3) Scale(s float32) Vec3   { return Vec3{a.X * s, a.Y * s, a.Z * s} }
func (a Vec3) Mul(b Vec3) Vec3        { return Vec3{a.X * b.X, a.Y * b.Y, a.Z * b.Z} }
func (a Vec3) Dot(b Vec3) float32     { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }
func (a Vec3) Norm() float32          { return Sqrt(a.Dot(a)) }
func (a Vec3) Neg() Vec3              { return Vec3{-a.X, -a.Y, -a.Z} }
func (a Vec3) Index(i int) float32    { return [3]float32{a.X, a.Y, a.Z}[i] }
func (a Vec3) Array() [3]float32      { return [3]float32{a.X, a.Y, a.Z} }
func Vec3FromArray(v [3]float32) Vec3 { return Vec3{v[0], v[1], v[2]} }

func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}

// Normalized returns a unit vector, or the zero vector if a is zero.
func (a Vec3) Normalized() Vec3 {
	n := a.Norm()
	if n == 0 {
		return Vec3{}
	}
	return a.Scale(1 / n)
}

// Clamp limits each axis to [-lim, lim] of the matching axis in lim.
func (a Vec3) Clamp(lim Vec3) Vec3 {
	return Vec3{Clamp(a.X, -lim.X, lim.X), Clamp(a.Y, -lim.Y, lim.Y), Clamp(a.Z, -lim.Z, lim.Z)}
}
