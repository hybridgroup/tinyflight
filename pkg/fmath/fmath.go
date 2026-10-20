// Package fmath has small float32 math helpers for vectors, rotations and limits.
package fmath

import "github.com/orsinium-labs/tinymath"

const (
	Pi      = float32(tinymath.Pi)
	Gravity = float32(9.80665)
)

// Sin returns the sine of x in radians. It uses tinymath with a maximum error of about 0.003.
func Sin(x float32) float32 { return tinymath.Sin(x) }

// Cos returns the cosine of x in radians. It uses tinymath with a maximum error of about 0.003.
func Cos(x float32) float32 { return tinymath.Cos(x) }

// Abs returns the absolute value of x.
func Abs(x float32) float32 { return tinymath.Abs(x) }

// SinCos returns the sine and cosine of x in radians, with the same error as Sin and Cos.
func SinCos(x float32) (s, c float32) { return tinymath.SinCos(x) }

// Atan2 returns the angle of (x, y) in radians, in (-Pi, Pi].
func Atan2(y, x float32) float32 { return tinymath.Atan2(y, x) }

// Asin uses Atan2 and Sqrt, since tinymath.Asin uses a rough inverse square root and is off by up to 0.018 rad.
func Asin(x float32) float32 {
	x = Clamp(x, -1, 1)
	return Atan2(x, Sqrt(1-x*x))
}

// Sqrt refines the tinymath estimate, which is off by up to about 6%, with three Newton steps.
// See https://en.wikipedia.org/wiki/Newton%27s_method#Square_root
func Sqrt(x float32) float32 {
	if x <= 0 {
		return 0
	}
	y := tinymath.Sqrt(x)
	y = 0.5 * (y + x/y)
	y = 0.5 * (y + x/y)
	return 0.5 * (y + x/y)
}

// Clamp limits x to [lo, hi].
func Clamp(x, lo, hi float32) float32 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

// Min returns the smaller of a and b.
func Min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

// Max returns the larger of a and b.
func Max(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

// WrapPi returns the angle wrapped into [-Pi, Pi).
func WrapPi(a float32) float32 {
	for a >= Pi {
		a -= 2 * Pi
	}
	for a < -Pi {
		a += 2 * Pi
	}
	return a
}

// Radians converts degrees to radians.
func Radians(deg float32) float32 { return deg * Pi / 180 }

// Degrees converts radians to degrees.
func Degrees(rad float32) float32 { return rad * 180 / Pi }

// Deadband returns 0 inside [-band, band] and rescales the rest back to [-1, 1].
func Deadband(x, band float32) float32 {
	switch {
	case x > band:
		return (x - band) / (1 - band)
	case x < -band:
		return (x + band) / (1 - band)
	}
	return 0
}
