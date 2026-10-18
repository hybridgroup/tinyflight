// Package fmath has small float32 math helpers for vectors, rotations and limits.
package fmath

import "github.com/orsinium-labs/tinymath"

const (
	Pi      = float32(tinymath.Pi)
	Gravity = float32(9.80665)
)

// Trig functions are float32 approximations from tinymath with a maximum error of about 0.003 rad.
func Sin(x float32) float32           { return tinymath.Sin(x) }
func Cos(x float32) float32           { return tinymath.Cos(x) }
func Abs(x float32) float32           { return tinymath.Abs(x) }
func SinCos(x float32) (s, c float32) { return tinymath.SinCos(x) }

// Atan2 maps tinymath.Atan2Norm itself, since tinymath.Atan2 v1.2.0 is wrong when y and x are both negative.
func Atan2(y, x float32) float32 {
	n := tinymath.Atan2Norm(y, x)
	if n > 2 {
		n -= 4
	}
	return Pi / 2 * n
}

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

func Clamp(x, lo, hi float32) float32 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}

func Min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

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

func Radians(deg float32) float32 { return deg * Pi / 180 }
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
