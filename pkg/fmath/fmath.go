// Package fmath has small float32 math helpers for vectors, rotations and limits.
package fmath

import "math"

const (
	Pi      = float32(math.Pi)
	Gravity = float32(9.80665)
)

func Sqrt(x float32) float32 { return float32(math.Sqrt(float64(x))) }
func Sin(x float32) float32  { return float32(math.Sin(float64(x))) }
func Cos(x float32) float32  { return float32(math.Cos(float64(x))) }
func Tan(x float32) float32  { return float32(math.Tan(float64(x))) }
func Abs(x float32) float32  { return float32(math.Abs(float64(x))) }

func Atan2(y, x float32) float32 { return float32(math.Atan2(float64(y), float64(x))) }

// Asin clamps its input to [-1, 1] so rounding errors do not produce NaN.
func Asin(x float32) float32 { return float32(math.Asin(float64(Clamp(x, -1, 1)))) }

func Pow(x, y float32) float32 { return float32(math.Pow(float64(x), float64(y))) }

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
