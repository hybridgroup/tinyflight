// Package filter has simple digital filters that smooth noisy sensor values.
package filter

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// LowPass1 is a first order low-pass filter. A cutoff of 0 passes values through unchanged.
type LowPass1 struct {
	alpha float32
	y     float32
	init  bool
}

func NewLowPass1(cutoffHz, sampleHz float32) LowPass1 {
	f := LowPass1{alpha: 1}
	if cutoffHz > 0 && sampleHz > 0 {
		dt := 1 / sampleHz
		rc := 1 / (2 * fmath.Pi * cutoffHz)
		f.alpha = dt / (rc + dt)
	}
	return f
}

func (f *LowPass1) Apply(x float32) float32 {
	if !f.init {
		f.y, f.init = x, true
		return x
	}
	f.y += f.alpha * (x - f.y)
	return f.y
}

func (f *LowPass1) Value() float32 { return f.y }
func (f *LowPass1) Reset()         { f.init = false }

// Biquad is a second order filter, used as a low-pass or a notch.
// Coefficients follow the RBJ Audio EQ Cookbook, https://www.w3.org/TR/audio-eq-cookbook/
type Biquad struct {
	b0, b1, b2, a1, a2 float32
	z1, z2             float32
	pass, init         bool
}

// NewLowPass2 makes a Butterworth low-pass. A cutoff of 0 passes values through unchanged.
func NewLowPass2(cutoffHz, sampleHz float32) Biquad {
	if cutoffHz <= 0 || sampleHz <= 0 || cutoffHz >= sampleHz/2 {
		return Biquad{pass: true}
	}
	cs, alpha := coeffs(cutoffHz, sampleHz, 0.70710678)
	a0 := 1 + alpha
	return Biquad{
		b0: (1 - cs) / 2 / a0,
		b1: (1 - cs) / a0,
		b2: (1 - cs) / 2 / a0,
		a1: -2 * cs / a0,
		a2: (1 - alpha) / a0,
	}
}

// NewNotch removes a narrow band around centerHz, such as motor vibration. A center of 0 disables it.
func NewNotch(centerHz, q, sampleHz float32) Biquad {
	if centerHz <= 0 || sampleHz <= 0 || q <= 0 || centerHz >= sampleHz/2 {
		return Biquad{pass: true}
	}
	cs, alpha := coeffs(centerHz, sampleHz, q)
	a0 := 1 + alpha
	return Biquad{
		b0: 1 / a0,
		b1: -2 * cs / a0,
		b2: 1 / a0,
		a1: -2 * cs / a0,
		a2: (1 - alpha) / a0,
	}
}

func coeffs(f, fs, q float32) (cs, alpha float32) {
	sn, cs := sinCos(2 * fmath.Pi * f / fs)
	return cs, sn / (2 * q)
}

// sinCos is accurate to about 1e-7 for |w| up to Pi, unlike the fast trig in fmath.
// Low cutoffs depend on 1-cos(w), which is too small for an approximation. Setup only.
func sinCos(w float32) (sn, cs float32) {
	// Taylor series on w/4, then the double angle formulas twice.
	x := w / 4
	x2 := x * x
	sn = x * (1 - x2/6*(1-x2/20*(1-x2/42*(1-x2/72))))
	cs = 1 - x2/2*(1-x2/12*(1-x2/30*(1-x2/56)))
	for range 2 {
		sn, cs = 2*sn*cs, cs*cs-sn*sn
	}
	return sn, cs
}

func (f *Biquad) Apply(x float32) float32 {
	if f.pass {
		return x
	}
	if !f.init {
		f.Reset(x)
	}
	y := f.b0*x + f.z1
	f.z1 = f.b1*x - f.a1*y + f.z2
	f.z2 = f.b2*x - f.a2*y
	return y
}

// Reset sets the filter state as if x had been the input forever.
func (f *Biquad) Reset(x float32) {
	f.z2 = x * (f.b2 - f.a2)
	f.z1 = x*(f.b1-f.a1) + f.z2
	f.init = true
}

// LowPass1Vec3 filters each axis of a vector.
type LowPass1Vec3 [3]LowPass1

func NewLowPass1Vec3(cutoffHz, sampleHz float32) LowPass1Vec3 {
	f := NewLowPass1(cutoffHz, sampleHz)
	return LowPass1Vec3{f, f, f}
}

func (f *LowPass1Vec3) Apply(v fmath.Vec3) fmath.Vec3 {
	return fmath.Vec3{X: f[0].Apply(v.X), Y: f[1].Apply(v.Y), Z: f[2].Apply(v.Z)}
}

// BiquadVec3 filters each axis of a vector.
type BiquadVec3 [3]Biquad

func NewBiquadVec3(f Biquad) BiquadVec3 { return BiquadVec3{f, f, f} }

func (f *BiquadVec3) Apply(v fmath.Vec3) fmath.Vec3 {
	return fmath.Vec3{X: f[0].Apply(v.X), Y: f[1].Apply(v.Y), Z: f[2].Apply(v.Z)}
}
