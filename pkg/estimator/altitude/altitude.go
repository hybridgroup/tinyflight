// Package altitude estimates height and vertical speed with a small Kalman filter.
// Vertical accel drives the prediction, then baro and downward range readings correct it.
package altitude

import (
	"github.com/hybridgroup/tinyflight/pkg/estimator/axis"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// Config holds the noise and gating settings for the filter.
type Config struct {
	AccelNoise float32 // m/s², accel noise density used in the prediction
	BiasNoise  float32 // m/s², how fast the accel bias may wander per second
	BaroNoise  float32 // m, baro height standard deviation
	RangeNoise float32 // m, range height standard deviation
	Gate       float32 // reject readings further than this many standard deviations away, 0 disables
	RangeMax   float32 // m, ignore range readings beyond this
	RangeHold  float32 // s, how long a range reading keeps the baro out of the filter
	BaroTrack  float32 // 0 to 1, how fast the baro offset follows while range is in use
	RangeDelay float32 // s, how old a range reading is when it arrives, up to about 0.3 s at 400 Hz
	// GroundAccelNoise replaces AccelNoise while on the ground, where bumps swing the accel. 0 keeps AccelNoise.
	GroundAccelNoise float32
}

// historyLen is how many past height changes are kept for late range readings.
const historyLen = 128

// Filter state is height Z (NED, down is positive), vertical speed VZ and accel bias.
type Filter struct {
	cfg Config
	k   axis.Filter

	baroRef     float32
	baroRefSet  bool
	rangeRef    float32
	rangeRefSet bool
	rangeAge    float32

	history [historyLen]float32 // height change from each Predict
	histN   int
	lastDt  float32
	ground  bool
}

// SetOnGround picks GroundAccelNoise while the vehicle sits on the ground.
func (f *Filter) SetOnGround(ground bool) { f.ground = ground }

// New returns a filter at zero height.
func New(cfg Config) *Filter {
	f := &Filter{cfg: cfg}
	f.Reset()
	return f
}

// Reset makes the current position the zero height point. Call it on the ground.
func (f *Filter) Reset() {
	f.k.Reset(1, 1, 0.1)
	f.baroRefSet = false
	f.rangeRefSet = false
	f.rangeAge = f.cfg.RangeHold + 1
	f.histN = 0
}

// Z returns the height in m, NED so up is negative.
func (f *Filter) Z() float32 { return f.k.X[axis.Pos] }

// VZ returns the vertical speed in m/s, NED so climbing is negative.
func (f *Filter) VZ() float32 { return f.k.X[axis.Vel] }

// Bias returns the estimated vertical accel bias in m/s².
func (f *Filter) Bias() float32 { return f.k.X[axis.Bias] }

// Ground returns the height above the ground in m from the range reference. Use it only while RangeActive.
func (f *Filter) Ground() float32 { return f.rangeRef - f.k.X[axis.Pos] }

// RangeActive reports whether a recent range reading is driving the height.
func (f *Filter) RangeActive() bool { return f.rangeAge <= f.cfg.RangeHold }

// Predict moves the state forward using earth frame vertical accel in m/s² with gravity removed.
func (f *Filter) Predict(accelZ, dt float32) {
	noise := f.cfg.AccelNoise
	if f.ground && f.cfg.GroundAccelNoise > 0 {
		noise = f.cfg.GroundAccelNoise
	}
	before := f.k.X[axis.Pos]
	f.k.Predict(accelZ, dt, noise, f.cfg.BiasNoise)
	f.rangeAge += dt
	f.history[f.histN%historyLen] = f.k.X[axis.Pos] - before
	f.histN++
	f.lastDt = dt
}

// heightBefore returns where the current estimate puts the vehicle RangeDelay ago. It takes off the
// predicted movement since then, so corrections made in the meantime carry back too and are not repeated.
func (f *Filter) heightBefore() float32 {
	z := f.k.X[axis.Pos]
	if f.cfg.RangeDelay <= 0 || f.lastDt <= 0 {
		return z
	}
	back := min(int(f.cfg.RangeDelay/f.lastDt+0.5), historyLen, f.histN)
	for i := 1; i <= back; i++ {
		z -= f.history[(f.histN-i)%historyLen]
	}
	return z
}

// UpdateBaro corrects with a baro height in m above sea level.
func (f *Filter) UpdateBaro(alt float32) bool {
	if !f.baroRefSet {
		f.baroRef = alt + f.k.X[axis.Pos]
		f.baroRefSet = true
	}
	if f.RangeActive() {
		// The baro drifts, so while range is good only its offset is tracked.
		f.baroRef += f.cfg.BaroTrack * (alt + f.k.X[axis.Pos] - f.baroRef)
		return false
	}
	return f.update(f.baroRef-alt, f.cfg.BaroNoise)
}

// UpdateRange corrects with a downward range reading in m and the cosine of the vehicle tilt.
func (f *Filter) UpdateRange(dist, cosTilt float32) bool {
	if dist <= 0 || dist > f.cfg.RangeMax || cosTilt < 0.7 {
		return false
	}
	h := dist * cosTilt
	switch {
	case !f.rangeRefSet:
		// First reading after Reset, so the vehicle is on the ground at zero height.
		f.rangeRef = h
		f.rangeRefSet = true
		f.k.X[axis.Pos] = 0
	case !f.RangeActive():
		// Pick up from the current height so the estimate does not jump.
		// The speed and bias may have drifted meanwhile, so let the range correct them quickly.
		f.rangeRef = h + f.heightBefore()
		f.k.P[axis.Vel][axis.Vel] = fmath.Max(f.k.P[axis.Vel][axis.Vel], 1)
		f.k.P[axis.Bias][axis.Bias] = fmath.Max(f.k.P[axis.Bias][axis.Bias], 0.1)
	}
	// A late reading is compared with the height estimate from when it was taken, and the
	// difference corrects the current state, as in delayed fusion in PX4 EKF2.
	// Only accepted readings keep range active, so after a knock the next reading picks up afresh.
	if !f.k.Correct(axis.Pos, -(h-f.rangeRef)-f.heightBefore(), f.cfg.RangeNoise, f.cfg.Gate) {
		return false
	}
	f.rangeAge = 0
	return true
}

// update is a Kalman correction for a direct height measurement z with standard deviation sd.
func (f *Filter) update(z, sd float32) bool {
	return f.k.Update(axis.Pos, z, sd, f.cfg.Gate)
}

// BaroHeight returns the height in m of pressure pa above reference pressure ref, both in Pa, at tempC °C.
// Hypsometric equation with ln(ref/pa) ≈ 2(ref-pa)/(ref+pa), https://en.wikipedia.org/wiki/Hypsometric_equation
func BaroHeight(pa, ref, tempC float32) float32 {
	const rd = 287.05 // J/(kg·K), specific gas constant of dry air
	scale := rd * (tempC + 273.15) / fmath.Gravity
	return scale * 2 * (ref - pa) / (ref + pa)
}
