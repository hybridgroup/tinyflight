// Package altitude estimates height and vertical speed with a small Kalman filter.
// Vertical accel drives the prediction, then baro and downward range readings correct it.
package altitude

import (
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
}

// Filter state is height Z (NED, down is positive), vertical speed VZ and accel bias.
type Filter struct {
	cfg Config
	x   [3]float32
	p   [3][3]float32

	baroRef     float32
	baroRefSet  bool
	rangeRef    float32
	rangeRefSet bool
	rangeAge    float32
}

// New returns a filter at zero height.
func New(cfg Config) *Filter {
	f := &Filter{cfg: cfg}
	f.Reset()
	return f
}

// Reset makes the current position the zero height point. Call it on the ground.
func (f *Filter) Reset() {
	f.x = [3]float32{}
	f.p = [3][3]float32{{1, 0, 0}, {0, 1, 0}, {0, 0, 0.1}}
	f.baroRefSet = false
	f.rangeRefSet = false
	f.rangeAge = f.cfg.RangeHold + 1
}

// Z returns the height in m, NED so up is negative.
func (f *Filter) Z() float32 { return f.x[0] }

// VZ returns the vertical speed in m/s, NED so climbing is negative.
func (f *Filter) VZ() float32 { return f.x[1] }

// Bias returns the estimated vertical accel bias in m/s².
func (f *Filter) Bias() float32 { return f.x[2] }

// RangeActive reports whether a recent range reading is driving the height.
func (f *Filter) RangeActive() bool { return f.rangeAge <= f.cfg.RangeHold }

// Predict moves the state forward using earth frame vertical accel in m/s² with gravity removed.
func (f *Filter) Predict(accelZ, dt float32) {
	a := accelZ - f.x[2]
	f.x[0] += f.x[1]*dt + 0.5*a*dt*dt
	f.x[1] += a * dt
	f.rangeAge += dt

	// P = F P Fᵀ + Q with F = [[1, dt, -dt²/2], [0, 1, -dt], [0, 0, 1]]
	h := 0.5 * dt * dt
	fm := [3][3]float32{{1, dt, -h}, {0, 1, -dt}, {0, 0, 1}}
	var fp [3][3]float32
	for i := range 3 {
		for j := range 3 {
			fp[i][j] = fm[i][0]*f.p[0][j] + fm[i][1]*f.p[1][j] + fm[i][2]*f.p[2][j]
		}
	}
	for i := range 3 {
		for j := range 3 {
			f.p[i][j] = fp[i][0]*fm[j][0] + fp[i][1]*fm[j][1] + fp[i][2]*fm[j][2]
		}
	}

	// Accel noise enters through G = [dt²/2, dt, 0], bias noise as a random walk.
	qa := f.cfg.AccelNoise * f.cfg.AccelNoise
	g := [2]float32{h, dt}
	for i := range 2 {
		for j := range 2 {
			f.p[i][j] += g[i] * g[j] * qa
		}
	}
	f.p[2][2] += f.cfg.BiasNoise * f.cfg.BiasNoise * dt
}

// UpdateBaro corrects with a baro height in m above sea level.
func (f *Filter) UpdateBaro(alt float32) bool {
	if !f.baroRefSet {
		f.baroRef = alt + f.x[0]
		f.baroRefSet = true
	}
	if f.RangeActive() {
		// The baro drifts, so while range is good only its offset is tracked.
		f.baroRef += f.cfg.BaroTrack * (alt + f.x[0] - f.baroRef)
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
		f.x[0] = 0
	case !f.RangeActive():
		// Pick up from the current height so the estimate does not jump.
		f.rangeRef = h + f.x[0]
	}
	f.rangeAge = 0
	return f.update(-(h - f.rangeRef), f.cfg.RangeNoise)
}

// update is a Kalman correction for a direct height measurement z with standard deviation sd.
func (f *Filter) update(z, sd float32) bool {
	innov := z - f.x[0]
	s := f.p[0][0] + sd*sd
	if f.cfg.Gate > 0 && innov*innov > f.cfg.Gate*f.cfg.Gate*s {
		return false
	}
	var k [3]float32
	for i := range 3 {
		k[i] = f.p[i][0] / s
		f.x[i] += k[i] * innov
	}
	row := f.p[0]
	for i := range 3 {
		for j := range 3 {
			f.p[i][j] -= k[i] * row[j]
		}
	}
	return true
}

// BaroHeight returns the height in m of pressure pa above reference pressure ref, both in Pa, at tempC °C.
// Hypsometric equation with ln(ref/pa) ≈ 2(ref-pa)/(ref+pa), https://en.wikipedia.org/wiki/Hypsometric_equation
func BaroHeight(pa, ref, tempC float32) float32 {
	const rd = 287.05 // J/(kg·K), specific gas constant of dry air
	scale := rd * (tempC + 273.15) / fmath.Gravity
	return scale * 2 * (ref - pa) / (ref + pa)
}
