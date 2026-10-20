// Package horizontal estimates north and east position and speed.
// Horizontal accel drives the prediction, then optical flow speed corrects it.
package horizontal

import (
	"github.com/hybridgroup/tinyflight/pkg/estimator/axis"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// Config holds the noise and gating settings for the filter.
type Config struct {
	AccelNoise float32 // m/s², accel noise density used in the prediction
	BiasNoise  float32 // m/s², how fast the accel bias may wander per second
	FlowNoise  float32 // rad/s, flow rate standard deviation
	Gate       float32 // reject readings further than this many standard deviations away, 0 disables
	MinQuality uint8   // lowest flow surface quality to use
	MinHeight  float32 // m, the flow sensor cannot focus closer than this
	MaxHeight  float32 // m, ignore flow beyond this
	FlowHold   float32 // s, how long the estimate stays valid without flow
	// MaxRate skips readings with a flow or gyro rate above this in rad/s, since the sensor cannot follow them.
	// Same idea as SENS_FLOW_MAXR in PX4, https://docs.px4.io/main/en/advanced_config/parameter_reference.html
	MaxRate float32
}

// Filter runs one axis filter for north and one for east.
type Filter struct {
	cfg     Config
	n, e    axis.Filter
	flowAge float32

	meas    fmath.Vec3 // last flow speed, earth frame
	acceptN bool
	acceptE bool
}

// New returns a filter at the origin.
func New(cfg Config) *Filter {
	f := &Filter{cfg: cfg}
	f.Reset()
	return f
}

// Reset makes the current position the origin. Call it on the ground.
func (f *Filter) Reset() {
	f.n.Reset(0, 1, 0.1)
	f.e.Reset(0, 1, 0.1)
	f.flowAge = f.cfg.FlowHold + 1
}

// X returns the position north in m.
func (f *Filter) X() float32 { return f.n.X[axis.Pos] }

// Y returns the position east in m.
func (f *Filter) Y() float32 { return f.e.X[axis.Pos] }

// VX returns the speed north in m/s.
func (f *Filter) VX() float32 { return f.n.X[axis.Vel] }

// VY returns the speed east in m/s.
func (f *Filter) VY() float32 { return f.e.X[axis.Vel] }

// LastFlow returns the earth frame speed from the last flow reading used and whether each axis accepted it.
func (f *Filter) LastFlow() (v fmath.Vec3, acceptN, acceptE bool) {
	return f.meas, f.acceptN, f.acceptE
}

// Valid reports whether recent flow readings are driving the estimate.
func (f *Filter) Valid() bool { return f.flowAge <= f.cfg.FlowHold }

// Predict moves the state forward using earth frame accel in m/s² with gravity removed.
func (f *Filter) Predict(accel fmath.Vec3, dt float32) {
	f.n.Predict(accel.X, dt, f.cfg.AccelNoise, f.cfg.BiasNoise)
	f.e.Predict(accel.Y, dt, f.cfg.AccelNoise, f.cfg.BiasNoise)
	f.flowAge += dt
	if !f.Valid() {
		// Accel alone drifts away fast, so stop and wait for flow.
		hold(&f.n)
		hold(&f.e)
	}
}

// hold keeps the position and bias but zeroes the speed and makes it uncertain.
func hold(a *axis.Filter) {
	pos, bias, pv, bv := a.X[axis.Pos], a.X[axis.Bias], a.P[axis.Pos][axis.Pos], a.P[axis.Bias][axis.Bias]
	a.Reset(pv, 1, bv)
	a.X[axis.Pos], a.X[axis.Bias] = pos, bias
}

// UpdateFlow corrects with a flow rate in body axes and the mean gyro rate over the same time, both in rad/s.
// dist is the distance to the ground along the body Z axis in m, vz the vertical speed and q the attitude.
func (f *Filter) UpdateFlow(rate, gyro fmath.Vec3, dist, vz float32, q fmath.Quat, quality uint8) bool {
	if quality < f.cfg.MinQuality || dist < f.cfg.MinHeight || dist > f.cfg.MaxHeight || q.Down().Z < 0.7 {
		return false
	}
	if m := f.cfg.MaxRate; m > 0 && (fmath.Abs(rate.X) > m || fmath.Abs(rate.Y) > m || fmath.Abs(gyro.X) > m || fmath.Abs(gyro.Y) > m) {
		return false
	}
	// Turning moves the image too, so remove the gyro rate to leave only the part from moving.
	vb := fmath.Vec3{X: dist * (rate.X - gyro.Y), Y: dist * (rate.Y + gyro.X)}
	// Flow cannot see speed along the body Z axis, so take it from the current estimate.
	vb.Z = q.RotateInv(fmath.Vec3{X: f.VX(), Y: f.VY(), Z: vz}).Z
	v := q.Rotate(vb)
	sd := f.cfg.FlowNoise * dist
	okN := f.n.Update(axis.Vel, v.X, sd, f.cfg.Gate)
	okE := f.e.Update(axis.Vel, v.Y, sd, f.cfg.Gate)
	f.meas, f.acceptN, f.acceptE = v, okN, okE
	if okN || okE {
		f.flowAge = 0
	}
	return okN || okE
}
