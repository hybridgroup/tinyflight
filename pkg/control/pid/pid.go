// Package pid has a PID controller, the basic building block of every control loop here.
// It pushes harder the bigger the error (P), the longer it lasts (I) and the faster it changes (D).
package pid

import (
	"github.com/hybridgroup/tinyflight/pkg/filter"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// Config holds the gains and limits for one PID.
type Config struct {
	P, I, D       float32
	FF            float32 // feedforward, output added straight from the setpoint
	IntegralLimit float32 // largest output the I term may hold
	OutputLimit   float32 // 0 means no limit
	DCutoffHz     float32 // low-pass on the D term, 0 disables
	SampleHz      float32
}

// PID is a single axis PID controller with a filtered D term and integral anti windup.
type PID struct {
	cfg      Config
	integral float32
	prev     float32
	hasPrev  bool
	dFilter  filter.LowPass1
}

// New returns a PID controller using cfg.
func New(cfg Config) PID {
	return PID{cfg: cfg, dFilter: filter.NewLowPass1(cfg.DCutoffHz, cfg.SampleHz)}
}

// Reset clears the integral and derivative state.
func (c *PID) Reset() {
	c.integral = 0
	c.hasPrev = false
	c.dFilter.Reset()
}

// Integral returns the current output held by the I term.
func (c *PID) Integral() float32 { return c.integral }

// Update returns the output for one step of dt seconds.
// D acts on the measurement, not the error, so setpoint jumps do not kick the output.
func (c *PID) Update(setpoint, measured, dt float32) float32 {
	err := setpoint - measured

	var d float32
	if c.hasPrev && dt > 0 {
		d = c.dFilter.Apply(-(measured - c.prev) / dt)
	}
	c.prev, c.hasPrev = measured, true

	out := c.cfg.P*err + c.integral + c.cfg.D*d + c.cfg.FF*setpoint
	if c.cfg.OutputLimit > 0 {
		limited := fmath.Clamp(out, -c.cfg.OutputLimit, c.cfg.OutputLimit)
		// Stop integrating while saturated in the direction of the error.
		if limited != out && (out > 0) == (err > 0) {
			return limited
		}
		out = limited
	}
	c.integral = fmath.Clamp(c.integral+c.cfg.I*err*dt, -c.cfg.IntegralLimit, c.cfg.IntegralLimit)
	return out
}

// Vec3 runs one PID per axis, such as roll, pitch and yaw.
type Vec3 [3]PID

// NewVec3 returns one PID per axis, each with its own config.
func NewVec3(x, y, z Config) Vec3 { return Vec3{New(x), New(y), New(z)} }

// Update runs each axis PID for one step of dt seconds.
func (v *Vec3) Update(setpoint, measured fmath.Vec3, dt float32) fmath.Vec3 {
	return fmath.Vec3{
		X: v[0].Update(setpoint.X, measured.X, dt),
		Y: v[1].Update(setpoint.Y, measured.Y, dt),
		Z: v[2].Update(setpoint.Z, measured.Z, dt),
	}
}

// Reset clears the state of every axis.
func (v *Vec3) Reset() {
	for i := range v {
		v[i].Reset()
	}
}
