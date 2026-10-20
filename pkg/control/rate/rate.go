// Package rate is the innermost loop. It turns a wanted spin rate into a torque request.
// Same role as mc_rate_control in PX4.
package rate

import (
	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// Config holds the PID settings for each axis.
type Config struct {
	Roll, Pitch, Yaw pid.Config
	// Trim is the torque the integrators start from and go back to on the ground. It is the steady
	// torque the vehicle needs to hover level, for example from motors that are not quite matched.
	Trim fmath.Vec3
}

// Controller turns body rate error into a torque request.
type Controller struct {
	pids pid.Vec3
	trim fmath.Vec3
}

// New returns a rate controller with one PID each for roll, pitch and yaw.
func New(cfg Config) *Controller {
	c := &Controller{pids: pid.NewVec3(cfg.Roll, cfg.Pitch, cfg.Yaw), trim: cfg.Trim}
	c.Reset()
	return c
}

// Update returns a normalized torque from the rate setpoint and measured rate, both in rad/s.
func (c *Controller) Update(setpoint, rate fmath.Vec3, dt float32) fmath.Vec3 {
	return c.pids.Update(setpoint, rate, dt)
}

// Integral returns the torque each integrator holds, which in a steady hover is the trim the vehicle needs.
func (c *Controller) Integral() fmath.Vec3 {
	return fmath.Vec3{X: c.pids[0].Integral(), Y: c.pids[1].Integral(), Z: c.pids[2].Integral()}
}

// SetTrim replaces the trim used by Reset, for example with what the integrators held on the last flight.
func (c *Controller) SetTrim(trim fmath.Vec3) { c.trim = trim }

// Trim returns the trim used by Reset.
func (c *Controller) Trim() fmath.Vec3 { return c.trim }

// Reset puts the integrators back to the trim, which is done on the ground so they do not wind up
// before takeoff but still hold what the vehicle needs at liftoff.
func (c *Controller) Reset() {
	c.pids.Reset()
	c.pids[0].SetIntegral(c.trim.X)
	c.pids[1].SetIntegral(c.trim.Y)
	c.pids[2].SetIntegral(c.trim.Z)
}
