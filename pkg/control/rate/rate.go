// Package rate is the innermost loop. It turns a wanted spin rate into a torque request.
// Same role as mc_rate_control in PX4.
package rate

import (
	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

type Config struct {
	Roll, Pitch, Yaw pid.Config
}

type Controller struct {
	pids pid.Vec3
}

func New(cfg Config) *Controller {
	return &Controller{pids: pid.NewVec3(cfg.Roll, cfg.Pitch, cfg.Yaw)}
}

// Update returns a normalized torque from the rate setpoint and measured rate, both in rad/s.
func (c *Controller) Update(setpoint, rate fmath.Vec3, dt float32) fmath.Vec3 {
	return c.pids.Update(setpoint, rate, dt)
}

// Reset clears the integrators, which is done on the ground so they do not wind up before takeoff.
func (c *Controller) Reset() { c.pids.Reset() }
