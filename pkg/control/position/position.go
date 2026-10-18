// Package position holds a height by turning height error into a climb rate, then climb rate error into thrust.
// Same role as the vertical part of mc_pos_control in PX4. Horizontal hold comes later.
package position

import (
	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

type Config struct {
	ZP          float32 // m/s of climb rate per m of height error
	Climb       pid.Config
	HoverThrust float32 // thrust that holds the vehicle level in still air
	MaxClimb    float32 // m/s
	MaxDescend  float32 // m/s
	MinThrust   float32
	MaxThrust   float32
}

type Vertical struct {
	cfg   Config
	climb pid.PID
}

func NewVertical(cfg Config) *Vertical {
	return &Vertical{cfg: cfg, climb: pid.New(cfg.Climb)}
}

func (c *Vertical) Reset() { c.climb.Reset() }

// Update returns collective thrust from 0 to 1. Heights and speeds are NED, so up is negative.
// climbFF is an extra climb rate in m/s, for example from the throttle stick.
func (c *Vertical) Update(zSetpoint, z, vz, climbFF, cosTilt, dt float32) float32 {
	climbSp := c.cfg.ZP*(z-zSetpoint) + climbFF
	climbSp = fmath.Clamp(climbSp, -c.cfg.MaxDescend, c.cfg.MaxClimb)
	u := c.climb.Update(climbSp, -vz, dt)
	// Tilting points some thrust sideways, so add more to keep the same lift.
	thrust := (c.cfg.HoverThrust + u) / fmath.Max(cosTilt, 0.5)
	return fmath.Clamp(thrust, c.cfg.MinThrust, c.cfg.MaxThrust)
}
