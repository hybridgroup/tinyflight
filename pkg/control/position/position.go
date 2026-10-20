// Package position holds a height by turning height error into a climb rate, then climb rate error into thrust,
// and a position by turning position error into a speed, then speed error into tilt. Same role as mc_pos_control in PX4.
package position

import (
	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// Config holds the height and climb rate gains and limits.
type Config struct {
	ZP          float32 // m/s of climb rate per m of height error
	Climb       pid.Config
	HoverThrust float32 // thrust that holds the vehicle level in still air
	MaxClimb    float32 // m/s
	MaxDescend  float32 // m/s
	MinThrust   float32
	MaxThrust   float32
	// MaxHover caps thrust at this multiple of hover thrust, so a wrong height estimate cannot run off at full power. 0 for no cap.
	MaxHover float32
}

// Vertical turns a height setpoint into collective thrust.
type Vertical struct {
	cfg   Config
	climb pid.PID
	hover float32
}

// NewVertical returns a height controller using cfg.
func NewVertical(cfg Config) *Vertical {
	return &Vertical{cfg: cfg, climb: pid.New(cfg.Climb), hover: cfg.HoverThrust}
}

// SetHover replaces the hover thrust from the config, for example with one worked out from the battery voltage.
func (c *Vertical) SetHover(thrust float32) { c.hover = thrust }

// Reset clears the climb rate integrator.
func (c *Vertical) Reset() { c.climb.Reset() }

// Update returns collective thrust from 0 to 1. Heights and speeds are NED, so up is negative.
// climbFF is an extra climb rate in m/s, for example from the throttle stick.
func (c *Vertical) Update(zSetpoint, z, vz, climbFF, cosTilt, dt float32) float32 {
	climbSp := c.cfg.ZP*(z-zSetpoint) + climbFF
	climbSp = fmath.Clamp(climbSp, -c.cfg.MaxDescend, c.cfg.MaxClimb)
	u := c.climb.Update(climbSp, -vz, dt)
	// Tilting points some thrust sideways, so add more to keep the same lift.
	thrust := (c.hover + u) / fmath.Max(cosTilt, 0.5)
	hi := c.cfg.MaxThrust
	if c.cfg.MaxHover > 0 {
		hi = fmath.Min(hi, c.cfg.MaxHover*c.hover)
	}
	return fmath.Clamp(thrust, c.cfg.MinThrust, hi)
}
