package position

import (
	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// HorizontalConfig holds the position and speed gains and limits.
type HorizontalConfig struct {
	XYP      float32    // m/s of speed per m of position error
	Vel      pid.Config // speed error in m/s to accel in m/s²
	MaxSpeed float32    // m/s
	MaxTilt  float32    // rad
}

// Horizontal turns a north and east position setpoint into roll and pitch angles.
type Horizontal struct {
	cfg    HorizontalConfig
	vn, ve pid.PID
}

// NewHorizontal returns a horizontal position controller using cfg.
func NewHorizontal(cfg HorizontalConfig) *Horizontal {
	return &Horizontal{cfg: cfg, vn: pid.New(cfg.Vel), ve: pid.New(cfg.Vel)}
}

// Reset clears the speed integrators.
func (c *Horizontal) Reset() {
	c.vn.Reset()
	c.ve.Reset()
}

// Update returns roll and pitch in rad. Positions and speeds are NED and only X and Y are used.
// velFF is an extra speed in m/s, for example from the sticks, and yaw is the heading in rad.
func (c *Horizontal) Update(posSp, pos, vel, velFF fmath.Vec3, yaw, dt float32) (roll, pitch float32) {
	velSp := fmath.Vec3{X: posSp.X - pos.X, Y: posSp.Y - pos.Y}.Scale(c.cfg.XYP).Add(velFF)
	velSp.Z = 0
	if n := velSp.Norm(); n > c.cfg.MaxSpeed {
		velSp = velSp.Scale(c.cfg.MaxSpeed / n)
	}
	an := c.vn.Update(velSp.X, vel.X, dt)
	ae := c.ve.Update(velSp.Y, vel.Y, dt)

	// Turn the accel into the heading frame, then tilt toward it. Nose down speeds up forward.
	s, co := fmath.SinCos(yaw)
	fwd := co*an + s*ae
	right := -s*an + co*ae
	pitch = fmath.Clamp(-fmath.Atan2(fwd, fmath.Gravity), -c.cfg.MaxTilt, c.cfg.MaxTilt)
	roll = fmath.Clamp(fmath.Atan2(right, fmath.Gravity), -c.cfg.MaxTilt, c.cfg.MaxTilt)
	return roll, pitch
}
