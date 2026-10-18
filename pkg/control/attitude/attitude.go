// Package attitude turns a wanted orientation into a wanted spin rate for the rate loop.
// Same role as mc_att_control in PX4.
package attitude

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// Config holds the attitude gains and rate limits.
type Config struct {
	P       fmath.Vec3 // rad/s of rate per rad of error, per axis
	MaxRate fmath.Vec3 // rad/s
}

// Controller turns attitude error into a body rate setpoint.
type Controller struct {
	cfg Config
}

// New returns an attitude controller using cfg.
func New(cfg Config) *Controller { return &Controller{cfg: cfg} }

// Update returns a body rate setpoint in rad/s.
// yawRate in rad/s is fed straight through so turning does not lag behind the stick.
func (c *Controller) Update(q, setpoint fmath.Quat, yawRate float32) fmath.Vec3 {
	e := Error(q, setpoint)
	rate := e.Mul(c.cfg.P).Add(q.Down().Scale(yawRate))
	return rate.Clamp(c.cfg.MaxRate)
}

// Error is the rotation from q to setpoint as a body frame axis times angle, in rad.
func Error(q, setpoint fmath.Quat) fmath.Vec3 {
	qe := q.Conj().Mul(setpoint)
	if qe.W < 0 {
		// Take the short way around.
		qe = fmath.Quat{W: -qe.W, X: -qe.X, Y: -qe.Y, Z: -qe.Z}
	}
	return fmath.Vec3{X: qe.X, Y: qe.Y, Z: qe.Z}.Scale(2)
}
