// Package mixer splits a thrust and torque request into one command per motor.
// Same role as control_allocator in PX4.
package mixer

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Motor says how much one motor helps each torque axis.
// Positive yaw means the motor spins counter clockwise seen from above.
type Motor struct {
	Roll, Pitch, Yaw float32
}

// QuadX is a four motor X frame in the order front left, front right, rear left, rear right.
// Front right and rear left spin counter clockwise, like the PX4 Generic Quadcopter X.
var QuadX = []Motor{
	{Roll: 1, Pitch: 1, Yaw: -1},
	{Roll: -1, Pitch: 1, Yaw: 1},
	{Roll: 1, Pitch: -1, Yaw: 1},
	{Roll: -1, Pitch: -1, Yaw: -1},
}

// Config holds the motor layout and thrust settings.
type Config struct {
	Geometry []Motor
	// ThrustModel is 0 for thrust linear in command and 1 for thrust proportional to command squared.
	// Same as PX4 THR_MDL_FAC.
	ThrustModel float32
	Idle        float32 // command that keeps armed motors spinning
	AirMode     bool    // raise thrust at low throttle to keep full torque authority
}

// Mixer turns thrust and torque into motor commands.
type Mixer struct {
	cfg Config
	rp  [msg.MaxMotors]float32
}

// New returns a mixer using cfg.
func New(cfg Config) *Mixer { return &Mixer{cfg: cfg} }

// Count returns the number of motors.
func (m *Mixer) Count() int { return len(m.cfg.Geometry) }

// Mix fills out with motor commands from 0 to 1. Roll and pitch win over thrust, thrust wins over yaw.
func (m *Mixer) Mix(req *msg.ThrustTorque, out *msg.MotorOutputs) {
	n := len(m.cfg.Geometry)
	out.Count = n
	thrust := fmath.Clamp(req.Thrust, 0, 1)

	lo, hi := float32(0), float32(0)
	for i, g := range m.cfg.Geometry {
		m.rp[i] = req.Torque.X*g.Roll + req.Torque.Y*g.Pitch
		lo, hi = fmath.Min(lo, m.rp[i]), fmath.Max(hi, m.rp[i])
	}

	// Shrink roll and pitch if they need more range than the motors have.
	scale := float32(1)
	if hi-lo > 1 {
		scale = 1 / (hi - lo)
		lo, hi = lo*scale, hi*scale
	}
	if thrust+hi > 1 {
		thrust = 1 - hi
	}
	if thrust+lo < 0 {
		if m.cfg.AirMode {
			thrust = -lo
		} else if lo < 0 {
			scale *= thrust / -lo
		}
	}

	// Yaw gets whatever room is left.
	yawScale := float32(1)
	for i, g := range m.cfg.Geometry {
		y := req.Torque.Z * g.Yaw
		base := thrust + m.rp[i]*scale
		switch {
		case y > 0:
			yawScale = fmath.Min(yawScale, (1-base)/y)
		case y < 0:
			yawScale = fmath.Min(yawScale, base/-y)
		}
	}
	yawScale = fmath.Max(yawScale, 0)

	for i, g := range m.cfg.Geometry {
		t := thrust + m.rp[i]*scale + req.Torque.Z*g.Yaw*yawScale
		u := m.command(fmath.Clamp(t, 0, 1))
		out.Value[i] = m.cfg.Idle + (1-m.cfg.Idle)*u
	}
}

// command finds u where ThrustModel·u² + (1-ThrustModel)·u equals the wanted thrust t.
// Thrust returns the thrust from 0 to 1 that Mix turns into motor command duty, ignoring torque.
func (m *Mixer) Thrust(duty float32) float32 {
	u := fmath.Clamp((duty-m.cfg.Idle)/(1-m.cfg.Idle), 0, 1)
	k := m.cfg.ThrustModel
	return k*u*u + (1-k)*u
}

func (m *Mixer) command(t float32) float32 {
	k := m.cfg.ThrustModel
	if k <= 0 {
		return t
	}
	b := 1 - k
	return (-b + fmath.Sqrt(b*b+4*k*t)) / (2 * k)
}

// Stop sets every motor to 0.
func (m *Mixer) Stop(out *msg.MotorOutputs) {
	out.Count = len(m.cfg.Geometry)
	for i := range out.Value {
		out.Value[i] = 0
	}
}
