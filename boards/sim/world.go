// Package sim is a simulated StampFly built on the pkg/sim physics, so the firmware can fly in a browser.
package sim

import (
	"math"

	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/mixer"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"github.com/hybridgroup/tinyflight/pkg/params"
	physics "github.com/hybridgroup/tinyflight/pkg/sim"
)

// MAC is the address of the simulated drone. Joystick packets start with its last 3 bytes.
var MAC = [6]byte{0x02, 0x54, 0x46, 0x53, 0x49, 0x4D}

// Substeps is how many physics steps run for each pass of the flight loop.
const Substeps = 4

// World is the simulated drone and the hardware the flight stack sees.
type World struct {
	Quad     *physics.Quad
	Joystick Joystick
	LED      LED
	Hardware flight.Hardware
	now      msg.Time
	dt       float32
	// imbalance is torque per N of total thrust.
	imbalance fmath.Vec3
}

// New returns a drone resting on the ground, with sensors that match the chip axes in cfg.
func New(cfg params.Config) *World {
	p := physics.StampFly()
	p.FlowScale = cfg.Sensors.FlowScale
	p.RangeDelay = cfg.AltEst.RangeDelay
	q := physics.New(p)
	w := &World{Quad: q, dt: 1 / cfg.LoopHz, imbalance: imbalance(cfg, p)}
	w.Joystick.dec = input.NewAtomJoystick(MAC)
	w.Hardware = flight.Hardware{
		IMU:    chipIMU{q, cfg.Sensors.Rotation.Transpose()},
		Motors: q,
		Input:  &w.Joystick,
		Baro:   q,
		Range:  q,
		Flow:   chipFlow{q, cfg.Sensors.FlowRotation.Transpose()},
		Power:  q,
		LED:    &w.LED,
	}
	return w
}

// imbalance is the torque per N of thrust that cfg.Rate.Trim cancels at hover. The real StampFly needs
// that trim to lift off level, see boards/stampfly/config.go, so the model gets the same imbalance.
func imbalance(cfg params.Config, p physics.Params) fmath.Vec3 {
	hover := cfg.Hover.Offset + cfg.Hover.Slope*p.Voltage
	if cfg.Hover.Slope == 0 {
		hover = cfg.Vertical.HoverThrust
	}
	m := mixer.New(cfg.Mixer)
	var level, trimmed msg.MotorOutputs
	m.Mix(&msg.ThrustTorque{Thrust: hover}, &level)
	m.Mix(&msg.ThrustTorque{Thrust: hover, Torque: cfg.Rate.Trim}, &trimmed)
	var t fmath.Vec3
	var total float32
	for i, g := range p.Geometry {
		a, b := trimmed.Value[i], level.Value[i]
		d := p.MaxThrust * (a*a - b*b)
		t = t.Add(fmath.Vec3{X: g.Roll * p.Arm * d, Y: g.Pitch * p.Arm * d, Z: g.Yaw * p.YawCoef * d})
		total += p.MaxThrust * b * b
	}
	return t.Scale(-1 / total)
}

// Now is the simulated time since power on.
func (w *World) Now() msg.Time { return w.now }

// Step advances the physics by one loop period, then runs one pass of the flight loop.
func (w *World) Step(f *flight.Flight) {
	for range Substeps {
		var thrust float32
		for _, t := range w.Quad.Thrust {
			thrust += t
		}
		w.Quad.Disturb = w.imbalance.Scale(thrust)
		w.Quad.Step(w.dt / Substeps)
	}
	w.now += msg.Time(w.dt * 1e6)
	f.Step(w.now)
}

// Offsets into the state record that State fills.
const (
	StateTime       = 0  // s
	StatePos        = 1  // x, y, z in m, NED with the ground at z = 0
	StateVel        = 4  // m/s, NED
	StateQuat       = 7  // w, x, y, z, body to NED
	StateMotors     = 11 // 4 commands, 0 to 1
	StateLED        = 15 // r, g, b, 0 to 255
	StateEstPos     = 18 // estimated x, y, z in m
	StateArmed      = 21
	StateMode       = 22 // msg.FlightMode
	StateFailsafe   = 23 // msg.Failsafe
	StateLanding    = 24
	StateBattery    = 25 // V
	StateCalibrated = 26
	StateXYValid    = 27
	StateSize       = 28
)

// State fills out with the true and estimated state of the drone.
func (w *World) State(f *flight.Flight, out *[StateSize]float32) {
	q := w.Quad
	out[StateTime] = float32(w.now) * 1e-6
	putVec(out[StatePos:], q.Pos)
	putVec(out[StateVel:], q.Vel)
	out[StateQuat], out[StateQuat+1], out[StateQuat+2], out[StateQuat+3] = q.Q.W, q.Q.X, q.Q.Y, q.Q.Z
	for i := range 4 {
		out[StateMotors+i] = q.Cmd[i]
	}
	out[StateLED], out[StateLED+1], out[StateLED+2] = float32(w.LED.R), float32(w.LED.G), float32(w.LED.B)
	pos := f.Position()
	putVec(out[StateEstPos:], fmath.Vec3{X: pos.X, Y: pos.Y, Z: pos.Z})
	s := f.Status()
	out[StateArmed] = flag(s.Armed)
	out[StateMode] = float32(s.Mode)
	out[StateFailsafe] = float32(s.Failsafe)
	out[StateLanding] = flag(s.Landing)
	out[StateBattery] = f.Battery().Voltage
	out[StateCalibrated] = flag(f.Calibrated())
	out[StateXYValid] = flag(pos.XYValid)
}

func putVec(out []float32, v fmath.Vec3) { out[0], out[1], out[2] = v.X, v.Y, v.Z }

func flag(b bool) float32 {
	if b {
		return 1
	}
	return 0
}

// Joystick is the radio link. It decodes Atom Joystick packets with the same decoder as the real radio.
type Joystick struct {
	dec      *input.AtomJoystick
	packet   [input.AtomPacketSize]byte
	seq      uint32
	read     uint32
	Packets  uint32
	Rejected uint32
}

// Send delivers a packet as if it came over the air.
func (j *Joystick) Send(p []byte) {
	j.packet = [input.AtomPacketSize]byte{}
	copy(j.packet[:], p)
	j.seq++
}

// ReadInput decodes the newest packet, if there is one.
func (j *Joystick) ReadInput(in *msg.ManualInput) bool {
	if j.read == j.seq {
		return false
	}
	j.read = j.seq
	if err := j.dec.Decode(j.packet[:], 0, in); err != nil {
		j.Rejected++
		return false
	}
	j.Packets++
	return true
}

// LED keeps the color the flight stack last set.
type LED struct{ R, G, B uint8 }

// SetColor stores the color.
func (l *LED) SetColor(r, g, b uint8) { l.R, l.G, l.B = r, g, b }

// Full scale of the BMI270 as boards/stampfly/stampfly.go sets it up. A reading past it clips.
const (
	accelRange = 16 * fmath.Gravity
	gyroRange  = 2000 * math.Pi / 180
)

// chipIMU turns the body axes of the physics into the BMI270 chip axes the config expects.
type chipIMU struct {
	q   *physics.Quad
	inv fmath.Mat3
}

func (c chipIMU) ReadIMU(s *msg.IMUSample) error {
	if err := c.q.ReadIMU(s); err != nil {
		return err
	}
	s.Accel = clip(c.inv.MulVec(s.Accel), accelRange)
	s.Gyro = clip(c.inv.MulVec(s.Gyro), gyroRange)
	return nil
}

func clip(v fmath.Vec3, r float32) fmath.Vec3 {
	return fmath.Vec3{X: fmath.Clamp(v.X, -r, r), Y: fmath.Clamp(v.Y, -r, r), Z: fmath.Clamp(v.Z, -r, r)}
}

// chipFlow turns the body axes of the physics into the PMW3901 chip axes the config expects.
type chipFlow struct {
	q   *physics.Quad
	inv fmath.Mat3
}

func (c chipFlow) ReadFlow(s *msg.FlowSample) error {
	if err := c.q.ReadFlow(s); err != nil {
		return err
	}
	v := c.inv.MulVec(fmath.Vec3{X: float32(s.DeltaX), Y: float32(s.DeltaY)})
	s.DeltaX, s.DeltaY = int16(v.X), int16(v.Y)
	return nil
}
