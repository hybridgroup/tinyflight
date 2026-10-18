// Package sim is a simple quadcopter physics model with fake sensors, so the flight stack can fly on a PC.
// It runs only on the host and is used by the closed loop tests.
package sim

import (
	"math/rand"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/mixer"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type Params struct {
	Mass      float32    // kg
	Inertia   fmath.Vec3 // kg·m² about each body axis
	Arm       float32    // m from the center to each motor along X and along Y
	YawCoef   float32    // m, yaw torque per N of thrust
	MaxThrust float32    // N per motor at full command
	MotorTau  float32    // s, motor spin up time constant
	Drag      float32    // 1/s, linear air drag
	Geometry  []mixer.Motor

	GyroNoise  float32 // rad/s
	AccelNoise float32 // m/s²
	BaroNoise  float32 // Pa
	RangeNoise float32 // m
	GyroBias   fmath.Vec3
	SeaLevelPa float32
	GroundAlt  float32 // m above sea level
	RangeMax   float32 // m
	Voltage    float32 // V
}

// StampFly is a rough model of the M5Stack StampFly, about 37 g with 0.2 N motors.
func StampFly() Params {
	return Params{
		Mass:       0.037,
		Inertia:    fmath.Vec3{X: 1.0e-5, Y: 1.0e-5, Z: 2.0e-5},
		Arm:        0.023,
		YawCoef:    0.006,
		MaxThrust:  0.21,
		MotorTau:   0.02,
		Drag:       0.3,
		Geometry:   mixer.QuadX,
		GyroNoise:  0.01,
		AccelNoise: 0.1,
		BaroNoise:  3,
		RangeNoise: 0.005,
		GyroBias:   fmath.Vec3{X: 0.01, Y: -0.02, Z: 0.005},
		SeaLevelPa: 101325,
		GroundAlt:  100,
		RangeMax:   3,
		Voltage:    3.9,
	}
}

// Quad is the simulated vehicle. Position and velocity are NED with the ground at Z = 0.
type Quad struct {
	P      Params
	Pos    fmath.Vec3
	Vel    fmath.Vec3
	Q      fmath.Quat
	Omega  fmath.Vec3
	Thrust [msg.MaxMotors]float32 // N per motor, after spin up lag
	Cmd    [msg.MaxMotors]float32
	// Disturb is an extra body torque in N·m, for testing recovery.
	Disturb fmath.Vec3
	// IMUFail makes ReadIMU return an error.
	IMUFail bool

	accel fmath.Vec3
	rnd   *rand.Rand
}

func New(p Params) *Quad {
	return &Quad{P: p, Q: fmath.IdentityQuat(), rnd: rand.New(rand.NewSource(1))}
}

func (s *Quad) OnGround() bool { return s.Pos.Z >= 0 }

// Step advances the physics by dt seconds.
func (s *Quad) Step(dt float32) {
	p := &s.P
	var total float32
	var torque fmath.Vec3
	for i, g := range p.Geometry {
		want := p.MaxThrust * s.Cmd[i] * s.Cmd[i]
		s.Thrust[i] += (want - s.Thrust[i]) * fmath.Min(dt/p.MotorTau, 1)
		t := s.Thrust[i]
		total += t
		torque = torque.Add(fmath.Vec3{X: g.Roll * p.Arm * t, Y: g.Pitch * p.Arm * t, Z: g.Yaw * p.YawCoef * t})
	}
	torque = torque.Add(s.Disturb)

	force := s.Q.Rotate(fmath.Vec3{Z: -total}).Scale(1 / p.Mass)
	acc := force.Add(fmath.Vec3{Z: fmath.Gravity}).Sub(s.Vel.Scale(p.Drag))

	// I·ω' = τ - ω × (I·ω)
	iw := s.Omega.Mul(p.Inertia)
	wdot := torque.Sub(s.Omega.Cross(iw))
	wdot = fmath.Vec3{X: wdot.X / p.Inertia.X, Y: wdot.Y / p.Inertia.Y, Z: wdot.Z / p.Inertia.Z}

	s.Omega = s.Omega.Add(wdot.Scale(dt))
	s.Q = s.Q.Integrate(s.Omega, dt)
	s.Vel = s.Vel.Add(acc.Scale(dt))
	s.Pos = s.Pos.Add(s.Vel.Scale(dt))
	s.accel = acc

	if s.Pos.Z >= 0 && s.Vel.Z >= 0 {
		// Resting on the ground keeps it level and still.
		s.Pos.Z = 0
		s.Vel = fmath.Vec3{}
		s.Omega = fmath.Vec3{}
		s.Q = fmath.QuatFromEuler(0, 0, s.Q.Yaw())
		s.accel = fmath.Vec3{}
	}
}

func (s *Quad) noise(sd float32) float32 { return float32(s.rnd.NormFloat64()) * sd }

func (s *Quad) noiseVec(sd float32) fmath.Vec3 {
	return fmath.Vec3{X: s.noise(sd), Y: s.noise(sd), Z: s.noise(sd)}
}

// ReadIMU reports specific force and body rate, with the body axes as the chip axes.
func (s *Quad) ReadIMU(out *msg.IMUSample) error {
	if s.IMUFail {
		return errIMU
	}
	f := s.Q.RotateInv(s.accel.Sub(fmath.Vec3{Z: fmath.Gravity}))
	out.Accel = f.Add(s.noiseVec(s.P.AccelNoise))
	out.Gyro = s.Omega.Add(s.P.GyroBias).Add(s.noiseVec(s.P.GyroNoise))
	out.Temperature = 25
	return nil
}

func (s *Quad) ReadBaro(out *msg.BaroSample) error {
	alt := s.P.GroundAlt - s.Pos.Z
	out.Pressure = s.P.SeaLevelPa*fmath.Pow(1-alt/44330, 5.255) + s.noise(s.P.BaroNoise)
	out.Temperature = 25
	return nil
}

func (s *Quad) ReadRange(out *msg.RangeSample) error {
	c := s.Q.Down().Z
	out.Valid = false
	if c < 0.5 {
		return nil
	}
	d := (-s.Pos.Z+0.01)/c + s.noise(s.P.RangeNoise)
	out.Distance = d
	out.Valid = d > 0 && d < s.P.RangeMax
	return nil
}

func (s *Quad) ReadBattery(out *msg.BatteryStatus) error {
	out.Voltage = s.P.Voltage
	return nil
}

func (s *Quad) SetMotors(m *msg.MotorOutputs) error {
	for i := range m.Count {
		s.Cmd[i] = fmath.Clamp(m.Value[i], 0, 1)
	}
	return nil
}

type simError string

func (e simError) Error() string { return string(e) }

const errIMU = simError("sim: imu failed")

// Pilot is a scripted pilot. Set In and it is delivered on the next read unless Silent is true.
type Pilot struct {
	In     msg.ManualInput
	Silent bool
}

func (p *Pilot) ReadInput(in *msg.ManualInput) bool {
	if p.Silent {
		return false
	}
	*in = p.In
	in.Valid = true
	return true
}
