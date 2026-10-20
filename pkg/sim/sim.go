// Package sim is a simple quadcopter physics model with fake sensors, so the flight stack can fly on a PC.
// It runs only on the host and is used by the closed loop tests.
package sim

import (
	"math"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/mixer"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Params holds the physical properties and sensor noise of the simulated quad.
type Params struct {
	Mass      float32    // kg
	Inertia   fmath.Vec3 // kg·m² about each body axis
	Arm       float32    // m from the center to each motor along X and along Y
	YawCoef   float32    // m, yaw torque per N of thrust
	MaxThrust float32    // N per motor at full command
	ThrustExp float32    // thrust goes with command to this power, 0 means 2
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
	RangeDelay float32 // s, how old a range reading is when it arrives
	Voltage    float32 // V
	FlowScale  float32 // rad of image motion per flow count
	FlowNoise  float32 // counts per reading
	FlowMin    float32 // m, closest the flow sensor can focus
	FlowMax    float32 // rad/s, fastest image motion the flow sensor can follow
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
		FlowScale:  0.00222,
		FlowNoise:  1,
		FlowMin:    0.08,
		FlowMax:    7.4,
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
	// FlowLost makes the flow sensor report a surface it cannot track.
	FlowLost bool
	// FlowCloseValid keeps flow quality good below FlowMin, as the PMW3901 did on the bench when set down.
	FlowCloseValid bool

	accel fmath.Vec3
	flow  fmath.Vec3 // flow counts gathered since the last ReadFlow
	time  float32
	hist  [512]heightAt // recent heights for RangeDelay
	histN int
	rnd   *Rand
}

// New returns a level quad resting on the ground.
func New(p Params) *Quad {
	return &Quad{P: p, Q: fmath.IdentityQuat(), rnd: NewRand(1)}
}

// OnGround reports whether the quad is at or below ground level.
func (s *Quad) OnGround() bool { return s.Pos.Z >= 0 }

// Step advances the physics by dt seconds.
func (s *Quad) Step(dt float32) {
	p := &s.P
	var total float32
	var torque fmath.Vec3
	for i, g := range p.Geometry {
		want := p.MaxThrust * s.Cmd[i] * s.Cmd[i]
		if p.ThrustExp != 0 && p.ThrustExp != 2 {
			want = p.MaxThrust * float32(math.Pow(float64(s.Cmd[i]), float64(p.ThrustExp)))
		}
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
	s.flow = s.flow.Add(s.flowRate().Scale(dt / p.FlowScale))
	s.time += dt
	s.hist[s.histN%len(s.hist)] = heightAt{s.time, s.Pos.Z}
	s.histN++
}

type heightAt struct{ t, z float32 }

// heightBefore returns the height RangeDelay seconds ago, or now when there is no delay.
func (s *Quad) heightBefore() float32 {
	if s.P.RangeDelay <= 0 || s.histN == 0 {
		return s.Pos.Z
	}
	want := s.time - s.P.RangeDelay
	n := min(s.histN, len(s.hist))
	for i := 1; i <= n; i++ {
		h := s.hist[(s.histN-i)%len(s.hist)]
		if h.t <= want {
			return h.z
		}
	}
	return s.hist[(s.histN-n)%len(s.hist)].z
}

// flowRate is how fast the ground below appears to move in body axes, in rad/s.
// Translation moves it by speed over distance, and rotation by the body rate.
func (s *Quad) flowRate() fmath.Vec3 {
	d := s.flowDistance()
	if d <= 0 {
		return fmath.Vec3{}
	}
	v := s.Q.RotateInv(s.Vel)
	m := s.P.FlowMax
	return fmath.Vec3{X: fmath.Clamp(v.X/d+s.Omega.Y, -m, m), Y: fmath.Clamp(v.Y/d-s.Omega.X, -m, m)}
}

// flowDistance is the distance to the ground along the body Z axis, or 0 when it cannot see the ground.
func (s *Quad) flowDistance() float32 {
	c := s.Q.Down().Z
	if c < 0.5 {
		return 0
	}
	return (-s.Pos.Z + 0.01) / c
}

func (s *Quad) noise(sd float32) float32 { return s.rnd.Normal() * sd }

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

// ReadBaro reports the pressure at the current height with noise added.
func (s *Quad) ReadBaro(out *msg.BaroSample) error {
	// Inverse of altitude.BaroHeight, from sea level.
	const temp = 25
	scale := 287.05 * (temp + 273.15) / fmath.Gravity
	h := s.P.GroundAlt - s.Pos.Z
	out.Pressure = s.P.SeaLevelPa*(2*scale-h)/(2*scale+h) + s.noise(s.P.BaroNoise)
	out.Temperature = temp
	return nil
}

// ReadRange reports the distance to the ground along the body Z axis. It is invalid when tilted too far.
func (s *Quad) ReadRange(out *msg.RangeSample) error {
	c := s.Q.Down().Z
	out.Valid = false
	if c < 0.5 {
		return nil
	}
	d := (-s.heightBefore()+0.01)/c + s.noise(s.P.RangeNoise)
	out.Distance = d
	out.Valid = d > 0 && d < s.P.RangeMax
	return nil
}

// ReadFlow reports the flow counts since the last read in body axes, with noise added.
func (s *Quad) ReadFlow(out *msg.FlowSample) error {
	c := fmath.Vec3{X: round(s.flow.X + s.noise(s.P.FlowNoise)), Y: round(s.flow.Y + s.noise(s.P.FlowNoise))}
	s.flow = s.flow.Sub(c)
	out.DeltaX, out.DeltaY = int16(c.X), int16(c.Y)
	out.Quality = 100
	out.Valid = true
	if s.FlowLost || (s.flowDistance() < s.P.FlowMin && !s.FlowCloseValid) {
		out.Quality = 10
		out.Valid = false
	}
	return nil
}

func round(x float32) float32 {
	if x < 0 {
		return -float32(int32(-x + 0.5))
	}
	return float32(int32(x + 0.5))
}

// ReadBattery reports the configured voltage.
func (s *Quad) ReadBattery(out *msg.BatteryStatus) error {
	out.Voltage = s.P.Voltage
	return nil
}

// SetMotors stores the motor commands for the next Step.
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

// ReadInput delivers In unless Silent is true.
func (p *Pilot) ReadInput(in *msg.ManualInput) bool {
	if p.Silent {
		return false
	}
	*in = p.In
	in.Valid = true
	return true
}
