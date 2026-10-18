// Package attitude estimates orientation by integrating the gyro and correcting drift with gravity and north.
// Mahony filter, see Mahony et al., "Nonlinear Complementary Filters on the Special Orthogonal Group", IEEE TAC 2008.
package attitude

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

type Config struct {
	Kp        float32 // how fast accel and mag pull the estimate back
	Ki        float32 // how fast gyro bias is learned
	AccelGate float32 // skip accel correction when |accel| differs from g by more than this fraction
	UseMag    bool
}

type Mahony struct {
	cfg      Config
	q        fmath.Quat
	integral fmath.Vec3
	mag      fmath.Vec3
	hasMag   bool
	init     bool
}

func New(cfg Config) *Mahony {
	return &Mahony{cfg: cfg, q: fmath.IdentityQuat()}
}

func (m *Mahony) Attitude() fmath.Quat { return m.q }

// Reset sets roll and pitch from a still accelerometer reading and yaw to 0.
func (m *Mahony) Reset(accel fmath.Vec3) {
	roll := fmath.Atan2(-accel.Y, -accel.Z)
	pitch := fmath.Atan2(accel.X, fmath.Sqrt(accel.Y*accel.Y+accel.Z*accel.Z))
	m.q = fmath.QuatFromEuler(roll, pitch, 0)
	m.integral = fmath.Vec3{}
	m.init = true
}

// SetMag stores a body frame mag reading for the next Update.
func (m *Mahony) SetMag(field fmath.Vec3) {
	m.mag = field
	m.hasMag = true
}

// Update advances the estimate with gyro in rad/s and accel in m/s² over dt seconds.
func (m *Mahony) Update(gyro, accel fmath.Vec3, dt float32) {
	if !m.init {
		m.Reset(accel)
	}

	down := m.q.Down()
	var e fmath.Vec3

	// Still, the accelerometer reads the opposite of gravity, so it should point along -down.
	if n := accel.Norm(); n > 0 && fmath.Abs(n-fmath.Gravity) < m.cfg.AccelGate*fmath.Gravity {
		measured := accel.Scale(-1 / n)
		e = e.Add(measured.Cross(down))
	}

	if m.cfg.UseMag && m.hasMag {
		e = e.Add(m.magError(down))
		m.hasMag = false
	}

	if m.cfg.Ki > 0 {
		m.integral = m.integral.Add(e.Scale(m.cfg.Ki * dt))
	}
	corrected := gyro.Add(e.Scale(m.cfg.Kp)).Add(m.integral)
	m.q = m.q.Integrate(corrected, dt)
}

// magError only corrects yaw, so a disturbed mag cannot tilt the roll and pitch estimate.
func (m *Mahony) magError(down fmath.Vec3) fmath.Vec3 {
	if m.mag.Norm() == 0 {
		return fmath.Vec3{}
	}
	mb := m.mag.Normalized()
	h := m.q.Rotate(mb)
	ref := fmath.Vec3{X: fmath.Sqrt(h.X*h.X + h.Y*h.Y), Z: h.Z}
	expected := m.q.RotateInv(ref)
	e := mb.Cross(expected)
	return down.Scale(e.Dot(down))
}
