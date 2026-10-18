// Package sensors cleans up raw sensor readings before the estimators use them.
// It turns chip axes into body axes, removes gyro bias and filters out noise.
package sensors

import (
	"github.com/hybridgroup/tinyflight/pkg/filter"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type Config struct {
	Rotation      fmath.Mat3 // IMU chip axes to body axes
	MagRotation   fmath.Mat3 // mag chip axes to body axes
	SampleHz      float32
	GyroCutoffHz  float32
	AccelCutoffHz float32
	NotchHz       float32 // 0 disables the gyro notch
	NotchQ        float32
	AccelOffset   fmath.Vec3 // m/s², subtracted after rotation
	MagOffset     fmath.Vec3 // µT hard iron offset, subtracted after rotation

	CalibrationSamples int
	StillThreshold     float32 // rad/s, gyro spread allowed while calibrating
}

type IMU struct {
	cfg        Config
	gyroLPF    filter.BiquadVec3
	gyroNotch  filter.BiquadVec3
	accelLPF   filter.BiquadVec3
	gyroBias   fmath.Vec3
	calibrated bool

	calSum      fmath.Vec3
	calMin      fmath.Vec3
	calMax      fmath.Vec3
	calCount    int
	calibrating bool
}

func NewIMU(cfg Config) *IMU {
	return &IMU{
		cfg:       cfg,
		gyroLPF:   filter.NewBiquadVec3(filter.NewLowPass2(cfg.GyroCutoffHz, cfg.SampleHz)),
		gyroNotch: filter.NewBiquadVec3(filter.NewNotch(cfg.NotchHz, cfg.NotchQ, cfg.SampleHz)),
		accelLPF:  filter.NewBiquadVec3(filter.NewLowPass2(cfg.AccelCutoffHz, cfg.SampleHz)),
	}
}

// Calibrate starts measuring gyro bias. The vehicle must stay still until Calibrated returns true.
func (p *IMU) Calibrate() {
	p.calibrating = true
	p.calCount = 0
	p.calSum = fmath.Vec3{}
}

func (p *IMU) Calibrated() bool { return p.calibrated }

func (p *IMU) GyroBias() fmath.Vec3 { return p.gyroBias }

// Update turns a raw sample into a body frame sample with bias removed and noise filtered.
func (p *IMU) Update(raw *msg.IMUSample, out *msg.IMUSample) {
	gyro := p.cfg.Rotation.MulVec(raw.Gyro)
	accel := p.cfg.Rotation.MulVec(raw.Accel).Sub(p.cfg.AccelOffset)

	if p.calibrating {
		p.calibrate(gyro)
	}

	gyro = gyro.Sub(p.gyroBias)
	out.Time = raw.Time
	out.Temperature = raw.Temperature
	out.Gyro = p.gyroLPF.Apply(p.gyroNotch.Apply(gyro))
	out.Accel = p.accelLPF.Apply(accel)
}

func (p *IMU) calibrate(gyro fmath.Vec3) {
	if p.calCount == 0 {
		p.calMin, p.calMax = gyro, gyro
	}
	p.calMin = fmath.Vec3{X: fmath.Min(p.calMin.X, gyro.X), Y: fmath.Min(p.calMin.Y, gyro.Y), Z: fmath.Min(p.calMin.Z, gyro.Z)}
	p.calMax = fmath.Vec3{X: fmath.Max(p.calMax.X, gyro.X), Y: fmath.Max(p.calMax.Y, gyro.Y), Z: fmath.Max(p.calMax.Z, gyro.Z)}
	spread := p.calMax.Sub(p.calMin)
	if spread.X > p.cfg.StillThreshold || spread.Y > p.cfg.StillThreshold || spread.Z > p.cfg.StillThreshold {
		// The vehicle moved, so start over.
		p.Calibrate()
		return
	}
	p.calSum = p.calSum.Add(gyro)
	p.calCount++
	if p.calCount >= p.cfg.CalibrationSamples {
		p.gyroBias = p.calSum.Scale(1 / float32(p.calCount))
		p.calibrated = true
		p.calibrating = false
	}
}

// Mag turns a raw mag sample into body axes with the hard iron offset removed.
func (p *IMU) Mag(raw *msg.MagSample, out *msg.MagSample) {
	out.Time = raw.Time
	out.Field = p.cfg.MagRotation.MulVec(raw.Field).Sub(p.cfg.MagOffset)
}
