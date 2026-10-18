package sensors

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

func testConfig() Config {
	return Config{
		Rotation:           fmath.Mat3{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},
		MagRotation:        fmath.Identity3(),
		SampleHz:           400,
		GyroCutoffHz:       80,
		AccelCutoffHz:      30,
		CalibrationSamples: 100,
		StillThreshold:     0.05,
	}
}

func TestRotationAndBias(t *testing.T) {
	p := NewIMU(testConfig())
	p.Calibrate()
	raw := msg.IMUSample{Accel: fmath.Vec3{Z: 9.8}, Gyro: fmath.Vec3{X: 0.01, Y: 0.02, Z: 0.03}}
	var out msg.IMUSample
	for range 100 {
		p.Update(&raw, &out)
	}
	if !p.Calibrated() {
		t.Fatal("not calibrated")
	}
	if b := p.GyroBias().Sub(fmath.Vec3{X: 0.02, Y: 0.01, Z: -0.03}); b.Norm() > 1e-6 {
		t.Errorf("bias error %v", b)
	}
	for range 100 {
		p.Update(&raw, &out)
	}
	if fmath.Abs(out.Accel.Z+9.8) > 1e-3 {
		t.Errorf("accel z %v, want -9.8 in FRD", out.Accel.Z)
	}
	if out.Gyro.Norm() > 1e-4 {
		t.Errorf("gyro after bias %v", out.Gyro)
	}
}

func TestCalibrationRestartsOnMotion(t *testing.T) {
	p := NewIMU(testConfig())
	p.Calibrate()
	var out msg.IMUSample
	for i := range 150 {
		raw := msg.IMUSample{}
		if i == 50 {
			raw.Gyro.X = 1
		}
		p.Update(&raw, &out)
	}
	if p.Calibrated() {
		t.Error("calibrated even though the vehicle moved late in the window")
	}
	raw := msg.IMUSample{}
	for range 100 {
		p.Update(&raw, &out)
	}
	if !p.Calibrated() {
		t.Error("did not calibrate once still")
	}
}
