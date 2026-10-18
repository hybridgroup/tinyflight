package flight

import (
	"sync/atomic"
	"testing"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"github.com/hybridgroup/tinyflight/pkg/params"
)

type stillIMU struct{}

func (stillIMU) ReadIMU(s *msg.IMUSample) error {
	s.Accel = fmath.Vec3{Z: -fmath.Gravity}
	s.Gyro = fmath.Vec3{}
	return nil
}

type offMotors struct{}

func (offMotors) SetMotors(*msg.MotorOutputs) error { return nil }

type noInput struct{}

func (noInput) ReadInput(*msg.ManualInput) bool { return false }

type countingBaro struct{ reads atomic.Int32 }

func (b *countingBaro) ReadBaro(s *msg.BaroSample) error {
	b.reads.Add(1)
	s.Pressure = 100000
	return nil
}

func TestRunSensorsFeedsLoop(t *testing.T) {
	baro := &countingBaro{}
	f := New(Hardware{IMU: stillIMU{}, Motors: offMotors{}, Input: noInput{}, Baro: baro}, params.Default())
	go f.RunSensors()
	for !f.slow.async.Load() {
		time.Sleep(time.Millisecond)
	}

	start := time.Now()
	for time.Since(start) < 200*time.Millisecond {
		f.Step(f.now())
		time.Sleep(2500 * time.Microsecond)
	}
	if n := baro.reads.Load(); n < 5 || n > 15 {
		t.Errorf("baro read %d times in 200 ms at 50 Hz", n)
	}
	if f.Baro().Pressure != 100000 {
		t.Errorf("loop never got a baro reading")
	}
	if !f.Position().ZValid {
		t.Errorf("baro reading never reached the altitude estimator")
	}
}
