// Package hal has the interfaces a board provides to the flight stack.
// Each read fills a msg struct in SI units. IMU and mag values stay in the chip's own axes.
package hal

import (
	"errors"

	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// ErrNotReady means the device has no new data yet.
var ErrNotReady = errors.New("hal: no new data")

// IMU reads accel and gyro.
type IMU interface {
	ReadIMU(s *msg.IMUSample) error
}

// Mag reads the magnetic field.
type Mag interface {
	ReadMag(s *msg.MagSample) error
}

// Baro reads pressure and temperature.
type Baro interface {
	ReadBaro(s *msg.BaroSample) error
}

// Rangefinder reads the distance to the ground.
type Rangefinder interface {
	ReadRange(s *msg.RangeSample) error
}

// Flow reads optical flow motion.
type Flow interface {
	ReadFlow(s *msg.FlowSample) error
}

// Power reads the battery voltage and current.
type Power interface {
	ReadBattery(s *msg.BatteryStatus) error
}

// Motors takes values from 0 to 1 in the order of the mixer geometry.
type Motors interface {
	SetMotors(m *msg.MotorOutputs) error
}

// Input gives the latest pilot input. It returns false when nothing new has arrived.
type Input interface {
	ReadInput(in *msg.ManualInput) bool
}

// LED sets a color from 0 to 255 per channel.
type LED interface {
	SetColor(r, g, b uint8)
}

// Buzzer plays a tone at hz. Zero turns it off.
type Buzzer interface {
	Tone(hz uint32)
}
