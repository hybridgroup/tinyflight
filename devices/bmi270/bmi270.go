// Package bmi270 adapts the Bosch BMI270 IMU driver to hal.IMU.
package bmi270

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/drivers"
	"tinygo.org/x/drivers/bmi270"
)

var _ hal.IMU = (*IMU)(nil)

type IMU struct {
	Dev *bmi270.Device
}

func New(dev *bmi270.Device) *IMU { return &IMU{Dev: dev} }

const (
	microGToMS2   = fmath.Gravity * 1e-6
	microDegToRad = fmath.Pi / 180 * 1e-6
)

// ReadIMU returns accel and gyro in the chip's own axes.
func (d *IMU) ReadIMU(s *msg.IMUSample) error {
	if err := d.Dev.Update(drivers.Acceleration | drivers.AngularVelocity); err != nil {
		return err
	}
	ax, ay, az := d.Dev.Acceleration()
	gx, gy, gz := d.Dev.AngularVelocity()
	s.Accel = fmath.Vec3{X: float32(ax) * microGToMS2, Y: float32(ay) * microGToMS2, Z: float32(az) * microGToMS2}
	s.Gyro = fmath.Vec3{X: float32(gx) * microDegToRad, Y: float32(gy) * microDegToRad, Z: float32(gz) * microDegToRad}
	return nil
}
