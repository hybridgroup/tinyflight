// Package bmm150 adapts the Bosch BMM150 magnetometer driver to hal.Mag.
package bmm150

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/drivers"
	"tinygo.org/x/drivers/bmm150"
)

var _ hal.Mag = (*Mag)(nil)

type Mag struct {
	Dev *bmm150.Device
}

func New(dev *bmm150.Device) *Mag { return &Mag{Dev: dev} }

// ReadMag returns the field in µT in the chip's own axes.
func (d *Mag) ReadMag(s *msg.MagSample) error {
	if err := d.Dev.Update(drivers.MagneticField); err != nil {
		return err
	}
	x, y, z := d.Dev.MagneticField()
	s.Field = fmath.Vec3{X: float32(x) * 1e-3, Y: float32(y) * 1e-3, Z: float32(z) * 1e-3}
	return nil
}
