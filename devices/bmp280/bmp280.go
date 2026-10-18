// Package bmp280 adapts the Bosch BMP280 barometer driver to hal.Baro.
package bmp280

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/drivers"
	"tinygo.org/x/drivers/bmp280"
)

var _ hal.Baro = (*Baro)(nil)

type Baro struct {
	Dev *bmp280.Device
}

func New(dev *bmp280.Device) *Baro { return &Baro{Dev: dev} }

func (d *Baro) ReadBaro(s *msg.BaroSample) error {
	if err := d.Dev.Update(drivers.Temperature | drivers.Pressure); err != nil {
		return err
	}
	t, p := d.Dev.Measurements()
	s.Pressure = float32(p) * 1e-3
	s.Temperature = float32(t) * 1e-3
	return nil
}
