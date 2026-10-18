// Package ina3221 adapts the TI INA3221 power monitor driver to hal.Power.
package ina3221

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/drivers/ina3221"
)

var _ hal.Power = (*Power)(nil)

// Power reads the battery on one INA3221 channel, numbered from 1.
type Power struct {
	Dev     *ina3221.Device
	Channel int
}

func New(dev *ina3221.Device, channel int) *Power { return &Power{Dev: dev, Channel: channel} }

func (d *Power) ReadBattery(s *msg.BatteryStatus) error {
	v, err := d.Dev.BusVoltage(d.Channel)
	if err != nil {
		return err
	}
	i, err := d.Dev.Current(d.Channel)
	if err != nil {
		return err
	}
	s.Voltage = float32(v) * 1e-6
	s.Current = float32(i) * 1e-6
	return nil
}
