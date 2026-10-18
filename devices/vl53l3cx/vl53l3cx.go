// Package vl53l3cx adapts the ST VL53L3CX time of flight driver to hal.Rangefinder.
package vl53l3cx

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/drivers/vl53l3cx"
)

var _ hal.Rangefinder = (*Range)(nil)

type Range struct {
	Dev *vl53l3cx.Device
	m   vl53l3cx.Measurement
}

func New(dev *vl53l3cx.Device) *Range { return &Range{Dev: dev} }

// ReadRange returns hal.ErrNotReady until a new measurement is done, so it never waits.
// The closest valid target is used.
func (d *Range) ReadRange(s *msg.RangeSample) error {
	ready, err := d.Dev.DataReady()
	if err != nil {
		return err
	}
	if !ready {
		return hal.ErrNotReady
	}
	if err := d.Dev.Read(&d.m); err != nil {
		return err
	}
	s.Valid = false
	for i := 0; i < int(d.m.NumTargets) && i < vl53l3cx.MaxTargets; i++ {
		t := &d.m.Targets[i]
		if t.Status != vl53l3cx.RangeValid {
			continue
		}
		dist := float32(t.Distance) * 1e-3
		if !s.Valid || dist < s.Distance {
			s.Distance = dist
			s.Valid = true
		}
	}
	return nil
}
