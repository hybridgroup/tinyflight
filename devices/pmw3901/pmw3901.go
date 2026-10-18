// Package pmw3901 adapts the PixArt PMW3901 optical flow driver to hal.Flow.
package pmw3901

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/drivers/pmw3901"
)

var _ hal.Flow = (*Flow)(nil)

type Flow struct {
	Dev *pmw3901.Device
}

func New(dev *pmw3901.Device) *Flow { return &Flow{Dev: dev} }

func (d *Flow) ReadFlow(s *msg.FlowSample) error {
	m, err := d.Dev.ReadMotionBurst()
	if err != nil {
		return err
	}
	s.DeltaX = m.DeltaX
	s.DeltaY = m.DeltaY
	s.Quality = m.Squal
	s.Valid = m.Valid()
	return nil
}
