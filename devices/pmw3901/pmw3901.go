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
	// m.Valid is false whenever nothing moved, but zero motion is a good reading, so check only the surface.
	// Low surface check from get_motion in https://github.com/pimoroni/pmw3901-python
	s.Valid = !(m.Squal < 0x19 && m.Shutter>>8 == 0x1F)
	return nil
}
