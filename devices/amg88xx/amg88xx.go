// Package amg88xx turns frames from the Panasonic AMG88xx 8x8 thermal sensor into camera packets.
package amg88xx

import (
	"github.com/hybridgroup/tinyflight/pkg/input"
	"tinygo.org/x/drivers/amg88xx"
)

const size = 8

// Camera reads one 8x8 frame at a time.
type Camera struct {
	Dev    *amg88xx.Device
	seq    uint16
	pixels [size * size]int32
}

func New(dev *amg88xx.Device) *Camera { return &Camera{Dev: dev} }

// ReadFrame fills f with the next frame in 0.25 °C units.
func (c *Camera) ReadFrame(f *input.CameraFrame) error {
	if err := c.Dev.ReadPixels(&c.pixels); err != nil {
		return err
	}
	f.Seq = c.seq
	c.seq++
	f.Format = input.CameraThermal
	f.Width, f.Height = size, size
	f.Chunk, f.Chunks = 0, 1
	f.Count = size * size
	for i, v := range c.pixels {
		f.Pixels[i] = int16(v / amg88xx.PIXEL_TEMP_CONVERSION)
	}
	return nil
}
