//go:build tinygo && (esp32c3 || esp32s3) && !m5stamp_c3

package main

import (
	"image/color"
	"machine"

	"github.com/hybridgroup/tinyflight/pkg/input"
	"tinygo.org/x/drivers/ssd1306"
	"tinygo.org/x/tinyfont"
	"tinygo.org/x/tinyfont/proggy"
)

var white = color.RGBA{255, 255, 255, 255}

type display struct {
	dev   ssd1306.Device
	image thermalImage
	text  []byte
}

// newDisplay looks for a 128 by 64 SSD1306 on the board's default I2C pins.
// It returns nil if there is none.
func newDisplay() *display {
	i2c := machine.I2C0
	if i2c.Configure(machine.I2CConfig{Frequency: 400 * machine.KHz}) != nil {
		return nil
	}
	// The SA0 pin picks 0x3C or 0x3D, from section 8.1.5 of the SSD1306 datasheet.
	for _, addr := range []uint16{0x3C, 0x3D} {
		// 0xAE turns the display off, which Configure undoes.
		if i2c.Tx(addr, []byte{0x00, 0xAE}, nil) != nil {
			continue
		}
		d := &display{dev: *ssd1306.NewI2C(i2c)}
		d.dev.Configure(ssd1306.Config{Width: 128, Height: 64, Address: addr, VccState: ssd1306.SWITCHCAPVCC})
		d.message("waiting")
		return d
	}
	return nil
}

func (d *display) message(s string) {
	d.dev.ClearBuffer()
	tinyfont.WriteLine(&d.dev, &proggy.TinySZ8pt7b, 0, 12, s, white)
	d.dev.Display()
}

// show draws the image once its last chunk arrives, with the hottest, coldest and center temperatures.
func (d *display) show(f *input.CameraFrame) {
	if !d.image.add(f) {
		return
	}
	d.dev.ClearBuffer()
	lo, hi := d.image.span()
	for y := range imageSize {
		for x := range imageSize {
			if d.image.lit(x, y, lo, hi) {
				d.dev.SetPixel(int16(x), int16(y), white)
			}
		}
	}
	d.line(12, "hi ", hi)
	d.line(32, "mid ", d.image.center())
	d.line(52, "lo ", lo)
	d.dev.Display()
}

func (d *display) line(y int16, label string, v int16) {
	d.text = appendTemp(append(d.text[:0], label...), v)
	tinyfont.WriteLine(&d.dev, &proggy.TinySZ8pt7b, imageSize+4, y, string(d.text), white)
}
