//go:build m5stamp_fly

package stampfly

import (
	"machine"

	"github.com/hybridgroup/tinyflight/devices/amg88xx"

	drvamg88xx "tinygo.org/x/drivers/amg88xx"
)

// NewCamera looks for an AMG88xx on the Grove port, which has its own I2C bus.
// It returns nil if there is none. See docs/camera.md.
func NewCamera() *amg88xx.Camera {
	i2c := machine.I2C1
	if i2c.Configure(machine.I2CConfig{Frequency: i2cHz, SDA: machine.GROVE_SDA, SCL: machine.GROVE_SCL}) != nil {
		return nil
	}
	for _, addr := range []uint16{drvamg88xx.AddressHigh, drvamg88xx.AddressLow} {
		dev := drvamg88xx.New(i2c)
		dev.Address = addr
		if dev.Configure(drvamg88xx.Config{}) != nil {
			continue
		}
		// The sensor averages each pixel over two frames, which cuts the noise by about a third.
		if dev.SetMovingAverageMode(true) == nil {
			return amg88xx.New(&dev)
		}
	}
	return nil
}
