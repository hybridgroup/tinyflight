//go:build tinygo && (esp32c3 || esp32s3)

// Camera receiver for the ground. It listens for camera packets from a StampFly over ESP-NOW and
// prints each one to USB serial for tools/cameraview, with the MAC of the drone that sent it. It never sends,
// so it cannot disturb the joystick. An SSD1306 display on the default I2C pins, if there is one, shows the
// image from the first drone it hears.
//
//	tinygo flash -target xiao-esp32c3 ./firmware/camera-receiver
package main

import (
	"os"
	"time"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"tinygo.org/x/espradio"
)

type frame struct {
	drone [6]byte
	input.CameraFrame
}

// frames is small since the newest frame matters most. A full queue drops new frames.
var frames = make(chan frame, 4)

func main() {
	time.Sleep(time.Second)
	if err := start(); err != nil {
		for {
			println("camera receiver:", err.Error())
			time.Sleep(time.Second)
		}
	}
	println("camera receiver ready on channel", espnowinput.Channel())
	disp := newDisplay()
	if disp != nil {
		println("display found")
	}
	var line []byte
	var shown [6]byte
	heard := false
	for f := range frames {
		line = input.AppendCameraLine(line[:0], f.drone, &f.CameraFrame)
		os.Stdout.Write(line)
		if !heard {
			shown, heard = f.drone, true
		}
		if disp != nil && f.drone == shown {
			disp.show(&f.CameraFrame)
		}
	}
}

func start() error {
	if err := espradio.Enable(espradio.Config{}); err != nil {
		return err
	}
	if err := espradio.Start(); err != nil {
		return err
	}
	if err := espradio.SetChannel(espnowinput.Channel()); err != nil {
		return err
	}
	if err := espradio.ESPNowInit(); err != nil {
		return err
	}
	espradio.ESPNowSetReceiveHandler(receive)
	return nil
}

func receive(r espradio.ESPNowReceive) {
	f := frame{drone: r.SourceAddress}
	if input.DecodeCamera(r.Data, &f.CameraFrame) != nil {
		return
	}
	select {
	case frames <- f:
	default:
	}
}
