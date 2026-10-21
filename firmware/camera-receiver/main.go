//go:build tinygo && (esp32c3 || esp32s3)

// Camera receiver for the ground. It listens for camera packets from a StampFly over ESP-NOW and
// prints each one to USB serial for tools/cameraview. It never sends, so it cannot disturb the joystick.
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

// frames is small since the newest frame matters most. A full queue drops new frames.
var frames = make(chan input.CameraFrame, 4)

func main() {
	time.Sleep(time.Second)
	if err := start(); err != nil {
		for {
			println("camera receiver:", err.Error())
			time.Sleep(time.Second)
		}
	}
	println("camera receiver ready on channel", espnowinput.Channel)
	var line []byte
	for f := range frames {
		line = input.AppendCameraLine(line[:0], &f)
		os.Stdout.Write(line)
	}
}

func start() error {
	if err := espradio.Enable(espradio.Config{}); err != nil {
		return err
	}
	if err := espradio.Start(); err != nil {
		return err
	}
	if err := espradio.SetChannel(espnowinput.Channel); err != nil {
		return err
	}
	if err := espradio.ESPNowInit(); err != nil {
		return err
	}
	espradio.ESPNowSetReceiveHandler(receive)
	return nil
}

func receive(r espradio.ESPNowReceive) {
	var f input.CameraFrame
	if input.DecodeCamera(r.Data, &f) != nil {
		return
	}
	select {
	case frames <- f:
	default:
	}
}
