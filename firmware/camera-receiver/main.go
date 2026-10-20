//go:build tinygo && (esp32c3 || esp32s3)

// Camera receiver for the ground. It listens for camera packets from a StampFly over ESP-NOW and
// prints each one to USB serial for tools/cameraview. It never sends, so it cannot disturb the joystick.
//
//	tinygo flash -target xiao-esp32c3 ./firmware/camera-receiver
package main

import (
	"os"
	"strconv"
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
		line = appendLine(line[:0], &f)
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

// appendLine writes "cam seq format width height chunk chunks count pixels... sum",
// where sum adds every value before it so the viewer can drop damaged lines.
func appendLine(b []byte, f *input.CameraFrame) []byte {
	b = append(b, "cam"...)
	var sum int64
	put := func(v int64) {
		b = append(b, ' ')
		b = strconv.AppendInt(b, v, 10)
		sum += v
	}
	put(int64(f.Seq))
	put(int64(f.Format))
	put(int64(f.Width))
	put(int64(f.Height))
	put(int64(f.Chunk))
	put(int64(f.Chunks))
	put(int64(f.Count))
	for _, p := range f.Pixels[:f.Count] {
		put(int64(p))
	}
	b = append(b, ' ')
	b = strconv.AppendInt(b, sum, 10)
	return append(b, '\n')
}
