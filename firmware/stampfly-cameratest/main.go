//go:build m5stamp_fly

// Bench tool for the AMG88xx thermal camera. The motors stay off and the radio is not used.
// It prints each frame over USB in the camera receiver format, so tools/cameraview can read
// the StampFly directly. See docs/camera.md.
package main

import (
	"os"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

func main() {
	time.Sleep(2 * time.Second)
	// New also stops the motors.
	b := stampfly.New()
	for _, err := range b.Errors {
		println("setup:", err.Error())
	}
	camera := stampfly.NewCamera()
	for camera == nil {
		println("no camera")
		time.Sleep(time.Second)
		camera = stampfly.NewCamera()
	}
	println("camera found")

	var f input.CameraFrame
	var line []byte
	// The driver Configure sets the AMG88xx to 10 frames per second.
	for next := time.Now(); ; {
		next = next.Add(100 * time.Millisecond)
		time.Sleep(time.Until(next))
		if err := camera.ReadFrame(&f); err != nil {
			println("camera:", err.Error())
			continue
		}
		line = input.AppendCameraLine(line[:0], &f)
		os.Stdout.Write(line)
	}
}
