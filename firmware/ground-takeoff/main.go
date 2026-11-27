//go:build !tinygo || esp32c3 || esp32s3

// Ground station example that flies a Tinyflight drone with the command protocol.
// It takes off, flies right, hovers, flies back left, and lands. See docs/command-protocol.md.
//
//	tinygo flash -target xiao-esp32c3 ./firmware/ground-takeoff
//
// Or fly the drone in tinyflightsim from a computer.
//
//	go run ./firmware/ground-takeoff
package main

import (
	"time"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/groundstation"
	"github.com/hybridgroup/tinyflight/pkg/command"
)

func main() {
	time.Sleep(3 * time.Second)
	station, err := groundstation.Start()
	must("start radio", err)
	println("ground station ready on channel", espnowinput.Channel())

	var infos []command.Info
	for {
		found, err := station.Find(time.Second)
		if err == nil {
			infos = found
			break
		}
		println("looking for a drone...")
	}
	info := infos[0]
	println("found drone", mac(info.MAC), "firmware", info.Firmware[0], info.Firmware[1], info.Firmware[2])

	drone := station.Drone(info.MAC)
	drone.FlyingStateChange(func(s int) { println("flying state", groundstation.FlyingState(s)) })
	drone.Start()

	println("takeoff")
	must("take off", drone.TakeOff())
	time.Sleep(5 * time.Second)

	println("right")
	drone.Right(50)
	time.Sleep(2 * time.Second)

	println("hover")
	drone.Hover()
	time.Sleep(3 * time.Second)

	println("left")
	drone.Left(50)
	time.Sleep(2 * time.Second)

	println("hover")
	drone.Hover()
	time.Sleep(3 * time.Second)

	println("land")
	must("land", drone.Land())
	time.Sleep(5 * time.Second)
	drone.Halt()
	println("done")
	for {
		time.Sleep(time.Second)
	}
}

func must(action string, err error) {
	if err != nil {
		for {
			println("failed to", action+":", err.Error())
			time.Sleep(time.Second)
		}
	}
}

func mac(m [6]byte) string {
	const digits = "0123456789ABCDEF"
	b := make([]byte, 0, 17)
	for i, v := range m {
		if i > 0 {
			b = append(b, ':')
		}
		b = append(b, digits[v>>4], digits[v&15])
	}
	return string(b)
}
