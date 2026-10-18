//go:build m5stamp_fly

// Bench tool that spins each motor in turn. REMOVE THE PROPELLERS FIRST.
// Press the button to run the sequence front left, front right, rear left, rear right.
package main

import (
	"machine"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

const (
	testPower = 0.2
	spinTime  = time.Second
)

var names = [4]string{"front left", "front right", "rear left", "rear right"}

func main() {
	time.Sleep(2 * time.Second)
	b := stampfly.New()
	for _, err := range b.Errors {
		println("setup:", err.Error())
	}
	machine.BUTTON.Configure(machine.PinConfig{Mode: machine.PinInputPullup})

	var out msg.MotorOutputs
	out.Count = 4
	for {
		println("props off! press the button to spin each motor")
		for machine.BUTTON.Get() {
			time.Sleep(10 * time.Millisecond)
		}
		for i := range names {
			println("spinning", names[i])
			out.Value[i] = testPower
			b.Motors.SetMotors(&out)
			time.Sleep(spinTime)
			out.Value[i] = 0
			b.Motors.SetMotors(&out)
			time.Sleep(spinTime / 2)
		}
		println("front right and rear left should spin counter clockwise seen from above")
	}
}
