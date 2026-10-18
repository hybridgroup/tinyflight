//go:build m5stamp_fly

// Tinyflight firmware for the M5Stack StampFly.
// Bench control over USB serial with lines like "rc roll pitch yaw throttle arm mode".
package main

import (
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

func main() {
	time.Sleep(2 * time.Second)
	b := stampfly.New()
	for _, err := range b.Errors {
		println("setup:", err.Error())
	}
	if b.Hardware.IMU == nil {
		println("no IMU, not flying")
		for {
			time.Sleep(time.Second)
		}
	}

	f := flight.New(b.Hardware, stampfly.Config())
	go report(f)
	f.Run()
}

func report(f *flight.Flight) {
	for {
		time.Sleep(500 * time.Millisecond)
		s := f.Status()
		r, p, y := f.Attitude().Q.Euler()
		st := f.Stats()
		println("armed", s.Armed, "mode", s.Mode.String(), "failsafe", s.Failsafe.String(),
			"rpy", deg(r), deg(p), deg(y),
			"z_mm", int(f.Position().Z*1000), "batt_mV", int(f.Battery().Voltage*1000),
			"overruns", st.Overruns, "step_us", st.MaxStep.Microseconds())
	}
}

func deg(rad float32) int { return int(fmath.Degrees(rad)) }
