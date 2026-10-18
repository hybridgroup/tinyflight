//go:build m5stamp_fly

// Tinyflight firmware for the M5Stack StampFly. Fly with the M5 Atom Joystick over ESP-NOW,
// or on the bench with USB serial lines like "rc roll pitch yaw throttle arm mode".
package main

import (
	"machine"
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/input"
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

	if radio, err := espnowinput.Start(); err != nil {
		println("esp-now:", err.Error())
	} else {
		mac := radio.MAC()
		println("esp-now ready, mac", hex(mac[0]), hex(mac[1]), hex(mac[2]), hex(mac[3]), hex(mac[4]), hex(mac[5]))
		joystick = radio
		b.Hardware.Input = input.Merge{radio, b.Hardware.Input}
	}

	f := flight.New(b.Hardware, stampfly.Config())
	startOnCores(f)
}

func report(f *flight.Flight) {
	var ms runtime.MemStats
	var lastMallocs uint64
	var lastGC uint32
	for {
		time.Sleep(500 * time.Millisecond)
		runtime.ReadMemStats(&ms)
		s := f.Status()
		r, p, y := f.Attitude().Q.Euler()
		st := f.Stats()
		m := f.Motors().Value
		println("armed", s.Armed, "mode", s.Mode.String(), "failsafe", s.Failsafe.String(),
			"rpy", deg(r), deg(p), deg(y),
			"z_mm", int(f.Position().Z*1000), "batt_mV", int(f.Battery().Voltage*1000),
			"overruns", st.Overruns, "step_us", st.MaxStep.Microseconds(),
			"motors_fl_fr_rl_rr", milli(m[0]), milli(m[1]), milli(m[2]), milli(m[3]),
			"radio_ok", radioPackets(), "radio_bad", radioRejected(),
			"gc", ms.NumGC-lastGC, "mallocs", ms.Mallocs-lastMallocs, "heap_kb", ms.HeapAlloc/1024)
		lastGC, lastMallocs = ms.NumGC, ms.Mallocs
	}
}

func deg(rad float32) int { return int(fmath.Degrees(rad)) }

// startOnCores runs the flight loop alone on core 1. Core 0 takes interrupts, the radio,
// the slow sensors and the printout. Build with -scheduler=cores to use both cores.
func startOnCores(f *flight.Flight) {
	go func() {
		machine.AssignCore(0)
		report(f)
	}()
	if runtime.NumCPU() < 2 {
		f.Run()
	}
	go func() {
		machine.AssignCore(0)
		f.RunSensors()
	}()
	machine.AssignCore(1)
	f.RunDedicated()
}

func milli(v float32) int { return int(v * 1000) }

var joystick *espnowinput.Input

func radioPackets() uint32 {
	if joystick == nil {
		return 0
	}
	return joystick.Packets.Load()
}

func radioRejected() uint32 {
	if joystick == nil {
		return 0
	}
	return joystick.Rejected.Load()
}

func hex(b byte) string {
	const digits = "0123456789ABCDEF"
	return string([]byte{digits[b>>4], digits[b&15]})
}
