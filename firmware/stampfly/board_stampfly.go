//go:build m5stamp_fly

package main

import (
	"machine"
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

// setup starts the board and the ESP-NOW joystick. It reports false when there is no IMU.
func setup() (flight.Hardware, bool) {
	b := stampfly.New()
	for _, err := range b.Errors {
		println("setup:", err.Error())
	}
	if b.Hardware.IMU == nil {
		return b.Hardware, false
	}
	if radio, err := espnowinput.Start(); err != nil {
		println("esp-now:", err.Error())
	} else {
		mac := radio.MAC()
		println("esp-now ready, mac", hex(mac[0]), hex(mac[1]), hex(mac[2]), hex(mac[3]), hex(mac[4]), hex(mac[5]))
		joystick = radio
		b.Hardware.Input = input.Merge{radio, b.Hardware.Input}
	}
	return b.Hardware, true
}

func startTelemetry(f *flight.Flight) {
	if joystick != nil {
		go joystick.RunTelemetry(100*time.Millisecond, func(t *input.AtomTelemetry) { fillTelemetry(f, t) })
	}
}

// startOnCores runs the flight loop alone on core 1. Core 0 takes interrupts, the radio, the slow
// sensors, the printout and any blackbox log. Build with -scheduler=cores to use both cores.
func startOnCores(f *flight.Flight) {
	go func() {
		machine.AssignCore(0)
		report(f)
	}()
	startBlackbox(f)
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

var joystick *espnowinput.Input

func radioPackets() uint32 {
	if joystick == nil {
		return 0
	}
	return joystick.Packets.Load()
}

func telemetrySent() uint32 {
	if joystick == nil {
		return 0
	}
	return joystick.Sent.Load()
}

func telemetryFailed() uint32 {
	if joystick == nil {
		return 0
	}
	return joystick.SendFailed.Load()
}

func radioRejected() uint32 {
	if joystick == nil {
		return 0
	}
	return joystick.Rejected.Load()
}
