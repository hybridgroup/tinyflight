//go:build tinyflightsim

package main

import (
	simboard "github.com/hybridgroup/tinyflight/boards/sim"
	"github.com/hybridgroup/tinyflight/pkg/flight"
)

// setup returns the simulated StampFly. See boards/sim.
func setup() (flight.Hardware, bool) { return simboard.Board().Hardware, true }

// startTelemetry does nothing, since simboard.Run sends the telemetry.
func startTelemetry(f *flight.Flight) {}

func startOnCores(f *flight.Flight) {
	go report(f)
	simboard.Run(f, fillTelemetry)
}

func radioPackets() uint32    { return simboard.Board().Joystick.Packets }
func radioRejected() uint32   { return simboard.Board().Joystick.Rejected }
func telemetrySent() uint32   { return simboard.TelemetrySent }
func telemetryFailed() uint32 { return 0 }
func cameraSent() uint32      { return 0 }
