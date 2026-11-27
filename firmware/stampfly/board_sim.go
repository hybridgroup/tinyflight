//go:build tinyflightsim

package main

import (
	"github.com/hybridgroup/tinyflight"
	simboard "github.com/hybridgroup/tinyflight/boards/sim"
	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

// setup returns the simulated StampFly. See boards/sim.
func setup() (flight.Hardware, bool) { return simboard.Board().Hardware, true }

// startTelemetry does nothing, since simboard.Run sends the telemetry.
func startTelemetry(f *flight.Flight) {}

func startOnCores(f *flight.Flight) {
	go report(f)
	simboard.Run(f, espnowinput.Telemetry{
		Info:      command.Info{Firmware: tinyflight.VersionNumbers(), Flags: command.InfoPosHold},
		FillAtom:  func(t *input.AtomTelemetry) { fillTelemetry(f, t) },
		FillState: func(s *command.State) { command.FillState(f, s) },
	})
}

func radioPackets() uint32    { return simboard.Board().Radio.Packets.Load() }
func radioRejected() uint32   { return simboard.Board().Radio.Rejected.Load() }
func telemetrySent() uint32   { return simboard.TelemetrySent }
func telemetryFailed() uint32 { return 0 }
