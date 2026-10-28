//go:build tinyflightsim && wasm

package sim

import (
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

// The browser runner provides these. readInput copies the newest joystick packet and returns its sequence number.
//
//go:wasmimport tinyflightsim read_input
func readInput(buf *byte, n uint32) uint32

//go:wasmimport tinyflightsim publish_state
func publishState(buf *float32, n uint32)

//go:wasmimport tinyflightsim telemetry
func sendTelemetry(buf *byte, n uint32)

var board = New(stampfly.Config())

// TelemetrySent counts telemetry packets handed to the browser.
var TelemetrySent uint32

// Board returns the simulated StampFly.
func Board() *World { return board }

const (
	paceSteps      = 4  // 10 ms at 400 Hz
	stateSteps     = 6  // about 66 per second
	telemetrySteps = 20 // 20 per second, like the radio
	maxBehind      = 100 * time.Millisecond
)

// Run flies f in the simulated world forever, keeping simulated time in step with the wall clock.
func Run(f *flight.Flight, fill func(*flight.Flight, *input.AtomTelemetry)) {
	w := board
	var packet [input.AtomPacketSize]byte
	var lastSeq uint32
	var state [StateSize]float32
	var tel input.AtomTelemetry
	var telPacket [input.AtomTelemetrySize]byte
	start := time.Now()
	base := w.Now()
	for step := 1; ; step++ {
		if seq := readInput(&packet[0], uint32(len(packet))); seq != lastSeq {
			lastSeq = seq
			w.Joystick.Send(packet[:])
		}
		w.Step(f)
		if step%stateSteps == 0 {
			w.State(f, &state)
			publishState(&state[0], StateSize)
		}
		if step%telemetrySteps == 0 {
			fill(f, &tel)
			tel.Encode(&telPacket)
			sendTelemetry(&telPacket[0], uint32(len(telPacket)))
			TelemetrySent++
		}
		if step%paceSteps != 0 {
			continue
		}
		ahead := time.Duration(w.Now()-base)*time.Microsecond - time.Since(start)
		switch {
		case ahead > 0:
			time.Sleep(ahead)
		case ahead < -maxBehind:
			// After a pause, carry on from now instead of racing to catch up.
			start, base = time.Now(), w.Now()
		default:
			runtime.Gosched()
		}
	}
}
