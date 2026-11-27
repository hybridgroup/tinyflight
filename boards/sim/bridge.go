//go:build tinyflightsim && wasm

package sim

import (
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/flight"
)

// The browser runner provides these. readPacket copies the next packet that came over the air and its
// sender, and returns its length, or 0 when there is none.
//
//go:wasmimport tinyflightsim read_packet
func readPacket(src *byte, buf *byte, n uint32) uint32

//go:wasmimport tinyflightsim send_packet
func sendPacket(dst *byte, buf *byte, n uint32)

//go:wasmimport tinyflightsim publish_state
func publishState(buf *float32, n uint32)

// readTerrain copies the terrain record and returns how many floats it wrote. See World.SetTerrain.
//
//go:wasmimport tinyflightsim read_terrain
func readTerrain(buf *float32, n uint32) uint32

var terrain [TerrainHeader + TerrainMaxCells]float32

var board = New(stampfly.Config())

// TelemetrySent counts packets the drone sent to the browser.
var TelemetrySent uint32

// Board returns the simulated StampFly.
func Board() *World { return board }

const (
	paceSteps      = 4  // 10 ms at 400 Hz
	stateSteps     = 6  // about 66 per second
	telemetrySteps = 20 // 20 per second, like the radio
	maxBehind      = 100 * time.Millisecond
)

// send hands a packet from the drone to the browser.
func send(addr [6]byte, data []byte) {
	sendPacket(&addr[0], &data[0], uint32(len(data)))
	TelemetrySent++
}

// Run flies f in the simulated world forever, keeping simulated time in step with the wall clock.
// tel answers the joystick or ground station on the page, and needs only its fill funcs and Info set.
func Run(f *flight.Flight, tel espnowinput.Telemetry) {
	w := board
	w.Radio.SetVehicle(f)
	tel.Link = w.Radio
	tel.Info.MAC = MAC
	var src [6]byte
	var packet [64]byte
	var state [StateSize]float32
	w.SetTerrain(terrain[:readTerrain(&terrain[0], uint32(len(terrain)))])
	start := time.Now()
	base := w.Now()
	for step := 1; ; step++ {
		for {
			n := readPacket(&src[0], &packet[0], uint32(len(packet)))
			if n == 0 {
				break
			}
			w.Receive(src, packet[:n])
		}
		w.Step(f)
		if step%stateSteps == 0 {
			w.State(f, &state)
			publishState(&state[0], StateSize)
		}
		if step%telemetrySteps == 0 {
			tel.Tick(send)
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
