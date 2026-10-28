//go:build m5stamp_fly || tinyflightsim

// Tinyflight firmware for the M5Stack StampFly. Fly with the M5 Atom Joystick over ESP-NOW,
// or on the bench with USB serial lines like "rc roll pitch yaw throttle arm mode".
// Built with -target=wasip1 -tags=tinyflightsim it flies the simulated drone in boards/sim.
package main

import (
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight"
	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

func main() {
	time.Sleep(2 * time.Second)
	println("Tinyflight", tinyflight.Version())
	hw, ok := setup()
	if !ok {
		println("no IMU, not flying")
		for {
			time.Sleep(time.Second)
		}
	}
	f := flight.New(hw, stampfly.Config())
	startTelemetry(f)
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
		in := f.Input()
		println("armed", s.Armed, "mode", s.Mode.String(), "failsafe", s.Failsafe.String(),
			"rpy", deg(r), deg(p), deg(y),
			"z_mm", int(f.Position().Z*1000), "xy_mm", int(f.Position().X*1000), int(f.Position().Y*1000), f.Position().XYValid, "batt_mV", int(f.Battery().Voltage*1000),
			"overruns", st.Overruns, "step_us", st.MaxStep.Microseconds(),
			"motors_fl_fr_rl_rr", milli(m[0]), milli(m[1]), milli(m[2]), milli(m[3]),
			"radio_ok", radioPackets(), "radio_bad", radioRejected(), "tel_sent", telemetrySent(), "tel_failed", telemetryFailed(),
			"in_rpy_thr_climb", milli(in.Roll), milli(in.Pitch), milli(in.Yaw), milli(in.Throttle), milli(in.Climb), "in_arm", in.Arm,
			"gc", ms.NumGC-lastGC, "mallocs", ms.Mallocs-lastMallocs, "heap_kb", ms.HeapAlloc/1024)
		lastGC, lastMallocs = ms.NumGC, ms.Mallocs
		println("prof imu", flight.Prof[0].Microseconds(), "input", flight.Prof[1].Microseconds(), "loopsens", flight.Prof[2].Microseconds(), "apply", flight.Prof[3].Microseconds(), "est", flight.Prof[4].Microseconds(), "cmd_ctl", flight.Prof[5].Microseconds(), "motors", flight.Prof[6].Microseconds())
	}
}

func deg(rad float32) int { return int(fmath.Degrees(rad)) }

func milli(v float32) int { return int(v * 1000) }

func hex(b byte) string {
	const digits = "0123456789ABCDEF"
	return string([]byte{digits[b>>4], digits[b&15]})
}

// fillTelemetry maps the flight state to what the stock joystick firmware expects.
func fillTelemetry(f *flight.Flight, t *input.AtomTelemetry) {
	r, p, y := f.Attitude().Q.Euler()
	s := f.Status()
	t.Roll, t.Pitch, t.Yaw = fmath.Degrees(r), fmath.Degrees(p), fmath.Degrees(y)
	t.Voltage = f.Battery().Voltage
	t.Altitude = -f.Position().Z
	t.AltFlag = 0
	switch {
	case !s.Armed:
		t.Mode = input.AtomModeParking
	case s.Landing:
		t.Mode = input.AtomModeLanding
	default:
		t.Mode = input.AtomModeFlight
		if s.Mode == msg.ModeAltHold || s.Mode == msg.ModePosHold {
			t.AltFlag = 1
		}
	}
}
