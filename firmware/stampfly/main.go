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
	"github.com/hybridgroup/tinyflight/pkg/msg"
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
	if joystick != nil {
		go joystick.RunTelemetry(100*time.Millisecond, func(t *input.AtomTelemetry) { fillTelemetry(f, t) })
	}
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

func milli(v float32) int { return int(v * 1000) }

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
