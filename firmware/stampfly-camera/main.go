//go:build m5stamp_fly

// Tinyflight firmware for the M5Stack StampFly with an AMG88xx thermal camera on the Grove port.
// It flies like firmware/stampfly and also streams camera frames over ESP-NOW. See docs/camera.md.
package main

import (
	"machine"
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/devices/amg88xx"
	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var (
	joystick *espnowinput.Input
	camera   *amg88xx.Camera
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
	camera = stampfly.NewCamera()
	if camera == nil {
		println("no camera")
	} else {
		println("camera found")
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
		go joystick.RunTelemetry(50*time.Millisecond, func(t *input.AtomTelemetry) { fillTelemetry(f, t) })
		if camera != nil {
			// The driver Configure sets the AMG88xx to 10 frames per second.
			go joystick.RunCamera(100*time.Millisecond, func(fr *input.CameraFrame) bool { return camera.ReadFrame(fr) == nil })
		}
	}
	startOnCores(f)
}

// startOnCores runs the flight loop alone on core 1. Core 0 takes interrupts, the radio, the
// camera, the slow sensors and the printout. Build with -scheduler=cores to use both cores.
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
		var radioOK, radioBad, telSent, telFailed, camSent, camFailed uint32
		if joystick != nil {
			radioOK, radioBad = joystick.Packets.Load(), joystick.Rejected.Load()
			telSent, telFailed = joystick.Sent.Load(), joystick.SendFailed.Load()
			camSent, camFailed = joystick.CameraSent.Load(), joystick.CameraFailed.Load()
		}
		println("armed", s.Armed, "mode", s.Mode.String(), "failsafe", s.Failsafe.String(),
			"rpy", deg(r), deg(p), deg(y),
			"z_mm", int(f.Position().Z*1000), "xy_mm", int(f.Position().X*1000), int(f.Position().Y*1000), f.Position().XYValid, "batt_mV", int(f.Battery().Voltage*1000),
			"overruns", st.Overruns, "step_us", st.MaxStep.Microseconds(),
			"motors_fl_fr_rl_rr", milli(m[0]), milli(m[1]), milli(m[2]), milli(m[3]),
			"radio_ok", radioOK, "radio_bad", radioBad, "tel_sent", telSent, "tel_failed", telFailed, "cam_sent", camSent, "cam_failed", camFailed,
			"in_rpy_thr_climb", milli(in.Roll), milli(in.Pitch), milli(in.Yaw), milli(in.Throttle), milli(in.Climb), "in_arm", in.Arm,
			"gc", ms.NumGC-lastGC, "mallocs", ms.Mallocs-lastMallocs, "heap_kb", ms.HeapAlloc/1024)
		lastGC, lastMallocs = ms.NumGC, ms.Mallocs
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
