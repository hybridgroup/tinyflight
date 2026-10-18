//go:build m5stamp_fly

// Bench tool that runs the flight loop with the motors disconnected and prints every sensor.
// Use it to check units, axis directions and loop timing.
package main

import (
	"machine"
	"runtime"
	"time"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// noMotors keeps the motors off whatever the flight loop asks for.
type noMotors struct{}

func (noMotors) SetMotors(*msg.MotorOutputs) error { return nil }

func main() {
	time.Sleep(2 * time.Second)
	b := stampfly.New()
	for _, err := range b.Errors {
		println("setup:", err.Error())
	}
	if b.Hardware.IMU == nil {
		println("no IMU")
		for {
			time.Sleep(time.Second)
		}
	}
	b.Hardware.Motors = noMotors{}

	cfg := stampfly.Config()
	cfg.Schedule.FlowHz = 20
	f := flight.New(b.Hardware, cfg)
	startOnCores(f)
}

func report(f *flight.Flight) {
	var ms runtime.MemStats
	var lastMallocs uint64
	for {
		time.Sleep(time.Second)
		imu := f.IMU()
		r, p, y := f.Attitude().Q.Euler()
		mag := f.Mag().Field
		flow := f.Flow()
		st := f.Stats()
		runtime.ReadMemStats(&ms)

		println("accel_mms2", mm(imu.Accel.X), mm(imu.Accel.Y), mm(imu.Accel.Z),
			"gyro_mdps", mdeg(imu.Gyro.X), mdeg(imu.Gyro.Y), mdeg(imu.Gyro.Z),
			"calibrated", f.Calibrated())
		println("rpy_deg", deg(r), deg(p), deg(y),
			"baro_pa", int(f.Baro().Pressure), "range_mm", mm(f.Range().Distance), f.Range().Valid,
			"z_mm", mm(f.Position().Z))
		println("mag_nT", mm(mag.X), mm(mag.Y), mm(mag.Z),
			"flow", flow.DeltaX, flow.DeltaY, flow.Quality,
			"batt_mV", mm(f.Battery().Voltage), "mA", mm(f.Battery().Current))
		println("ticks", st.Ticks, "overruns", st.Overruns, "step_us", st.MaxStep.Microseconds(),
			"jitter_us", st.MaxJitter.Microseconds(), "imu_err", st.IMUErrors, "sensor_err", st.SensorErrors,
			"mallocs", ms.Mallocs-lastMallocs)
		println()
		lastMallocs = ms.Mallocs
	}
}

func mm(v float32) int   { return int(v * 1000) }
func deg(v float32) int  { return int(fmath.Degrees(v)) }
func mdeg(v float32) int { return int(fmath.Degrees(v) * 1000) }

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
