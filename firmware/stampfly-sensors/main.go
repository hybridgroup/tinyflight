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
	"github.com/hybridgroup/tinyflight/pkg/sensors"
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
	f := flight.New(b.Hardware, cfg)
	startOnCores(f, cfg.Sensors)
}

// sumFlow adds up the flow counts and gyro rotation over one second, polling faster than the flow read rate.
func sumFlow(f *flight.Flight) (counts, turn fmath.Vec3) {
	const poll = 2 * time.Millisecond
	last := f.Flow().Time
	for range int(time.Second / poll) {
		time.Sleep(poll)
		turn = turn.Add(f.IMU().Gyro.Scale(float32(poll.Seconds())))
		if s := f.Flow(); s.Time != last {
			last = s.Time
			counts = counts.Add(fmath.Vec3{X: float32(s.DeltaX), Y: float32(s.DeltaY)})
		}
	}
	return counts, turn
}

func report(f *flight.Flight, sc sensors.Config) {
	var ms runtime.MemStats
	var lastMallocs uint64
	for {
		counts, turn := sumFlow(f)
		// Image motion in body axes, then with the part from turning removed, as in pkg/estimator/horizontal.
		motion := sc.FlowRotation.MulVec(counts).Scale(sc.FlowScale)
		moved := fmath.Vec3{X: motion.X - turn.Y, Y: motion.Y + turn.X}
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
			"z_mm", mm(f.Position().Z), "xy_mm", mm(f.Position().X), mm(f.Position().Y), f.Position().XYValid)
		println("mag_nT", mm(mag.X), mm(mag.Y), mm(mag.Z),
			"flow_counts", int(counts.X), int(counts.Y), "quality", flow.Quality, flow.Valid,
			"flow_mrad", mm(motion.X), mm(motion.Y), "moved_mrad", mm(moved.X), mm(moved.Y),
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
func startOnCores(f *flight.Flight, sc sensors.Config) {
	go func() {
		machine.AssignCore(0)
		report(f, sc)
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
