// Package params has every tunable value in one place. Boards start from Default and change what differs.
package params

import (
	"github.com/hybridgroup/tinyflight/pkg/commander"
	"github.com/hybridgroup/tinyflight/pkg/control/attitude"
	"github.com/hybridgroup/tinyflight/pkg/control/pid"
	"github.com/hybridgroup/tinyflight/pkg/control/position"
	"github.com/hybridgroup/tinyflight/pkg/control/rate"
	altest "github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	attest "github.com/hybridgroup/tinyflight/pkg/estimator/attitude"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/mixer"
	"github.com/hybridgroup/tinyflight/pkg/sensors"
)

// Config holds the settings for every part of the flight stack.
type Config struct {
	LoopHz    float32
	Sensors   sensors.Config
	AttEst    attest.Config
	AltEst    altest.Config
	Rate      rate.Config
	Attitude  attitude.Config
	Vertical  position.Config
	Mixer     mixer.Config
	Commander commander.Config
	Manual    Manual
	Schedule  Schedule
}

// Manual sets how far full stick goes.
type Manual struct {
	MaxTilt    float32    // rad in Stabilize and AltHold
	AcroRate   fmath.Vec3 // rad/s in Acro
	MaxYawRate float32    // rad/s
	MaxClimb   float32    // m/s in AltHold
}

// Schedule sets how often the slower sensors are read, in Hz. 0 skips a sensor.
type Schedule struct {
	BaroHz, RangeHz, FlowHz, MagHz, BatteryHz float32
}

// Default fits a small brushed quad like the StampFly running the loop at 400 Hz.
func Default() Config {
	const hz = 400
	ratePID := func(p, i, d float32) pid.Config {
		return pid.Config{P: p, I: i, D: d, IntegralLimit: 0.15, OutputLimit: 0.5, DCutoffHz: 40, SampleHz: hz}
	}
	return Config{
		LoopHz: hz,
		Sensors: sensors.Config{
			Rotation:           fmath.Identity3(),
			MagRotation:        fmath.Identity3(),
			SampleHz:           hz,
			GyroCutoffHz:       80,
			AccelCutoffHz:      30,
			CalibrationSamples: hz,
			StillThreshold:     0.1,
		},
		// Low Kp because in flight the accelerometer cannot tell tilt apart from sideways acceleration.
		AttEst: attest.Config{Kp: 0.2, Ki: 0.02, AccelGate: 0.2},
		AltEst: altest.Config{
			AccelNoise: 0.5,
			BiasNoise:  0.01,
			BaroNoise:  0.5,
			RangeNoise: 0.03,
			Gate:       5,
			RangeMax:   2,
			RangeHold:  0.3,
			BaroTrack:  0.02,
		},
		Rate: rate.Config{
			Roll:  ratePID(0.03, 0.06, 0.0006),
			Pitch: ratePID(0.03, 0.06, 0.0006),
			Yaw:   ratePID(0.1, 0.05, 0),
		},
		Attitude: attitude.Config{
			P:       fmath.Vec3{X: 8, Y: 8, Z: 2},
			MaxRate: fmath.Vec3{X: 6, Y: 6, Z: 3},
		},
		Vertical: position.Config{
			ZP:          1.5,
			Climb:       pid.Config{P: 0.15, I: 0.1, IntegralLimit: 0.2, SampleHz: hz},
			HoverThrust: 0.43,
			MaxClimb:    0.5,
			MaxDescend:  0.4,
			MinThrust:   0.1,
			MaxThrust:   0.9,
		},
		Mixer: mixer.Config{Geometry: mixer.QuadX, ThrustModel: 1, Idle: 0.05},
		Commander: commander.Config{
			InputTimeout:   0.5,
			LowBattery:     3.34,
			LowBatteryTime: 2,
			MaxTilt:        1.2,
			ArmMaxTilt:     0.35,
			ArmMaxThrottle: 0.05,
			LandTime:       4,
		},
		Manual: Manual{
			MaxTilt:    0.4,
			AcroRate:   fmath.Vec3{X: 4, Y: 4, Z: 3},
			MaxYawRate: 3,
			MaxClimb:   0.5,
		},
		Schedule: Schedule{BaroHz: 50, RangeHz: 30, FlowHz: 0, MagHz: 20, BatteryHz: 10},
	}
}
