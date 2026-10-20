package stampfly

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/params"
)

// Config returns the flight settings for the StampFly.
func Config() params.Config {
	cfg := params.Default()
	// IMU to body axes, from Accel_x_raw = acc_y and so on in sensor.cpp of the M5StampFly firmware.
	cfg.Sensors.Rotation = fmath.Mat3{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}
	// Flow chip to body axes, from ReadMotionBurst in sf_hal_pmw3901/src/pmw3901_wrapper.cpp
	// of https://github.com/M5Fly-kanazawa/stampfly_ecosystem
	cfg.Sensors.FlowRotation = fmath.Mat3{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}
	// Measured on the bench, where 0.00222 read about 1.4 times too much.
	// See https://github.com/hybridgroup/tinyflight/pull/1
	cfg.Sensors.FlowScale = 0.0016
	cfg.AttEst.UseMag = false
	// Hover duty from get_trim_duty in flight_control.cpp of https://github.com/m5stack/M5StampFly
	cfg.Hover = params.Hover{Slope: -0.2448, Offset: 1.5892}
	cfg.Vertical.HoverThrust = 0.35
	// Indoor test flights, see https://github.com/hybridgroup/tinyflight/pull/1
	cfg.Manual.MaxHeight = 1.2
	// A fit of flight 10 gave 12.7 m/s² per unit thrust near hover, 0.45 times the sim the gains were tuned in.
	// See https://github.com/hybridgroup/tinyflight/pull/1
	cfg.Vertical.Climb.P = 0.33
	cfg.Vertical.Climb.I = 0.22
	// The same motor response sets yaw torque. On flight 18 the heading wandered 27 degrees RMS.
	cfg.Rate.Yaw.P *= 2.2
	cfg.Rate.Yaw.I *= 2.2
	// Mean rate integrator torque in a steady hover on flight 20. Without it the drone tipped at liftoff.
	// See https://github.com/hybridgroup/tinyflight/pull/1
	cfg.Rate.Trim = fmath.Vec3{X: 0.020, Y: -0.012, Z: 0.012}
	// On flight 12 a wrong height estimate pushed full thrust and shot the drone up.
	// See https://github.com/hybridgroup/tinyflight/pull/1
	cfg.Vertical.MaxHover = 1.7
	// Replaying flight 14 through the height filter, the bottom ToF reading fits the accel best 0.2 s late.
	// See https://github.com/hybridgroup/tinyflight/pull/1
	cfg.AltEst.RangeDelay = 0.2
	return cfg
}
