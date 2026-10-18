// Package msg has the data passed between the stages of the flight stack.
// Units are SI. Body frame is FRD and earth frame is NED, so Z points down.
package msg

import "github.com/hybridgroup/tinyflight/pkg/fmath"

// Time is microseconds since boot.
type Time uint64

type IMUSample struct {
	Time        Time
	Accel       fmath.Vec3 // m/s², reads (0, 0, -9.8) when level and still
	Gyro        fmath.Vec3 // rad/s
	Temperature float32    // °C
}

type MagSample struct {
	Time  Time
	Field fmath.Vec3 // µT
}

type BaroSample struct {
	Time        Time
	Pressure    float32 // Pa
	Temperature float32 // °C
}

type RangeSample struct {
	Time     Time
	Distance float32 // m
	Valid    bool
}

type FlowSample struct {
	Time    Time
	DeltaX  int16 // raw sensor counts
	DeltaY  int16
	Quality uint8
	Valid   bool
}

type BatteryStatus struct {
	Time    Time
	Voltage float32 // V
	Current float32 // A
}

type Attitude struct {
	Time Time
	Q    fmath.Quat
	Rate fmath.Vec3 // rad/s, filtered body rate
}

type LocalPosition struct {
	Time    Time
	Z       float32 // m, negative is above the takeoff point
	VZ      float32 // m/s, negative is climbing
	ZValid  bool
	Ground  float32 // m, distance to the ground below
	GValid  bool
	AccBias float32 // m/s², estimated vertical accel bias
}

type RateSetpoint struct {
	Rate   fmath.Vec3 // rad/s
	Thrust float32    // 0 to 1
}

type AttitudeSetpoint struct {
	Q       fmath.Quat
	YawRate float32 // rad/s feedforward
	Thrust  float32 // 0 to 1
}

// ThrustTorque is the request to the mixer. Torque is normalized to about -1 to 1.
type ThrustTorque struct {
	Thrust float32
	Torque fmath.Vec3
}

const MaxMotors = 8

type MotorOutputs struct {
	Count int
	Value [MaxMotors]float32 // 0 to 1
}

type FlightMode uint8

const (
	ModeAcro FlightMode = iota
	ModeStabilize
	ModeAltHold
)

func (m FlightMode) String() string {
	switch m {
	case ModeAcro:
		return "acro"
	case ModeStabilize:
		return "stabilize"
	case ModeAltHold:
		return "althold"
	}
	return "unknown"
}

// ManualInput is what the pilot asks for. Sticks are -1 to 1, throttle is 0 to 1.
type ManualInput struct {
	Time     Time
	Roll     float32 // positive rolls right
	Pitch    float32 // positive pitches nose up
	Yaw      float32 // positive turns right
	Throttle float32
	Climb    float32 // -1 to 1, climb request used in AltHold
	Arm      bool
	Mode     FlightMode
	Valid    bool
}

type Failsafe uint8

const (
	FailsafeNone Failsafe = iota
	FailsafeInputLost
	FailsafeLowBattery
	FailsafeTilt
)

func (f Failsafe) String() string {
	switch f {
	case FailsafeNone:
		return "none"
	case FailsafeInputLost:
		return "input lost"
	case FailsafeLowBattery:
		return "low battery"
	case FailsafeTilt:
		return "tilt"
	}
	return "unknown"
}

type VehicleStatus struct {
	Time     Time
	Armed    bool
	Mode     FlightMode
	Landing  bool
	Failsafe Failsafe
}
