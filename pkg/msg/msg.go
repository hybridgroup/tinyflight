// Package msg has the data passed between the stages of the flight stack.
// Units are SI. Body frame is FRD and earth frame is NED, so Z points down.
package msg

import "github.com/hybridgroup/tinyflight/pkg/fmath"

// Time is microseconds since boot.
type Time uint64

// IMUSample is one accel and gyro reading.
type IMUSample struct {
	Time        Time
	Accel       fmath.Vec3 // m/s², reads (0, 0, -9.8) when level and still
	Gyro        fmath.Vec3 // rad/s
	Temperature float32    // °C
}

// MagSample is one magnetic field reading.
type MagSample struct {
	Time  Time
	Field fmath.Vec3 // µT
}

// BaroSample is one pressure and temperature reading.
type BaroSample struct {
	Time        Time
	Pressure    float32 // Pa
	Temperature float32 // °C
}

// RangeSample is one distance reading from the rangefinder.
type RangeSample struct {
	Time     Time
	Distance float32 // m
	Valid    bool
}

// FlowSample is the optical flow motion since the last reading.
type FlowSample struct {
	Time    Time
	DeltaX  int16 // raw sensor counts
	DeltaY  int16
	Quality uint8
	Valid   bool
	// Rate is the ground image motion in body axes, rad/s. X is positive when moving forward
	// and Y when moving right, before the part caused by rotation is removed.
	Rate fmath.Vec3
}

// BatteryStatus is one battery voltage and current reading.
type BatteryStatus struct {
	Time    Time
	Voltage float32 // V
	Current float32 // A
}

// Attitude is the estimated orientation and body rate.
type Attitude struct {
	Time Time
	Q    fmath.Quat
	Rate fmath.Vec3 // rad/s, filtered body rate
}

// LocalPosition is the estimated position and speed relative to the takeoff point.
type LocalPosition struct {
	Time    Time
	X       float32 // m north
	Y       float32 // m east
	VX      float32 // m/s north
	VY      float32 // m/s east
	XYValid bool
	Z       float32 // m, negative is above the takeoff point
	VZ      float32 // m/s, negative is climbing
	ZValid  bool
	Ground  float32 // m, distance to the ground below
	GValid  bool
	AccBias float32 // m/s², estimated vertical accel bias
}

// RateSetpoint is the wanted body rate and thrust.
type RateSetpoint struct {
	Rate   fmath.Vec3 // rad/s
	Thrust float32    // 0 to 1
}

// AttitudeSetpoint is the wanted orientation, yaw rate and thrust.
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

// MotorOutputs holds one command per motor.
type MotorOutputs struct {
	Count int
	Value [MaxMotors]float32 // 0 to 1
}

// FlightMode selects how pilot input is used.
type FlightMode uint8

const (
	ModeAcro FlightMode = iota
	ModeStabilize
	ModeAltHold
	ModePosHold
)

// String returns the mode name.
func (m FlightMode) String() string {
	switch m {
	case ModeAcro:
		return "acro"
	case ModeStabilize:
		return "stabilize"
	case ModeAltHold:
		return "althold"
	case ModePosHold:
		return "poshold"
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
	Climb    float32 // -1 to 1, climb request used in AltHold and PosHold
	Arm      bool
	Mode     FlightMode
	Valid    bool
}

// Failsafe is the reason for the last automatic landing or disarm.
type Failsafe uint8

const (
	FailsafeNone Failsafe = iota
	FailsafeInputLost
	FailsafeLowBattery
	FailsafeTilt
)

// String returns the failsafe name.
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

// VehicleStatus is the arm state, flight mode and failsafe state.
type VehicleStatus struct {
	Time     Time
	Armed    bool
	Mode     FlightMode
	Landing  bool
	Failsafe Failsafe
}
