// Package input turns controller messages into pilot input for the flight stack.
package input

import (
	"encoding/binary"
	"errors"
	"math"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var (
	ErrShort    = errors.New("input: packet too short")
	ErrNotForUs = errors.New("input: packet for another vehicle")
	ErrChecksum = errors.New("input: bad checksum")
)

// AtomPacketSize is the ESP-NOW packet the M5 Atom Joystick sends to a StampFly.
// Layout from OnDataRecv in https://github.com/m5stack/M5StampFly/blob/main/src/rc.cpp
const AtomPacketSize = 25

// Mode byte values from https://github.com/m5stack/M5StampFly/blob/main/src/flight_control.hpp
const (
	atomRateControl = 1
	atomAutoAlt     = 4
	stickDeadband   = 0.2
)

// AtomJoystick decodes Atom Joystick packets. The arm button toggles arming each time it is released.
type AtomJoystick struct {
	mac       [3]byte
	armed     bool
	buttonWas bool
}

// NewAtomJoystick takes the vehicle's own MAC address, whose last 3 bytes start every packet.
func NewAtomJoystick(mac [6]byte) *AtomJoystick {
	return &AtomJoystick{mac: [3]byte{mac[3], mac[4], mac[5]}}
}

// Decode checks a packet is for this vehicle and turns it into pilot input stamped with now.
func (a *AtomJoystick) Decode(data []byte, now msg.Time, out *msg.ManualInput) error {
	if len(data) < AtomPacketSize {
		return ErrShort
	}
	if data[0] != a.mac[0] || data[1] != a.mac[1] || data[2] != a.mac[2] {
		return ErrNotForUs
	}
	var sum byte
	for _, b := range data[:24] {
		sum += b
	}
	if sum != data[24] {
		return ErrChecksum
	}

	yaw := f32(data[3:])
	throttle := f32(data[7:])
	roll := f32(data[11:])
	pitch := f32(data[15:])

	button := data[19] != 0
	if a.buttonWas && !button {
		a.armed = !a.armed
	}
	a.buttonWas = button

	out.Time = now
	out.Roll = fmath.Clamp(roll, -1, 1)
	out.Pitch = fmath.Clamp(pitch, -1, 1)
	out.Yaw = fmath.Clamp(yaw, -1, 1)
	out.Throttle = 0
	if throttle > stickDeadband {
		out.Throttle = fmath.Clamp(throttle, 0, 1)
	}
	out.Climb = fmath.Deadband(fmath.Clamp(throttle, -1, 1), stickDeadband)
	out.Arm = a.armed
	out.Mode = msg.ModeStabilize
	if data[21] == atomRateControl {
		out.Mode = msg.ModeAcro
	}
	if data[22] == atomAutoAlt {
		out.Mode = msg.ModeAltHold
	}
	out.Valid = true
	return nil
}

func f32(b []byte) float32 {
	v := math.Float32frombits(binary.LittleEndian.Uint32(b))
	if v != v {
		return 0
	}
	return v
}
