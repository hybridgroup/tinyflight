package input

import (
	"encoding/binary"
	"math"
)

// AtomTelemetrySize is the packet the StampFly sends back to the Atom Joystick, MAXINDEX in telemetry.cpp.
// Layout from make_telemetry_data in https://github.com/m5stack/M5StampFly/blob/main/src/telemetry.cpp
const AtomTelemetrySize = 120

// Flight modes the joystick shows, from flight_control.hpp in the M5StampFly firmware.
const (
	AtomModeFlight  = 2
	AtomModeParking = 3
	AtomModeLanding = 5
)

// AtomTelemetry is the part of the stock telemetry the joystick displays.
type AtomTelemetry struct {
	Roll, Pitch, Yaw float32 // degrees
	Voltage          float32 // V
	Altitude         float32 // m above the takeoff point
	AltFlag          uint8   // 1 when height hold is active
	Mode             uint8   // one of the AtomMode values
	FrontRange       uint16  // mm
}

// Slot numbers count 4 byte floats after the two header bytes, starting at 1.
const (
	slotRoll     = 3
	slotPitch    = 4
	slotYaw      = 5
	slotVoltage  = 15
	slotAltitude = 25
	slotFlags    = 28
)

// Encode fills p with a telemetry packet. Fields the joystick does not show are left at zero.
func (t *AtomTelemetry) Encode(p *[AtomTelemetrySize]byte) {
	*p = [AtomTelemetrySize]byte{}
	p[0], p[1] = 88, 88
	putSlot(p, slotRoll, t.Roll)
	putSlot(p, slotPitch, t.Pitch)
	putSlot(p, slotYaw, t.Yaw)
	putSlot(p, slotVoltage, t.Voltage)
	putSlot(p, slotAltitude, t.Altitude)
	i := slotOffset(slotFlags)
	p[i] = t.AltFlag
	p[i+1] = t.Mode
	binary.LittleEndian.PutUint16(p[i+2:], t.FrontRange)
}

func slotOffset(slot int) int { return 2 + 4*(slot-1) }

func putSlot(p *[AtomTelemetrySize]byte, slot int, v float32) {
	binary.LittleEndian.PutUint32(p[slotOffset(slot):], math.Float32bits(v))
}
