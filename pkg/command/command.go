// Package command is the wire format of the Tinyflight command protocol, used by a ground station to fly
// a drone with commands such as take off, hover and land. See docs/command-protocol.md.
package command

import (
	"encoding/binary"
	"errors"
)

var (
	ErrNotCommand = errors.New("command: not a command packet")
	ErrChecksum   = errors.New("command: bad checksum")
)

// Version is the protocol version this package speaks.
const Version = 1

// Type is the message type.
type Type uint8

// Ground station to drone.
const (
	TypeHello     Type = 0x01
	TypeTakeOff   Type = 0x02
	TypeLand      Type = 0x03
	TypeEmergency Type = 0x04
	TypePcmd      Type = 0x05
)

// Drone to ground station.
const (
	TypeInfo  Type = 0x81
	TypeState Type = 0x82
)

const headerSize = 6

// MaxSize is the longest packet, State.
const MaxSize = headerSize + statePayload + 1

const (
	takeOffPayload = 2
	pcmdPayload    = 4
	infoPayload    = 11
	statePayload   = 16
)

// Size returns the full packet length for t, or 0 for an unknown type.
func Size(t Type) int {
	n := 0
	switch t {
	case TypeHello, TypeLand, TypeEmergency:
	case TypeTakeOff:
		n = takeOffPayload
	case TypePcmd:
		n = pcmdPayload
	case TypeInfo:
		n = infoPayload
	case TypeState:
		n = statePayload
	default:
		return 0
	}
	return headerSize + n + 1
}

// Flying states. The values match FlyingState in https://github.com/hybridgroup/tinygo-minidrone
const (
	FlyingLanded    = 0
	FlyingTakingOff = 1
	FlyingHovering  = 2
	FlyingFlying    = 3
	FlyingLanding   = 4
	FlyingEmergency = 5
)

// Info flags.
const (
	InfoOwned   = 1 << 0
	InfoPosHold = 1 << 1
	InfoCamera  = 1 << 2
)

// State flags.
const (
	StateArmed       = 1 << 0
	StateHeightValid = 1 << 1
	StatePosValid    = 1 << 2
)

// Pcmd is a velocity request. Each axis is -100 to 100.
type Pcmd struct {
	Right, Forward, Yaw, Up int8
}

// Info is the drone's answer to Hello.
type Info struct {
	Protocol uint8
	Firmware [3]uint8 // major, minor, patch
	MAC      [6]byte
	Flags    uint8
}

// State is what the drone reports to its owner.
type State struct {
	Flying           uint8
	Failsafe         uint8
	Flags            uint8
	Mode             uint8
	Battery          uint16 // mV
	Height           int16  // cm above the takeoff point
	Roll, Pitch, Yaw int16  // 0.01 degree
	Ack              uint16 // sequence of the last TakeOff, Land or Emergency acted on
}

// Packet is any command protocol message. Only the field for Type is used.
type Packet struct {
	Type   Type
	Seq    uint16
	Height uint16 // TakeOff, cm, 0 for the default
	Pcmd   Pcmd
	Info   Info
	State  State
}

// Match reports whether data is a well formed command packet of a known type. It does not check the checksum.
// No type is 25 bytes long, so an Atom Joystick control packet never matches.
func Match(data []byte) bool {
	if len(data) < headerSize || data[0] != 'T' || data[1] != 'F' || data[2] != Version {
		return false
	}
	n := Size(Type(data[3]))
	return n != 0 && len(data) == n
}

// Encode writes p into buf and returns the packet length, or 0 for an unknown type.
func (p *Packet) Encode(buf *[MaxSize]byte) int {
	n := Size(p.Type)
	if n == 0 {
		return 0
	}
	buf[0], buf[1], buf[2], buf[3] = 'T', 'F', Version, byte(p.Type)
	binary.LittleEndian.PutUint16(buf[4:], p.Seq)
	b := buf[headerSize:]
	switch p.Type {
	case TypeTakeOff:
		binary.LittleEndian.PutUint16(b, p.Height)
	case TypePcmd:
		b[0], b[1], b[2], b[3] = byte(p.Pcmd.Right), byte(p.Pcmd.Forward), byte(p.Pcmd.Yaw), byte(p.Pcmd.Up)
	case TypeInfo:
		i := &p.Info
		b[0] = i.Protocol
		copy(b[1:4], i.Firmware[:])
		copy(b[4:10], i.MAC[:])
		b[10] = i.Flags
	case TypeState:
		s := &p.State
		b[0], b[1], b[2], b[3] = s.Flying, s.Failsafe, s.Flags, s.Mode
		binary.LittleEndian.PutUint16(b[4:], s.Battery)
		binary.LittleEndian.PutUint16(b[6:], uint16(s.Height))
		binary.LittleEndian.PutUint16(b[8:], uint16(s.Roll))
		binary.LittleEndian.PutUint16(b[10:], uint16(s.Pitch))
		binary.LittleEndian.PutUint16(b[12:], uint16(s.Yaw))
		binary.LittleEndian.PutUint16(b[14:], s.Ack)
	}
	buf[n-1] = checksum(buf[:n-1])
	return n
}

// Decode reads a command packet into p.
func Decode(data []byte, p *Packet) error {
	if !Match(data) {
		return ErrNotCommand
	}
	n := len(data)
	if checksum(data[:n-1]) != data[n-1] {
		return ErrChecksum
	}
	p.Type = Type(data[3])
	p.Seq = binary.LittleEndian.Uint16(data[4:])
	b := data[headerSize:]
	switch p.Type {
	case TypeTakeOff:
		p.Height = binary.LittleEndian.Uint16(b)
	case TypePcmd:
		p.Pcmd = Pcmd{Right: int8(b[0]), Forward: int8(b[1]), Yaw: int8(b[2]), Up: int8(b[3])}
	case TypeInfo:
		i := &p.Info
		i.Protocol = b[0]
		copy(i.Firmware[:], b[1:4])
		copy(i.MAC[:], b[4:10])
		i.Flags = b[10]
	case TypeState:
		s := &p.State
		s.Flying, s.Failsafe, s.Flags, s.Mode = b[0], b[1], b[2], b[3]
		s.Battery = binary.LittleEndian.Uint16(b[4:])
		s.Height = int16(binary.LittleEndian.Uint16(b[6:]))
		s.Roll = int16(binary.LittleEndian.Uint16(b[8:]))
		s.Pitch = int16(binary.LittleEndian.Uint16(b[10:]))
		s.Yaw = int16(binary.LittleEndian.Uint16(b[12:]))
		s.Ack = binary.LittleEndian.Uint16(b[14:])
	}
	return nil
}

func checksum(b []byte) byte {
	var s byte
	for _, v := range b {
		s += v
	}
	return s
}
