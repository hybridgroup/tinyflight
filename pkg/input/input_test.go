package input

import (
	"encoding/binary"
	"math"
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var mac = [6]byte{0x4c, 0x75, 0x25, 0xad, 0x8b, 0x20}

func packet(yaw, throttle, roll, pitch float32, button, ctrl, alt byte) []byte {
	p := make([]byte, AtomPacketSize)
	copy(p, mac[3:])
	for i, v := range []float32{yaw, throttle, roll, pitch} {
		binary.LittleEndian.PutUint32(p[3+4*i:], math.Float32bits(v))
	}
	p[19], p[21], p[22] = button, ctrl, alt
	var sum byte
	for _, b := range p[:24] {
		sum += b
	}
	p[24] = sum
	return p
}

func TestAtomDecode(t *testing.T) {
	a := NewAtomJoystick(mac)
	var in msg.ManualInput
	if err := a.Decode(packet(0.5, 0.6, -0.25, 0.75, 0, 0, 5), 7, &in); err != nil {
		t.Fatal(err)
	}
	if fmath.Abs(in.Climb-0.5) > 1e-5 || fmath.Abs(in.Throttle-0.5) > 1e-5 {
		t.Errorf("climb %v throttle %v, want both 0.5 after the deadband", in.Climb, in.Throttle)
	}
	in.Climb, in.Throttle = 0, 0
	want := msg.ManualInput{Time: 7, Roll: -0.25, Pitch: 0.75, Yaw: 0.5, Mode: msg.ModeStabilize, Valid: true}
	if in != want {
		t.Errorf("got %+v", in)
	}
	a.Decode(packet(0, 0.1, 0, 0, 0, 0, 4), 8, &in)
	if in.Mode != msg.ModeAltHold || in.Throttle != 0 || in.Climb != 0 {
		t.Errorf("althold or deadband wrong %+v", in)
	}
	a.Decode(packet(0, 0, 0, 0, 0, 1, 4), 8, &in)
	if in.Mode != msg.ModePosHold {
		t.Errorf("rate with altitude mode gave %v, want poshold", in.Mode)
	}
	a.Decode(packet(0, 0, 0, 0, 0, 1, 5), 9, &in)
	if in.Mode != msg.ModeAcro {
		t.Errorf("mode %v", in.Mode)
	}
}

func TestAtomArmToggle(t *testing.T) {
	a := NewAtomJoystick(mac)
	var in msg.ManualInput
	for i, step := range []struct {
		button byte
		arm    bool
	}{{0, false}, {1, false}, {1, false}, {0, true}, {1, true}, {0, false}} {
		a.Decode(packet(0, 0, 0, 0, step.button, 0, 5), 0, &in)
		if in.Arm != step.arm {
			t.Errorf("step %d arm %v", i, in.Arm)
		}
	}
}

func TestAtomRejects(t *testing.T) {
	a := NewAtomJoystick(mac)
	var in msg.ManualInput
	p := packet(0, 0, 0, 0, 0, 0, 5)
	p[24]++
	if a.Decode(p, 0, &in) != ErrChecksum {
		t.Error("bad checksum accepted")
	}
	p = packet(0, 0, 0, 0, 0, 0, 5)
	p[0]++
	if a.Decode(p, 0, &in) != ErrNotForUs {
		t.Error("other vehicle accepted")
	}
	if a.Decode(p[:10], 0, &in) != ErrShort {
		t.Error("short packet accepted")
	}
}

func TestParseLine(t *testing.T) {
	var in msg.ManualInput
	if err := ParseLine([]byte("rc 0.1 -0.25 0 .5 1 2\r\n"), 3, &in); err != nil {
		t.Fatal(err)
	}
	if in.Roll != 0.1 || in.Pitch != -0.25 || in.Throttle != 0.5 || in.Climb != 0 || !in.Arm || in.Mode != msg.ModeAltHold || !in.Valid {
		t.Errorf("got %+v", in)
	}
	if ParseLine([]byte("rc 1 2"), 0, &in) != ErrBadLine || ParseLine([]byte("xx 1 2 3 4 5 6"), 0, &in) != ErrBadLine {
		t.Error("bad line accepted")
	}
	if n := testing.AllocsPerRun(100, func() { ParseLine([]byte("rc 0 0 0 0 0 1"), 0, &in) }); n > 0 {
		t.Errorf("ParseLine allocates %v times", n)
	}
}

func TestAtomTelemetry(t *testing.T) {
	var p [AtomTelemetrySize]byte
	tel := AtomTelemetry{Roll: 1.5, Pitch: -2, Yaw: 90, Voltage: 3.9, Altitude: 0.75, AltFlag: 1, Mode: AtomModeFlight, FrontRange: 1234}
	tel.Encode(&p)
	f := func(off int) float32 { return math.Float32frombits(binary.LittleEndian.Uint32(p[off:])) }
	// Offsets as the joystick reads them, 2 + 4*(slot-1), in OnDataRecv of the Atom-JoyStick StampFlyController.
	if p[0] != 88 || p[1] != 88 {
		t.Fatalf("header %d %d", p[0], p[1])
	}
	if f(10) != 1.5 || f(14) != -2 || f(18) != 90 || f(58) != 3.9 || f(98) != 0.75 {
		t.Errorf("floats %v %v %v %v %v", f(10), f(14), f(18), f(58), f(98))
	}
	if p[110] != 1 || p[111] != AtomModeFlight || binary.LittleEndian.Uint16(p[112:]) != 1234 {
		t.Errorf("flags %d %d %d", p[110], p[111], binary.LittleEndian.Uint16(p[112:]))
	}
	if n := testing.AllocsPerRun(100, func() { tel.Encode(&p) }); n > 0 {
		t.Errorf("Encode allocates %v times", n)
	}
}
