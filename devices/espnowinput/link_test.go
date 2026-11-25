package espnowinput

import (
	"encoding/binary"
	"math"
	"testing"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var (
	droneMAC = [6]byte{0x4c, 0x75, 0x25, 0xad, 0x8b, 0x20}
	joyA     = [6]byte{1, 1, 1, 1, 1, 1}
	joyB     = [6]byte{2, 2, 2, 2, 2, 2}
	groundA  = [6]byte{3, 3, 3, 3, 3, 3}
)

type vehicle struct{ armed bool }

func (v *vehicle) Status() msg.VehicleStatus   { return msg.VehicleStatus{Armed: v.armed} }
func (v *vehicle) Position() msg.LocalPosition { return msg.LocalPosition{} }

func atomPacket(roll float32) []byte {
	p := make([]byte, 25)
	copy(p, droneMAC[3:])
	binary.LittleEndian.PutUint32(p[11:], math.Float32bits(roll))
	var sum byte
	for _, b := range p[:24] {
		sum += b
	}
	p[24] = sum
	return p
}

func cmdPacket(typ command.Type, seq uint16) []byte {
	var buf [command.MaxSize]byte
	n := (&command.Packet{Type: typ, Seq: seq}).Encode(&buf)
	return buf[:n]
}

func TestLinkJoystickOwns(t *testing.T) {
	l := NewLink(droneMAC)
	var in msg.ManualInput
	if l.ReadInput(&in) {
		t.Fatal("input with no owner")
	}
	l.Receive(joyA, atomPacket(0.5), 0)
	if _, kind := l.Owner(); kind != OwnerJoystick {
		t.Fatalf("owner %d", kind)
	}
	if !l.ReadInput(&in) || in.Roll != 0.5 {
		t.Fatalf("input %+v", in)
	}
	l.Receive(joyB, atomPacket(-1), time.Millisecond)
	l.Receive(groundA, cmdPacket(command.TypeTakeOff, 1), time.Millisecond)
	if l.ReadInput(&in) {
		t.Errorf("took input from another controller %+v", in)
	}
	if l.Rejected.Load() != 2 || l.Packets.Load() != 1 {
		t.Errorf("packets %d rejected %d", l.Packets.Load(), l.Rejected.Load())
	}
}

func TestLinkGroundOwns(t *testing.T) {
	l := NewLink(droneMAC)
	l.SetVehicle(&vehicle{})
	l.Receive(groundA, cmdPacket(command.TypePcmd, 1), 0)
	if owner, kind := l.Owner(); kind != OwnerGround || owner != groundA {
		t.Fatalf("owner %v %d", owner, kind)
	}
	l.Receive(joyA, atomPacket(1), time.Millisecond)
	var in msg.ManualInput
	if !l.ReadInput(&in) || in.Roll != 0 || in.Mode != msg.ModePosHold {
		t.Fatalf("input %+v", in)
	}
	l.Receive(groundA, cmdPacket(command.TypeTakeOff, 2), 2*time.Millisecond)
	var s command.State
	l.FillState(&s)
	if s.Ack != 2 {
		t.Errorf("ack %d", s.Ack)
	}
}

func TestLinkHello(t *testing.T) {
	l := NewLink(droneMAC)
	l.Receive(joyA, atomPacket(0), 0)
	l.Receive(groundA, cmdPacket(command.TypeHello, 1), 0)
	if src, ok := l.TakeHello(); !ok || src != groundA {
		t.Fatalf("hello from %v %v", src, ok)
	}
	if _, ok := l.TakeHello(); ok {
		t.Error("hello answered twice")
	}
	if _, kind := l.Owner(); kind != OwnerJoystick {
		t.Errorf("hello changed the owner to %d", kind)
	}
}

func TestLinkRelease(t *testing.T) {
	v := &vehicle{armed: true}
	l := NewLink(droneMAC)
	l.SetVehicle(v)
	l.Receive(joyA, atomPacket(0), 0)
	l.Receive(groundA, cmdPacket(command.TypePcmd, 1), Release+time.Second)
	if _, kind := l.Owner(); kind != OwnerJoystick {
		t.Fatal("taken over while armed")
	}
	v.armed = false
	l.Receive(groundA, cmdPacket(command.TypePcmd, 2), Release/2)
	if _, kind := l.Owner(); kind != OwnerJoystick {
		t.Fatal("taken over while the owner was still talking")
	}
	l.Receive(groundA, cmdPacket(command.TypePcmd, 3), 2*Release)
	if owner, kind := l.Owner(); kind != OwnerGround || owner != groundA {
		t.Fatalf("not taken over, owner %v %d", owner, kind)
	}
}

func TestLinkBadPackets(t *testing.T) {
	l := NewLink(droneMAC)
	bad := cmdPacket(command.TypeTakeOff, 1)
	bad[len(bad)-1]++
	l.Receive(groundA, bad, 0)
	l.Receive(joyA, []byte{1, 2, 3}, 0)
	if _, kind := l.Owner(); kind != OwnerNone {
		t.Errorf("bad packet made owner %d", kind)
	}
	if l.Rejected.Load() != 2 {
		t.Errorf("rejected %d", l.Rejected.Load())
	}
}
