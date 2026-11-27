package espnowinput

import (
	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

// Telemetry makes the packets the drone sends back on a Link. Call Tick 20 times a second.
// It does not allocate, and the data passed to send is only valid during the call.
type Telemetry struct {
	Link      *Link
	Info      command.Info // the answer to Hello, Protocol and the owned flag are filled in
	FillAtom  func(t *input.AtomTelemetry)
	FillState func(s *command.State)
	tick      int
	seq       uint16
	atom      input.AtomTelemetry
	atomBuf   [input.AtomTelemetrySize]byte
	packet    command.Packet
	buf       [command.MaxSize]byte
}

// Tick answers a waiting Hello with Info, and sends Atom telemetry to a joystick owner or State
// to a ground station owner every other tick. See docs/command-protocol.md.
func (t *Telemetry) Tick(send func(addr [6]byte, data []byte)) {
	t.tick++
	p := &t.packet
	if addr, ok := t.Link.TakeHello(); ok {
		*p = command.Packet{Type: command.TypeInfo, Info: t.Info}
		p.Info.Protocol = command.Version
		if _, kind := t.Link.Owner(); kind != OwnerNone {
			p.Info.Flags |= command.InfoOwned
		}
		send(addr, t.encode())
	}
	addr, kind := t.Link.Owner()
	switch kind {
	case OwnerJoystick:
		t.FillAtom(&t.atom)
		t.atom.Encode(&t.atomBuf)
		send(addr, t.atomBuf[:])
	case OwnerGround:
		if t.tick%2 != 0 {
			return
		}
		*p = command.Packet{Type: command.TypeState}
		t.FillState(&p.State)
		t.Link.FillState(&p.State)
		send(addr, t.encode())
	}
}

func (t *Telemetry) encode() []byte {
	t.seq++
	t.packet.Seq = t.seq
	return t.buf[:t.packet.Encode(&t.buf)]
}
