package espnowinput

import (
	"sync"
	"sync/atomic"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Owner is the kind of controller flying the drone.
type Owner uint8

const (
	OwnerNone Owner = iota
	OwnerJoystick
	OwnerGround
)

// Release is how long the owner must be quiet, with the drone disarmed, before another controller can take over.
const Release = 2 * time.Second

// Link sorts packets into the Atom Joystick and command protocols, and keeps one owner at a time.
// See docs/command-protocol.md. It is safe for the radio and the flight loop to use at the same time.
type Link struct {
	mu      sync.Mutex
	atom    *input.AtomJoystick
	cmds    input.Commands
	vehicle input.Vehicle
	latest  msg.ManualInput
	fresh   bool
	owner   [6]byte
	kind    Owner
	heard   time.Duration
	hello   [6]byte
	pending bool
	packet  command.Packet

	Packets  atomic.Uint32
	Rejected atomic.Uint32
}

// NewLink takes the drone's own MAC, since Atom packets start with its last 3 bytes.
func NewLink(mac [6]byte) *Link {
	return &Link{atom: input.NewAtomJoystick(mac)}
}

// SetVehicle gives the link the arm state and height it needs, usually a *flight.Flight.
func (l *Link) SetVehicle(v input.Vehicle) {
	l.mu.Lock()
	l.vehicle = v
	l.cmds.Vehicle = v
	l.mu.Unlock()
}

// Receive handles one packet from src. now is any clock that only goes forward.
func (l *Link) Receive(src [6]byte, data []byte, now time.Duration) {
	l.mu.Lock()
	ok := l.receive(src, data, now)
	l.mu.Unlock()
	if ok {
		l.Packets.Add(1)
	} else {
		l.Rejected.Add(1)
	}
}

func (l *Link) receive(src [6]byte, data []byte, now time.Duration) bool {
	if command.Match(data) {
		p := &l.packet
		if command.Decode(data, p) != nil {
			return false
		}
		if p.Type == command.TypeHello {
			l.hello, l.pending = src, true
			return true
		}
		if !l.claim(src, OwnerGround, now) {
			return false
		}
		l.cmds.Handle(p)
		return true
	}
	var in msg.ManualInput
	if l.atom.Decode(data, 0, &in) != nil || !l.claim(src, OwnerJoystick, now) {
		return false
	}
	l.latest, l.fresh = in, true
	return true
}

// claim reports whether src may fly the drone, and makes it the owner if there is none.
func (l *Link) claim(src [6]byte, kind Owner, now time.Duration) bool {
	if l.kind != OwnerNone && src != l.owner && now-l.heard > Release && !l.armed() {
		l.kind = OwnerNone
	}
	switch {
	case l.kind == OwnerNone:
		l.owner, l.kind = src, kind
		l.fresh = false
		l.cmds = input.Commands{Vehicle: l.vehicle}
	case src != l.owner || kind != l.kind:
		return false
	}
	l.heard = now
	return true
}

func (l *Link) armed() bool { return l.vehicle != nil && l.vehicle.Status().Armed }

// Owner returns the address and kind of the controller flying the drone.
func (l *Link) Owner() ([6]byte, Owner) {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.owner, l.kind
}

// TakeHello returns the sender of a Hello waiting for an Info answer, if there is one.
func (l *Link) TakeHello() ([6]byte, bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	ok := l.pending
	l.pending = false
	return l.hello, ok
}

// FillState sets the flying state and command ack in s. The rest is up to the caller.
func (l *Link) FillState(s *command.State) {
	l.mu.Lock()
	s.Flying = l.cmds.Flying()
	s.Ack, _ = l.cmds.Ack()
	l.mu.Unlock()
}

// ReadInput gives the owner's latest input.
func (l *Link) ReadInput(out *msg.ManualInput) bool {
	l.mu.Lock()
	defer l.mu.Unlock()
	switch l.kind {
	case OwnerJoystick:
		ok := l.fresh
		if ok {
			*out = l.latest
			l.fresh = false
		}
		return ok
	case OwnerGround:
		return l.cmds.ReadInput(out)
	}
	return false
}
