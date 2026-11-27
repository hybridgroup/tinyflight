package espnowinput

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

type sent struct {
	addr [6]byte
	data []byte
}

func tick(t *Telemetry) []sent {
	var out []sent
	t.Tick(func(addr [6]byte, data []byte) { out = append(out, sent{addr, append([]byte(nil), data...)}) })
	return out
}

func TestTelemetry(t *testing.T) {
	l := NewLink(droneMAC)
	l.SetVehicle(&vehicle{})
	tel := &Telemetry{
		Link:      l,
		Info:      command.Info{Firmware: [3]uint8{1, 2, 3}, MAC: droneMAC},
		FillAtom:  func(a *input.AtomTelemetry) { a.Altitude = 1 },
		FillState: func(s *command.State) { s.Battery = 3900 },
	}
	if out := tick(tel); len(out) != 0 {
		t.Fatalf("sent %d packets with no owner and no Hello", len(out))
	}

	l.Receive(groundA, cmdPacket(command.TypeHello, 1), 0)
	out := tick(tel)
	var p command.Packet
	if len(out) != 1 || out[0].addr != groundA || command.Decode(out[0].data, &p) != nil || p.Type != command.TypeInfo {
		t.Fatalf("Hello answered with %+v", out)
	}
	if p.Info.Protocol != command.Version || p.Info.MAC != droneMAC || p.Info.Flags&command.InfoOwned != 0 {
		t.Errorf("info %+v", p.Info)
	}

	l.Receive(groundA, cmdPacket(command.TypeTakeOff, 2), 0)
	var states int
	for range 4 {
		for _, s := range tick(tel) {
			if s.addr != groundA || command.Decode(s.data, &p) != nil || p.Type != command.TypeState {
				t.Fatalf("sent %+v to a ground station owner", s)
			}
			if p.State.Battery != 3900 || p.State.Ack != 2 || p.State.Flying != command.FlyingTakingOff {
				t.Errorf("state %+v", p.State)
			}
			states++
		}
	}
	if states != 2 {
		t.Errorf("%d State packets in 4 ticks, want 2", states)
	}

	l.Receive(groundA, cmdPacket(command.TypeHello, 3), 0)
	out = tick(tel)
	if len(out) == 0 || command.Decode(out[0].data, &p) != nil || p.Info.Flags&command.InfoOwned == 0 {
		t.Errorf("Info while owned %+v", out)
	}
}

func TestTelemetryJoystick(t *testing.T) {
	l := NewLink(droneMAC)
	tel := &Telemetry{Link: l, FillAtom: func(a *input.AtomTelemetry) { a.Altitude = 1 }}
	l.Receive(joyA, atomPacket(0), 0)
	out := tick(tel)
	if len(out) != 1 || out[0].addr != joyA || len(out[0].data) != input.AtomTelemetrySize {
		t.Fatalf("joystick telemetry %+v", out)
	}
}
