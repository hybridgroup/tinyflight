package command

import "testing"

func TestRoundTrip(t *testing.T) {
	for _, p := range []Packet{
		{Type: TypeHello, Seq: 1},
		{Type: TypeTakeOff, Seq: 2, Height: 80},
		{Type: TypeLand, Seq: 3},
		{Type: TypeEmergency, Seq: 4},
		{Type: TypePcmd, Seq: 0xfffe, Pcmd: Pcmd{Right: -100, Forward: 100, Yaw: -1, Up: 30}},
		{Type: TypeInfo, Seq: 5, Info: Info{Protocol: 1, Firmware: [3]uint8{0, 4, 0}, MAC: [6]byte{1, 2, 3, 4, 5, 6}, Flags: InfoPosHold}},
		{Type: TypeState, Seq: 6, State: State{Flying: FlyingHovering, Failsafe: 2, Flags: StateArmed | StateHeightValid,
			Mode: 3, Battery: 3900, Height: -12, Roll: -1800, Pitch: 250, Yaw: -17999, Ack: 4}},
	} {
		var buf [MaxSize]byte
		n := p.Encode(&buf)
		if n != Size(p.Type) {
			t.Fatalf("type %x: length %d, want %d", p.Type, n, Size(p.Type))
		}
		if n == 25 {
			t.Errorf("type %x is as long as an Atom control packet", p.Type)
		}
		var got Packet
		if err := Decode(buf[:n], &got); err != nil {
			t.Fatalf("type %x: %v", p.Type, err)
		}
		if got != p {
			t.Errorf("got %+v, want %+v", got, p)
		}
	}
}

func TestBadPackets(t *testing.T) {
	var buf [MaxSize]byte
	p := Packet{Type: TypeTakeOff, Seq: 9, Height: 50}
	n := p.Encode(&buf)
	var got Packet

	bad := buf
	bad[n-1]++
	if err := Decode(bad[:n], &got); err != ErrChecksum {
		t.Errorf("bad checksum gave %v", err)
	}
	if err := Decode(buf[:n-1], &got); err != ErrNotCommand {
		t.Errorf("short packet gave %v", err)
	}
	bad = buf
	bad[2] = 2
	if err := Decode(bad[:n], &got); err != ErrNotCommand {
		t.Errorf("version 2 gave %v", err)
	}
	bad = buf
	bad[3] = 0x7f
	if err := Decode(bad[:n], &got); err != ErrNotCommand {
		t.Errorf("unknown type gave %v", err)
	}
	if n := (&Packet{Type: 0x7f}).Encode(&buf); n != 0 {
		t.Errorf("unknown type encoded %d bytes", n)
	}
}

func TestAtomPacketNeverMatches(t *testing.T) {
	// A drone whose MAC ends 54 46 01 gets Atom packets that start like a command packet.
	p := make([]byte, 25)
	p[0], p[1], p[2] = 'T', 'F', Version
	for typ := range 256 {
		p[3] = byte(typ)
		if Match(p) {
			t.Errorf("Atom packet with type byte %x matched", typ)
		}
	}
}
