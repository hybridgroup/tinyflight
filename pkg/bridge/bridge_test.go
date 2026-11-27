package bridge

import (
	"bytes"
	"testing"
)

func TestRoundTrip(t *testing.T) {
	addr := [6]byte{0x02, 0x54, 0x46, 0x53, 0x49, 0x4d}
	data := []byte{0x54, 0x46, 0x01, 0x01, 0x00, 0xff}
	line := Append(nil, Send, addr, data)
	if want := "tx 02544653494d 5446010100ff\n"; string(line) != want {
		t.Fatalf("got %q, want %q", line, want)
	}
	var gotAddr [6]byte
	var got [MaxPayload]byte
	n, err := Parse(line, Send, &gotAddr, &got)
	if err != nil || gotAddr != addr || !bytes.Equal(got[:n], data) {
		t.Fatalf("parsed %x %x %v", gotAddr, got[:n], err)
	}
	if _, err := Parse(line, Recv, &gotAddr, &got); err != ErrNotPacket {
		t.Errorf("wrong tag gave %v", err)
	}
	full := Append(nil, Recv, addr, make([]byte, MaxPayload))
	if n, err := Parse(full, Recv, &gotAddr, &got); err != nil || n != MaxPayload {
		t.Errorf("full packet gave %d %v", n, err)
	}
}

func TestBadLines(t *testing.T) {
	var addr [6]byte
	var data [MaxPayload]byte
	for _, c := range []struct {
		line string
		err  error
	}{
		{"", ErrNotPacket},
		{"radio bridge ready on channel 1", ErrNotPacket},
		{"txx 02544653494d 54", ErrNotPacket},
		{"tx 02544653494d", ErrBadLine},
		{"tx 02544653494d ", ErrBadLine},
		{"tx 0254465349 54", ErrBadLine},
		{"tx 02544653494g 54", ErrBadLine},
		{"tx 02544653494d 545", ErrBadLine},
		{"tx 02544653494d 5z", ErrBadLine},
		{"tx 02544653494d " + string(bytes.Repeat([]byte("00"), MaxPayload+1)), ErrBadLine},
	} {
		if _, err := Parse([]byte(c.line), Send, &addr, &data); err != c.err {
			t.Errorf("%q: got %v, want %v", c.line, err, c.err)
		}
	}
	if n, err := Parse([]byte("tx 02544653494D AB\r\n"), Send, &addr, &data); err != nil || n != 1 || data[0] != 0xab || addr[5] != 0x4d {
		t.Errorf("upper case with CRLF gave %d %v", n, err)
	}
}
