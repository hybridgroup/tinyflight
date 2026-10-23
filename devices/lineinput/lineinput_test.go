package lineinput

import (
	"errors"
	"strings"
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type port struct {
	data []byte
	fail bool
}

func (p *port) Buffered() int { return len(p.data) }

func (p *port) ReadByte() (byte, error) {
	if p.fail {
		return 0, errors.New("read failed")
	}
	b := p.data[0]
	p.data = p.data[1:]
	return b, nil
}

func TestReadsLines(t *testing.T) {
	p := &port{data: []byte("rc 0.5 0 0 0.4 1 2\r\n")}
	in := New(p)
	var out msg.ManualInput
	if !in.ReadInput(&out) {
		t.Fatal("no input from a full line")
	}
	if out.Roll != 0.5 || !out.Arm || out.Mode != msg.ModeAltHold {
		t.Errorf("parsed %+v", out)
	}
	p.data = []byte("rc 0 0")
	if in.ReadInput(&out) {
		t.Error("reported input from a partial line")
	}
	p.data = []byte(" 0 0.4 1 1\n")
	if !in.ReadInput(&out) || out.Mode != msg.ModeStabilize {
		t.Errorf("did not finish the split line, %+v", out)
	}
}

func TestDropsBadLines(t *testing.T) {
	long := "rc " + strings.Repeat("0 ", 40) + "\n"
	p := &port{data: []byte(long + "junk\n")}
	var out msg.ManualInput
	if New(p).ReadInput(&out) {
		t.Error("took a line that was too long or bad")
	}
}

func TestReadError(t *testing.T) {
	var out msg.ManualInput
	if New(&port{data: []byte("x"), fail: true}).ReadInput(&out) {
		t.Error("reported input on a read error")
	}
}
