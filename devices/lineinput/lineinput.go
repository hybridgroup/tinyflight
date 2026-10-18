// Package lineinput reads bench control lines like "rc 0 0 0 0.4 1 1" from a serial port.
package lineinput

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var _ hal.Input = (*Input)(nil)

// Port is the part of machine.Serial that is needed here.
type Port interface {
	Buffered() int
	ReadByte() (byte, error)
}

type Input struct {
	port Port
	buf  [64]byte
	n    int
}

func New(port Port) *Input { return &Input{port: port} }

// ReadInput takes whatever bytes are waiting and parses each full line, without blocking.
func (in *Input) ReadInput(out *msg.ManualInput) bool {
	got := false
	for in.port.Buffered() > 0 {
		b, err := in.port.ReadByte()
		if err != nil {
			break
		}
		if b == '\n' || b == '\r' {
			if in.n > 0 && input.ParseLine(in.buf[:in.n], 0, out) == nil {
				got = true
			}
			in.n = 0
			continue
		}
		if in.n < len(in.buf) {
			in.buf[in.n] = b
			in.n++
		} else {
			// Line too long, drop it.
			in.n = 0
		}
	}
	return got
}
