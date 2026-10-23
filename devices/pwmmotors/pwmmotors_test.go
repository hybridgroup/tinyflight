package pwmmotors

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type pwm map[uint8]uint32

func (p pwm) Set(channel uint8, value uint32) { p[channel] = value }
func (p pwm) Top() uint32                     { return 1000 }

func TestSetMotors(t *testing.T) {
	p := pwm{}
	m := New(p, 3, 2, 1, 0)
	out := msg.MotorOutputs{Count: 4, Value: [msg.MaxMotors]float32{0.5, -1, 2, 0.25}}
	m.SetMotors(&out)
	want := pwm{3: 500, 2: 0, 1: 1000, 0: 250}
	for ch, v := range want {
		if p[ch] != v {
			t.Errorf("channel %d = %d, want %d", ch, p[ch], v)
		}
	}
	m.Stop()
	for ch := range want {
		if p[ch] != 0 {
			t.Errorf("channel %d = %d after stop", ch, p[ch])
		}
	}
}
