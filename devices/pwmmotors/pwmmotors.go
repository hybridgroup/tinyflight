// Package pwmmotors drives brushed motors straight from PWM channels, as on the StampFly.
package pwmmotors

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var _ hal.Motors = (*Motors)(nil)

// PWM is the part of a TinyGo machine PWM peripheral that is needed here.
type PWM interface {
	Set(channel uint8, value uint32)
	Top() uint32
}

type Motors struct {
	pwm      PWM
	channels [msg.MaxMotors]uint8
	count    int
}

// New takes PWM channels in the same order as the mixer geometry.
func New(pwm PWM, channels ...uint8) *Motors {
	m := &Motors{pwm: pwm, count: len(channels)}
	copy(m.channels[:], channels)
	return m
}

func (m *Motors) SetMotors(out *msg.MotorOutputs) error {
	top := float32(m.pwm.Top())
	for i := 0; i < m.count && i < out.Count; i++ {
		v := out.Value[i]
		if v < 0 {
			v = 0
		} else if v > 1 {
			v = 1
		}
		m.pwm.Set(m.channels[i], uint32(v*top))
	}
	return nil
}

// Stop sets every motor to 0.
func (m *Motors) Stop() {
	for i := 0; i < m.count; i++ {
		m.pwm.Set(m.channels[i], 0)
	}
}
