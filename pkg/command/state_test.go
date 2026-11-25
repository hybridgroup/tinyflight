package command

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type vehicle struct {
	status msg.VehicleStatus
	pos    msg.LocalPosition
	att    msg.Attitude
	batt   msg.BatteryStatus
}

func (v *vehicle) Status() msg.VehicleStatus   { return v.status }
func (v *vehicle) Position() msg.LocalPosition { return v.pos }
func (v *vehicle) Attitude() msg.Attitude      { return v.att }
func (v *vehicle) Battery() msg.BatteryStatus  { return v.batt }

func TestFillState(t *testing.T) {
	v := &vehicle{
		status: msg.VehicleStatus{Armed: true, Mode: msg.ModePosHold, Failsafe: msg.FailsafeLowBattery},
		pos:    msg.LocalPosition{Z: -0.5, ZValid: true},
		att:    msg.Attitude{Q: fmath.QuatFromEuler(0, 0, fmath.Pi/2)},
		batt:   msg.BatteryStatus{Voltage: 3.9},
	}
	s := State{Flying: FlyingHovering, Ack: 7}
	FillState(v, &s)
	want := State{Flying: FlyingHovering, Ack: 7, Failsafe: uint8(msg.FailsafeLowBattery), Mode: uint8(msg.ModePosHold),
		Flags: StateArmed | StateHeightValid, Battery: 3900, Height: 50, Yaw: 9000}
	if d := s.Yaw - want.Yaw; d < -1 || d > 1 {
		t.Errorf("yaw %d, want %d", s.Yaw, want.Yaw)
	}
	s.Yaw = want.Yaw
	if d := int(s.Battery) - int(want.Battery); d < -1 || d > 1 {
		t.Errorf("battery %d, want %d", s.Battery, want.Battery)
	}
	s.Battery = want.Battery
	if s != want {
		t.Errorf("got %+v, want %+v", s, want)
	}

	v.pos.Z = -1000
	FillState(v, &s)
	if s.Height != 32767 {
		t.Errorf("height %d, want clamped to 32767", s.Height)
	}
}
