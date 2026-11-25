package command

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Vehicle is the flight state a State is made from, such as a *flight.Flight.
type Vehicle interface {
	Status() msg.VehicleStatus
	Position() msg.LocalPosition
	Attitude() msg.Attitude
	Battery() msg.BatteryStatus
}

// FillState fills s from v, apart from Flying and Ack, which come from the command input.
func FillState(v Vehicle, s *State) {
	st := v.Status()
	pos := v.Position()
	s.Failsafe = uint8(st.Failsafe)
	s.Mode = uint8(st.Mode)
	s.Flags = 0
	if st.Armed {
		s.Flags |= StateArmed
	}
	if pos.ZValid {
		s.Flags |= StateHeightValid
	}
	if pos.XYValid {
		s.Flags |= StatePosValid
	}
	s.Battery = uint16(fmath.Clamp(v.Battery().Voltage*1000, 0, 65535))
	s.Height = toInt16(-pos.Z * 100)
	r, p, y := v.Attitude().Q.Euler()
	s.Roll = toInt16(fmath.Degrees(r) * 100)
	s.Pitch = toInt16(fmath.Degrees(p) * 100)
	s.Yaw = toInt16(fmath.Degrees(y) * 100)
}

func toInt16(v float32) int16 { return int16(fmath.Clamp(v, -32768, 32767)) }
