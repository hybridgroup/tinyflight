package input

import (
	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Vehicle is what Commands needs to know about the drone, such as a *flight.Flight.
type Vehicle interface {
	Status() msg.VehicleStatus
	Position() msg.LocalPosition
}

const (
	// DefaultTakeOffHeight is the hover height in m when TakeOff gives none.
	DefaultTakeOffHeight = 0.5
	// takeOffSlow is the distance in m below the target height where the climb starts to slow, so it does not overshoot.
	takeOffSlow = 0.3
	// takeOffMin is the slowest climb request while taking off.
	takeOffMin = 0.2
	// takeOffMargin is how far below the target height in m counts as there.
	takeOffMargin = 0.03
	// armReads is how many reads may pass after TakeOff before giving up on arming.
	armReads = 10
)

// Commands turns command protocol messages into pilot input in PosHold. See docs/command-protocol.md.
// It is not safe for concurrent use, so a radio driver must guard it.
type Commands struct {
	Vehicle Vehicle
	state   uint8
	target  float32
	climb   float32
	pcmd    command.Pcmd
	ack     uint16
	acked   bool
	armed   bool
	waited  int
	fresh   bool
	status  msg.VehicleStatus
}

// Handle acts on one packet from the ground station.
func (c *Commands) Handle(p *command.Packet) {
	switch p.Type {
	case command.TypePcmd:
		c.pcmd = p.Pcmd
	case command.TypeTakeOff, command.TypeLand, command.TypeEmergency:
		if c.acked && int16(p.Seq-c.ack) <= 0 {
			// A retry of a command already acted on, or an older one.
			c.fresh = true
			return
		}
		c.ack, c.acked = p.Seq, true
		c.command(p)
	default:
		return
	}
	c.fresh = true
}

func (c *Commands) command(p *command.Packet) {
	switch p.Type {
	case command.TypeTakeOff:
		if c.state != command.FlyingLanded {
			return
		}
		c.state = command.FlyingTakingOff
		c.target = DefaultTakeOffHeight
		if p.Height > 0 {
			c.target = float32(p.Height) / 100
		}
		c.armed, c.waited, c.climb = false, 0, 0
	case command.TypeLand:
		switch c.state {
		case command.FlyingTakingOff, command.FlyingHovering, command.FlyingFlying:
			c.state = command.FlyingLanding
		}
	case command.TypeEmergency:
		c.state = command.FlyingEmergency
	}
}

// Ack returns the sequence of the last TakeOff, Land or Emergency acted on, and false if there was none.
func (c *Commands) Ack() (uint16, bool) { return c.ack, c.acked }

// Flying returns the flying state, as of the last ReadInput.
func (c *Commands) Flying() uint8 {
	if c.status.Armed && c.status.Landing {
		return command.FlyingLanding
	}
	return c.state
}

// ReadInput fills in the input for the current state when a packet has arrived since the last read.
// Only reading when packets arrive keeps the lost input failsafes working.
func (c *Commands) ReadInput(out *msg.ManualInput) bool {
	if !c.fresh {
		return false
	}
	c.fresh = false
	if c.Vehicle != nil {
		c.status = c.Vehicle.Status()
		c.step(c.Vehicle.Position())
	}

	*out = msg.ManualInput{Mode: msg.ModePosHold, Valid: true}
	switch c.state {
	case command.FlyingTakingOff:
		out.Arm = true
		out.Climb = c.climb
	case command.FlyingHovering:
		out.Arm = true
	case command.FlyingFlying:
		out.Arm = true
		out.Roll = speedStick(c.pcmd.Right)
		out.Pitch = -speedStick(c.pcmd.Forward)
		out.Yaw = float32(c.pcmd.Yaw) / 100
		out.Climb = float32(c.pcmd.Up) / 100
	case command.FlyingLanding:
		out.Arm = true
		out.Climb = -1
	}
	return true
}

// speedStick turns a Pcmd percent into a stick past the PosHold centre, so N percent asks for N percent of the top speed.
func speedStick(v int8) float32 {
	s := fmath.Clamp(float32(v)/100, -1, 1) * (1 - msg.StickCentre)
	switch {
	case v > 0:
		return s + msg.StickCentre
	case v < 0:
		return s - msg.StickCentre
	}
	return 0
}

// step moves the state machine on from what the vehicle is doing.
func (c *Commands) step(pos msg.LocalPosition) {
	armed := c.status.Armed
	switch c.state {
	case command.FlyingTakingOff:
		switch {
		case armed:
			c.armed = true
			c.climb = 1
			if !pos.ZValid {
				break
			}
			left := c.target + pos.Z
			if left <= takeOffMargin {
				c.state = command.FlyingHovering
				c.climb = 0
				break
			}
			c.climb = fmath.Clamp(left/takeOffSlow, takeOffMin, 1)
		case c.armed:
			c.state = command.FlyingLanded
		default:
			// The arming checks failed, such as the gyro still calibrating.
			c.waited++
			if c.waited > armReads {
				c.state = command.FlyingLanded
			}
		}
	case command.FlyingHovering, command.FlyingFlying:
		switch {
		case !armed:
			c.state = command.FlyingLanded
		case c.pcmd != command.Pcmd{}:
			c.state = command.FlyingFlying
		default:
			c.state = command.FlyingHovering
		}
	case command.FlyingLanding, command.FlyingEmergency:
		if !armed {
			c.state = command.FlyingLanded
		}
	}
}
