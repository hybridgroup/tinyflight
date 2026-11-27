package input

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type fakeVehicle struct {
	status msg.VehicleStatus
	pos    msg.LocalPosition
}

func (v *fakeVehicle) Status() msg.VehicleStatus   { return v.status }
func (v *fakeVehicle) Position() msg.LocalPosition { return v.pos }

func send(c *Commands, typ command.Type, seq uint16) {
	c.Handle(&command.Packet{Type: typ, Seq: seq})
}

func read(t *testing.T, c *Commands) msg.ManualInput {
	t.Helper()
	var in msg.ManualInput
	if !c.ReadInput(&in) {
		t.Fatal("no input after a packet")
	}
	return in
}

func TestCommandsTakeOffAndLand(t *testing.T) {
	v := &fakeVehicle{}
	c := &Commands{Vehicle: v}
	var in msg.ManualInput
	if c.ReadInput(&in) {
		t.Fatal("input before any packet")
	}
	send(c, command.TypePcmd, 1)
	in = read(t, c)
	if in.Arm || in.Mode != msg.ModePosHold || !in.Valid || in.ArmToggle {
		t.Fatalf("landed input %+v", in)
	}
	if c.ReadInput(&in) {
		t.Fatal("input again with no new packet")
	}

	c.Handle(&command.Packet{Type: command.TypeTakeOff, Seq: 2, Height: 100})
	if ack, ok := c.Ack(); !ok || ack != 2 {
		t.Errorf("ack %d %v", ack, ok)
	}
	in = read(t, c)
	if !in.Arm || in.Climb != 0 || c.Flying() != command.FlyingTakingOff {
		t.Fatalf("climbed before arming %+v", in)
	}
	v.status.Armed = true
	send(c, command.TypePcmd, 3)
	if in = read(t, c); in.Climb <= 0 {
		t.Fatalf("not climbing %+v", in)
	}
	v.pos = msg.LocalPosition{Z: -0.5, ZValid: true}
	send(c, command.TypePcmd, 4)
	if read(t, c); c.Flying() != command.FlyingTakingOff {
		t.Fatalf("stopped at 0.5 m on the way to 1 m")
	}
	v.pos.Z = -0.98
	send(c, command.TypePcmd, 5)
	if in = read(t, c); c.Flying() != command.FlyingHovering || in.Climb != 0 || !in.Arm {
		t.Fatalf("not hovering, state %d input %+v", c.Flying(), in)
	}

	c.Handle(&command.Packet{Type: command.TypePcmd, Seq: 6, Pcmd: command.Pcmd{Right: 50, Forward: 20, Yaw: -10, Up: 30}})
	in = read(t, c)
	if c.Flying() != command.FlyingFlying || !near(fmath.Deadband(in.Roll, msg.StickCentre), 0.5) || !near(fmath.Deadband(in.Pitch, msg.StickCentre), -0.2) || in.Yaw != -0.1 || in.Climb != 0.3 {
		t.Fatalf("pcmd gave state %d input %+v", c.Flying(), in)
	}
	send(c, command.TypePcmd, 7)
	if in = read(t, c); c.Flying() != command.FlyingHovering || in.Roll != 0 {
		t.Fatalf("zero pcmd gave state %d input %+v", c.Flying(), in)
	}

	send(c, command.TypeLand, 8)
	if in = read(t, c); c.Flying() != command.FlyingLanding || in.Climb >= 0 || !in.Arm {
		t.Fatalf("land gave state %d input %+v", c.Flying(), in)
	}
	v.status.Armed = false
	send(c, command.TypePcmd, 9)
	if in = read(t, c); c.Flying() != command.FlyingLanded || in.Arm {
		t.Fatalf("after disarm state %d input %+v", c.Flying(), in)
	}
}

func TestCommandsArmFails(t *testing.T) {
	c := &Commands{Vehicle: &fakeVehicle{}}
	send(c, command.TypeTakeOff, 1)
	for i := range armReads + 1 {
		read(t, c)
		send(c, command.TypePcmd, uint16(i+2))
	}
	if in := read(t, c); c.Flying() != command.FlyingLanded || in.Arm {
		t.Fatalf("state %d input %+v, want landed", c.Flying(), in)
	}
}

func TestCommandsEmergency(t *testing.T) {
	v := &fakeVehicle{status: msg.VehicleStatus{Armed: true}}
	c := &Commands{Vehicle: v, state: command.FlyingFlying}
	send(c, command.TypeEmergency, 1)
	if in := read(t, c); in.Arm || c.Flying() != command.FlyingEmergency {
		t.Fatalf("state %d input %+v", c.Flying(), in)
	}
	v.status.Armed = false
	send(c, command.TypePcmd, 2)
	if read(t, c); c.Flying() != command.FlyingLanded {
		t.Fatalf("state %d after disarm", c.Flying())
	}
}

func TestCommandsRetryIgnored(t *testing.T) {
	v := &fakeVehicle{}
	c := &Commands{Vehicle: v}
	send(c, command.TypeTakeOff, 1)
	read(t, c)
	v.status.Armed = true
	send(c, command.TypeLand, 2)
	read(t, c)
	v.status.Armed = false
	send(c, command.TypePcmd, 3)
	read(t, c)
	// A late TakeOff retry must not start a new flight.
	send(c, command.TypeTakeOff, 1)
	if read(t, c); c.Flying() != command.FlyingLanded {
		t.Fatalf("state %d", c.Flying())
	}
	send(c, command.TypeTakeOff, 4)
	if read(t, c); c.Flying() != command.FlyingTakingOff {
		t.Fatalf("new TakeOff gave state %d", c.Flying())
	}
}

func TestCommandsPcmdIgnoredWhileTakingOff(t *testing.T) {
	v := &fakeVehicle{status: msg.VehicleStatus{Armed: true}}
	c := &Commands{Vehicle: v}
	send(c, command.TypeTakeOff, 1)
	read(t, c)
	c.Handle(&command.Packet{Type: command.TypePcmd, Seq: 2, Pcmd: command.Pcmd{Right: 100}})
	if in := read(t, c); in.Roll != 0 || c.Flying() != command.FlyingTakingOff {
		t.Fatalf("state %d input %+v", c.Flying(), in)
	}
}

func TestCommandsFailsafeLanding(t *testing.T) {
	v := &fakeVehicle{status: msg.VehicleStatus{Armed: true, Landing: true}}
	c := &Commands{Vehicle: v, state: command.FlyingHovering}
	send(c, command.TypePcmd, 1)
	if read(t, c); c.Flying() != command.FlyingLanding {
		t.Fatalf("state %d during a failsafe landing", c.Flying())
	}
}

// near reports whether a speed request is within 0.1% of want.
func near(got, want float32) bool { return fmath.Abs(got-want) < 0.001 }
