package sim

import (
	"testing"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/input"
)

// groundMAC is the address of the ground station in the tests.
var groundMAC = [6]byte{0x02, 0x47, 0x53, 0, 0, 1}

// cmdRig flies the simulated drone with the command protocol over its radio, like a ground station would.
type cmdRig struct {
	t    *testing.T
	w    *World
	f    *flight.Flight
	seq  uint16
	pcmd command.Pcmd
	// beat is false to stop the heartbeat, as if the link was lost.
	beat bool
}

func newCmdRig(t *testing.T) *cmdRig {
	w := New(stampfly.Config())
	f := flight.New(w.Hardware, stampfly.Config())
	w.Radio.SetVehicle(f)
	return &cmdRig{t: t, w: w, f: f, beat: true}
}

func (r *cmdRig) send(p command.Packet) {
	r.seq++
	p.Seq = r.seq
	var buf [command.MaxSize]byte
	r.w.Receive(groundMAC, buf[:p.Encode(&buf)])
}

// flying is the flying state the drone reports in State.
func (r *cmdRig) flying() uint8 {
	var s command.State
	r.w.Radio.FillState(&s)
	return s.Flying
}

// fly sends the Pcmd heartbeat 20 times a second for d seconds.
func (r *cmdRig) fly(d float32) {
	for i := range int(d * 400) {
		if i%20 == 0 && r.beat {
			r.send(command.Packet{Type: command.TypePcmd, Pcmd: r.pcmd})
		}
		r.w.Step(r.f)
	}
}

// until flies until the flying state is want, for at most d seconds.
func (r *cmdRig) until(want uint8, d float32) {
	r.t.Helper()
	for range int(d * 20) {
		r.fly(0.05)
		if r.flying() == want {
			return
		}
	}
	r.t.Fatalf("flying state %d after %v s, want %d, status %+v", r.flying(), d, want, r.f.Status())
}

func (r *cmdRig) takeOff() {
	r.fly(1.5)
	if !r.f.Calibrated() {
		r.t.Fatal("gyro calibration did not finish")
	}
	r.send(command.Packet{Type: command.TypeTakeOff})
	r.until(command.FlyingHovering, 5)
	r.fly(2)
}

func TestCommandFlight(t *testing.T) {
	r := newCmdRig(t)
	q := r.w.Quad
	r.takeOff()
	if h := -q.Pos.Z; fmath.Abs(h-input.DefaultTakeOffHeight) > 0.05 {
		t.Errorf("hovering at %.2f m, want %.2f", h, input.DefaultTakeOffHeight)
	}

	start := q.Pos
	r.pcmd = command.Pcmd{Right: 50}
	r.fly(2)
	if r.flying() != command.FlyingFlying {
		t.Errorf("flying state %d while moving", r.flying())
	}
	r.pcmd = command.Pcmd{}
	r.fly(3)
	if r.flying() != command.FlyingHovering {
		t.Errorf("flying state %d after stopping", r.flying())
	}
	d := q.Pos.Sub(start)
	if d.Y < 0.3 || fmath.Abs(d.X) > 0.15 {
		t.Errorf("moved %.2f m north and %.2f m east, want about 0.6 m east", d.X, d.Y)
	}
	if s := q.Vel.Norm(); s > 0.05 {
		t.Errorf("still moving at %.2f m/s after Hover", s)
	}
	if h := -q.Pos.Z; fmath.Abs(h-input.DefaultTakeOffHeight) > 0.1 {
		t.Errorf("height %.2f m after moving", h)
	}

	r.pcmd = command.Pcmd{Up: 100}
	r.fly(1)
	r.pcmd = command.Pcmd{}
	r.fly(1)
	if h := -q.Pos.Z; h < input.DefaultTakeOffHeight+0.3 {
		t.Errorf("height %.2f m after climbing", h)
	}

	r.send(command.Packet{Type: command.TypeLand})
	r.until(command.FlyingLanded, 10)
	if r.f.Status().Armed {
		t.Error("still armed after landing")
	}
	if h := -q.Pos.Z; h > 0.05 {
		t.Errorf("landed at %.2f m", h)
	}
}

// TestCommandSpeed checks a Pcmd percent asks for that share of the top speed, with no loss to the stick centre.
func TestCommandSpeed(t *testing.T) {
	r := newCmdRig(t)
	q := r.w.Quad
	r.takeOff()
	r.pcmd = command.Pcmd{Right: 30}
	r.fly(3)
	want := 0.3 * stampfly.Config().Horizontal.MaxSpeed
	if v := q.Vel.Y; fmath.Abs(v-want) > 0.03 {
		t.Errorf("flying right at %.2f m/s, want %.2f", v, want)
	}
}

func TestCommandTakeOffHeight(t *testing.T) {
	r := newCmdRig(t)
	r.fly(1.5)
	r.send(command.Packet{Type: command.TypeTakeOff, Height: 100})
	r.until(command.FlyingHovering, 8)
	r.fly(2)
	if h := -r.w.Quad.Pos.Z; fmath.Abs(h-1) > 0.05 {
		t.Errorf("hovering at %.2f m, want 1", h)
	}
}

func TestCommandEmergency(t *testing.T) {
	r := newCmdRig(t)
	r.takeOff()
	r.send(command.Packet{Type: command.TypeEmergency})
	r.fly(0.1)
	if r.f.Status().Armed {
		t.Fatal("still armed after Emergency")
	}
	r.fly(0.1)
	if r.flying() != command.FlyingLanded {
		t.Errorf("flying state %d after Emergency", r.flying())
	}
}

func TestCommandLinkLost(t *testing.T) {
	r := newCmdRig(t)
	r.takeOff()
	r.beat = false
	r.fly(10)
	if r.f.Status().Armed {
		t.Fatalf("still armed 10 s after the link was lost, status %+v", r.f.Status())
	}
	if h := -r.w.Quad.Pos.Z; h > 0.05 {
		t.Errorf("at %.2f m after the link was lost", h)
	}
}

func TestCommandTakeOffBeforeCalibration(t *testing.T) {
	r := newCmdRig(t)
	r.send(command.Packet{Type: command.TypeTakeOff})
	r.fly(1)
	if r.flying() != command.FlyingLanded || r.f.Status().Armed {
		t.Errorf("flying state %d armed %v, want landed", r.flying(), r.f.Status().Armed)
	}
}
