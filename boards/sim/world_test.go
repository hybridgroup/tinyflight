package sim

import (
	"encoding/binary"
	"math"
	"testing"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// stick is what the Atom Joystick sends. Layout from docs/protocol.md.
type stick struct {
	yaw, throttle, roll, pitch float32
	arm                        bool
	rate                       bool
	autoAlt                    bool
}

func (s stick) packet(mac [6]byte) []byte {
	p := make([]byte, input.AtomPacketSize)
	p[0], p[1], p[2] = mac[3], mac[4], mac[5]
	for i, v := range []float32{s.yaw, s.throttle, s.roll, s.pitch} {
		binary.LittleEndian.PutUint32(p[3+4*i:], math.Float32bits(v))
	}
	if s.arm {
		p[19] = 1
	}
	if s.rate {
		p[21] = 1
	}
	p[22] = 5
	if s.autoAlt {
		p[22] = 4
	}
	var sum byte
	for _, b := range p[:24] {
		sum += b
	}
	p[24] = sum
	return p
}

type rig struct {
	t *testing.T
	w *World
	f *flight.Flight
	s stick
}

func newRig(t *testing.T) *rig {
	w := New(stampfly.Config())
	return &rig{t: t, w: w, f: flight.New(w.Hardware, stampfly.Config())}
}

// fly sends the stick 100 times a second, like the joystick, for d seconds.
func (r *rig) fly(d float32, check func()) {
	for i := range int(d * 400) {
		if i%4 == 0 {
			r.w.Joystick.Send(r.s.packet(MAC))
		}
		r.w.Step(r.f)
		if check != nil {
			check()
		}
	}
}

func (r *rig) arm() {
	r.fly(1.5, nil)
	if !r.f.Calibrated() {
		r.t.Fatal("gyro calibration did not finish")
	}
	r.s.arm = true
	r.fly(0.2, nil)
	r.s.arm = false
	r.fly(0.1, nil)
	if !r.f.Status().Armed {
		r.t.Fatalf("did not arm, status %+v", r.f.Status())
	}
}

func TestAltHoldHover(t *testing.T) {
	r := newRig(t)
	r.s.autoAlt = true
	r.arm()
	if r.f.Status().Mode != msg.ModeAltHold {
		t.Fatalf("mode %v", r.f.Status().Mode)
	}
	r.s.throttle = 1
	r.fly(2, nil)
	r.s.throttle = 0
	r.fly(3, nil)
	start := -r.w.Quad.Pos.Z
	if start < 0.5 || start > 1.3 {
		t.Fatalf("height after takeoff %v m", start)
	}
	var maxErr float32
	r.fly(5, func() { maxErr = fmath.Max(maxErr, fmath.Abs(-r.w.Quad.Pos.Z-start)) })
	t.Logf("hover at %.2f m, height error %.3f m", start, maxErr)
	if maxErr > 0.15 {
		t.Errorf("height wandered %v m", maxErr)
	}
	if r.w.LED != (LED{R: 32}) {
		t.Errorf("LED %+v while armed, want red", r.w.LED)
	}
}

func TestBadPacketsIgnored(t *testing.T) {
	r := newRig(t)
	r.fly(1.5, nil)
	other := MAC
	other[5]++
	r.s.arm = true
	r.w.Joystick.Send(r.s.packet(other))
	r.w.Step(r.f)
	bad := r.s.packet(MAC)
	bad[24]++
	r.w.Joystick.Send(bad)
	r.w.Step(r.f)
	r.s.arm = false
	r.fly(0.2, nil)
	if r.f.Status().Armed {
		t.Error("armed from bad packets")
	}
	if r.w.Joystick.Rejected != 2 {
		t.Errorf("rejected %d packets, want 2", r.w.Joystick.Rejected)
	}
}

func TestSignalLost(t *testing.T) {
	r := newRig(t)
	r.s.autoAlt = true
	r.arm()
	r.s.throttle = 1
	r.fly(1, nil)
	for range 400 {
		r.w.Step(r.f)
	}
	if r.f.Status().Failsafe == msg.FailsafeNone {
		t.Errorf("no failsafe after 1 s without packets, status %+v", r.f.Status())
	}
}
