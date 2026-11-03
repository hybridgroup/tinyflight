package sim

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	physics "github.com/hybridgroup/tinyflight/pkg/sim"
)

type linkLog struct {
	failsafe  msg.Failsafe
	disarmed  bool
	drift     float32 // m, furthest from the hold point
	heightErr float32 // m
	tilt      float32 // rad
}

// hoverThrough takes off in PosHold or AltHold over a flat floor, then hovers for d seconds
// with packets passed by pass, holding the right stick at pitch.
func hoverThrough(t *testing.T, posHold bool, pitch float32, d float32, pass func() bool) (*rig, linkLog) {
	r := newRig(t)
	r.s.autoAlt, r.s.rate = true, posHold
	r.arm()
	r.s.throttle = 1
	r.fly(2, nil)
	r.s.throttle = 0
	r.fly(3, nil)
	q := r.w.Quad
	hold := q.Pos
	r.s.pitch = pitch
	r.pass = pass
	var l linkLog
	r.fly(d, func() {
		s := r.f.Status()
		if s.Failsafe != msg.FailsafeNone {
			l.failsafe = s.Failsafe
		}
		if !s.Armed {
			l.disarmed = true
		}
		dp := q.Pos.Sub(hold)
		l.heightErr = fmath.Max(l.heightErr, fmath.Abs(dp.Z))
		dp.Z = 0
		l.drift = fmath.Max(l.drift, dp.Norm())
		l.tilt = fmath.Max(l.tilt, q.Q.Tilt())
	})
	t.Logf("failsafe %v, disarmed %v, drift %.2f m, height error %.2f m, tilt %.1f°", l.failsafe, l.disarmed, l.drift, l.heightErr, fmath.Degrees(l.tilt))
	return r, l
}

// lossy drops packets with a Link, stepping it once per joystick packet.
func lossy(l *physics.Link) func() bool { return func() bool { return l.Pass(0.01) } }

// gap drops every packet between from and to seconds after the link starts.
func gap(from, to float32) func() bool {
	n := 0
	return func() bool {
		s := float32(n) * 0.01
		n++
		return s < from || s >= to
	}
}

// TestConferenceWiFi hovers through heavy random loss with bursts, like a crowded hall.
func TestConferenceWiFi(t *testing.T) {
	for _, c := range []struct {
		name string
		link func() *physics.Link
	}{
		{"30% loss", func() *physics.Link { return physics.NewLink(0.3, 0, 0, 0, 1) }},
		{"80% loss", func() *physics.Link { return physics.NewLink(0.8, 0, 0, 0, 2) }},
		{"short bursts", func() *physics.Link { return physics.NewLink(0.2, 0.98, 2, 0.3, 3) }},
		{"long bursts", func() *physics.Link { return physics.NewLink(0.3, 1, 4, 0.8, 4) }},
	} {
		for _, posHold := range []bool{false, true} {
			name := c.name + "/althold"
			if posHold {
				name = c.name + "/poshold"
			}
			t.Run(name, func(t *testing.T) {
				_, l := hoverThrough(t, posHold, 0, 20, lossy(c.link()))
				if l.failsafe != msg.FailsafeNone || l.disarmed {
					t.Errorf("failsafe %v, disarmed %v", l.failsafe, l.disarmed)
				}
				if l.heightErr > 0.2 {
					t.Errorf("height changed %.2f m", l.heightErr)
				}
				if posHold && l.drift > 0.3 {
					t.Errorf("drifted %.2f m", l.drift)
				}
			})
		}
	}
}

// TestSignalGapStickHeld loses every packet for 1.5 s with the stick held forward. The drone must stop
// and hold. Before it kept flying on the old stick for 0.5 s, then landed.
func TestSignalGapStickHeld(t *testing.T) {
	r, _ := hoverThrough(t, true, -1, 1, nil)
	q := r.w.Quad
	start := q.Pos
	r.pass = gap(0, 1.5)
	r.fly(1.5, nil)
	d := q.Pos.Sub(start)
	d.Z = 0
	speed := fmath.Vec3{X: q.Vel.X, Y: q.Vel.Y}.Norm()
	t.Logf("moved %.2f m, now at %.2f m/s", d.Norm(), speed)
	if d.Norm() > 0.5 || speed > 0.1 || r.f.Status().Failsafe != msg.FailsafeNone {
		t.Errorf("moved %.2f m, at %.2f m/s, failsafe %v", d.Norm(), speed, r.f.Status().Failsafe)
	}
}

// TestSignalBackDuringLanding loses the link long enough to start the failsafe landing, then gets it back.
func TestSignalBackDuringLanding(t *testing.T) {
	r, l := hoverThrough(t, true, 0, 6, gap(0, 3))
	if l.failsafe != msg.FailsafeInputLost {
		t.Fatalf("no failsafe after 3 s without packets, got %v", l.failsafe)
	}
	s := r.f.Status()
	if !s.Armed || s.Landing || s.Failsafe != msg.FailsafeNone {
		t.Errorf("still landing after the link came back, status %+v", s)
	}
	if h := r.w.Quad.Clearance(); h < 0.3 {
		t.Errorf("only %.2f m up after the link came back", h)
	}
}

// TestSignalGoneLands loses the link for good. The drone must land and disarm.
func TestSignalGoneLands(t *testing.T) {
	r, _ := hoverThrough(t, true, 0, 12, func() bool { return false })
	if s := r.f.Status(); s.Armed || s.Failsafe != msg.FailsafeInputLost || !r.w.Quad.OnGround() {
		t.Errorf("status %+v, %.2f m up", s, r.w.Quad.Clearance())
	}
}
