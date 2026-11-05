package sim

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// TestPosHoldStopsOnRelease flies against heavy drag so the drone lags the stick, then lets go.
// The setpoint once ran ahead while it lagged, so it kept flying on after the stick was let go.
func TestPosHoldStopsOnRelease(t *testing.T) {
	r := newRig(t)
	r.w.Quad.P.Drag = 4
	r.s.autoAlt, r.s.rate = true, true
	r.arm()
	r.s.throttle = 1
	r.fly(2, nil)
	r.s.throttle = 0
	r.fly(3, nil)
	q := r.w.Quad
	for _, pitch := range []float32{-1, 1} {
		r.s.pitch = pitch
		r.fly(3, nil)
		r.s.pitch = 0
		start := q.Pos
		r.fly(3, nil)
		d := q.Pos.Sub(start)
		d.Z = 0
		if d.Norm() > 0.15 {
			t.Errorf("stick %v: moved %.2f m after letting go", pitch, d.Norm())
		}
		if s := fmath.Abs(q.Vel.X); s > 0.05 {
			t.Errorf("stick %v: still moving at %.2f m/s", pitch, s)
		}
	}
}
