package sim

import (
	"testing"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// shortRange loses the floor beyond max, as the ToF does over a dark floor.
type shortRange struct {
	r   hal.Rangefinder
	max float32
}

func (s shortRange) ReadRange(out *msg.RangeSample) error {
	err := s.r.ReadRange(out)
	if err == nil && out.Distance > s.max {
		out.Valid = false
	}
	return err
}

// TestClimbPastRange climbs in PosHold above where the range reads, lets go and holds on the baro.
// A centred climb stick once read as zero thrust, so with no range the drone counted as landed and disarmed.
func TestClimbPastRange(t *testing.T) {
	w := New(stampfly.Config())
	w.Hardware.Range = shortRange{w.Hardware.Range, 1.2}
	r := &rig{t: t, w: w, f: flight.New(w.Hardware, stampfly.Config())}
	r.s.autoAlt = true
	r.arm()
	r.s.throttle = 1
	r.fly(2, nil)
	r.s.throttle = 0
	r.fly(3, nil)
	r.s.throttle = 1
	r.fly(0.8, nil)
	r.s.throttle = 0
	r.fly(0.5, nil)
	want := -r.f.HeightSetpoint()
	var worst float32
	r.fly(5, func() { worst = fmath.Max(worst, fmath.Abs(-w.Quad.Pos.Z-want)) })
	if !r.f.Status().Armed {
		t.Fatal("disarmed in the air")
	}
	if worst > 0.3 {
		t.Errorf("height strayed %.2f m from %.2f m", worst, want)
	}
}
