package altitude_test

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/params"
)

// TestStageEdge hovers 0.3 m above a 0.5 m high stage and flies off its edge. The range reads part
// of the step, waits gap readings, then ramps to the floor. The height must stay near where it was.
func TestStageEdge(t *testing.T) {
	cases := []struct {
		first     float32
		gap, ramp int
	}{
		{0.5, 0, 12}, {0.5, 20, 1}, {0.5, 20, 3}, {0.5, 20, 6}, {0.5, 40, 12}, {0.6, 20, 1}, {0.6, 20, 3},
	}
	for _, c := range cases {
		cfg := params.Default().AltEst
		cfg.RangeDelay = 0.2
		f := altitude.New(cfg)
		const dt = 0.0025
		seed := uint32(7)
		noise := func() float32 {
			seed = seed*1664525 + 1013904223
			return float32(seed>>8)/(1<<24) - 0.5
		}
		f.UpdateRange(0.3, 1)
		var worst float32
		n := 0
		for i := range 6000 {
			f.Predict(noise(), dt)
			if i%13 == 0 {
				r := float32(0.3)
				if i >= 1000 {
					k := n - c.gap
					n++
					switch {
					case k < 0:
						r = c.first
					case k < c.ramp:
						r = c.first + (0.8-c.first)*float32(k+1)/float32(c.ramp)
					default:
						r = 0.8
					}
				}
				f.UpdateRange(r+0.01*noise(), 1)
			}
			if i%50 == 0 {
				f.UpdateBaro(100 + 0.2*noise())
			}
			worst = fmath.Max(worst, fmath.Abs(f.Z()))
		}
		if worst > 0.8 || !f.RangeActive() || fmath.Abs(f.Ground()-0.8) > 0.05 {
			t.Errorf("%+v: height off by up to %v m, range active %v, ground %v m", c, worst, f.RangeActive(), f.Ground())
		}
	}
}
