package altitude_test

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/params"
	"github.com/hybridgroup/tinyflight/pkg/sim"
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

// TestRampNoRunaway ramps the range from one surface to another over 1 to 16 readings, so some ramp steps
// sit near StepGate. A wrong speed estimate once made step detection move the reference and run away.
func TestRampNoRunaway(t *testing.T) {
	for ramp := 1; ramp <= 16; ramp++ {
		cfg := params.Default().AltEst
		cfg.RangeDelay = 0.2
		f := altitude.New(cfg)
		seed := uint32(7)
		noise := func() float32 {
			seed = seed*1664525 + 1013904223
			return float32(seed>>8)/(1<<24) - 0.5
		}
		f.UpdateRange(0.3, 1)
		var worst float32
		n := 0
		for i := range 6000 {
			f.Predict(noise(), 0.0025)
			if i%13 == 0 {
				r := float32(0.3)
				if i >= 1000 {
					r = 0.3 + 0.5*fmath.Min(float32(n)/float32(ramp), 1)
					n++
				}
				f.UpdateRange(r+0.01*noise(), 1)
			}
			if i%50 == 0 {
				f.UpdateBaro(100 + 0.2*noise())
			}
			worst = fmath.Max(worst, fmath.Abs(f.Z()))
		}
		if worst > 0.8 || fmath.Abs(f.Ground()-0.8) > 0.05 {
			t.Errorf("ramp over %d readings: height off by up to %v m, ground %v m", ramp, worst, f.Ground())
		}
	}
}

// TestRoughSteps hovers still for 30 s over ground that steps up and down at random, like rocks.
// Without StepGate the height was off by 0.18, 0.19, 1.49 and 0.88 m in these cases.
func TestRoughSteps(t *testing.T) {
	cases := []struct{ rocks, noise, limit float32 }{
		{0.15, 0.005, 0.2}, {0.15, 0.02, 0.3}, {0.4, 0.005, 0.15}, {0.4, 0.02, 0.5},
	}
	for _, c := range cases {
		cfg := params.Default().AltEst
		cfg.RangeDelay = 0.2
		f := altitude.New(cfg)
		rnd := sim.NewRand(3)
		f.UpdateRange(0.8, 1)
		var surface, worst float32
		for i := range 12000 {
			f.Predict(0.2*rnd.Normal(), 0.0025)
			if i%13 == 0 {
				if i > 1000 && (i/13)%8 == 0 {
					surface = c.rocks * rnd.Float32()
				}
				f.UpdateRange(0.8-surface+c.noise*rnd.Normal(), 1)
			}
			if i%50 == 0 {
				f.UpdateBaro(100 + 0.5*rnd.Normal())
			}
			worst = fmath.Max(worst, fmath.Abs(f.Z()))
		}
		t.Logf("%+v: height off by up to %.3f m", c, worst)
		if worst > c.limit {
			t.Errorf("%+v: height off by up to %v m", c, worst)
		}
	}
}
