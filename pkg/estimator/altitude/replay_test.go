package altitude_test

import (
	"bufio"
	"os"
	"strconv"
	"strings"
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/params"
)

type traceStep struct {
	t, accel, dt, rng, cos, baro float32
	flags                        int
}

// loadTrace reads a blackbox height trace, as printed between saved trace start and saved trace end.
func loadTrace(t *testing.T, name string) []traceStep {
	file, err := os.Open(name)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	var steps []traceStep
	sc := bufio.NewScanner(file)
	for sc.Scan() {
		f := strings.Fields(sc.Text())
		if len(f) != 14 || f[0] != "trace" || f[1] == "us" {
			continue
		}
		var v [13]float32
		for i := range v {
			n, err := strconv.Atoi(f[i+1])
			if err != nil {
				t.Fatalf("bad trace line %q", sc.Text())
			}
			v[i] = float32(n)
		}
		steps = append(steps, traceStep{t: v[0] * 1e-6, accel: v[1] * 1e-3, dt: v[2] * 1e-6,
			rng: v[3] * 1e-3, cos: v[4] * 1e-4, baro: v[5] * 1e-3, flags: int(v[11])})
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	return steps
}

// replay runs the trace through a filter and returns the time the height went stale, or 0,
// and the biggest height change from one reading.
func replay(steps []traceStep, cfg altitude.Config) (stale, jump float32) {
	f := altitude.New(cfg)
	for _, s := range steps {
		before := f.Z()
		if s.flags&4 != 0 {
			f.UpdateBaro(s.baro)
		}
		if s.flags&1 != 0 {
			f.UpdateRange(s.rng, s.cos)
		}
		jump = fmath.Max(jump, fmath.Abs(f.Z()-before))
		f.SetOnGround(s.flags&16 != 0)
		f.Predict(s.accel, s.dt)
		if stale == 0 && !f.Fresh() {
			stale = s.t
		}
	}
	return stale, jump
}

// TestSunlightReplay replays a PosHold flight where sunlight blinded the ToF from about 3.4 s and the
// baro drifted meters. The height ran on accel alone into the ceiling at 5.91 s, then snapped to a far off baro reading.
func TestSunlightReplay(t *testing.T) {
	steps := loadTrace(t, "testdata/sunlight.txt")
	if len(steps) != 2400 {
		t.Fatalf("%d trace steps", len(steps))
	}

	old := params.Default().AltEst
	old.FixTimeout, old.MaxPosSD = 0, 0
	_, oldJump := replay(steps, old)
	t.Logf("without the fix one reading moves the height %.2f m", oldJump)
	if oldJump < 3 {
		t.Errorf("replay no longer shows the jump, %v m", oldJump)
	}

	stale, jump := replay(steps, params.Default().AltEst)
	t.Logf("height stale at %.2f s, biggest move from one reading %.3f m", stale, jump)
	if stale < 3.4 || stale > 4.6 {
		t.Errorf("height went stale at %v s, want soon after the range was lost at 3.38 s", stale)
	}
	if jump > 1 {
		t.Errorf("one reading moved the height %v m", jump)
	}
}
