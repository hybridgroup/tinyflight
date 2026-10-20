package altitude_test

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/params"
)

// TestIgnoresBumps shakes the accel the way the drone bounced on its base at liftoff,
// while the range says it has not moved. The height must follow the range.
func TestIgnoresBumps(t *testing.T) {
	f := altitude.New(params.Default().AltEst)
	f.SetOnGround(true)
	const dt = 0.0025
	var worst float32
	for i := range 2000 {
		var a float32
		if i >= 800 && i < 1040 {
			// Swings of up to about 1 g with a net push, like the third tethered test.
			a = -2.5 + 7*fmath.Sin(fmath.WrapPi(float32(i)*0.7))
		}
		f.Predict(a, dt)
		if i%13 == 0 {
			f.UpdateRange(0.01, 1)
		}
		worst = fmath.Max(worst, fmath.Abs(f.Z()))
	}
	t.Logf("worst height error %.3f m", worst)
	if worst > 0.1 || !f.RangeActive() {
		t.Errorf("height moved %v m from accel bumps, range active %v", worst, f.RangeActive())
	}
}
