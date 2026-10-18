package altitude

import (
	"math"
	"math/rand"
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

func testConfig() Config {
	return Config{
		AccelNoise: 0.5,
		BiasNoise:  0.01,
		BaroNoise:  0.3,
		RangeNoise: 0.02,
		Gate:       5,
		RangeMax:   2,
		RangeHold:  0.2,
		BaroTrack:  0.01,
	}
}

// truth is a climb to 1 m with a slow bob, as height up in m and vertical accel up in m/s².
func truth(t float64) (h, a float64) {
	w := 2 * math.Pi * 0.2
	h = 1 - math.Cos(w*t)*0.5 - 0.5
	a = 0.5 * w * w * math.Cos(w*t)
	return
}

func run(t *testing.T, useRange bool) (rms, bias float32) {
	r := rand.New(rand.NewSource(1))
	f := New(testConfig())
	const dt = 0.0025
	const accelBias = 0.2
	var sum float64
	n := 0
	for i := range 8000 {
		tm := float64(i) * dt
		h, a := truth(tm)
		f.Predict(float32(-a+accelBias+r.NormFloat64()*0.3), dt)
		if i%8 == 0 {
			f.UpdateBaro(float32(120 + h + r.NormFloat64()*0.3))
		}
		if useRange && i%13 == 0 {
			f.UpdateRange(float32(h+0.01+r.NormFloat64()*0.01), 1)
		}
		if tm > 10 {
			e := float64(f.Z()) + h
			sum += e * e
			n++
		}
	}
	rms, bias = float32(math.Sqrt(sum/float64(n))), f.Bias()
	t.Logf("rms %.4f m, bias %.3f m/s²", rms, bias)
	return
}

func TestBaroOnly(t *testing.T) {
	rms, bias := run(t, false)
	if rms > 0.15 {
		t.Errorf("baro only height rms error %v m", rms)
	}
	if fmath.Abs(bias-0.2) > 0.1 {
		t.Errorf("bias %v, want 0.2", bias)
	}
}

func TestWithRange(t *testing.T) {
	rms, bias := run(t, true)
	if rms > 0.03 {
		t.Errorf("baro and range height rms error %v m", rms)
	}
	if fmath.Abs(bias-0.2) > 0.05 {
		t.Errorf("bias %v, want 0.2", bias)
	}
}

func TestGateRejectsOutlier(t *testing.T) {
	f := New(testConfig())
	for range 400 {
		f.Predict(0, 0.0025)
		f.UpdateBaro(100)
	}
	if f.UpdateBaro(150) {
		t.Error("50 m baro jump was accepted")
	}
}

func TestPressureToAltitude(t *testing.T) {
	if a := PressureToAltitude(101325); fmath.Abs(a) > 0.01 {
		t.Errorf("sea level gave %v", a)
	}
	if a := PressureToAltitude(89876); fmath.Abs(a-1000) > 5 {
		t.Errorf("89876 Pa gave %v, want about 1000 m", a)
	}
}
