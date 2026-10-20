package altitude

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/sim"
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
func truth(t float32) (h, a float32) {
	w := 2 * fmath.Pi * 0.2
	c := fmath.Cos(fmath.WrapPi(w * t))
	h = 0.5 - 0.5*c
	a = 0.5 * w * w * c
	return
}

func run(t *testing.T, useRange bool) (rms, bias float32) {
	r := sim.NewRand(1)
	f := New(testConfig())
	const dt = 0.0025
	const accelBias = 0.2
	var sum float32
	n := 0
	for i := range 8000 {
		tm := float32(i) * dt
		h, a := truth(tm)
		f.Predict(-a+accelBias+r.Normal()*0.3, dt)
		if i%8 == 0 {
			f.UpdateBaro(120 + h + r.Normal()*0.3)
		}
		if useRange && i%13 == 0 {
			f.UpdateRange(h+0.01+r.Normal()*0.01, 1)
		}
		if tm > 10 {
			e := f.Z() + h
			sum += e * e
			n++
		}
	}
	rms, bias = fmath.Sqrt(sum/float32(n)), f.Bias()
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

func TestBaroHeight(t *testing.T) {
	if h := BaroHeight(101325, 101325, 15); h != 0 {
		t.Errorf("same pressure gave %v", h)
	}
	// About 0.12 hPa per m near sea level at 15 °C.
	if h := BaroHeight(101325-12.0, 101325, 15); fmath.Abs(h-1.0) > 0.01 {
		t.Errorf("12 Pa drop gave %v m, want about 1", h)
	}
	// Standard atmosphere, 89876 Pa is 1000 m above 101325 Pa with a mean temperature near 11.5 °C.
	if h := BaroHeight(89876, 101325, 11.5); fmath.Abs(h-1000) > 5 {
		t.Errorf("89876 Pa gave %v, want about 1000 m", h)
	}
}

// TestRecoversFromKnock pushes the estimate far from the range reading, as a hard knock did in flight.
// Range readings then fail the gate, and the filter must not lock them out for good.
func TestRecoversFromKnock(t *testing.T) {
	f := New(testConfig())
	r := sim.NewRand(1)
	const dt, h = 0.0025, 0.5
	for i := range 4000 {
		a := r.Normal() * 0.3
		if i >= 800 && i < 920 {
			a += 15
		}
		f.Predict(a, dt)
		if i%8 == 0 {
			f.UpdateBaro(120 + h + r.Normal()*0.3)
		}
		if i%13 == 0 {
			f.UpdateRange(h+0.01+r.Normal()*0.01, 1)
		}
	}
	t.Logf("height %.3f m, ground %.3f m, speed %.3f m/s, range active %v", -f.Z(), f.Ground(), f.VZ(), f.RangeActive())
	if !f.RangeActive() || fmath.Abs(f.Ground()-h) > 0.05 || fmath.Abs(f.VZ()) > 0.1 {
		t.Error("did not recover from the knock")
	}
}
