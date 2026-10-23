package filter

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
)

// amplitude runs a sine through f and returns the output amplitude after settling.
func amplitude(apply func(float32) float32, freq, fs float32) float32 {
	var peak float32
	n := int(fs * 4)
	for i := range n {
		y := apply(fmath.Sin(fmath.WrapPi(2 * fmath.Pi * freq * float32(i) / fs)))
		if i > n/2 && y > peak {
			peak = y
		}
	}
	return peak
}

func TestLowPass1(t *testing.T) {
	f := NewLowPass1(10, 1000)
	for range 1000 {
		f.Apply(1)
	}
	if v := f.Value(); fmath.Abs(v-1) > 1e-3 {
		t.Errorf("step settled at %v", v)
	}
	g := NewLowPass1(10, 1000)
	if a := amplitude(g.Apply, 200, 1000); a > 0.1 {
		t.Errorf("200 Hz passed with amplitude %v", a)
	}
}

func TestLowPass2(t *testing.T) {
	f := NewLowPass2(50, 1000)
	if a := amplitude(f.Apply, 5, 1000); a < 0.95 {
		t.Errorf("5 Hz amplitude %v", a)
	}
	f = NewLowPass2(50, 1000)
	if a := amplitude(f.Apply, 50, 1000); a < 0.65 || a > 0.75 {
		t.Errorf("cutoff amplitude %v, want about 0.707", a)
	}
	f = NewLowPass2(50, 1000)
	if a := amplitude(f.Apply, 300, 1000); a > 0.05 {
		t.Errorf("300 Hz amplitude %v", a)
	}
}

func TestNotch(t *testing.T) {
	f := NewNotch(100, 2, 1000)
	if a := amplitude(f.Apply, 100, 1000); a > 0.05 {
		t.Errorf("notch center amplitude %v", a)
	}
	f = NewNotch(100, 2, 1000)
	if a := amplitude(f.Apply, 10, 1000); a < 0.95 {
		t.Errorf("notch passband amplitude %v", a)
	}
}

func TestPassThrough(t *testing.T) {
	f := NewLowPass2(0, 1000)
	if f.Apply(3) != 3 {
		t.Error("disabled filter changed value")
	}
	g := NewLowPass2(100, 1000)
	if v := g.Apply(5); fmath.Abs(v-5) > 1e-5 {
		t.Errorf("first sample %v, want 5", v)
	}
}

func TestSinCosAccuracy(t *testing.T) {
	for _, c := range [][3]float32{
		{0.0157, 0.01569936, 0.99987676},
		{0.5, 0.47942554, 0.87758256},
		{1.5707964, 1, 0},
		{3.1415927, 0, -1},
	} {
		sn, cs := sinCos(c[0])
		if fmath.Abs(sn-c[1]) > 1e-6 || fmath.Abs(cs-c[2]) > 1e-6 {
			t.Errorf("sinCos(%v) = %v %v, want %v %v", c[0], sn, cs, c[1], c[2])
		}
	}
}

func TestLowCutoff(t *testing.T) {
	f := NewLowPass2(1, 400)
	if a := amplitude(f.Apply, 0.1, 400); a < 0.95 {
		t.Errorf("0.1 Hz through a 1 Hz low-pass gave %v", a)
	}
	f = NewLowPass2(1, 400)
	if a := amplitude(f.Apply, 20, 400); a > 0.01 {
		t.Errorf("20 Hz through a 1 Hz low-pass gave %v", a)
	}
}

func TestVec3Filters(t *testing.T) {
	lp := NewLowPass1Vec3(10, 1000)
	lp.Apply(fmath.Vec3{X: 1, Y: 2, Z: 3})
	if v := lp.Apply(fmath.Vec3{}); v.X <= 0 || v.X >= 1 || v.Z <= v.Y {
		t.Errorf("low-pass step %+v", v)
	}
	bq := NewBiquadVec3(NewLowPass2(10, 1000))
	bq.Apply(fmath.Vec3{X: 1, Y: 2, Z: 3})
	if v := bq.Apply(fmath.Vec3{}); v.X <= 0 || v.X >= 1 || v.Z <= v.Y {
		t.Errorf("biquad step %+v", v)
	}
}

func TestLowPass1Reset(t *testing.T) {
	f := NewLowPass1(10, 1000)
	f.Apply(5)
	f.Reset()
	if v := f.Apply(-2); v != -2 || f.Value() != -2 {
		t.Errorf("after reset %v, want -2", v)
	}
}

func TestNotchDisabled(t *testing.T) {
	for _, f := range []Biquad{NewNotch(0, 1, 1000), NewNotch(100, 0, 1000), NewNotch(600, 1, 1000)} {
		if f.Apply(3) != 3 {
			t.Error("disabled notch changed value")
		}
	}
}
