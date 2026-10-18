package filter

import (
	"math"
	"testing"
)

// amplitude runs a sine through f and returns the output amplitude after settling.
func amplitude(apply func(float32) float32, freq, fs float32) float32 {
	var peak float32
	n := int(fs * 4)
	for i := range n {
		y := apply(float32(math.Sin(2 * math.Pi * float64(freq) * float64(i) / float64(fs))))
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
	if v := f.Value(); math.Abs(float64(v-1)) > 1e-3 {
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
	if v := g.Apply(5); math.Abs(float64(v-5)) > 1e-5 {
		t.Errorf("first sample %v, want 5", v)
	}
}
