package axis

import (
	"math"
	"testing"
)

func TestTracksPositionAndBias(t *testing.T) {
	var f Filter
	f.Reset(1, 1, 0.1)
	const dt, bias = 0.01, 0.3
	var pos, vel float32
	for i := range 3000 {
		a := float32(math.Sin(float64(i) * dt))
		vel += a * dt
		pos += vel * dt
		f.Predict(a+bias, dt, 0.1, 0.01)
		if i%10 == 0 {
			f.Update(Pos, pos, 0.01, 0)
		}
	}
	if d := f.X[Pos] - pos; d > 0.02 || d < -0.02 {
		t.Errorf("pos %v, want %v", f.X[Pos], pos)
	}
	if d := f.X[Bias] - bias; d > 0.05 || d < -0.05 {
		t.Errorf("bias %v, want %v", f.X[Bias], bias)
	}
}

func TestGateRejects(t *testing.T) {
	var f Filter
	f.Reset(0.01, 0.01, 0.01)
	if f.Update(Pos, 10, 0.1, 3) {
		t.Error("accepted a reading far outside the gate")
	}
	if f.X[Pos] != 0 {
		t.Errorf("state moved to %v on a rejected reading", f.X[Pos])
	}
	if !f.Update(Vel, 0.1, 0.1, 3) || f.X[Vel] <= 0 {
		t.Errorf("did not take a close reading, vel %v", f.X[Vel])
	}
}

func TestPredictGrowsVariance(t *testing.T) {
	var f Filter
	f.Reset(0, 0, 0)
	f.Predict(0, 0.1, 1, 1)
	for i := range 3 {
		if f.P[i][i] <= 0 {
			t.Errorf("P[%d][%d] = %v after predict", i, i, f.P[i][i])
		}
	}
}
