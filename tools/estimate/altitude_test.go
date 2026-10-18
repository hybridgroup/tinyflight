// Package estimate checks the on-board filters against github.com/milosgajdos/go-estimate.
// It runs only on a PC and has its own go.mod so gonum never enters the firmware.
package estimate

import (
	"math"
	"math/rand"
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	"github.com/milosgajdos/go-estimate/kalman/kf"
	"gonum.org/v1/gonum/mat"
)

// model is the altitude filter's state model in go-estimate form, with x = [z, vz, bias] and u = accel z.
type model struct {
	a, b, c *mat.Dense
}

func newModel(dt float64) *model {
	h := 0.5 * dt * dt
	return &model{
		a: mat.NewDense(3, 3, []float64{1, dt, -h, 0, 1, -dt, 0, 0, 1}),
		b: mat.NewDense(3, 1, []float64{h, dt, 0}),
		c: mat.NewDense(1, 3, []float64{1, 0, 0}),
	}
}

func (m *model) Propagate(x, u, _ mat.Vector) (mat.Vector, error) {
	out := mat.NewVecDense(3, nil)
	out.MulVec(m.a, x)
	bu := mat.NewVecDense(3, nil)
	bu.MulVec(m.b, u)
	out.AddVec(out, bu)
	return out, nil
}

func (m *model) Observe(x, _, _ mat.Vector) (mat.Vector, error) {
	out := mat.NewVecDense(1, nil)
	out.MulVec(m.c, x)
	return out, nil
}

func (m *model) SystemDims() (nx, nu, ny, nz int) { return 3, 1, 1, 0 }
func (m *model) SystemMatrix() mat.Matrix         { return m.a }
func (m *model) ControlMatrix() mat.Matrix        { return m.b }
func (m *model) OutputMatrix() mat.Matrix         { return m.c }
func (m *model) FeedForwardMatrix() mat.Matrix    { return nil }

// covOnly is noise with a covariance but zero samples, since the filter should not add random noise to its state.
type covOnly struct {
	cov *mat.SymDense
}

func (n covOnly) Mean() []float64    { return make([]float64, n.cov.SymmetricDim()) }
func (n covOnly) Cov() mat.Symmetric { return n.cov }
func (n covOnly) Sample() mat.Vector { return mat.NewVecDense(n.cov.SymmetricDim(), nil) }
func (n covOnly) Reset()             {}

// initCond matches altitude.Filter after Reset.
type initCond struct{}

func (initCond) State() mat.Vector { return mat.NewVecDense(3, nil) }
func (initCond) Cov() mat.Symmetric {
	return mat.NewSymDense(3, []float64{1, 0, 0, 0, 1, 0, 0, 0, 0.1})
}

func TestAltitudeMatchesGoEstimate(t *testing.T) {
	const dt = 0.0025
	cfg := altitude.Config{AccelNoise: 0.5, BiasNoise: 0.01, BaroNoise: 0.3}

	h := 0.5 * dt * dt
	qa := float64(cfg.AccelNoise * cfg.AccelNoise)
	q := mat.NewSymDense(3, []float64{
		h * h * qa, h * dt * qa, 0,
		h * dt * qa, dt * dt * qa, 0,
		0, 0, float64(cfg.BiasNoise*cfg.BiasNoise) * dt,
	})
	r := mat.NewSymDense(1, []float64{float64(cfg.BaroNoise * cfg.BaroNoise)})

	ref, err := kf.New(newModel(dt), initCond{}, covOnly{q}, covOnly{r})
	if err != nil {
		t.Fatal(err)
	}
	ours := altitude.New(cfg)

	rnd := rand.New(rand.NewSource(1))
	x := mat.NewVecDense(3, nil)
	var baroRef float64
	var worst [3]float64
	for i := range 4000 {
		tm := float64(i) * dt
		accel := -0.5*math.Cos(tm) + 0.2 + rnd.NormFloat64()*0.3
		alt := 100 + 0.5*math.Cos(tm) + rnd.NormFloat64()*0.3
		if i == 0 {
			baroRef = alt
		}

		ours.Predict(float32(accel), dt)
		ours.UpdateBaro(float32(alt))

		est, err := ref.Predict(x, mat.NewVecDense(1, []float64{accel}))
		if err != nil {
			t.Fatal(err)
		}
		x.CopyVec(est.Val())
		est, err = ref.Update(x, nil, mat.NewVecDense(1, []float64{baroRef - alt}))
		if err != nil {
			t.Fatal(err)
		}
		x.CopyVec(est.Val())

		got := [3]float32{ours.Z(), ours.VZ(), ours.Bias()}
		for k := range 3 {
			worst[k] = math.Max(worst[k], math.Abs(float64(got[k])-x.AtVec(k)))
		}
	}
	t.Logf("largest difference: z %.2e m, vz %.2e m/s, bias %.2e m/s²", worst[0], worst[1], worst[2])
	for k, tol := range [3]float64{1e-3, 1e-3, 1e-3} {
		if worst[k] > tol {
			t.Errorf("state %d differs by %v", k, worst[k])
		}
	}
}
