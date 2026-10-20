// Package axis is a small Kalman filter for position, speed and accel bias along one axis.
// Accel drives the prediction, then position or speed readings correct it.
package axis

// Index of each value in the state.
const (
	Pos = iota
	Vel
	Bias
)

// Filter holds the state X and its covariance P.
type Filter struct {
	X [3]float32
	P [3][3]float32
}

// Reset zeroes the state and sets the variance of each value.
func (f *Filter) Reset(posVar, velVar, biasVar float32) {
	f.X = [3]float32{}
	f.P = [3][3]float32{{posVar, 0, 0}, {0, velVar, 0}, {0, 0, biasVar}}
}

// Predict moves the state forward by dt using accel in m/s².
// accelNoise is the accel noise density and biasNoise how fast the bias may wander per second.
func (f *Filter) Predict(accel, dt, accelNoise, biasNoise float32) {
	a := accel - f.X[Bias]
	f.X[Pos] += f.X[Vel]*dt + 0.5*a*dt*dt
	f.X[Vel] += a * dt

	// P = F P Fᵀ + Q with F = [[1, dt, -dt²/2], [0, 1, -dt], [0, 0, 1]]
	h := 0.5 * dt * dt
	fm := [3][3]float32{{1, dt, -h}, {0, 1, -dt}, {0, 0, 1}}
	var fp [3][3]float32
	for i := range 3 {
		for j := range 3 {
			fp[i][j] = fm[i][0]*f.P[0][j] + fm[i][1]*f.P[1][j] + fm[i][2]*f.P[2][j]
		}
	}
	for i := range 3 {
		for j := range 3 {
			f.P[i][j] = fp[i][0]*fm[j][0] + fp[i][1]*fm[j][1] + fp[i][2]*fm[j][2]
		}
	}

	// Accel noise enters through G = [dt²/2, dt, 0], bias noise as a random walk.
	qa := accelNoise * accelNoise
	g := [2]float32{h, dt}
	for i := range 2 {
		for j := range 2 {
			f.P[i][j] += g[i] * g[j] * qa
		}
	}
	f.P[Bias][Bias] += biasNoise * biasNoise * dt
}

// Update corrects with a direct reading z of state i with standard deviation sd.
// It rejects readings more than gate standard deviations away, unless gate is 0.
func (f *Filter) Update(i int, z, sd, gate float32) bool {
	return f.Correct(i, z-f.X[i], sd, gate)
}

// Correct is Update with the difference between reading and state already worked out,
// for example against an older state when the reading arrives late.
func (f *Filter) Correct(i int, innov, sd, gate float32) bool {
	s := f.P[i][i] + sd*sd
	if gate > 0 && innov*innov > gate*gate*s {
		return false
	}
	var k [3]float32
	for j := range 3 {
		k[j] = f.P[j][i] / s
		f.X[j] += k[j] * innov
	}
	row := f.P[i]
	for a := range 3 {
		for b := range 3 {
			f.P[a][b] -= k[a] * row[b]
		}
	}
	return true
}
