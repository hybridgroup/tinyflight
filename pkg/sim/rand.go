package sim

// Rand is a small float32 random source, so the simulator needs no float64.
// Xorshift32, see https://en.wikipedia.org/wiki/Xorshift
type Rand struct {
	state uint32
}

func NewRand(seed uint32) *Rand {
	if seed == 0 {
		seed = 1
	}
	return &Rand{state: seed}
}

func (r *Rand) Uint32() uint32 {
	x := r.state
	x ^= x << 13
	x ^= x >> 17
	x ^= x << 5
	r.state = x
	return x
}

// Float32 returns a value in [0, 1).
func (r *Rand) Float32() float32 {
	return float32(r.Uint32()>>8) / (1 << 24)
}

// Normal returns a value with mean 0 and standard deviation 1.
// It sums 12 uniform values, see https://en.wikipedia.org/wiki/Irwin%E2%80%93Hall_distribution
func (r *Rand) Normal() float32 {
	var s float32
	for range 12 {
		s += r.Float32()
	}
	return s - 6
}
