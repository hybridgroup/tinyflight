package sim

// Link drops radio packets in bursts, like a crowded 2.4 GHz band. It switches between a good and a bad
// state, the Gilbert-Elliott model, see https://en.wikipedia.org/wiki/Burst_error
type Link struct {
	Loss      float32 // chance a packet is lost in the good state
	BurstLoss float32 // chance a packet is lost in a burst
	BurstGap  float32 // s, mean time between bursts, 0 for no bursts
	BurstLen  float32 // s, mean burst length
	rnd       *Rand
	burst     bool
}

// NewLink returns a link with its own random source.
func NewLink(loss, burstLoss, burstGap, burstLen float32, seed uint32) *Link {
	return &Link{Loss: loss, BurstLoss: burstLoss, BurstGap: burstGap, BurstLen: burstLen, rnd: NewRand(seed)}
}

// Pass moves time on by dt seconds and reports whether a packet sent now gets through.
func (l *Link) Pass(dt float32) bool {
	switch {
	case l.burst && l.rnd.Float32() < dt/l.BurstLen:
		l.burst = false
	case !l.burst && l.BurstGap > 0 && l.rnd.Float32() < dt/l.BurstGap:
		l.burst = true
	}
	loss := l.Loss
	if l.burst {
		loss = l.BurstLoss
	}
	return l.rnd.Float32() >= loss
}
