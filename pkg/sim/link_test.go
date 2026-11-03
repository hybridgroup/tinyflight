package sim

import "testing"

func TestLink(t *testing.T) {
	count := func(l *Link) (lost, longest int) {
		run := 0
		for range 100000 {
			if l.Pass(0.01) {
				run = 0
				continue
			}
			lost++
			run++
			longest = max(longest, run)
		}
		return lost, longest
	}
	if lost, _ := count(NewLink(0.3, 0, 0, 0, 1)); lost < 29000 || lost > 31000 {
		t.Errorf("lost %d of 100000 at 30%%", lost)
	}
	lost, longest := count(NewLink(0, 1, 2, 0.5, 2))
	// Bursts take about 0.5 s of every 2.5 s, so about a fifth of the packets.
	if lost < 15000 || lost > 25000 || longest < 100 {
		t.Errorf("lost %d of 100000 in bursts, longest run %d", lost, longest)
	}
}
