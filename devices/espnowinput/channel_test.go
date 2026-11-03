package espnowinput

import "testing"

func TestParseChannel(t *testing.T) {
	for s, want := range map[string]uint8{"": 3, "11": 11, "1": 1, "13": 13, "0": 3, "14": 3, "x": 3} {
		if got := parseChannel(s); got != want {
			t.Errorf("%q gave channel %d, want %d", s, got, want)
		}
	}
	if Channel() != DefaultChannel {
		t.Errorf("channel %d with no setting", Channel())
	}
}

func TestQuietest(t *testing.T) {
	// Access points on 1, 6 and 11, the usual plan. Channel 13 only overlaps the quietest one.
	counts := [14]uint32{1: 900, 6: 1200, 11: 300}
	if c := Quietest(&counts); c != 13 {
		t.Errorf("picked channel %d, want 13", c)
	}
	counts = [14]uint32{1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500, 7: 500, 8: 500, 9: 500, 10: 500, 11: 500, 12: 10, 13: 10}
	if c := Quietest(&counts); c != 13 {
		t.Errorf("picked channel %d, want 13", c)
	}
}
