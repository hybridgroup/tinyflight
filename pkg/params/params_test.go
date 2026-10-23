package params

import "testing"

func TestDefaultLoopRate(t *testing.T) {
	c := Default()
	if c.LoopHz != 400 || c.Sensors.SampleHz != c.LoopHz || c.Rate.Roll.SampleHz != c.LoopHz {
		t.Errorf("loop %v, sensors %v, rate %v Hz", c.LoopHz, c.Sensors.SampleHz, c.Rate.Roll.SampleHz)
	}
	if len(c.Mixer.Geometry) != 4 {
		t.Errorf("%d motors", len(c.Mixer.Geometry))
	}
}
