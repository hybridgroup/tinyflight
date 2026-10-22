package commander

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/msg"
)

func testConfig() Config {
	return Config{
		InputTimeout:   0.5,
		LowBattery:     3.4,
		LowBatteryTime: 1,
		MaxTilt:        1,
		ArmMaxTilt:     0.2,
		ArmMaxThrottle: 0.05,
		LandTime:       3,
	}
}

const second = msg.Time(1000000)

type harness struct {
	c    *Commander
	in   msg.ManualInput
	batt msg.BatteryStatus
	now  msg.Time
	tilt float32
}

func newHarness() *harness {
	h := &harness{c: New(testConfig()), batt: msg.BatteryStatus{Voltage: 4.1}}
	h.in = msg.ManualInput{Valid: true, Mode: msg.ModeStabilize}
	return h
}

// step advances time by dt and sends fresh input unless stale is true.
func (h *harness) step(dt msg.Time, stale bool) msg.VehicleStatus {
	h.now += dt
	if !stale {
		h.in.Time = h.now
	}
	return h.c.Update(State{Now: h.now, Input: &h.in, Tilt: h.tilt, Battery: &h.batt, Ready: true})
}

func TestArmNeedsRisingEdgeAndLowThrottle(t *testing.T) {
	h := newHarness()
	h.in.Arm = true
	h.in.Throttle = 0.5
	if h.step(1000, false).Armed {
		t.Fatal("armed with throttle up")
	}
	h.in.Throttle = 0
	if h.step(1000, false).Armed {
		t.Fatal("armed without a new arm request")
	}
	h.in.Arm = false
	h.step(1000, false)
	h.in.Arm = true
	if !h.step(1000, false).Armed {
		t.Fatal("did not arm")
	}
	h.in.Arm = false
	if h.step(1000, false).Armed {
		t.Fatal("did not disarm")
	}
}

// press sends one arm button press like the Atom Joystick.
func (h *harness) press() msg.VehicleStatus {
	h.in.Arm = true
	s := h.step(1000, false)
	h.in.Arm = false
	h.step(1000, false)
	return s
}

func TestArmButtonFollowsArmState(t *testing.T) {
	h := newHarness()
	h.in.ArmToggle = true
	h.in.Throttle = 0.5
	if h.press().Armed {
		t.Fatal("armed with throttle up")
	}
	h.in.Throttle = 0
	if !h.press().Armed {
		t.Fatal("press after a refused arm did not arm")
	}
	if !h.step(1000, false).Armed {
		t.Fatal("disarmed when the button was let go")
	}
	h.step(second, true)
	h.step(4*second, true)
	if h.c.Status().Armed {
		t.Fatal("still armed after input loss")
	}
	if !h.press().Armed {
		t.Fatal("press after a failsafe did not arm")
	}
	if h.press().Armed {
		t.Fatal("press did not disarm")
	}
}

func arm(h *harness) {
	h.step(1000, false)
	h.in.Arm = true
	if !h.step(1000, false).Armed {
		panic("arm failed")
	}
}

func TestInputLossLandsThenDisarms(t *testing.T) {
	h := newHarness()
	arm(h)
	s := h.step(second, true)
	if !s.Landing || s.Failsafe != msg.FailsafeInputLost {
		t.Fatalf("status %+v", s)
	}
	s = h.step(4*second, true)
	if s.Armed || s.Failsafe != msg.FailsafeInputLost {
		t.Fatalf("status after landing %+v", s)
	}
}

func TestTiltStopsMotors(t *testing.T) {
	h := newHarness()
	arm(h)
	h.tilt = 1.5
	if s := h.step(1000, false); s.Armed || s.Failsafe != msg.FailsafeTilt {
		t.Fatalf("status %+v", s)
	}
}

func TestTiltAllowedInAcro(t *testing.T) {
	h := newHarness()
	h.in.Mode = msg.ModeAcro
	arm(h)
	h.tilt = 2
	if !h.step(1000, false).Armed {
		t.Fatal("acro flip disarmed")
	}
}

func TestLowBattery(t *testing.T) {
	h := newHarness()
	arm(h)
	h.batt.Voltage = 3.3
	if h.step(1000, false).Landing {
		t.Fatal("landed on first low reading")
	}
	if s := h.step(2*second, false); !s.Landing || s.Failsafe != msg.FailsafeLowBattery {
		t.Fatalf("status %+v", s)
	}
}
