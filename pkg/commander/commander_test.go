package commander

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/msg"
)

func testConfig() Config {
	return Config{
		InputTimeout:   0.5,
		StaleInput:     0.2,
		HoldTimeout:    2,
		InputRecover:   0.5,
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

// armIn arms in mode with fresh input.
func armIn(mode msg.FlightMode) *harness {
	h := newHarness()
	h.in.Mode = mode
	h.step(1000, false)
	h.in.Arm = true
	h.step(1000, false)
	return h
}

func TestStale(t *testing.T) {
	h := armIn(msg.ModeAltHold)
	h.step(second/10, true)
	if h.c.Stale(State{Now: h.now, Input: &h.in}) {
		t.Error("stale after 0.1 s")
	}
	h.step(second/5, true)
	if !h.c.Stale(State{Now: h.now, Input: &h.in}) {
		t.Error("not stale after 0.3 s")
	}
}

func TestHoldTimeout(t *testing.T) {
	h := armIn(msg.ModeAltHold)
	if s := h.step(3*second/2, true); s.Landing {
		t.Fatal("landing after 1.5 s without input in AltHold")
	}
	if s := h.step(second, true); !s.Landing || s.Failsafe != msg.FailsafeInputLost {
		t.Fatalf("not landing after 2.5 s without input, status %+v", s)
	}
	h = armIn(msg.ModeStabilize)
	if s := h.step(3*second/5, true); !s.Landing {
		t.Fatal("not landing after 0.6 s without input in Stabilize")
	}
}

func TestInputRecovers(t *testing.T) {
	h := armIn(msg.ModePosHold)
	h.step(3*second, true)
	for range 3 {
		h.step(second/10, false)
	}
	for range 3 {
		h.step(second/10, true)
	}
	for range 4 {
		h.step(second/10, false)
	}
	if s := h.step(second/20, false); !s.Landing {
		t.Fatal("stopped landing after a gap and 0.45 s of input")
	}
	h.step(second/10, false)
	if s := h.step(second/10, false); s.Landing || s.Failsafe != msg.FailsafeNone || !s.Armed {
		t.Fatalf("still landing after steady input, status %+v", s)
	}

	h = armIn(msg.ModeStabilize)
	h.step(second, true)
	for range 10 {
		h.step(second/10, false)
	}
	if s := h.step(second/10, false); !s.Landing {
		t.Error("Stabilize stopped landing when input came back")
	}
}
