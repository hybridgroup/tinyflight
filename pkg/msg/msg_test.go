package msg

import "testing"

func TestFlightModeString(t *testing.T) {
	for m, want := range map[FlightMode]string{
		ModeAcro: "acro", ModeStabilize: "stabilize", ModeAltHold: "althold", ModePosHold: "poshold", 99: "unknown",
	} {
		if got := m.String(); got != want {
			t.Errorf("%d: %q, want %q", m, got, want)
		}
	}
}

func TestFailsafeString(t *testing.T) {
	for f, want := range map[Failsafe]string{
		FailsafeNone: "none", FailsafeInputLost: "input lost", FailsafeLowBattery: "low battery", FailsafeTilt: "tilt", 99: "unknown",
	} {
		if got := f.String(); got != want {
			t.Errorf("%d: %q, want %q", f, got, want)
		}
	}
}
