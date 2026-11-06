// Package commander decides when the vehicle may arm, which mode it flies in, and when to give up and land.
// Same role as commander in PX4.
package commander

import (
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Config holds the failsafe and arming limits.
type Config struct {
	InputTimeout   float32 // s without pilot input before landing
	StaleInput     float32 // s without pilot input before the sticks count as centred
	HoldTimeout    float32 // s without pilot input before landing in AltHold and PosHold
	InputRecover   float32 // s of steady input that ends a lost input landing in AltHold and PosHold
	LowBattery     float32 // V
	LowBatteryTime float32 // s the voltage must stay low before landing
	MaxTilt        float32 // rad, beyond this in Stabilize or AltHold the motors stop
	ArmMaxTilt     float32 // rad, must be this level to arm
	ArmMaxThrottle float32
	LandTime       float32 // s, how long a failsafe landing takes before disarming
}

// State is what the commander needs to know each step.
type State struct {
	Now     msg.Time
	Input   *msg.ManualInput
	Tilt    float32 // rad
	Battery *msg.BatteryStatus
	Ready   bool // sensors are calibrated
	// HeightLost is true when the vehicle is flying with no recent height reading.
	HeightLost bool
}

// Commander tracks the arm state, flight mode and failsafes.
type Commander struct {
	cfg       Config
	status    msg.VehicleStatus
	prevArm   bool
	lowSince  msg.Time
	lowActive bool
	landStart msg.Time
	steady    bool // input arrives with no gap longer than StaleInput
	steadyAt  msg.Time
}

// New returns a Commander that starts disarmed.
func New(cfg Config) *Commander { return &Commander{cfg: cfg} }

// Status returns the vehicle status from the last Update.
func (c *Commander) Status() msg.VehicleStatus { return c.status }

// LandingElapsed returns seconds since a failsafe landing began.
func (c *Commander) LandingElapsed(now msg.Time) float32 {
	if !c.status.Landing {
		return 0
	}
	return seconds(now - c.landStart)
}

// Update handles arming, mode changes and failsafes for one step and returns the new status.
func (c *Commander) Update(s State) msg.VehicleStatus {
	in := s.Input
	st := &c.status
	st.Time = s.Now

	pressed := in.Valid && in.Arm && !c.prevArm
	c.prevArm = in.Valid && in.Arm
	c.trackSteady(s)

	if !st.Armed {
		// The last failsafe stays visible until the next arm.
		st.Mode = in.Mode
		st.Landing = false
		if pressed && c.canArm(s) {
			st.Armed = true
			st.Failsafe = msg.FailsafeNone
		}
		return *st
	}

	disarm := !in.Arm
	if in.ArmToggle {
		disarm = pressed
	}
	if disarm && in.Valid && c.inputFresh(s) {
		c.disarm()
		return *st
	}

	if st.Mode != msg.ModeAcro && s.Tilt > c.cfg.MaxTilt {
		c.disarm()
		st.Failsafe = msg.FailsafeTilt
		return *st
	}

	if st.Landing && st.Failsafe == msg.FailsafeInputLost && holdsHeight(st.Mode) && holdsHeight(in.Mode) &&
		c.steady && seconds(s.Now-c.steadyAt) >= c.cfg.InputRecover {
		st.Landing = false
		st.Failsafe = msg.FailsafeNone
	}

	if !st.Landing {
		timeout := c.cfg.InputTimeout
		if holdsHeight(st.Mode) {
			// Hovering in place is safe, so ride out longer gaps before landing.
			timeout = fmath.Max(timeout, c.cfg.HoldTimeout)
		}
		switch {
		case !c.inputAge(s, timeout):
			c.land(s.Now, msg.FailsafeInputLost)
		case c.batteryLow(s):
			c.land(s.Now, msg.FailsafeLowBattery)
		case holdsHeight(st.Mode) && s.HeightLost:
			c.land(s.Now, msg.FailsafeHeightLost)
		default:
			st.Mode = in.Mode
		}
	}

	if st.Landing && c.LandingElapsed(s.Now) > c.cfg.LandTime {
		f := st.Failsafe
		c.disarm()
		st.Failsafe = f
	}
	return *st
}

func (c *Commander) canArm(s State) bool {
	return s.Ready && c.inputFresh(s) && s.Input.Throttle <= c.cfg.ArmMaxThrottle && s.Tilt <= c.cfg.ArmMaxTilt && !c.batteryLow(s)
}

func (c *Commander) inputFresh(s State) bool { return c.inputAge(s, c.cfg.InputTimeout) }

// inputAge reports whether the last input is valid and at most max seconds old.
func (c *Commander) inputAge(s State, max float32) bool {
	return s.Input.Valid && s.Now >= s.Input.Time && seconds(s.Now-s.Input.Time) <= max
}

// Stale reports whether the last input is older than StaleInput, so the sticks should count as centred.
func (c *Commander) Stale(s State) bool {
	return c.cfg.StaleInput > 0 && !c.inputAge(s, c.cfg.StaleInput)
}

func (c *Commander) trackSteady(s State) {
	switch {
	case c.Stale(s) || !s.Input.Valid:
		c.steady = false
	case !c.steady:
		c.steady = true
		c.steadyAt = s.Now
	}
}

func holdsHeight(m msg.FlightMode) bool { return m == msg.ModeAltHold || m == msg.ModePosHold }

func (c *Commander) batteryLow(s State) bool {
	if s.Battery == nil || s.Battery.Voltage <= 0 || s.Battery.Voltage >= c.cfg.LowBattery {
		c.lowActive = false
		return false
	}
	if !c.lowActive {
		c.lowActive = true
		c.lowSince = s.Now
	}
	return seconds(s.Now-c.lowSince) >= c.cfg.LowBatteryTime
}

func (c *Commander) land(now msg.Time, f msg.Failsafe) {
	c.status.Landing = true
	c.status.Failsafe = f
	c.landStart = now
}

// Disarm stops the motors, for example when the flight loop sees the vehicle has landed.
// Any failsafe stays visible, so it shows why a failsafe landing ended.
func (c *Commander) Disarm() {
	f := c.status.Failsafe
	c.disarm()
	c.status.Failsafe = f
}

func (c *Commander) disarm() {
	c.status.Armed = false
	c.status.Landing = false
	c.status.Failsafe = msg.FailsafeNone
}

func seconds(t msg.Time) float32 { return float32(t) * 1e-6 }
