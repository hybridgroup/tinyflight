// Package commander decides when the vehicle may arm, which mode it flies in, and when to give up and land.
// Same role as commander in PX4.
package commander

import (
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Config holds the failsafe and arming limits.
type Config struct {
	InputTimeout   float32 // s without pilot input before landing
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
}

// Commander tracks the arm state, flight mode and failsafes.
type Commander struct {
	cfg       Config
	status    msg.VehicleStatus
	prevArm   bool
	lowSince  msg.Time
	lowActive bool
	landStart msg.Time
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

	armRequest := in.Valid && in.Arm && !c.prevArm
	c.prevArm = in.Valid && in.Arm

	if !st.Armed {
		// The last failsafe stays visible until the next arm.
		st.Mode = in.Mode
		st.Landing = false
		if armRequest && c.canArm(s) {
			st.Armed = true
			st.Failsafe = msg.FailsafeNone
		}
		return *st
	}

	if !in.Arm && in.Valid && c.inputFresh(s) {
		c.disarm()
		return *st
	}

	if st.Mode != msg.ModeAcro && s.Tilt > c.cfg.MaxTilt {
		c.disarm()
		st.Failsafe = msg.FailsafeTilt
		return *st
	}

	if !st.Landing {
		switch {
		case !c.inputFresh(s):
			c.land(s.Now, msg.FailsafeInputLost)
		case c.batteryLow(s):
			c.land(s.Now, msg.FailsafeLowBattery)
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

func (c *Commander) inputFresh(s State) bool {
	return s.Input.Valid && s.Now >= s.Input.Time && seconds(s.Now-s.Input.Time) <= c.cfg.InputTimeout
}

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
func (c *Commander) Disarm() { c.disarm() }

func (c *Commander) disarm() {
	c.status.Armed = false
	c.status.Landing = false
	c.status.Failsafe = msg.FailsafeNone
}

func seconds(t msg.Time) float32 { return float32(t) * 1e-6 }
