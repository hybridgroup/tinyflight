package flight

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"github.com/hybridgroup/tinyflight/pkg/params"
	"github.com/hybridgroup/tinyflight/pkg/sim"
)

type world struct {
	t     *testing.T
	quad  *sim.Quad
	pilot *sim.Pilot
	f     *Flight
	now   msg.Time
	cfg   params.Config
}

func newWorld(t *testing.T) *world { return newWorldConfig(t, params.Default()) }

func newWorldConfig(t *testing.T, cfg params.Config) *world {
	quad := sim.New(sim.StampFly())
	pilot := &sim.Pilot{}
	f := New(Hardware{IMU: quad, Motors: quad, Input: pilot, Baro: quad, Range: quad, Power: quad}, cfg)
	return &world{t: t, quad: quad, pilot: pilot, f: f, cfg: cfg}
}

// run steps the world for d seconds, calling check after every step if given.
func (w *world) run(d float32, check func()) {
	const sub = 4
	dt := 1 / w.cfg.LoopHz
	for range int(d * w.cfg.LoopHz) {
		for range sub {
			w.quad.Step(dt / sub)
		}
		w.now += msg.Time(dt * 1e6)
		w.f.Step(w.now)
		if check != nil {
			check()
		}
	}
}

func (w *world) arm(mode msg.FlightMode) {
	w.pilot.In = msg.ManualInput{Mode: mode}
	w.run(1.5, nil)
	if !w.f.Calibrated() {
		w.t.Fatal("gyro calibration did not finish")
	}
	w.pilot.In.Arm = true
	w.run(0.1, nil)
	if !w.f.Status().Armed {
		w.t.Fatalf("did not arm, status %+v", w.f.Status())
	}
}

// takeoff climbs in AltHold and lets go of the stick near 1 m.
func (w *world) takeoff() {
	w.arm(msg.ModeAltHold)
	w.pilot.In.Climb = 1
	w.run(2, nil)
	w.pilot.In.Climb = 0
	w.run(3, nil)
}

func TestAltHoldHover(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	start := -w.quad.Pos.Z
	if start < 0.5 || start > 1.5 {
		t.Fatalf("height after takeoff %v m", start)
	}
	var maxErr, maxTilt float32
	w.run(5, func() {
		maxErr = fmath.Max(maxErr, fmath.Abs(-w.quad.Pos.Z-start))
		maxTilt = fmath.Max(maxTilt, w.quad.Q.Tilt())
	})
	t.Logf("hover at %.2f m, height error %.3f m, tilt %.2f°", start, maxErr, fmath.Degrees(maxTilt))
	if maxErr > 0.1 {
		t.Errorf("height wandered %v m", maxErr)
	}
	if maxTilt > fmath.Radians(3) {
		t.Errorf("tilted %v° while hovering", fmath.Degrees(maxTilt))
	}
	if e := w.f.Position().Z - w.quad.Pos.Z; fmath.Abs(e) > 0.05 {
		t.Errorf("height estimate off by %v m", e)
	}
}

func TestTiltRecovery(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	// A knock that spins the quad hard enough to tip it past 30 degrees.
	w.quad.Omega.X = 25
	var peak float32
	var settled float32 = -1
	elapsed := float32(0)
	w.run(2, func() {
		elapsed += 1 / w.cfg.LoopHz
		tilt := w.quad.Q.Tilt()
		peak = fmath.Max(peak, tilt)
		if tilt > fmath.Radians(3) {
			settled = -1
		} else if settled < 0 {
			settled = elapsed
		}
	})
	t.Logf("peak tilt %.1f°, back within 3° after %.3f s", fmath.Degrees(peak), settled)
	if peak < fmath.Radians(30) {
		t.Errorf("knock only tilted %v°", fmath.Degrees(peak))
	}
	if settled < 0 || settled > 0.5 {
		t.Errorf("recovery took %v s", settled)
	}
	if !w.f.Status().Armed {
		t.Error("disarmed during recovery")
	}
}

func TestDisturbanceRejection(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	w.quad.Disturb = fmath.Vec3{X: 2e-4}
	var maxTilt float32
	w.run(3, func() { maxTilt = fmath.Max(maxTilt, w.quad.Q.Tilt()) })
	r, _, _ := w.quad.Q.Euler()
	t.Logf("peak tilt %.2f°, final roll %.2f°", fmath.Degrees(maxTilt), fmath.Degrees(r))
	if fmath.Abs(r) > fmath.Radians(1) {
		t.Errorf("steady roll %v° with constant disturbance", fmath.Degrees(r))
	}
}

func TestStepResponse(t *testing.T) {
	for _, axis := range []string{"roll", "pitch", "yaw"} {
		t.Run(axis, func(t *testing.T) {
			w := newWorld(t)
			w.takeoff()
			var want float32
			switch axis {
			case "roll":
				w.pilot.In.Roll = 0.5
				want = 0.5 * w.cfg.Manual.MaxTilt
			case "pitch":
				w.pilot.In.Pitch = 0.5
				want = 0.5 * w.cfg.Manual.MaxTilt
			case "yaw":
				w.pilot.In.Yaw = 0.5
				want = 0.5 * w.cfg.Manual.MaxYawRate
			}
			var peak, at200ms float32
			n := 0
			w.run(0.6, func() {
				n++
				r, p, _ := w.quad.Q.Euler()
				v := map[string]float32{"roll": r, "pitch": p, "yaw": w.quad.Omega.Z}[axis]
				peak = fmath.Max(peak, v)
				if n == int(0.2*w.cfg.LoopHz) {
					at200ms = v
				}
			})
			t.Logf("%s target %.3f, at 200 ms %.3f, peak %.3f", axis, want, at200ms, peak)
			if at200ms < 0.8*want {
				t.Errorf("slow, %v at 200 ms", at200ms)
			}
			if peak > 1.25*want {
				t.Errorf("overshoot, peak %v", peak)
			}
		})
	}
}

func TestInputLossLands(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	w.pilot.Silent = true
	w.run(w.cfg.Commander.LandTime+2, nil)
	s := w.f.Status()
	if s.Armed || s.Failsafe != msg.FailsafeInputLost {
		t.Errorf("status %+v", s)
	}
	if !w.quad.OnGround() {
		t.Errorf("still %v m up", -w.quad.Pos.Z)
	}
}

func TestStabilizeThrottle(t *testing.T) {
	w := newWorld(t)
	w.arm(msg.ModeStabilize)
	w.pilot.In.Throttle = 0.6
	w.run(1, nil)
	if w.quad.Pos.Z > -0.2 {
		t.Errorf("did not climb with throttle up, z %v", w.quad.Pos.Z)
	}
	if w.quad.Q.Tilt() > fmath.Radians(3) {
		t.Errorf("tilt %v° while climbing", fmath.Degrees(w.quad.Q.Tilt()))
	}
}

func TestDisarmedMotorsOff(t *testing.T) {
	w := newWorld(t)
	w.pilot.In = msg.ManualInput{Throttle: 0.8, Mode: msg.ModeStabilize}
	w.run(2, nil)
	for i, v := range w.f.Motors().Value {
		if v != 0 {
			t.Errorf("motor %d at %v while disarmed", i, v)
		}
	}
}

func TestIMUFailureStopsMotors(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	w.quad.IMUFail = true
	w.run(0.2, nil)
	for i := range 4 {
		if w.quad.Cmd[i] != 0 {
			t.Errorf("motor %d still at %v", i, w.quad.Cmd[i])
		}
	}
}

func TestStepDoesNotAllocate(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	n := testing.AllocsPerRun(400, func() {
		w.quad.Step(0.0025)
		w.now += 2500
		w.f.Step(w.now)
	})
	if n > 0 {
		t.Errorf("Step allocates %v times", n)
	}
}
