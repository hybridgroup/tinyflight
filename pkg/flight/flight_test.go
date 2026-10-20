package flight

import (
	"testing"

	altest "github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
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
	f := New(Hardware{IMU: quad, Motors: quad, Input: pilot, Baro: quad, Range: quad, Flow: quad, Power: quad}, cfg)
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
func (w *world) takeoff() { w.takeoffIn(msg.ModeAltHold) }

func (w *world) takeoffIn(mode msg.FlightMode) {
	w.arm(mode)
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

func TestPosHoldHover(t *testing.T) {
	w := newWorld(t)
	w.takeoffIn(msg.ModePosHold)
	if !w.f.Position().XYValid {
		t.Fatal("no position estimate after takeoff")
	}
	hold := w.quad.Pos
	var drift float32
	w.run(5, func() {
		d := w.quad.Pos.Sub(hold)
		d.Z = 0
		drift = fmath.Max(drift, d.Norm())
	})
	p := w.f.Position()
	t.Logf("drift %.3f m, estimate off by %.3f %.3f m", drift, p.X-w.quad.Pos.X, p.Y-w.quad.Pos.Y)
	// Flow noise makes the position estimate wander slowly, so allow a little drift.
	if drift > 0.15 {
		t.Errorf("drifted %v m while holding", drift)
	}
}

func TestPosHoldPush(t *testing.T) {
	w := newWorld(t)
	w.takeoffIn(msg.ModePosHold)
	hold := w.quad.Pos
	w.quad.Vel.X, w.quad.Vel.Y = 0.5, -0.3
	var far, maxTilt float32
	w.run(5, func() {
		d := w.quad.Pos.Sub(hold)
		d.Z = 0
		far = fmath.Max(far, d.Norm())
		maxTilt = fmath.Max(maxTilt, w.quad.Q.Tilt())
	})
	d := w.quad.Pos.Sub(hold)
	d.Z = 0
	t.Logf("pushed %.3f m away, back within %.3f m, peak tilt %.1f°", far, d.Norm(), fmath.Degrees(maxTilt))
	if d.Norm() > 0.1 {
		t.Errorf("ended %v m from the hold point", d.Norm())
	}
	if maxTilt > w.cfg.Horizontal.MaxTilt+fmath.Radians(5) {
		t.Errorf("tilted %v°", fmath.Degrees(maxTilt))
	}
}

func TestPosHoldStick(t *testing.T) {
	w := newWorld(t)
	w.takeoffIn(msg.ModePosHold)
	start := w.quad.Pos
	w.pilot.In.Pitch = -1
	w.run(2, nil)
	moved := w.quad.Pos.X - start.X
	if moved < 0.5 {
		t.Errorf("moved %v m forward with the stick held forward", moved)
	}
	w.pilot.In.Pitch = 0
	w.run(2, nil)
	stop := w.quad.Pos
	w.run(3, nil)
	d := w.quad.Pos.Sub(stop)
	d.Z = 0
	t.Logf("moved %.2f m, then drifted %.3f m after letting go", moved, d.Norm())
	if d.Norm() > 0.1 || fmath.Abs(w.quad.Vel.X) > 0.05 {
		t.Errorf("still moving after letting go, %v m and %v m/s", d.Norm(), w.quad.Vel.X)
	}
}

func TestPosHoldFlowLost(t *testing.T) {
	w := newWorld(t)
	w.takeoffIn(msg.ModePosHold)
	z := w.quad.Pos.Z
	w.quad.FlowLost = true
	var maxTilt float32
	w.run(3, func() { maxTilt = fmath.Max(maxTilt, w.quad.Q.Tilt()) })
	if w.f.Position().XYValid {
		t.Error("position still valid without flow")
	}
	if fmath.Abs(w.quad.Pos.Z-z) > 0.1 || maxTilt > fmath.Radians(3) {
		t.Errorf("height moved %v m, tilt %v° after losing flow", w.quad.Pos.Z-z, fmath.Degrees(maxTilt))
	}
}

// TestFlowSetDown drops the quad to just above the ground in one step while it slides, like setting it down by hand.
// The height filter rejects the sudden range change, so only the raw range shows it is too close for flow.
func TestFlowSetDown(t *testing.T) {
	w := newWorld(t)
	w.takeoffIn(msg.ModePosHold)
	w.quad.FlowCloseValid = true
	start := w.f.Position()
	w.run(1, func() {
		w.quad.Pos.Z = -0.005
		w.quad.Vel = fmath.Vec3{X: 0.05, Y: 0.05}
	})
	p := w.f.Position()
	if d := (fmath.Vec3{X: p.X - start.X, Y: p.Y - start.Y}).Norm(); d > 0.2 {
		t.Errorf("position estimate jumped %v m while set down", d)
	}
}

func TestAltHoldArmIdles(t *testing.T) {
	w := newWorld(t)
	w.arm(msg.ModeAltHold)
	w.run(2, nil)
	if !w.quad.OnGround() {
		t.Fatalf("lifted %v m after arming with the stick centred", -w.quad.Pos.Z)
	}
	if th := w.f.ThrustTorque().Thrust; th != 0 {
		t.Errorf("thrust %v after arming, want idle", th)
	}
	w.pilot.In.Climb = 1
	w.run(1, nil)
	if w.quad.Pos.Z > -0.2 {
		t.Errorf("did not climb with the stick up, z %v", w.quad.Pos.Z)
	}
}

func TestThrottleCurve(t *testing.T) {
	cfg := params.Default()
	cfg.Vertical.HoverThrust = 0.3
	cfg.Manual.FullStick = 2
	w := newWorldConfig(t, cfg)
	w.f.updateHover(0)
	for _, tc := range []struct{ stick, thrust float32 }{{0, 0}, {0.25, 0.15}, {0.5, 0.3}, {0.75, 0.45}, {1, 0.6}} {
		if got := w.f.throttleCurve(tc.stick); fmath.Abs(got-tc.thrust) > 1e-5 {
			t.Errorf("stick %v gave thrust %v, want %v", tc.stick, got, tc.thrust)
		}
	}
}

// TestGroundWindup sits armed just below hover while the yaw estimate drifts, as the gyro does
// on the real drone, then lifts off. Errors the drone cannot fix on the ground must not build up.
func TestGroundWindup(t *testing.T) {
	w := newWorld(t)
	w.arm(msg.ModeStabilize)
	w.quad.P.GyroBias = w.quad.P.GyroBias.Add(fmath.Vec3{X: 0.02, Z: 0.05})
	w.pilot.In.Throttle = 0.4
	w.run(5, nil)
	if !w.quad.OnGround() {
		t.Fatal("lifted off below hover")
	}
	yaw := w.quad.Q.Yaw()
	w.pilot.In.Throttle = 0.65
	var maxTilt, maxYaw float32
	w.run(1, func() {
		maxTilt = fmath.Max(maxTilt, w.quad.Q.Tilt())
		maxYaw = fmath.Max(maxYaw, fmath.Abs(fmath.WrapPi(w.quad.Q.Yaw()-yaw)))
	})
	t.Logf("after liftoff tilt %.1f°, yaw turned %.1f°", fmath.Degrees(maxTilt), fmath.Degrees(maxYaw))
	if w.quad.OnGround() {
		t.Fatal("did not lift off")
	}
	if maxTilt > fmath.Radians(5) || maxYaw > fmath.Radians(10) {
		t.Error("errors built up on the ground and kicked the drone at liftoff")
	}
}

// TestHoverFromBattery checks the StampFly hover model against get_trim_duty in the stock firmware.
func TestHoverFromBattery(t *testing.T) {
	for _, tc := range []struct{ volts, thrust float32 }{{4.2, 0.29}, {3.9, 0.38}, {3.7, 0.44}} {
		cfg := params.Default()
		cfg.Hover = params.Hover{Slope: -0.2448, Offset: 1.5892}
		w := newWorldConfig(t, cfg)
		w.quad.P.Voltage = tc.volts
		w.run(1, nil)
		for range 400 {
			w.f.updateHover(0.0025)
		}
		if fmath.Abs(w.f.hover-tc.thrust) > 0.01 {
			t.Errorf("%v V gave hover thrust %v, want about %v", tc.volts, w.f.hover, tc.thrust)
		}
	}
}

func TestMaxHeight(t *testing.T) {
	cfg := params.Default()
	cfg.Manual.MaxHeight = 1
	w := newWorldConfig(t, cfg)
	w.arm(msg.ModeAltHold)
	w.pilot.In.Climb = 1
	var top float32
	w.run(8, func() { top = fmath.Max(top, -w.quad.Pos.Z) })
	t.Logf("highest %.2f m with the stick held up", top)
	if top > 1.1 {
		t.Errorf("climbed to %v m past the 1 m limit", top)
	}
	if top < 0.9 {
		t.Errorf("only reached %v m", top)
	}
}

// TestAltHoldWeakMotors flies with lift that grows almost linearly with motor command and a range reading
// 0.2 s late, as on the real StampFly, and with the height gains and range delay set as on the StampFly board.
// Without the range delay in the height filter the height kept swinging about 20 cm.
func TestAltHoldWeakMotors(t *testing.T) {
	cfg := params.Default()
	cfg.Vertical.Climb.P *= 2.2
	cfg.Vertical.Climb.I *= 2.2
	cfg.AltEst.RangeDelay = 0.2
	w := newWorldConfig(t, cfg)
	w.quad.P.ThrustExp = 0.9
	w.quad.P.MaxThrust = 0.18
	w.quad.P.RangeDelay = 0.2
	w.takeoff()
	w.quad.Vel.Z = 0.5
	w.run(7, nil)
	lo, hi := float32(9), float32(-9)
	w.run(5, func() { lo, hi = fmath.Min(lo, -w.quad.Pos.Z), fmath.Max(hi, -w.quad.Pos.Z) })
	t.Logf("height %.2f to %.2f m after a knock", lo, hi)
	if hi-lo > 0.1 {
		t.Errorf("height still swinging %v m", hi-lo)
	}
}

// TestTraceHeightReplays checks the height trace has everything needed to replay the height filter off board.
// The filter is reset on the pass that arms, so a fresh filter replays every pass after that one.
func TestTraceHeightReplays(t *testing.T) {
	w := newWorld(t)
	w.pilot.In = msg.ManualInput{Mode: msg.ModeAltHold}
	w.run(1.5, nil)
	var steps []HeightStep
	armed := false
	w.f.TraceHeight(func(s *HeightStep) {
		if armed {
			steps = append(steps, *s)
		}
		armed = w.f.Status().Armed
	})
	w.pilot.In.Arm = true
	w.pilot.In.Climb = 1
	w.run(2, nil)
	w.pilot.In.Climb = 0
	w.run(2, nil)

	f := altest.New(w.cfg.AltEst)
	var ranges int
	var worst float32
	for i := range steps {
		s := &steps[i]
		if s.BaroNew {
			f.UpdateBaro(s.Baro)
		}
		if s.RangeNew {
			ranges++
			f.UpdateRange(s.Range, s.CosTilt)
		}
		f.SetOnGround(s.OnGround)
		f.Predict(s.AccelZ, s.Dt)
		worst = fmath.Max(worst, fmath.Abs(f.Z()-s.Z)+fmath.Abs(f.VZ()-s.VZ))
	}
	t.Logf("%d passes, %d range readings, worst replay difference %g", len(steps), ranges, worst)
	if len(steps) < 1500 || ranges < 100 {
		t.Fatalf("trace too short, %d passes and %d range readings", len(steps), ranges)
	}
	if worst > 1e-6 {
		t.Errorf("replay differs by %v", worst)
	}
}

func TestLandedDisarms(t *testing.T) {
	w := newWorld(t)
	w.takeoff()
	w.run(3, nil)
	if !w.f.Status().Armed {
		t.Fatal("disarmed while hovering")
	}
	w.pilot.In.Climb = -1
	var tm, landedAt, disarmedAt float32 = 0, -1, -1
	w.run(10, func() {
		tm += 1 / w.cfg.LoopHz
		if landedAt < 0 && w.quad.OnGround() {
			landedAt = tm
		}
		if disarmedAt < 0 && !w.f.Status().Armed {
			disarmedAt = tm
		}
	})
	t.Logf("disarmed %.2f s after touchdown", disarmedAt-landedAt)
	if disarmedAt < 0 || landedAt < 0 || disarmedAt-landedAt > 3 {
		t.Error("did not disarm within 3 s of landing")
	}
}

// TestLiftoffTrim gives the quad a steady torque, like motors that are not quite matched, measures the
// trim the rate integrators hold in a hover, then lifts off again with that trim. Without it the quad tips
// at liftoff, as the StampFly did on flights 17 to 20.
func TestLiftoffTrim(t *testing.T) {
	imbalance := fmath.Vec3{X: -3.6e-4, Y: 2.4e-4, Z: -1.2e-4}
	liftoff := func(trim fmath.Vec3) (tilt float32, held fmath.Vec3) {
		cfg := params.Default()
		cfg.Rate.Trim = trim
		w := newWorldConfig(t, cfg)
		w.quad.Disturb = imbalance
		w.arm(msg.ModeAltHold)
		w.pilot.In.Climb = 0.3
		w.run(2, func() {
			if !w.quad.OnGround() {
				tilt = fmath.Max(tilt, w.quad.Q.Tilt())
			}
		})
		w.pilot.In.Climb = 0
		w.run(5, nil)
		return tilt, w.f.RateIntegral()
	}
	untrimmed, trim := liftoff(fmath.Vec3{})
	trimmed, _ := liftoff(trim)
	t.Logf("trim %.4f %.4f %.4f, liftoff tilt %.1f° without and %.1f° with it", trim.X, trim.Y, trim.Z, fmath.Degrees(untrimmed), fmath.Degrees(trimmed))
	if trimmed > untrimmed/2 {
		t.Errorf("trim did not reduce the liftoff tilt enough")
	}
}

// TestTrimCarriesOver flies twice without a trim set. The second liftoff starts from what the first
// flight needed, so it does not tip.
func TestTrimCarriesOver(t *testing.T) {
	w := newWorld(t)
	w.quad.Disturb = fmath.Vec3{X: -3.6e-4, Y: 2.4e-4, Z: -1.2e-4}
	fly := func() (tilt float32) {
		w.pilot.In = msg.ManualInput{Mode: msg.ModeAltHold, Arm: true, Climb: 0.3}
		w.run(2, func() {
			if !w.quad.OnGround() {
				tilt = fmath.Max(tilt, w.quad.Q.Tilt())
			}
		})
		w.pilot.In.Climb = 0
		w.run(6, nil)
		w.pilot.In.Climb = -1
		w.run(6, nil)
		w.pilot.In.Arm = false
		w.run(1, nil)
		return tilt
	}
	w.arm(msg.ModeAltHold)
	w.pilot.In.Arm = false
	w.run(0.5, nil)
	first := fly()
	second := fly()
	t.Logf("liftoff tilt %.1f° on the first flight and %.1f° on the second", fmath.Degrees(first), fmath.Degrees(second))
	if second > first/2 {
		t.Errorf("trim did not carry over")
	}
}
