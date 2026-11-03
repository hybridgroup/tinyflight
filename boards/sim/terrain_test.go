package sim

import (
	"fmt"
	"testing"

	"github.com/hybridgroup/tinyflight/boards/stampfly"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	physics "github.com/hybridgroup/tinyflight/pkg/sim"
)

// cameraMass is the 5 g camera setup from docs/camera.md.
const cameraMass = 0.005

type terrainCase struct {
	name    string
	terrain func(x, y float32) float32
	climb   float32 // s of extra full climb after takeoff
	to      float32 // m to fly forward
	noise   float32 // m, range noise, 0 for the default
}

type flightLog struct {
	takeoff     float32 // m above the floor after takeoff
	disarmed    bool
	failsafe    msg.Failsafe
	tilt        float32 // rad, most while flying over
	speed       float32 // m/s, most horizontal speed
	clearance   float32 // m, least height above the surface
	heightErr   float32 // m, most change in height above the floor
	reached     float32 // m forward when the stick was let go
	settleDrift float32 // m moved after letting go and settling
}

// flyOver takes off in PosHold, flies forward with the stick held, lets go and holds.
func flyOver(t *testing.T, tc terrainCase, extraMass float32) flightLog {
	r := newRig(t)
	r.w.Quad.Terrain = tc.terrain
	r.w.Quad.P.Mass += extraMass
	if tc.noise > 0 {
		r.w.Quad.P.RangeNoise = tc.noise
	}
	r.s.autoAlt, r.s.rate = true, true
	var l flightLog
	watch := func() {
		s := r.f.Status()
		if !s.Armed {
			l.disarmed = true
		}
		if s.Failsafe != msg.FailsafeNone {
			l.failsafe = s.Failsafe
		}
	}
	r.arm()
	r.s.throttle = 1
	r.fly(2+tc.climb, watch)
	r.s.throttle = 0
	r.fly(3, watch)
	l.takeoff = -r.w.Quad.Pos.Z
	l.clearance = r.w.Quad.Clearance()
	q := r.w.Quad
	track := func() {
		watch()
		l.tilt = fmath.Max(l.tilt, q.Q.Tilt())
		l.speed = fmath.Max(l.speed, fmath.Vec3{X: q.Vel.X, Y: q.Vel.Y}.Norm())
		l.clearance = fmath.Min(l.clearance, q.Clearance())
		l.heightErr = fmath.Max(l.heightErr, fmath.Abs(-q.Pos.Z-l.takeoff))
	}
	r.s.pitch = -1
	for i := 0; q.Pos.X < tc.to && i < 30*400 && !l.disarmed; i++ {
		r.fly(1.0/400, track)
	}
	l.reached = q.Pos.X
	r.s.pitch = 0
	r.fly(3, track)
	stop := q.Pos
	r.fly(3, track)
	d := q.Pos.Sub(stop)
	d.Z = 0
	l.settleDrift = d.Norm()
	return l
}

// problems lists what went wrong on the flight.
func (l flightLog) problems(tc terrainCase) []string {
	var p []string
	if l.disarmed || l.failsafe != msg.FailsafeNone {
		p = append(p, fmt.Sprintf("disarmed %v, failsafe %v", l.disarmed, l.failsafe))
	}
	if l.reached < tc.to {
		p = append(p, fmt.Sprintf("only reached %.2f m of %.2f m", l.reached, tc.to))
	}
	if l.clearance < 0.1 {
		p = append(p, fmt.Sprintf("came within %.2f m of the surface", l.clearance))
	}
	if l.heightErr > 0.25 {
		p = append(p, fmt.Sprintf("height above the floor changed %.2f m", l.heightErr))
	}
	if max := 2 * stampfly.Config().Horizontal.MaxSpeed; l.speed > max {
		p = append(p, fmt.Sprintf("reached %.1f m/s, over %.1f m/s", l.speed, max))
	}
	if l.tilt > fmath.Radians(20) {
		p = append(p, fmt.Sprintf("tilted %.1f°", fmath.Degrees(l.tilt)))
	}
	if l.settleDrift > 0.3 {
		p = append(p, fmt.Sprintf("drifted %.2f m after letting go", l.settleDrift))
	}
	return p
}

func runTerrain(t *testing.T, cases []terrainCase) {
	for _, tc := range cases {
		for _, m := range []struct {
			name string
			mass float32
		}{{"bare", 0}, {"camera", cameraMass}} {
			t.Run(tc.name+"/"+m.name, func(t *testing.T) {
				l := flyOver(t, tc, m.mass)
				t.Logf("takeoff %.2f m, reached %.2f m, tilt %.1f°, speed %.2f m/s, clearance %.2f m, height error %.3f m, drift %.3f m",
					l.takeoff, l.reached, fmath.Degrees(l.tilt), l.speed, l.clearance, l.heightErr, l.settleDrift)
				if p := l.problems(tc); len(p) > 0 {
					t.Error(p)
				}
			})
		}
	}
}

func TestFlatFloor(t *testing.T) {
	// A noisy range at liftoff once fooled step detection and the drone shot up to 2.2 m.
	runTerrain(t, []terrainCase{{name: "flat", to: 3}, {name: "noisy range", to: 3, noise: 0.03}})
}

func TestFlyOverHole(t *testing.T) {
	runTerrain(t, []terrainCase{
		{name: "step down", terrain: physics.Hole(1, 2, 0.5), to: 3},
		{name: "narrow", terrain: physics.Hole(1, 1.3, 0.5), to: 3},
		{name: "out of range", terrain: physics.Hole(1, 2.5, 3), to: 3.5},
		{name: "drop off", terrain: physics.Hole(1, 100, 0.8), to: 3},
	})
}

func TestFlyOverChairs(t *testing.T) {
	aisle := physics.DefaultChairs(1, 4)
	aisle.MidAisle = 1
	runTerrain(t, []terrainCase{
		{name: "low", terrain: physics.DefaultChairs(1, 4).Height, climb: 0.4, to: 5},
		{name: "rows", terrain: physics.DefaultChairs(1, 4).Height, climb: 1, to: 5},
		{name: "aisle", terrain: aisle.Height, climb: 1, to: 5},
		{name: "takeoff from a seat", terrain: physics.DefaultChairs(-0.2, 4).Height, climb: 1, to: 4},
	})
}

func TestFlyOverRough(t *testing.T) {
	runTerrain(t, []terrainCase{
		{name: "pebbles", terrain: physics.Rough{Cell: 0.05, Max: 0.03, Seed: 1}.Height, to: 3},
		{name: "rocks", terrain: physics.Rough{Cell: 0.2, Max: 0.15, Seed: 2}.Height, to: 3},
		{name: "boulders", terrain: physics.Rough{Cell: 0.5, Max: 0.4, Seed: 3}.Height, to: 3},
	})
}
