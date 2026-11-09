package sim

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

func TestHole(t *testing.T) {
	h := Hole(1, 2, 0.5)
	for _, c := range []struct{ x, want float32 }{{0.9, 0}, {1, -0.5}, {1.9, -0.5}, {2, 0}} {
		if got := h(c.x, 3); got != c.want {
			t.Errorf("x %v height %v, want %v", c.x, got, c.want)
		}
	}
}

func TestChairs(t *testing.T) {
	c := DefaultChairs(1, 2)
	for _, p := range []struct{ x, y, want float32 }{
		{0.5, 0, 0},       // in front of the first row
		{1.1, 0.2, 0.45},  // seat
		{1.43, 0.2, 0.85}, // back
		{1.6, 0.2, 0},     // between rows
		{2, -0.2, 0.45},   // second row seat
		{1.1, 0.52, 0},    // gap between chairs
		{2.9, 0.2, 0},     // past the last row
	} {
		if got := c.Height(p.x, p.y); got != p.want {
			t.Errorf("at %v, %v height %v, want %v", p.x, p.y, got, p.want)
		}
	}
	c.MidAisle = 1
	if c.Height(1.1, 0.3) != 0 || c.Height(1.1, 0.7) != 0.45 {
		t.Error("no aisle in the middle")
	}
}

func TestRough(t *testing.T) {
	r := Rough{Cell: 0.2, Max: 0.15, Seed: 2}
	var lo, hi float32 = 1, -1
	for i := range 100 {
		h := r.Height(float32(i)*0.2-10, float32(i)*0.13)
		lo, hi = fmath.Min(lo, h), fmath.Max(hi, h)
	}
	if lo < 0 || hi > r.Max || hi-lo < r.Max/2 {
		t.Errorf("heights %v to %v m", lo, hi)
	}
	if r.Height(0.01, 0.01) != r.Height(0.19, 0.19) {
		t.Error("height changes inside a cell")
	}
}

func TestTerrainSensors(t *testing.T) {
	p := StampFly()
	p.RangeNoise = 0
	q := New(p)
	q.Terrain = Hole(1, 2, 0.5)
	q.Pos = fmath.Vec3{X: 1.5, Z: -1}
	var r msg.RangeSample
	q.ReadRange(&r)
	if !r.Valid || fmath.Abs(r.Distance-1.51) > 1e-4 {
		t.Errorf("range over the hole %+v", r)
	}
	if q.Clearance() != 1.5 || q.OnGround() {
		t.Errorf("clearance %v", q.Clearance())
	}
	q.Terrain = func(x, y float32) float32 { return 0.3 }
	q.Pos.Z = 0
	q.Step(0.001)
	if !q.OnGround() || fmath.Abs(q.Pos.Z+0.3) > 1e-6 {
		t.Errorf("not resting on the raised surface, z %v", q.Pos.Z)
	}
}

func TestGrid(t *testing.T) {
	g := Grid{X0: -1, Y0: -1, Cell: 0.5, Cols: 4, Rows: 2, H: []float32{1, 2, 3, 4, 5, 6, 7, 8}}
	for _, p := range []struct{ x, y, want float32 }{
		{-1, -1, 1},
		{0.9, -0.6, 4},
		{-0.4, -0.4, 6},
		{0.99, -0.01, 8},
		{1, 0, 0},     // past the last column
		{0, 0, 0},     // past the last row
		{-1.01, 0, 0}, // before the first column
	} {
		if got := g.Height(p.x, p.y); got != p.want {
			t.Errorf("at %v, %v height %v, want %v", p.x, p.y, got, p.want)
		}
	}
}
