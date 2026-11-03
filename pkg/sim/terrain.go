package sim

// Hole is a pit Depth m deep between X0 and X1 that runs the full width in Y.
func Hole(x0, x1, depth float32) func(x, y float32) float32 {
	return func(x, y float32) float32 {
		if x >= x0 && x < x1 {
			return -depth
		}
		return 0
	}
}

// Chairs is Rows rows of chairs that run the full width in Y, starting at X0 and facing -X.
type Chairs struct {
	X0       float32
	Rows     int
	Spacing  float32 // m from one row to the next
	Seat     float32 // m, seat height
	SeatLen  float32 // m, seat depth along X
	Back     float32 // m, height of the top of the back
	BackLen  float32 // m, back thickness along X
	Width    float32 // m, chair width along Y
	Gap      float32 // m between chairs in a row
	MidAisle float32 // m, width of an aisle at Y = 0, 0 for none
}

// DefaultChairs is rows of plain stacking chairs set out for a talk.
func DefaultChairs(x0 float32, rows int) Chairs {
	return Chairs{X0: x0, Rows: rows, Spacing: 0.9, Seat: 0.45, SeatLen: 0.45, Back: 0.85, BackLen: 0.04, Width: 0.5, Gap: 0.05}
}

// Height returns the height of the chairs at x, y.
func (c Chairs) Height(x, y float32) float32 {
	if x < c.X0 {
		return 0
	}
	row := int((x - c.X0) / c.Spacing)
	if row >= c.Rows {
		return 0
	}
	if y < 0 {
		y = -y
	}
	if y < c.MidAisle/2 {
		return 0
	}
	if y -= c.MidAisle / 2; y-float32(int(y/(c.Width+c.Gap)))*(c.Width+c.Gap) >= c.Width {
		return 0
	}
	dx := x - c.X0 - float32(row)*c.Spacing
	switch {
	case dx < c.SeatLen-c.BackLen:
		return c.Seat
	case dx < c.SeatLen:
		return c.Back
	}
	return 0
}

// Rough is uneven ground made of square cells Cell m wide, each up to Max m high.
type Rough struct {
	Cell float32
	Max  float32
	Seed uint32
}

// Height returns the height of the cell under x, y.
func (r Rough) Height(x, y float32) float32 {
	i, j := floor(x/r.Cell), floor(y/r.Cell)
	h := uint32(i)*73856093 ^ uint32(j)*19349663 ^ r.Seed*83492791
	rnd := NewRand(h)
	rnd.Uint32()
	return rnd.Float32() * r.Max
}

func floor(x float32) int32 {
	i := int32(x)
	if float32(i) > x {
		i--
	}
	return i
}
