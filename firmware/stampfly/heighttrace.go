//go:build m5stamp_fly && blackbox

package main

import (
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

const (
	traceEntries   = 2400    // 6 s of loop passes at 400 Hz
	traceTrigger   = 0.25    // m of height error in AltHold or PosHold that freezes the trace
	traceSettle    = 3000000 // µs after arming before the trigger counts, to skip takeoff
	traceClimbWait = 1000000 // µs after the climb stick is centred, while the height catches up
	traceAfterTrig = 2000000 // µs kept after the trigger
)

// Flags in traceEntry.
const (
	traceRangeNew = 1 << iota
	traceRangeOK
	traceBaroNew
	traceBaroOK
	traceOnGround
)

// traceEntry is one flight loop pass of the height filter in mm, mm/s, mm/s² and thousandths.
type traceEntry struct {
	us     uint32 // since arming
	accelZ int16  // vertical accel passed to the filter
	dtUs   uint16
	rng    uint16 // range reading, when traceRangeNew is set
	cos    uint16 // cosine of the tilt with that reading, times 10000
	baro   int16  // baro height, when traceBaroNew is set
	z, vz  int16
	zSp    int16 // height setpoint
	bias   int16
	thrust int16
	flags  uint8
}

// heightTrace keeps the height filter's inputs and outputs on every loop pass of the last 6 s, so the
// filter can be replayed off board. It freezes 2 s after the height error first passes traceTrigger,
// so it holds the start of a height swing, and otherwise ends with the flight.
var trace heightTrace

type heightTrace struct {
	entries  [traceEntries]traceEntry
	n        int // passes recorded since arming
	start    msg.Time
	armed    bool
	frozen   bool
	trigAt   uint32 // µs after arming when the trigger fired, 0 if it did not
	climbAt  uint32 // µs after arming when the climb stick was last off centre
	isArmed  func() bool
	holding  func() bool // in AltHold or PosHold
	climbing func() bool // climb stick off centre
	setpoint func() float32
}

// record runs in the flight loop through flight.TraceHeight, so it must stay quick and not allocate.
func (t *heightTrace) record(s *flight.HeightStep) {
	armed := t.isArmed()
	if armed && !t.armed {
		// The filter was reset on this pass, so recording starts with the next one.
		t.n, t.start, t.frozen, t.trigAt, t.climbAt = 0, s.Time, false, 0, 0
		t.armed = true
		return
	}
	t.armed = armed
	if !armed || t.frozen {
		return
	}
	us := uint32(s.Time - t.start)
	var flags uint8
	if s.RangeNew {
		flags |= traceRangeNew
	}
	if s.RangeOK {
		flags |= traceRangeOK
	}
	if s.BaroNew {
		flags |= traceBaroNew
	}
	if s.BaroOK {
		flags |= traceBaroOK
	}
	if s.OnGround {
		flags |= traceOnGround
	}
	t.entries[t.n%traceEntries] = traceEntry{
		us:     us,
		accelZ: milli16(s.AccelZ),
		dtUs:   uint16(fmath.Clamp(s.Dt*1e6, 0, 65535)),
		rng:    uint16(fmath.Clamp(s.Range*1000, 0, 65535)),
		cos:    uint16(fmath.Clamp(s.CosTilt*10000, 0, 65535)),
		baro:   milli16(s.Baro),
		z:      milli16(s.Z),
		vz:     milli16(s.VZ),
		zSp:    milli16(t.setpoint()),
		bias:   milli16(s.Bias),
		thrust: milli16(s.Thrust),
		flags:  flags,
	}
	t.n++
	if t.climbing() {
		t.climbAt = us
	}
	// While climbing the setpoint runs ahead of the height, so only a steady height counts.
	if t.trigAt == 0 && us >= traceSettle && us >= t.climbAt+traceClimbWait && t.holding() &&
		fmath.Abs(s.Z-t.setpoint()) > traceTrigger {
		t.trigAt = us
	}
	if t.trigAt != 0 && us >= t.trigAt+traceAfterTrig {
		t.frozen = true
	}
}

func (t *heightTrace) count() int { return min(t.n, traceEntries) }

// inOrder rotates the ring so the oldest pass comes first. Call it only while disarmed.
func (t *heightTrace) inOrder() {
	n := t.count()
	k := t.n % traceEntries
	if n < traceEntries || k == 0 {
		return
	}
	reverse(t.entries[:k])
	reverse(t.entries[k:])
	reverse(t.entries[:])
	t.n = traceEntries
}

func reverse(s []traceEntry) {
	for i, j := 0, len(s)-1; i < j; i, j = i+1, j-1 {
		s[i], s[j] = s[j], s[i]
	}
}

func printTraceHeader() {
	println("trace us accel_z_mms2 dt_us range_mm cos_x10000 baro_mm z_mm vz_mms z_sp_mm bias_mms2 thrust flags check")
}

// printTraceEntry ends each line with the sum of its values, like printEntry.
func printTraceEntry(e *traceEntry) {
	sum := int64(e.us) + int64(e.accelZ) + int64(e.dtUs) + int64(e.rng) + int64(e.cos) + int64(e.baro) +
		int64(e.z) + int64(e.vz) + int64(e.zSp) + int64(e.bias) + int64(e.thrust) + int64(e.flags)
	println("trace", e.us, e.accelZ, e.dtUs, e.rng, e.cos, e.baro, e.z, e.vz, e.zSp, e.bias, e.thrust, e.flags, sum)
}
