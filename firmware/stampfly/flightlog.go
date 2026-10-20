//go:build m5stamp_fly && blackbox

package main

import (
	"machine"
	"time"
	"unsafe"

	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

const (
	logPeriod   = 100 * time.Millisecond
	logEntries  = 600 // 1 minute at logPeriod
	logInterval = 10 * time.Second
)

// logEntry is one sample in mm, mm/s, mm/s², tenths of a degree and thousandths of thrust.
type logEntry struct {
	ms          uint32
	x, y, z     int16
	vx, vy      int16
	roll, pitch int16
	yaw         int16
	thrust      int16
	rng         int16 // raw downward range, -1 when invalid
	accZ        int16 // body Z accel, about -9810 when level and still
	bias        int16 // vertical accel bias estimate
	flowX       int16 // flow rate in body axes, mrad/s
	flowY       int16
	gyroX       int16 // mean gyro rate over the same flow reading, mrad/s
	gyroY       int16
	measVX      int16 // speed the estimator made of the flow reading, mm/s
	measVY      int16
	flowFlags   uint8 // 1 sensor valid, 2 passed the height checks, 4 north accepted, 8 east accepted
	stickRoll   int16 // pilot input from -1000 to 1000, throttle from 0
	stickPitch  int16
	stickYaw    int16
	stickThr    int16
	stickClimb  int16
	vz          int16  // vertical speed estimate, mm/s, NED so climbing is negative
	zSp         int16  // height setpoint, mm, NED
	battery     uint16 // mV
	trimX       int16  // rate integrator torque, ten thousandths
	trimY       int16
	trimZ       int16
	quality     uint8
	mode        uint8
	xyValid     bool
	zValid      bool
	landing     bool
}

// flightLog keeps the last armed flight in memory, so it can be read over USB after landing.
// It is a global so the buffer is set aside at build time.
var flog flightLog

type flightLog struct {
	entries [logEntries]logEntry
	n       int
}

// rtcTail keeps the last part of a flight in RTC SLOW memory, which nothing writes at startup, so it
// survives a reset but not a power loss. RTC SLOW memory is 8 KB at 0x5000_0000, ESP32-S3 Technical
// Reference Manual, chapter 4 System and Memory.
type rtcTail struct {
	magic   uint32
	next    uint32
	count   uint32
	entries [tailEntries]logEntry
}

const (
	rtcSlowMem  = 0x50000000
	tailMagic   = 0x464c4f47
	tailEntries = uint32((8192 - 64) / unsafe.Sizeof(logEntry{}))
)

var tail = (*rtcTail)(unsafe.Pointer(uintptr(rtcSlowMem)))

const saveDelay = time.Second // after disarming, before the flight is saved to flash

// startBlackbox traces the height filter and runs the flight log on core 0.
func startBlackbox(f *flight.Flight) {
	trace.isArmed = func() bool { return f.Status().Armed }
	trace.holding = func() bool { m := f.Status().Mode; return m == msg.ModeAltHold || m == msg.ModePosHold }
	trace.climbing = func() bool { return f.Input().Climb != 0 }
	trace.setpoint = f.HeightSetpoint
	f.TraceHeight(trace.record)
	go func() {
		machine.AssignCore(0)
		flog.run(f)
	}()
}

// run samples at logPeriod while armed, saves the flight to flash after landing, and prints the
// latest saved flight every logInterval while disarmed. A tail left in RTC memory by a reset is
// saved at startup.
func (l *flightLog) run(f *flight.Flight) {
	var start, lastPrint, disarmed time.Time
	armed := func() bool { return f.Status().Armed }
	store.open()
	if tail.magic == tailMagic && tail.count > 0 && tail.count <= tailEntries && tail.next < tailEntries {
		first := (tail.next + tailEntries - tail.count) % tailEntries
		for i := range tail.count {
			l.entries[i] = tail.entries[(first+i)%tailEntries]
		}
		if err := store.save(slotTail, l.entries[:tail.count], nil, 0, armed); err != nil {
			println("save:", err.Error())
		}
		tail.magic = 0
	}
	wasArmed, unsaved := false, false
	for {
		time.Sleep(logPeriod)
		now := armed()
		switch {
		case now && !wasArmed:
			l.n = 0
			start = time.Now()
			tail.magic, tail.next, tail.count = tailMagic, 0, 0
			fallthrough
		case now:
			l.add(f, start)
			unsaved = true
			disarmed = time.Now()
		case unsaved && time.Since(disarmed) >= saveDelay:
			trace.inOrder()
			if err := store.save(slotFlight, l.entries[:l.n], trace.entries[:trace.count()], trace.trigAt, armed); err != nil {
				println("save:", err.Error())
			} else {
				// The whole flight is saved, so the tail is not needed after a reset.
				tail.magic = 0
			}
			unsaved = false
		case time.Since(lastPrint) < logInterval:
		case store.latest >= 0:
			store.print()
			lastPrint = time.Now()
		case l.n > 0:
			l.print()
			lastPrint = time.Now()
		}
		wasArmed = now
	}
}

func (l *flightLog) add(f *flight.Flight, start time.Time) {
	if l.n >= logEntries {
		return
	}
	p := f.Position()
	s := f.Status()
	r, pt, y := f.Attitude().Q.Euler()
	rng := int16(-1)
	if rs := f.Range(); rs.Valid {
		rng = int16(rs.Distance * 1000)
	}
	fl := f.Flow()
	gyro, used, speed, okN, okE := f.FlowDebug()
	var flags uint8
	for i, b := range [4]bool{fl.Valid, used, okN, okE} {
		if b {
			flags |= 1 << i
		}
	}
	in := f.Input()
	ri := f.RateIntegral()
	l.entries[l.n] = logEntry{
		ms:         uint32(time.Since(start) / time.Millisecond),
		x:          int16(p.X * 1000),
		y:          int16(p.Y * 1000),
		z:          int16(p.Z * 1000),
		vx:         int16(p.VX * 1000),
		vy:         int16(p.VY * 1000),
		roll:       decideg(r),
		pitch:      decideg(pt),
		yaw:        decideg(y),
		thrust:     int16(f.ThrustTorque().Thrust * 1000),
		rng:        rng,
		accZ:       int16(fmath.Clamp(f.IMU().Accel.Z*1000, -32767, 32767)),
		bias:       int16(p.AccBias * 1000),
		flowX:      milli16(fl.Rate.X),
		flowY:      milli16(fl.Rate.Y),
		gyroX:      milli16(gyro.X),
		gyroY:      milli16(gyro.Y),
		measVX:     milli16(speed.X),
		measVY:     milli16(speed.Y),
		flowFlags:  flags,
		stickRoll:  milli16(in.Roll),
		stickPitch: milli16(in.Pitch),
		stickYaw:   milli16(in.Yaw),
		stickThr:   milli16(in.Throttle),
		stickClimb: milli16(in.Climb),
		vz:         milli16(p.VZ),
		zSp:        milli16(f.HeightSetpoint()),
		battery:    uint16(f.Battery().Voltage * 1000),
		trimX:      int16(fmath.Clamp(ri.X*10000, -32767, 32767)),
		trimY:      int16(fmath.Clamp(ri.Y*10000, -32767, 32767)),
		trimZ:      int16(fmath.Clamp(ri.Z*10000, -32767, 32767)),
		quality:    f.Flow().Quality,
		mode:       uint8(s.Mode),
		xyValid:    p.XYValid,
		zValid:     p.ZValid,
		landing:    s.Landing,
	}
	tail.entries[tail.next] = l.entries[l.n]
	tail.next = (tail.next + 1) % tailEntries
	if tail.count < tailEntries {
		tail.count++
	}
	l.n++
}

func (l *flightLog) print() {
	println("log start", l.n, "entries")
	printHeader()
	for i := range l.n {
		printEntry(&l.entries[i])
	}
	println("log end")
}

func printHeader() {
	println("log ms x_mm y_mm z_mm vx_mms vy_mms roll_ddeg pitch_ddeg yaw_ddeg thrust range_mm accz_mms2 bias_mms2 quality mode xy_valid z_valid landing flow_x flow_y gyro_x gyro_y meas_vx meas_vy flow_flags stick_roll stick_pitch stick_yaw stick_throttle stick_climb vz_mms z_sp_mm battery_mv trim_x trim_y trim_z check")
}

// printEntry ends each line with the sum of its values, counting true as 1, since USB serial
// output now and then loses a character.
func printEntry(e *logEntry) {
	sum := int64(e.ms) + int64(e.x) + int64(e.y) + int64(e.z) + int64(e.vx) + int64(e.vy) + int64(e.roll) +
		int64(e.pitch) + int64(e.yaw) + int64(e.thrust) + int64(e.rng) + int64(e.accZ) + int64(e.bias) +
		int64(e.quality) + int64(e.mode) + b2i(e.xyValid) + b2i(e.zValid) + b2i(e.landing) + int64(e.flowX) +
		int64(e.flowY) + int64(e.gyroX) + int64(e.gyroY) + int64(e.measVX) + int64(e.measVY) + int64(e.flowFlags) +
		int64(e.stickRoll) + int64(e.stickPitch) + int64(e.stickYaw) + int64(e.stickThr) + int64(e.stickClimb) +
		int64(e.vz) + int64(e.zSp) + int64(e.battery) + int64(e.trimX) + int64(e.trimY) + int64(e.trimZ)
	println("log", e.ms, e.x, e.y, e.z, e.vx, e.vy, e.roll, e.pitch, e.yaw, e.thrust, e.rng, e.accZ, e.bias, e.quality, e.mode,
		e.xyValid, e.zValid, e.landing, e.flowX, e.flowY, e.gyroX, e.gyroY, e.measVX, e.measVY, e.flowFlags,
		e.stickRoll, e.stickPitch, e.stickYaw, e.stickThr, e.stickClimb, e.vz, e.zSp, e.battery, e.trimX, e.trimY, e.trimZ, sum)
}

func b2i(b bool) int64 {
	if b {
		return 1
	}
	return 0
}

func milli16(v float32) int16 { return int16(fmath.Clamp(v*1000, -32767, 32767)) }

func decideg(rad float32) int16 { return int16(fmath.Degrees(rad) * 10) }
