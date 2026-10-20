//go:build m5stamp_fly && blackbox

package main

import (
	"machine"
	"unsafe"
)

// Saved flights go in fixed slots counted down from the end of the usable flash, which stays put when
// the program size changes. Each slot is a header followed by the flight log and the height trace. The
// header is written last, so a slot is only valid once complete.
const (
	slotSize    = 256 * 1024
	maxSlots    = 5
	slotMagic   = 0x544c4654
	slotVersion = 2
	chunkSize   = 4096

	// The ESP32-S3 ROM flash routines refuse to erase at or past 2 MB, since nothing tells the ROM the
	// real chip size without the ESP-IDF bootloader. See https://github.com/hybridgroup/tinyflight/pull/1
	romFlashLimit = 0x200000
	flashDROM     = 0x3C000000 // flash is mapped 1:1 here, as in machine_esp32xx_flash.go
)

// Kinds of saved slot.
const (
	slotFlight = 1 + iota // a whole flight, saved after landing
	slotTail              // the RTC tail of a flight that ended in a reset
)

type slotHeader struct {
	magic     uint32
	version   uint32
	flight    uint32 // counts up with each save
	kind      uint32
	logN      uint32
	logSize   uint32
	traceN    uint32
	traceSize uint32
	traceTrig uint32 // µs after arming when the height trace was triggered, 0 if it was not
}

// errArmed stops a save, since writing to flash pauses both cores and so the flight loop.
type errArmed struct{}

func (errArmed) Error() string { return "armed during save" }

type storage struct {
	slots  int
	latest int // slot of the last save, -1 when none
	flight uint32
	head   slotHeader // header of the latest slot
}

var store storage

// open finds the latest saved flight.
func (s *storage) open() {
	s.slots = int(min(usableFlash()/slotSize, maxSlots))
	s.latest = -1
	var h slotHeader
	for i := range s.slots {
		if !s.readHeader(i, &h) {
			continue
		}
		if s.latest < 0 || h.flight > s.flight {
			s.latest, s.flight, s.head = i, h.flight, h
		}
	}
}

// usableFlash is the size of the flash data area below romFlashLimit.
func usableFlash() int64 {
	return min(machine.Flash.Size(), romFlashLimit-int64(machine.FlashDataStart()-flashDROM))
}

// slotOffset counts slots back from the end of the usable flash.
func slotOffset(slot int) int64 { return usableFlash() - int64(slot+1)*slotSize }

func (s *storage) readHeader(slot int, h *slotHeader) bool {
	_, err := machine.Flash.ReadAt(bytesOf(h), slotOffset(slot))
	return err == nil && h.magic == slotMagic && h.version == slotVersion &&
		h.logSize == uint32(unsafe.Sizeof(logEntry{})) && h.traceSize == uint32(unsafe.Sizeof(traceEntry{}))
}

// save writes a new slot. armed is checked before every flash operation, and the save stops if it is true.
func (s *storage) save(kind uint32, log []logEntry, tr []traceEntry, trig uint32, armed func() bool) error {
	if s.slots == 0 {
		return nil
	}
	slot := (s.latest + 1) % s.slots
	base := slotOffset(slot)
	logBytes := sliceBytes(log)
	traceBytes := sliceBytes(tr)
	h := slotHeader{
		magic: slotMagic, version: slotVersion, flight: s.flight + 1, kind: kind,
		logN: uint32(len(log)), logSize: uint32(unsafe.Sizeof(logEntry{})),
		traceN: uint32(len(tr)), traceSize: uint32(unsafe.Sizeof(traceEntry{})), traceTrig: trig,
	}
	hsize := int64(unsafe.Sizeof(h))
	total := hsize + int64(len(logBytes)+len(traceBytes))
	if total > slotSize {
		return nil
	}
	eb := machine.Flash.EraseBlockSize()
	for b := int64(0); b*eb < total; b++ {
		if armed() {
			return errArmed{}
		}
		if err := machine.Flash.EraseBlocks(base/eb+b, 1); err != nil {
			return err
		}
	}
	if err := writeChunks(base+hsize, logBytes, armed); err != nil {
		return err
	}
	if err := writeChunks(base+hsize+int64(len(logBytes)), traceBytes, armed); err != nil {
		return err
	}
	if armed() {
		return errArmed{}
	}
	if _, err := machine.Flash.WriteAt(bytesOf(&h), base); err != nil {
		return err
	}
	s.latest, s.flight, s.head = slot, h.flight, h
	return nil
}

func writeChunks(off int64, p []byte, armed func() bool) error {
	for len(p) > 0 {
		if armed() {
			return errArmed{}
		}
		n := min(len(p), chunkSize)
		if _, err := machine.Flash.WriteAt(p[:n], off); err != nil {
			return err
		}
		off += int64(n)
		p = p[n:]
	}
	return nil
}

// print writes the latest saved slot, reading one entry at a time.
func (s *storage) print() {
	if s.latest < 0 {
		return
	}
	h := &s.head
	base := slotOffset(s.latest) + int64(unsafe.Sizeof(*h))
	switch h.kind {
	case slotTail:
		println("saved log start", h.logN, "entries, flight", h.flight, "from before a reset")
	default:
		println("saved log start", h.logN, "entries, flight", h.flight)
	}
	printHeader()
	var e logEntry
	for i := range int64(h.logN) {
		machine.Flash.ReadAt(bytesOf(&e), base+i*int64(h.logSize))
		printEntry(&e)
	}
	println("saved log end")
	if h.traceN == 0 {
		return
	}
	base += int64(h.logN) * int64(h.logSize)
	if h.traceTrig != 0 {
		println("saved trace start", h.traceN, "entries, height error over", traceTrigger, "m at us", h.traceTrig)
	} else {
		println("saved trace start", h.traceN, "entries, ending with the flight")
	}
	printTraceHeader()
	var t traceEntry
	for i := range int64(h.traceN) {
		machine.Flash.ReadAt(bytesOf(&t), base+i*int64(h.traceSize))
		printTraceEntry(&t)
	}
	println("saved trace end")
}

func bytesOf[T any](v *T) []byte {
	return unsafe.Slice((*byte)(unsafe.Pointer(v)), unsafe.Sizeof(*v))
}

func sliceBytes[T any](s []T) []byte {
	if len(s) == 0 {
		return nil
	}
	return unsafe.Slice((*byte)(unsafe.Pointer(&s[0])), uintptr(len(s))*unsafe.Sizeof(s[0]))
}
