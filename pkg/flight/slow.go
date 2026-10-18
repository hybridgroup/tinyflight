package flight

import (
	"sync"
	"sync/atomic"
	"time"

	altest "github.com/hybridgroup/tinyflight/pkg/estimator/altitude"
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// box hands the latest reading from the sensor goroutine to the flight loop.
type box[T any] struct {
	mu    sync.Mutex
	v     T
	fresh bool
}

func (b *box[T]) put(v *T) {
	b.mu.Lock()
	b.v = *v
	b.fresh = true
	b.mu.Unlock()
}

func (b *box[T]) take(v *T) bool {
	b.mu.Lock()
	ok := b.fresh
	if ok {
		*v = b.v
		b.fresh = false
	}
	b.mu.Unlock()
	return ok
}

// task is one slow sensor read on its own schedule.
// inLoop tasks always run in the flight loop, because they share a bus with the IMU.
type task struct {
	period msg.Time
	due    msg.Time
	inLoop bool
	read   func(f *Flight, now msg.Time) error
}

// slowSensors reads sensors that are slower than the IMU, either inline or from RunSensors.
type slowSensors struct {
	tasks  [5]task
	next   int
	async  atomic.Bool
	errors atomic.Uint32

	// Read buffers, only touched by the goroutine that reads the sensor.
	baroIn  msg.BaroSample
	rangeIn msg.RangeSample
	magIn   msg.MagSample
	battIn  msg.BatteryStatus

	baro box[msg.BaroSample]
	rng  box[msg.RangeSample]
	mag  box[msg.MagSample]
	batt box[msg.BatteryStatus]
}

func (f *Flight) setupSlowSensors() {
	s, hw := f.cfg.Schedule, f.hw
	f.slow.tasks = [5]task{
		newTask(s.BaroHz, hw.Baro != nil, false, (*Flight).readBaro),
		newTask(s.RangeHz, hw.Range != nil, false, (*Flight).readRange),
		newTask(s.FlowHz, hw.Flow != nil, true, (*Flight).readFlow),
		newTask(s.MagHz, hw.Mag != nil, false, (*Flight).readMag),
		newTask(s.BatteryHz, hw.Power != nil, false, (*Flight).readBattery),
	}
}

func newTask(hz float32, present, inLoop bool, read func(*Flight, msg.Time) error) task {
	if hz <= 0 || !present {
		return task{}
	}
	return task{period: msg.Time(1e6 / hz), inLoop: inLoop, read: read}
}

// RunSensors reads the slow sensors forever. Run it in its own goroutine, ideally on another core,
// and the flight loop stops reading them itself. Without it the loop reads one per pass.
func (f *Flight) RunSensors() {
	f.slow.async.Store(true)
	for {
		now := f.now()
		next := now + msg.Time(time.Second/time.Microsecond)
		for i := range f.slow.tasks {
			t := &f.slow.tasks[i]
			if t.read == nil || t.inLoop {
				continue
			}
			if now >= t.due {
				f.runTask(t, now)
			}
			if t.due < next {
				next = t.due
			}
		}
		if wait := time.Duration(next-f.now()) * time.Microsecond; wait > 0 {
			time.Sleep(wait)
		}
	}
}

// runLoopSensors reads at most one due sensor, so no single pass takes too long.
func (f *Flight) runLoopSensors(now msg.Time) {
	async := f.slow.async.Load()
	n := len(f.slow.tasks)
	for i := range n {
		k := (f.slow.next + i) % n
		t := &f.slow.tasks[k]
		// Check ownership first, RunSensors owns the other tasks while it runs.
		if t.read == nil || (async && !t.inLoop) || now < t.due {
			continue
		}
		f.runTask(t, now)
		f.slow.next = k + 1
		return
	}
}

func (f *Flight) runTask(t *task, now msg.Time) {
	t.due += t.period
	if t.due < now {
		t.due = now + t.period
	}
	if err := t.read(f, now); err != nil && err != hal.ErrNotReady {
		f.slow.errors.Add(1)
	}
}

func (f *Flight) readBaro(now msg.Time) error {
	if err := f.hw.Baro.ReadBaro(&f.slow.baroIn); err != nil {
		return err
	}
	f.slow.baroIn.Time = now
	f.slow.baro.put(&f.slow.baroIn)
	return nil
}

func (f *Flight) readRange(now msg.Time) error {
	if err := f.hw.Range.ReadRange(&f.slow.rangeIn); err != nil {
		return err
	}
	f.slow.rangeIn.Time = now
	f.slow.rng.put(&f.slow.rangeIn)
	return nil
}

func (f *Flight) readMag(now msg.Time) error {
	if err := f.hw.Mag.ReadMag(&f.slow.magIn); err != nil {
		return err
	}
	f.slow.magIn.Time = now
	f.slow.mag.put(&f.slow.magIn)
	return nil
}

func (f *Flight) readBattery(now msg.Time) error {
	if err := f.hw.Power.ReadBattery(&f.slow.battIn); err != nil {
		return err
	}
	f.slow.battIn.Time = now
	f.slow.batt.put(&f.slow.battIn)
	return nil
}

// readFlow runs in the flight loop, so it writes straight to the loop's copy.
func (f *Flight) readFlow(now msg.Time) error {
	if err := f.hw.Flow.ReadFlow(&f.flow); err != nil {
		return err
	}
	f.flow.Time = now
	return nil
}

// applySlowSensors feeds any new slow readings to the estimators.
func (f *Flight) applySlowSensors() {
	if f.slow.baro.take(&f.baro) {
		if f.altEst.UpdateBaro(altest.PressureToAltitude(f.baro.Pressure)) {
			f.pos.ZValid = true
		}
	}
	if f.slow.rng.take(&f.rng) && f.rng.Valid {
		cosTilt := f.att.Q.Down().Z
		f.pos.Ground = f.rng.Distance * cosTilt
		if f.altEst.UpdateRange(f.rng.Distance, cosTilt) {
			f.pos.ZValid = true
		}
	}
	if f.slow.mag.take(&f.magRaw) {
		f.imu.Mag(&f.magRaw, &f.magS)
		f.attEst.SetMag(f.magS.Field)
	}
	f.slow.batt.take(&f.batt)
}
