package groundstation

import (
	"sync"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/command"
)

// Drone is one drone flown by a Station.
type Drone struct {
	station *Station
	addr    [6]byte
	mu      sync.Mutex
	pcmd    command.Pcmd
	state   command.State
	heard   bool
	stop    chan struct{}
	onState func(flying int)
}

// Address returns the drone's MAC address.
func (d *Drone) Address() [6]byte { return d.addr }

// Start sends the Pcmd heartbeat until Halt. Without it the drone lands after a few seconds.
func (d *Drone) Start() {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.stop != nil {
		return
	}
	d.stop = make(chan struct{})
	go d.heartbeat(d.stop)
}

// Halt stops the heartbeat. A flying drone treats this as a lost link and lands.
func (d *Drone) Halt() {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.stop != nil {
		close(d.stop)
		d.stop = nil
	}
}

func (d *Drone) heartbeat(stop chan struct{}) {
	t := time.NewTicker(Heartbeat)
	defer t.Stop()
	for {
		select {
		case <-stop:
			return
		case <-t.C:
			d.mu.Lock()
			p := command.Packet{Type: command.TypePcmd, Pcmd: d.pcmd}
			d.mu.Unlock()
			d.station.send(d.addr, &p)
		}
	}
}

// TakeOff climbs to the default height of 50 cm and hovers.
func (d *Drone) TakeOff() error { return d.TakeOffTo(0) }

// TakeOffTo climbs to height in cm and hovers.
func (d *Drone) TakeOffTo(height uint16) error {
	return d.command(command.Packet{Type: command.TypeTakeOff, Height: height})
}

// Land descends until the drone lands and stops its motors.
func (d *Drone) Land() error { return d.command(command.Packet{Type: command.TypeLand}) }

// Emergency stops the motors at once, even in the air.
func (d *Drone) Emergency() error { return d.command(command.Packet{Type: command.TypeEmergency}) }

// command clears any movement, then sends p until the drone reports it in State, for up to RetryFor.
func (d *Drone) command(p command.Packet) error {
	d.Hover()
	until := time.Now().Add(RetryFor)
	if err := d.station.send(d.addr, &p); err != nil {
		return err
	}
	seq := p.Seq
	for time.Now().Before(until) {
		time.Sleep(Retry)
		if d.acked(seq) {
			return nil
		}
		// Retries keep the first sequence number, so the drone acts only once.
		var buf [command.MaxSize]byte
		p.Seq = seq
		d.station.radio.Send(d.addr, buf[:p.Encode(&buf)])
	}
	if d.acked(seq) {
		return nil
	}
	return ErrNoAck
}

func (d *Drone) acked(seq uint16) bool {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.heard && int16(d.state.Ack-seq) >= 0
}

func (d *Drone) setState(s command.State) {
	d.mu.Lock()
	changed := !d.heard || s.Flying != d.state.Flying
	d.state, d.heard = s, true
	f := d.onState
	d.mu.Unlock()
	if changed && f != nil {
		f(int(s.Flying))
	}
}

// State returns the last State the drone sent, and false if none has arrived.
func (d *Drone) State() (command.State, bool) {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.state, d.heard
}

// FlyingStateChange calls handler with the new flying state each time it changes.
// It runs on the radio receive path, so keep it short.
func (d *Drone) FlyingStateChange(handler func(flying int)) {
	d.mu.Lock()
	d.onState = handler
	d.mu.Unlock()
}

// Up climbs at val from 0 to 100 percent of the top speed.
func (d *Drone) Up(val int) error { return d.set(&d.pcmd.Up, val) }

// Down descends at val from 0 to 100 percent.
func (d *Drone) Down(val int) error { return d.set(&d.pcmd.Up, -val) }

// Forward flies forward at val from 0 to 100 percent.
func (d *Drone) Forward(val int) error { return d.set(&d.pcmd.Forward, val) }

// Backward flies backward at val from 0 to 100 percent.
func (d *Drone) Backward(val int) error { return d.set(&d.pcmd.Forward, -val) }

// Right flies right at val from 0 to 100 percent.
func (d *Drone) Right(val int) error { return d.set(&d.pcmd.Right, val) }

// Left flies left at val from 0 to 100 percent.
func (d *Drone) Left(val int) error { return d.set(&d.pcmd.Right, -val) }

// Clockwise turns clockwise at val from 0 to 100 percent.
func (d *Drone) Clockwise(val int) error { return d.set(&d.pcmd.Yaw, val) }

// CounterClockwise turns counter clockwise at val from 0 to 100 percent.
func (d *Drone) CounterClockwise(val int) error { return d.set(&d.pcmd.Yaw, -val) }

// Hover stops all movement and holds position.
func (d *Drone) Hover() error {
	d.mu.Lock()
	d.pcmd = command.Pcmd{}
	d.mu.Unlock()
	return nil
}

func (d *Drone) set(axis *int8, val int) error {
	d.mu.Lock()
	*axis = int8(min(max(val, -100), 100))
	d.mu.Unlock()
	return nil
}

// FlyingState returns the name of a flying state, as in tinygo-minidrone.
func FlyingState(flying int) string {
	switch flying {
	case command.FlyingLanded:
		return "landed"
	case command.FlyingTakingOff:
		return "takeoff"
	case command.FlyingHovering:
		return "hovering"
	case command.FlyingFlying:
		return "flying"
	case command.FlyingLanding:
		return "landing"
	case command.FlyingEmergency:
		return "emergency"
	}
	return "unknown"
}
