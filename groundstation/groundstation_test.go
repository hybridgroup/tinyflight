package groundstation

import (
	"sync"
	"testing"
	"time"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var (
	stationMAC = [6]byte{0x10, 0, 0, 0, 0, 1}
	droneMAC   = [6]byte{0x20, 0, 0, 0, 0, 2}
)

type vehicle struct {
	mu     sync.Mutex
	status msg.VehicleStatus
}

func (v *vehicle) Status() msg.VehicleStatus {
	v.mu.Lock()
	defer v.mu.Unlock()
	return v.status
}
func (v *vehicle) Position() msg.LocalPosition { return msg.LocalPosition{} }

// air carries packets between the station and a drone running the real espnowinput.Link.
// The drone answers each packet with Info or State at once, standing in for its telemetry loop.
type air struct {
	station *Station
	link    *espnowinput.Link
	start   time.Time
	mu      sync.Mutex
	sent    []command.Packet
	drop    bool
}

func newAir(t *testing.T) *air {
	a := &air{link: espnowinput.NewLink(droneMAC), start: time.Now()}
	a.link.SetVehicle(&vehicle{})
	a.station = NewStation(a)
	Heartbeat, Retry, RetryFor = 5*time.Millisecond, 5*time.Millisecond, 50*time.Millisecond
	return a
}

func (a *air) Send(addr [6]byte, data []byte) error {
	var p command.Packet
	if err := command.Decode(data, &p); err != nil {
		panic(err)
	}
	a.mu.Lock()
	a.sent = append(a.sent, p)
	drop := a.drop
	a.mu.Unlock()
	if drop || (addr != droneMAC && addr != Broadcast) {
		return nil
	}
	a.link.Receive(stationMAC, data, time.Since(a.start))
	var reply command.Packet
	if src, ok := a.link.TakeHello(); ok && src == stationMAC {
		reply = command.Packet{Type: command.TypeInfo, Info: command.Info{Protocol: 1, MAC: droneMAC}}
	} else if owner, kind := a.link.Owner(); kind == espnowinput.OwnerGround && owner == stationMAC {
		reply = command.Packet{Type: command.TypeState}
		a.link.FillState(&reply.State)
	} else {
		return nil
	}
	var buf [command.MaxSize]byte
	a.station.Receive(droneMAC, buf[:reply.Encode(&buf)])
	return nil
}

func (a *air) count(typ command.Type) int {
	a.mu.Lock()
	defer a.mu.Unlock()
	n := 0
	for _, p := range a.sent {
		if p.Type == typ {
			n++
		}
	}
	return n
}

func TestFindAndFly(t *testing.T) {
	a := newAir(t)
	infos, err := a.station.Find(20 * time.Millisecond)
	if err != nil || len(infos) != 1 || infos[0].MAC != droneMAC {
		t.Fatalf("found %+v, %v", infos, err)
	}
	d := a.station.Drone(infos[0].MAC)
	if a.station.Drone(droneMAC) != d {
		t.Error("second Drone call gave a new drone")
	}
	var mu sync.Mutex
	var states []int
	d.FlyingStateChange(func(s int) {
		mu.Lock()
		states = append(states, s)
		mu.Unlock()
	})
	d.Start()
	defer d.Halt()

	if err := d.TakeOffTo(80); err != nil {
		t.Fatal(err)
	}
	if s, ok := d.State(); !ok || s.Flying != command.FlyingTakingOff {
		t.Errorf("state %+v %v after TakeOff", s, ok)
	}
	mu.Lock()
	if len(states) == 0 || states[len(states)-1] != command.FlyingTakingOff {
		t.Errorf("flying states %v", states)
	}
	mu.Unlock()

	d.Right(150)
	time.Sleep(4 * Heartbeat)
	a.mu.Lock()
	last := a.sent[len(a.sent)-1]
	a.mu.Unlock()
	if last.Type != command.TypePcmd || last.Pcmd != (command.Pcmd{Right: 100}) {
		t.Errorf("last packet %+v, want Pcmd right 100", last)
	}
	d.Left(20)
	d.Up(10)
	d.CounterClockwise(5)
	d.mu.Lock()
	if d.pcmd != (command.Pcmd{Right: -20, Up: 10, Yaw: -5}) {
		t.Errorf("pcmd %+v", d.pcmd)
	}
	d.mu.Unlock()

	if err := d.Land(); err != nil {
		t.Fatal(err)
	}
	d.mu.Lock()
	if d.pcmd != (command.Pcmd{}) {
		t.Errorf("Land left movement %+v", d.pcmd)
	}
	d.mu.Unlock()
	if s, _ := d.State(); s.Flying != command.FlyingLanding {
		t.Errorf("flying state %s after Land", FlyingState(int(s.Flying)))
	}
}

func TestNoAck(t *testing.T) {
	a := newAir(t)
	a.drop = true
	d := a.station.Drone(droneMAC)
	if err := d.TakeOff(); err != ErrNoAck {
		t.Fatalf("got %v, want ErrNoAck", err)
	}
	if n := a.count(command.TypeTakeOff); n < 3 {
		t.Errorf("sent TakeOff %d times, want retries", n)
	}
	a.mu.Lock()
	for _, p := range a.sent {
		if p.Type == command.TypeTakeOff && p.Seq != a.sent[0].Seq {
			t.Errorf("retry with sequence %d, want %d", p.Seq, a.sent[0].Seq)
		}
	}
	a.mu.Unlock()
	if _, err := a.station.Find(10 * time.Millisecond); err != ErrNoDrone {
		t.Errorf("Find gave %v, want ErrNoDrone", err)
	}
}

func TestHaltStopsHeartbeat(t *testing.T) {
	a := newAir(t)
	d := a.station.Drone(droneMAC)
	d.Start()
	d.Start()
	time.Sleep(5 * Heartbeat)
	d.Halt()
	n := a.count(command.TypePcmd)
	if n == 0 {
		t.Fatal("no heartbeat")
	}
	time.Sleep(5 * Heartbeat)
	if a.count(command.TypePcmd) > n+1 {
		t.Error("heartbeat kept going after Halt")
	}
}

func TestFlyingStateNames(t *testing.T) {
	for s, want := range map[int]string{0: "landed", 1: "takeoff", 2: "hovering", 3: "flying", 4: "landing", 5: "emergency", 9: "unknown"} {
		if got := FlyingState(s); got != want {
			t.Errorf("%d: %q, want %q", s, got, want)
		}
	}
}
