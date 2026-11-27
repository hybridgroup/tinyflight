package main

import (
	"sync"
	"testing"
	"time"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/groundstation"
	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var stationMAC = [6]byte{0x10, 0, 0, 0, 0, 1}

type vehicle struct{}

func (vehicle) Status() msg.VehicleStatus   { return msg.VehicleStatus{} }
func (vehicle) Position() msg.LocalPosition { return msg.LocalPosition{} }

// air carries packets between the station and several drones running the real espnowinput.Link.
type air struct {
	mu      sync.Mutex
	station *groundstation.Station
	links   map[[6]byte]*espnowinput.Link
	start   time.Time
}

func (a *air) Send(addr [6]byte, data []byte) error {
	a.mu.Lock()
	defer a.mu.Unlock()
	for mac, l := range a.links {
		if addr != mac && addr != groundstation.Broadcast {
			continue
		}
		l.Receive(stationMAC, data, time.Since(a.start))
		var reply command.Packet
		if src, ok := l.TakeHello(); ok && src == stationMAC {
			reply = command.Packet{Type: command.TypeInfo, Info: command.Info{Protocol: 1, MAC: mac}}
		} else if owner, kind := l.Owner(); kind == espnowinput.OwnerGround && owner == stationMAC {
			reply = command.Packet{Type: command.TypeState}
			l.FillState(&reply.State)
		} else {
			continue
		}
		var buf [command.MaxSize]byte
		go a.station.Receive(mac, buf[:reply.Encode(&buf)])
	}
	return nil
}

func TestFindAndTakeOffTogether(t *testing.T) {
	a := &air{links: map[[6]byte]*espnowinput.Link{}, start: time.Now()}
	for i := range 3 {
		mac := [6]byte{0x20, 0, 0, 0, 0, byte(i)}
		l := espnowinput.NewLink(mac)
		l.SetVehicle(vehicle{})
		a.links[mac] = l
	}
	a.station = groundstation.NewStation(a)

	infos := find(a.station, 3)
	if len(infos) != 3 {
		t.Fatalf("found %d drones, want 3", len(infos))
	}
	var drones []*groundstation.Drone
	for _, info := range infos {
		drones = append(drones, a.station.Drone(info.MAC))
	}
	if err := each(drones, func(d *groundstation.Drone) error { return d.TakeOff() }); err != nil {
		t.Fatal(err)
	}
	for _, d := range drones {
		if s, ok := d.State(); !ok || s.Flying != command.FlyingTakingOff {
			t.Errorf("%s state %+v", mac(d.Address()), s)
		}
	}
}
