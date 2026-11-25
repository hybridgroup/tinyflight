// Package groundstation flies Tinyflight drones with the command protocol over ESP-NOW, with an API like
// https://github.com/hybridgroup/tinygo-minidrone. See docs/command-protocol.md.
package groundstation

import (
	"errors"
	"sync"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/command"
)

var (
	ErrNoAck   = errors.New("groundstation: drone did not acknowledge the command")
	ErrNoDrone = errors.New("groundstation: no drone found")
)

// Broadcast is the ESP-NOW broadcast address.
var Broadcast = [6]byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}

// Timing of the protocol, from docs/command-protocol.md.
var (
	Heartbeat = 50 * time.Millisecond
	Retry     = 100 * time.Millisecond
	RetryFor  = time.Second
)

// Radio sends a packet to an address. It must be safe to call from several goroutines.
type Radio interface {
	Send(addr [6]byte, data []byte) error
}

// Station talks to every drone on the channel through one radio.
type Station struct {
	radio  Radio
	mu     sync.Mutex
	seq    uint16
	drones []*Drone
	found  chan command.Info
}

// NewStation returns a station that sends with radio. Feed it received packets with Receive.
func NewStation(radio Radio) *Station {
	return &Station{radio: radio, found: make(chan command.Info, 8)}
}

// Receive handles a packet from src. Call it from the radio receive handler.
func (s *Station) Receive(src [6]byte, data []byte) {
	var p command.Packet
	if command.Decode(data, &p) != nil {
		return
	}
	switch p.Type {
	case command.TypeInfo:
		select {
		case s.found <- p.Info:
		default:
		}
	case command.TypeState:
		if d := s.lookup(src); d != nil {
			d.setState(p.State)
		}
	}
}

// Find broadcasts Hello and returns the drones that answer within wait.
func (s *Station) Find(wait time.Duration) ([]command.Info, error) {
	for len(s.found) > 0 {
		<-s.found
	}
	if err := s.send(Broadcast, &command.Packet{Type: command.TypeHello}); err != nil {
		return nil, err
	}
	var infos []command.Info
	timeout := time.After(wait)
	for {
		select {
		case i := <-s.found:
			infos = append(infos, i)
		case <-timeout:
			if len(infos) == 0 {
				return nil, ErrNoDrone
			}
			return infos, nil
		}
	}
}

// Drone returns the drone at addr, creating it the first time.
func (s *Station) Drone(addr [6]byte) *Drone {
	s.mu.Lock()
	defer s.mu.Unlock()
	for _, d := range s.drones {
		if d.addr == addr {
			return d
		}
	}
	d := &Drone{station: s, addr: addr}
	s.drones = append(s.drones, d)
	return d
}

func (s *Station) lookup(addr [6]byte) *Drone {
	s.mu.Lock()
	defer s.mu.Unlock()
	for _, d := range s.drones {
		if d.addr == addr {
			return d
		}
	}
	return nil
}

// send stamps p with the next sequence number and sends it.
func (s *Station) send(addr [6]byte, p *command.Packet) error {
	s.mu.Lock()
	s.seq++
	p.Seq = s.seq
	s.mu.Unlock()
	var buf [command.MaxSize]byte
	return s.radio.Send(addr, buf[:p.Encode(&buf)])
}
