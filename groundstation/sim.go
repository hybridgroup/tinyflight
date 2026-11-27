//go:build !tinygo

package groundstation

import (
	"errors"
	"net"
	"time"
)

// SimAddr is the tinyflightsim server that Start connects to on a computer. See docs/simulator.md.
var SimAddr = "localhost:8080"

// Start connects to the drone in tinyflightsim at SimAddr and returns a station that flies it.
// Each UDP datagram is a 6 byte address and an ESP-NOW payload, the destination going out and the source coming in.
func Start() (*Station, error) {
	addr, err := net.ResolveUDPAddr("udp", SimAddr)
	if err != nil {
		return nil, err
	}
	conn, err := net.DialUDP("udp", nil, addr)
	if err != nil {
		return nil, err
	}
	s := NewStation(&udpRadio{conn: conn})
	go func() {
		var buf [256]byte
		for {
			n, err := conn.Read(buf[:])
			if errors.Is(err, net.ErrClosed) {
				return
			}
			if err != nil {
				// Refused until the sim server is up.
				time.Sleep(100 * time.Millisecond)
				continue
			}
			if n > 6 {
				s.Receive([6]byte(buf[:6]), buf[6:n])
			}
		}
	}()
	return s, nil
}

type udpRadio struct{ conn *net.UDPConn }

func (r *udpRadio) Send(addr [6]byte, data []byte) error {
	_, err := r.conn.Write(append(addr[:], data...))
	return err
}
