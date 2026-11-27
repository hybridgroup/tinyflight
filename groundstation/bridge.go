package groundstation

import (
	"bufio"
	"io"
	"sync"

	"github.com/hybridgroup/tinyflight/pkg/bridge"
)

// StartBridge returns a station that flies drones through firmware/radio-bridge on port, such as
// a USB serial port. Lines that are not packets, like the bridge log, go to logf if it is not nil.
func StartBridge(port io.ReadWriter, logf func(line string)) *Station {
	s := NewStation(&bridgeRadio{port: port})
	go func() {
		var addr [6]byte
		var data [bridge.MaxPayload]byte
		sc := bufio.NewScanner(port)
		for sc.Scan() {
			n, err := bridge.Parse(sc.Bytes(), bridge.Recv, &addr, &data)
			switch {
			case err == nil:
				s.Receive(addr, data[:n])
			case logf != nil && len(sc.Bytes()) > 0:
				logf(sc.Text())
			}
		}
	}()
	return s
}

type bridgeRadio struct {
	mu   sync.Mutex
	port io.Writer
	line []byte
}

func (r *bridgeRadio) Send(addr [6]byte, data []byte) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.line = bridge.Append(r.line[:0], bridge.Send, addr, data)
	_, err := r.port.Write(r.line)
	return err
}
