//go:build tinygo && (esp32s3 || esp32c3 || esp32)

package groundstation

import (
	"sync"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"tinygo.org/x/espradio"
)

// Start brings up the radio on espnowinput.Channel and returns a station that uses it.
func Start() (*Station, error) {
	if err := espradio.Enable(espradio.Config{}); err != nil {
		return nil, err
	}
	if err := espradio.Start(); err != nil {
		return nil, err
	}
	if err := espradio.SetChannel(espnowinput.Channel()); err != nil {
		return nil, err
	}
	if err := espradio.ESPNowInit(); err != nil {
		return nil, err
	}
	s := NewStation(&radio{channel: espnowinput.Channel()})
	espradio.ESPNowSetReceiveHandler(func(r espradio.ESPNowReceive) { s.Receive(r.SourceAddress, r.Data) })
	return s, nil
}

// radio sends over ESP-NOW, adding each address as a peer the first time.
type radio struct {
	mu      sync.Mutex
	channel uint8
}

func (r *radio) Send(addr [6]byte, data []byte) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if !espradio.ESPNowPeerExists(addr) {
		err := espradio.ESPNowAddPeer(espradio.ESPNowPeer{Address: addr, Channel: r.channel, If: espradio.WiFiInterfaceSTA})
		if err != nil {
			return err
		}
	}
	return espradio.ESPNowSend(&addr, data)
}
