//go:build tinygo && (esp32s3 || esp32c3 || esp32)

// Package espnowinput receives M5 Atom Joystick packets over ESP-NOW and pairs like the stock StampFly firmware.
// Protocol from rc_init, send_peer_info and OnDataRecv in https://github.com/m5stack/M5StampFly/blob/main/src/rc.cpp
package espnowinput

import (
	"sync"
	"sync/atomic"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"github.com/hybridgroup/tinyflight/pkg/msg"
	"tinygo.org/x/espradio"
)

// Channel is the Wi-Fi channel the stock joystick uses, CHANNEL in rc.hpp.
const Channel = 3

// The stock firmware announces 50 times at 50 ms only at boot. This keeps going until a joystick answers.
const announceEvery = 50 * time.Millisecond

// announceTag ends the pairing packet, peer_command in rc.cpp.
var announceTag = [4]byte{0xaa, 0x55, 0x16, 0x88}

var _ hal.Input = (*Input)(nil)

type Input struct {
	mac      [6]byte
	dec      *input.AtomJoystick
	mu       sync.Mutex
	latest   msg.ManualInput
	fresh    bool
	paired   atomic.Bool
	Packets  atomic.Uint32
	Rejected atomic.Uint32
}

// Start brings up the radio on Channel, listens for the joystick, and announces this vehicle until it pairs.
func Start() (*Input, error) {
	if err := espradio.Enable(espradio.Config{}); err != nil {
		return nil, err
	}
	if err := espradio.Start(); err != nil {
		return nil, err
	}
	if err := espradio.SetChannel(Channel); err != nil {
		return nil, err
	}
	mac, err := espradio.StationMAC()
	if err != nil {
		return nil, err
	}
	if err := espradio.ESPNowInit(); err != nil {
		return nil, err
	}
	in := &Input{mac: mac, dec: input.NewAtomJoystick(mac)}
	espradio.ESPNowSetReceiveHandler(in.receive)
	err = espradio.ESPNowAddPeer(espradio.ESPNowPeer{
		Address: [6]byte(espradio.ESPNowBroadcastAddr),
		Channel: Channel,
		If:      espradio.WiFiInterfaceSTA,
	})
	if err != nil {
		return nil, err
	}
	go in.announce()
	return in, nil
}

// RunTelemetry broadcasts telemetry forever so the joystick can show the vehicle state.
// fill is called before each send. It runs on core 0 and does not allocate.
func (in *Input) RunTelemetry(every time.Duration, fill func(t *input.AtomTelemetry)) {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var t input.AtomTelemetry
	var pkt [input.AtomTelemetrySize]byte
	addr := [6]byte(espradio.ESPNowBroadcastAddr)
	for {
		time.Sleep(every)
		if !in.paired.Load() {
			continue
		}
		fill(&t)
		t.Encode(&pkt)
		espradio.ESPNowSend(&addr, pkt[:])
	}
}

// Paired reports whether a joystick has sent a valid packet.
func (in *Input) Paired() bool { return in.paired.Load() }

// MAC is this vehicle's address. The joystick sends to its last three bytes.
func (in *Input) MAC() [6]byte { return in.mac }

// announce sends the pairing packet the joystick listens for, with the channel, MAC and tag.
func (in *Input) announce() {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var pkt [11]byte
	pkt[0] = Channel
	copy(pkt[1:7], in.mac[:])
	copy(pkt[7:], announceTag[:])
	addr := [6]byte(espradio.ESPNowBroadcastAddr)
	for !in.paired.Load() {
		espradio.ESPNowSend(&addr, pkt[:])
		time.Sleep(announceEvery)
	}
}

func (in *Input) receive(r espradio.ESPNowReceive) {
	in.mu.Lock()
	err := in.dec.Decode(r.Data, 0, &in.latest)
	if err == nil {
		in.fresh = true
	}
	in.mu.Unlock()
	if err == nil {
		in.paired.Store(true)
		in.Packets.Add(1)
	} else {
		in.Rejected.Add(1)
	}
}

func (in *Input) ReadInput(out *msg.ManualInput) bool {
	in.mu.Lock()
	ok := in.fresh
	if ok {
		*out = in.latest
		in.fresh = false
	}
	in.mu.Unlock()
	return ok
}
