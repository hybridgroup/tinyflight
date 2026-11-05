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

// The stock firmware announces 50 times at 50 ms only at boot. This keeps going until a joystick answers.
const announceEvery = 50 * time.Millisecond

// announceTag ends the pairing packet, peer_command in rc.cpp.
var announceTag = [4]byte{0xaa, 0x55, 0x16, 0x88}

var _ hal.Input = (*Input)(nil)

type Input struct {
	mac      [6]byte
	channel  uint8
	dec      *input.AtomJoystick
	mu       sync.Mutex
	latest   msg.ManualInput
	fresh    bool
	paired   atomic.Bool
	joyAddr  [6]byte // joystick MAC, written once before paired is set
	Packets  atomic.Uint32
	Rejected atomic.Uint32
	// Sent counts telemetry packets queued. SendFailed counts those that failed to queue or the joystick did not acknowledge.
	Sent         atomic.Uint32
	SendFailed   atomic.Uint32
	CameraSent   atomic.Uint32
	CameraFailed atomic.Uint32
}

// Start brings up the radio on Channel, listens for the joystick, and announces this vehicle until it pairs.
func Start() (*Input, error) {
	ch := Channel()
	if err := espradio.Enable(espradio.Config{}); err != nil {
		return nil, err
	}
	if err := espradio.Start(); err != nil {
		return nil, err
	}
	if err := espradio.SetChannel(ch); err != nil {
		return nil, err
	}
	mac, err := espradio.StationMAC()
	if err != nil {
		return nil, err
	}
	if err := espradio.ESPNowInit(); err != nil {
		return nil, err
	}
	in := &Input{mac: mac, dec: input.NewAtomJoystick(mac), channel: ch}
	espradio.ESPNowSetReceiveHandler(in.receive)
	espradio.ESPNowSetSendHandler(in.sent)
	err = espradio.ESPNowAddPeer(espradio.ESPNowPeer{
		Address: [6]byte(espradio.ESPNowBroadcastAddr),
		Channel: ch,
		If:      espradio.WiFiInterfaceSTA,
	})
	if err != nil {
		return nil, err
	}
	go in.announce()
	return in, nil
}

// RunTelemetry sends telemetry to the joystick forever so it can show the vehicle state.
// fill is called before each send. It runs on core 0 and does not allocate.
// It goes to the joystick's own address, as telemetry_send in rc.cpp does, since a broadcast gets no
// acknowledgement or retries and is easily lost on a busy channel.
func (in *Input) RunTelemetry(every time.Duration, fill func(t *input.AtomTelemetry)) {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var t input.AtomTelemetry
	var pkt [input.AtomTelemetrySize]byte
	var addr [6]byte
	added := false
	for {
		time.Sleep(every)
		if !in.paired.Load() {
			continue
		}
		if !added {
			addr = in.joyAddr
			err := espradio.ESPNowAddPeer(espradio.ESPNowPeer{Address: addr, Channel: in.channel, If: espradio.WiFiInterfaceSTA})
			if err != nil {
				in.SendFailed.Add(1)
				continue
			}
			added = true
		}
		fill(&t)
		t.Encode(&pkt)
		if espradio.ESPNowSend(&addr, pkt[:]) != nil {
			in.SendFailed.Add(1)
		} else {
			in.Sent.Add(1)
		}
	}
}

// RunCamera broadcasts camera frames forever once a joystick has paired. read fills the next
// frame and reports false when there is none. It runs on core 0 and does not allocate.
func (in *Input) RunCamera(every time.Duration, read func(f *input.CameraFrame) bool) {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var f input.CameraFrame
	var pkt [input.CameraPacketMax]byte
	addr := [6]byte(espradio.ESPNowBroadcastAddr)
	for {
		time.Sleep(every)
		if !in.paired.Load() || !read(&f) {
			continue
		}
		n := f.Encode(&pkt)
		if espradio.ESPNowSend(&addr, pkt[:n]) != nil {
			in.CameraFailed.Add(1)
		} else {
			in.CameraSent.Add(1)
		}
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
	pkt[0] = in.channel
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
		if !in.paired.Load() {
			in.joyAddr = r.SourceAddress
			in.paired.Store(true)
		}
		in.Packets.Add(1)
	} else {
		in.Rejected.Add(1)
	}
}

// sent counts telemetry the joystick did not acknowledge. Broadcasts are never acknowledged, so they are skipped.
func (in *Input) sent(r espradio.ESPNowSendReport) {
	if r.DestinationAddress != [6]byte(espradio.ESPNowBroadcastAddr) && r.Status != espradio.ESPNowSendSuccess {
		in.SendFailed.Add(1)
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
