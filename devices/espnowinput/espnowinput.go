//go:build tinygo && (esp32s3 || esp32c3 || esp32)

// Package espnowinput receives M5 Atom Joystick and command protocol packets over ESP-NOW, and pairs like the stock StampFly firmware.
// Protocol from rc_init, send_peer_info and OnDataRecv in https://github.com/m5stack/M5StampFly/blob/main/src/rc.cpp
package espnowinput

import (
	"sync/atomic"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/command"
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/input"
	"tinygo.org/x/espradio"
)

// The stock firmware announces 50 times at 50 ms only at boot. This keeps going while the drone has no owner.
const announceEvery = 50 * time.Millisecond

// announceTag ends the pairing packet, peer_command in rc.cpp.
var announceTag = [4]byte{0xaa, 0x55, 0x16, 0x88}

var _ hal.Input = (*Input)(nil)

type Input struct {
	*Link
	mac     [6]byte
	channel uint8
	start   time.Time
	// Sent counts telemetry packets queued. SendFailed counts those that failed to queue or the controller did not acknowledge.
	Sent         atomic.Uint32
	SendFailed   atomic.Uint32
	CameraSent   atomic.Uint32
	CameraFailed atomic.Uint32
}

// Start brings up the radio on Channel, listens for a joystick or ground station, and announces this vehicle while it has no owner.
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
	in := &Input{Link: NewLink(mac), mac: mac, channel: ch, start: time.Now()}
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

// RunTelemetry sends telemetry to the owner forever, Atom telemetry to a joystick and State to a ground station.
// It also answers Hello with info. fillAtom and fillState are called before each send.
// It runs on core 0 and does not allocate. Telemetry goes to the owner's own address, as telemetry_send
// in rc.cpp does, since a broadcast gets no acknowledgement or retries and is easily lost on a busy channel.
func (in *Input) RunTelemetry(every time.Duration, fillAtom func(t *input.AtomTelemetry), fillState func(s *command.State), info command.Info) {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var t input.AtomTelemetry
	var pkt [input.AtomTelemetrySize]byte
	var p command.Packet
	var buf [command.MaxSize]byte
	var seq uint16
	info.Protocol = command.Version
	info.MAC = in.mac
	for tick := 0; ; tick++ {
		time.Sleep(every)
		if addr, ok := in.TakeHello(); ok {
			p = command.Packet{Type: command.TypeInfo, Info: info}
			if _, kind := in.Owner(); kind != OwnerNone {
				p.Info.Flags |= command.InfoOwned
			}
			seq++
			p.Seq = seq
			in.send(addr, buf[:p.Encode(&buf)])
		}
		addr, kind := in.Owner()
		switch kind {
		case OwnerJoystick:
			fillAtom(&t)
			t.Encode(&pkt)
			in.send(addr, pkt[:])
		case OwnerGround:
			if tick%2 != 0 {
				continue
			}
			p = command.Packet{Type: command.TypeState}
			fillState(&p.State)
			in.FillState(&p.State)
			seq++
			p.Seq = seq
			in.send(addr, buf[:p.Encode(&buf)])
		}
	}
}

// send adds addr as a peer if needed and sends data to it, counting the result.
func (in *Input) send(addr [6]byte, data []byte) {
	if !espradio.ESPNowPeerExists(addr) {
		err := espradio.ESPNowAddPeer(espradio.ESPNowPeer{Address: addr, Channel: in.channel, If: espradio.WiFiInterfaceSTA})
		if err != nil {
			in.SendFailed.Add(1)
			return
		}
	}
	if espradio.ESPNowSend(&addr, data) != nil {
		in.SendFailed.Add(1)
	} else {
		in.Sent.Add(1)
	}
}

// RunCamera broadcasts camera frames forever while the drone has an owner. read fills the next
// frame and reports false when there is none. It runs on core 0 and does not allocate.
func (in *Input) RunCamera(every time.Duration, read func(f *input.CameraFrame) bool) {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var f input.CameraFrame
	var pkt [input.CameraPacketMax]byte
	addr := [6]byte(espradio.ESPNowBroadcastAddr)
	for {
		time.Sleep(every)
		if !in.Paired() || !read(&f) {
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

// Paired reports whether a controller owns the drone.
func (in *Input) Paired() bool {
	_, kind := in.Owner()
	return kind != OwnerNone
}

// MAC is this vehicle's address. The joystick sends to its last three bytes.
func (in *Input) MAC() [6]byte { return in.mac }

// announce sends the pairing packet the joystick listens for, with the channel, MAC and tag, while the drone has no owner.
func (in *Input) announce() {
	espradio.LockToCore0()
	defer espradio.UnlockFromCore0()
	var pkt [11]byte
	pkt[0] = in.channel
	copy(pkt[1:7], in.mac[:])
	copy(pkt[7:], announceTag[:])
	addr := [6]byte(espradio.ESPNowBroadcastAddr)
	for {
		if !in.Paired() {
			espradio.ESPNowSend(&addr, pkt[:])
		}
		time.Sleep(announceEvery)
	}
}

func (in *Input) receive(r espradio.ESPNowReceive) {
	in.Receive(r.SourceAddress, r.Data, time.Since(in.start))
}

// sent counts telemetry the controller did not acknowledge. Broadcasts are never acknowledged, so they are skipped.
func (in *Input) sent(r espradio.ESPNowSendReport) {
	if r.DestinationAddress != [6]byte(espradio.ESPNowBroadcastAddr) && r.Status != espradio.ESPNowSendSuccess {
		in.SendFailed.Add(1)
	}
}
