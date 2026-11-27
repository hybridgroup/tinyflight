//go:build tinygo && (esp32c3 || esp32s3)

// Radio bridge for the ground. It forwards ESP-NOW packets between USB serial and the drones,
// so a program on a computer can fly several drones at once. See docs/multi-drone.md.
//
//	tinygo flash -target xiao-esp32c3 ./firmware/radio-bridge
package main

import (
	"machine"
	"os"
	"time"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"github.com/hybridgroup/tinyflight/groundstation"
	"github.com/hybridgroup/tinyflight/pkg/bridge"
)

type packet struct {
	addr [6]byte
	n    int
	data [bridge.MaxPayload]byte
}

// received holds packets from the radio callback until the main loop writes them out. A full queue drops new packets.
var received = make(chan packet, 32)

func main() {
	time.Sleep(time.Second)
	radio, err := groundstation.StartRadio(receive)
	for err != nil {
		println("radio bridge:", err.Error())
		time.Sleep(time.Second)
	}
	println("radio bridge ready on channel", espnowinput.Channel())

	var in [2*bridge.MaxPayload + 32]byte
	var out []byte
	var addr [6]byte
	var data [bridge.MaxPayload]byte
	n := 0
	for {
		idle := true
		for machine.Serial.Buffered() > 0 {
			idle = false
			b, err := machine.Serial.ReadByte()
			if err != nil {
				break
			}
			if b != '\n' {
				if n < len(in) {
					in[n] = b
				}
				n++
				continue
			}
			if n <= len(in) {
				if size, err := bridge.Parse(in[:n], bridge.Send, &addr, &data); err == nil {
					if err := radio.Send(addr, data[:size]); err != nil {
						println("send:", err.Error())
					}
				}
			}
			n = 0
		}
	drain:
		for {
			select {
			case p := <-received:
				idle = false
				out = bridge.Append(out[:0], bridge.Recv, p.addr, p.data[:p.n])
				os.Stdout.Write(out)
			default:
				break drain
			}
		}
		if idle {
			time.Sleep(time.Millisecond)
		}
	}
}

func receive(src [6]byte, data []byte) {
	p := packet{addr: src, n: min(len(data), bridge.MaxPayload)}
	copy(p.data[:], data)
	select {
	case received <- p:
	default:
	}
}
