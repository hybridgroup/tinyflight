package groundstation

import (
	"bufio"
	"io"
	"testing"
	"time"

	"github.com/hybridgroup/tinyflight/pkg/bridge"
	"github.com/hybridgroup/tinyflight/pkg/command"
)

// pipePort is the computer end of a serial port.
type pipePort struct {
	io.Reader
	io.Writer
}

func TestBridge(t *testing.T) {
	bridgeIn, toBridge := io.Pipe()
	stationIn, fromBridge := io.Pipe()
	logs := make(chan string, 4)
	s := StartBridge(pipePort{stationIn, toBridge}, func(l string) { logs <- l })

	// The fake bridge answers a Hello with Info from droneMAC, after a log line.
	go func() {
		var addr [6]byte
		var data [bridge.MaxPayload]byte
		sc := bufio.NewScanner(bridgeIn)
		for sc.Scan() {
			n, err := bridge.Parse(sc.Bytes(), bridge.Send, &addr, &data)
			if err != nil || addr != Broadcast {
				t.Errorf("bridge got %q", sc.Text())
				continue
			}
			var p command.Packet
			if command.Decode(data[:n], &p) != nil || p.Type != command.TypeHello {
				t.Errorf("bridge got packet %+v", p)
				continue
			}
			reply := command.Packet{Type: command.TypeInfo, Info: command.Info{Protocol: 1, MAC: droneMAC}}
			var buf [command.MaxSize]byte
			io.WriteString(fromBridge, "radio bridge ready\n")
			fromBridge.Write(bridge.Append(nil, bridge.Recv, droneMAC, buf[:reply.Encode(&buf)]))
		}
	}()

	infos, err := s.Find(time.Second)
	if err != nil || len(infos) != 1 || infos[0].MAC != droneMAC {
		t.Fatalf("found %+v, %v", infos, err)
	}
	select {
	case l := <-logs:
		if l != "radio bridge ready" {
			t.Errorf("log %q", l)
		}
	case <-time.After(time.Second):
		t.Error("no log line")
	}
	toBridge.Close()
	fromBridge.Close()
}
