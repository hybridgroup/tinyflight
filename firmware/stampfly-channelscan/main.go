//go:build tinygo && (esp32s3 || esp32c3 || esp32)

// Counts Wi-Fi traffic on each channel and prints the quietest one to fly on.
package main

import (
	"time"

	"github.com/hybridgroup/tinyflight/devices/espnowinput"
	"tinygo.org/x/espradio"
)

const listen = 300 * time.Millisecond

func main() {
	time.Sleep(2 * time.Second)
	if err := espradio.Enable(espradio.Config{}); err != nil {
		println("radio:", err.Error())
		return
	}
	if err := espradio.Start(); err != nil {
		println("radio:", err.Error())
		return
	}
	var counts [14]uint32
	for {
		for c := uint8(1); c <= 13; c++ {
			n, err := espradio.SniffCountOnChannel(c, listen)
			if err != nil {
				println("channel", c, err.Error())
			}
			counts[c] = n
		}
		println()
		for c := 1; c <= 13; c++ {
			print("channel ", c, "\t", counts[c], "\t")
			for range min(counts[c]/10, 60) {
				print("#")
			}
			println()
		}
		println("quietest channel", espnowinput.Quietest(&counts), "now flying on", espnowinput.Channel())
	}
}
