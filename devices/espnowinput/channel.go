package espnowinput

import "strconv"

// DefaultChannel is the Wi-Fi channel the stock joystick firmware uses, CHANNEL in rc.hpp.
const DefaultChannel = 3

// ChannelSetting is the Wi-Fi channel, 1 to 13, empty for DefaultChannel. The joystick finds it when it pairs.
// Set it with -ldflags="-X github.com/hybridgroup/tinyflight/devices/espnowinput.ChannelSetting=13".
var ChannelSetting string

// Channel returns the Wi-Fi channel from ChannelSetting.
func Channel() uint8 { return parseChannel(ChannelSetting) }

func parseChannel(s string) uint8 {
	if c, err := strconv.Atoi(s); err == nil && c >= 1 && c <= 13 {
		return uint8(c)
	}
	return DefaultChannel
}

// overlap is how far in channels a 20 MHz Wi-Fi signal reaches, since channels are 5 MHz apart.
// See https://en.wikipedia.org/wiki/List_of_WLAN_channels#2.4_GHz_(802.11b/g/n/ax/be)
const overlap = 4

// Quietest returns the channel from 1 to 13 with the least traffic counted on it and on the channels
// it overlaps, nearer ones counting more. counts[c] is the traffic on channel c.
func Quietest(counts *[14]uint32) uint8 {
	best, bestScore := uint8(1), uint64(1<<63)
	for c := 1; c <= 13; c++ {
		var score uint64
		for d := -overlap; d <= overlap; d++ {
			if n := c + d; n >= 1 && n <= 13 {
				score += uint64(counts[n]) * uint64(overlap+1-abs(d))
			}
		}
		if score < bestScore {
			best, bestScore = uint8(c), score
		}
	}
	return best
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
