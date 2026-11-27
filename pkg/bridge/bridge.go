// Package bridge is the USB serial line format of firmware/radio-bridge, which forwards ESP-NOW
// packets between a computer and the drones. See docs/multi-drone.md.
//
//	tx 02544653494d 5446010101002c   computer to bridge, sent to that address
//	rx 02544653494d 5446018201...    bridge to computer, from that address
//
// Each line is a tag, the 6 byte address in hex, and the payload in hex. Other lines are log text.
package bridge

import "errors"

// MaxPayload is the ESP-NOW payload limit, ESP_NOW_MAX_DATA_LEN in esp_now.h of ESP-IDF.
const MaxPayload = 250

// Line tags.
const (
	Send = "tx"
	Recv = "rx"
)

var (
	ErrNotPacket = errors.New("bridge: not a packet line")
	ErrBadLine   = errors.New("bridge: bad packet line")
)

const digits = "0123456789abcdef"

// Append adds a line with tag, addr and data to b, ending in a newline.
func Append(b []byte, tag string, addr [6]byte, data []byte) []byte {
	b = append(b, tag...)
	b = append(b, ' ')
	b = appendHex(b, addr[:])
	b = append(b, ' ')
	b = appendHex(b, data)
	return append(b, '\n')
}

func appendHex(b, data []byte) []byte {
	for _, v := range data {
		b = append(b, digits[v>>4], digits[v&15])
	}
	return b
}

// Parse reads a line with tag into addr and data, and returns the payload length.
// A trailing newline or carriage return is allowed. It does not allocate.
func Parse(line []byte, tag string, addr *[6]byte, data *[MaxPayload]byte) (int, error) {
	for len(line) > 0 && (line[len(line)-1] == '\n' || line[len(line)-1] == '\r') {
		line = line[:len(line)-1]
	}
	if len(line) < len(tag)+1 || string(line[:len(tag)]) != tag || line[len(tag)] != ' ' {
		return 0, ErrNotPacket
	}
	line = line[len(tag)+1:]
	if len(line) < 2*len(addr)+1 || line[2*len(addr)] != ' ' {
		return 0, ErrBadLine
	}
	if !decodeHex(addr[:], line[:2*len(addr)]) {
		return 0, ErrBadLine
	}
	line = line[2*len(addr)+1:]
	n := len(line) / 2
	if n == 0 || len(line)%2 != 0 || n > MaxPayload || !decodeHex(data[:n], line) {
		return 0, ErrBadLine
	}
	return n, nil
}

func decodeHex(dst, src []byte) bool {
	for i := range dst {
		hi, ok1 := fromHex(src[2*i])
		lo, ok2 := fromHex(src[2*i+1])
		if !ok1 || !ok2 {
			return false
		}
		dst[i] = hi<<4 | lo
	}
	return true
}

func fromHex(c byte) (byte, bool) {
	switch {
	case c >= '0' && c <= '9':
		return c - '0', true
	case c >= 'a' && c <= 'f':
		return c - 'a' + 10, true
	case c >= 'A' && c <= 'F':
		return c - 'A' + 10, true
	}
	return 0, false
}
