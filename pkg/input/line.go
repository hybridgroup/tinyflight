package input

import (
	"errors"

	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

var ErrBadLine = errors.New("input: bad line")

// ParseLine reads a bench control line such as "rc 0.1 -0.2 0 0.4 1 1".
// Fields are roll, pitch, yaw, throttle, arm (0 or 1) and mode (0 acro, 1 stabilize, 2 althold, 3 poshold).
func ParseLine(line []byte, now msg.Time, out *msg.ManualInput) error {
	var v [6]float32
	rest, ok := expect(line, "rc")
	if !ok {
		return ErrBadLine
	}
	for i := range v {
		v[i], rest, ok = number(rest)
		if !ok {
			return ErrBadLine
		}
	}
	out.Time = now
	out.Roll = fmath.Clamp(v[0], -1, 1)
	out.Pitch = fmath.Clamp(v[1], -1, 1)
	out.Yaw = fmath.Clamp(v[2], -1, 1)
	out.Throttle = fmath.Clamp(v[3], 0, 1)
	out.Climb = fmath.Deadband(out.Throttle*2-1, 0.1)
	out.Arm = v[4] > 0.5
	out.Mode = msg.FlightMode(fmath.Clamp(v[5], 0, float32(msg.ModePosHold)))
	out.Valid = true
	return nil
}

func skipSpace(b []byte) []byte {
	for len(b) > 0 && (b[0] == ' ' || b[0] == '\t' || b[0] == '\r' || b[0] == '\n') {
		b = b[1:]
	}
	return b
}

func expect(b []byte, word string) ([]byte, bool) {
	b = skipSpace(b)
	if len(b) < len(word) || string(b[:len(word)]) != word {
		return b, false
	}
	return b[len(word):], true
}

// number parses a plain decimal like -0.25 without allocating.
func number(b []byte) (float32, []byte, bool) {
	b = skipSpace(b)
	neg := false
	if len(b) > 0 && (b[0] == '-' || b[0] == '+') {
		neg = b[0] == '-'
		b = b[1:]
	}
	var v float32
	digits := 0
	for len(b) > 0 && b[0] >= '0' && b[0] <= '9' {
		v = v*10 + float32(b[0]-'0')
		b = b[1:]
		digits++
	}
	if len(b) > 0 && b[0] == '.' {
		b = b[1:]
		scale := float32(0.1)
		for len(b) > 0 && b[0] >= '0' && b[0] <= '9' {
			v += float32(b[0]-'0') * scale
			scale /= 10
			b = b[1:]
			digits++
		}
	}
	if digits == 0 {
		return 0, b, false
	}
	if neg {
		v = -v
	}
	return v, b, true
}
