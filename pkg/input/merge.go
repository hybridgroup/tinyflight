package input

import (
	"github.com/hybridgroup/tinyflight/pkg/hal"
	"github.com/hybridgroup/tinyflight/pkg/msg"
)

// Merge reads from several inputs, such as a radio and a serial port. The first one with new input wins.
type Merge []hal.Input

// ReadInput fills in from the first input that has new data and reports whether any did.
func (m Merge) ReadInput(in *msg.ManualInput) bool {
	for _, src := range m {
		if src != nil && src.ReadInput(in) {
			return true
		}
	}
	return false
}
