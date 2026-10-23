package input

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/msg"
)

type fixedInput struct {
	ok   bool
	roll float32
}

func (f *fixedInput) ReadInput(in *msg.ManualInput) bool {
	if f.ok {
		in.Roll = f.roll
	}
	return f.ok
}

func TestMergeFirstWins(t *testing.T) {
	var in msg.ManualInput
	m := Merge{nil, &fixedInput{}, &fixedInput{ok: true, roll: 0.5}, &fixedInput{ok: true, roll: -0.5}}
	if !m.ReadInput(&in) || in.Roll != 0.5 {
		t.Errorf("got %v, want roll 0.5", in.Roll)
	}
	if (Merge{&fixedInput{}}).ReadInput(&in) {
		t.Error("reported input when none had any")
	}
}
