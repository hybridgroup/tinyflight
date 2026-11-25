package tinyflight

import (
	"fmt"
	"testing"
)

func TestVersionNumbers(t *testing.T) {
	v := VersionNumbers()
	if got := fmt.Sprintf("%d.%d.%d", v[0], v[1], v[2]); got != Version() {
		t.Errorf("numbers give %q, want %q", got, Version())
	}
}
