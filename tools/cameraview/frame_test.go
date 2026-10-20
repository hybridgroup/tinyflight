package main

import (
	"fmt"
	"strings"
	"testing"
)

func line(seq, index, count int, pixels []int) string {
	v := []int{seq, formatThermal, 8, 8, index, count, len(pixels)}
	v = append(v, pixels...)
	sum := 0
	for _, n := range v {
		sum += n
	}
	var b strings.Builder
	b.WriteString("cam")
	for _, n := range append(v, sum) {
		fmt.Fprintf(&b, " %d", n)
	}
	return b.String()
}

func TestParseLine(t *testing.T) {
	px := make([]int, 64)
	for i := range px {
		px[i] = 80 + i
	}
	c, err := parseLine(line(7, 0, 1, px))
	if err != nil {
		t.Fatal(err)
	}
	if c.seq != 7 || c.width != 8 || len(c.pixels) != 64 || c.pixels[63] != 143 {
		t.Errorf("got %+v", c)
	}
	if _, err := parseLine(line(7, 0, 1, px) + "1"); err != errSum {
		t.Errorf("bad sum gave %v", err)
	}
	if _, err := parseLine("esp-now ready"); err != errNotCamera {
		t.Errorf("other line gave %v", err)
	}
	if _, err := parseLine("cam 1 2 3"); err != errBadLine {
		t.Errorf("short line gave %v", err)
	}
}

func TestAssembler(t *testing.T) {
	var a assembler
	whole := make([]int, 64)
	for i := range whole {
		whole[i] = i
	}
	c, _ := parseLine(line(1, 0, 1, whole))
	f, ok := a.add(c)
	if !ok || f.Seq != 1 || f.Pixels[10] != 10 || f.Scale != 0.25 {
		t.Fatalf("single chunk frame %v %+v", ok, f)
	}

	// Two chunks of a frame with more pixels than fit in one packet.
	big := make([]int, chunkPixels)
	c0 := chunk{seq: 3, format: formatThermal, width: 31, height: 4, index: 0, count: 2, pixels: big}
	c1 := chunk{seq: 3, format: formatThermal, width: 31, height: 4, index: 1, count: 2, pixels: []int{1, 2, 3, 4, 5}}
	if _, ok := a.add(c0); ok {
		t.Error("frame done after the first of two chunks")
	}
	lost := c1
	lost.seq = 2
	if _, ok := a.add(lost); ok {
		t.Error("chunk from another frame finished it")
	}
	if _, ok := a.add(c1); ok {
		t.Error("frame done without its first chunk")
	}
	f, ok = a.add(c0)
	if !ok || f.Pixels[chunkPixels+4] != 5 {
		t.Errorf("two chunk frame %v", ok)
	}
}
