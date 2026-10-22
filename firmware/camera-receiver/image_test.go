package main

import (
	"testing"

	"github.com/hybridgroup/tinyflight/pkg/input"
)

func TestAddScalesFrame(t *testing.T) {
	f := input.CameraFrame{Format: input.CameraThermal, Width: 8, Height: 8, Chunks: 1, Count: 64}
	for i := range 64 {
		f.Pixels[i] = int16(i)
	}
	var m thermalImage
	if !m.add(&f) {
		t.Fatal("single chunk frame not complete")
	}
	if got := m.cells[63][63]; got != 63 {
		t.Errorf("bottom right cell %d, want 63", got)
	}
	if got := m.cells[8][0]; got != 8 {
		t.Errorf("second row cell %d, want 8", got)
	}
	if lo, hi := m.span(); lo != 0 || hi != 63 {
		t.Errorf("span %d %d, want 0 63", lo, hi)
	}
}

func TestAddChunks(t *testing.T) {
	f := input.CameraFrame{Format: input.CameraThermal, Width: 16, Height: 16, Chunks: 3, Count: input.CameraChunkPixels}
	var m thermalImage
	if m.add(&f) {
		t.Error("first of three chunks reported complete")
	}
	f.Chunk, f.Count, f.Pixels[0] = 2, 256-2*input.CameraChunkPixels, 99
	if !m.add(&f) {
		t.Error("last chunk not complete")
	}
	p := 2 * input.CameraChunkPixels
	if got := m.cells[p/16*4][p%16*4]; got != 99 {
		t.Errorf("last chunk cell %d, want 99", got)
	}
}

func TestAddSkipsOtherFormats(t *testing.T) {
	f := input.CameraFrame{Format: 2, Width: 8, Height: 8, Chunks: 1, Count: 1}
	var m thermalImage
	if m.add(&f) {
		t.Error("unknown format added")
	}
}

func TestLit(t *testing.T) {
	var m thermalImage
	m.cells[0][0] = 0
	m.cells[1][1] = 200
	if m.lit(0, 0, 0, 200) {
		t.Error("coldest pixel lit")
	}
	if !m.lit(1, 1, 0, 200) {
		t.Error("hottest pixel dark")
	}
	m.cells[0][1] = 2
	if m.lit(1, 0, 0, 4) {
		t.Error("noise in a calm scene lit up")
	}
}

func TestAppendTemp(t *testing.T) {
	for v, want := range map[int16]string{0: "0.0", 101: "25.3", 100: "25.0", -3: "-0.8", 320: "80.0"} {
		if got := string(appendTemp(nil, v)); got != want {
			t.Errorf("appendTemp(%d) = %s, want %s", v, got, want)
		}
	}
}
