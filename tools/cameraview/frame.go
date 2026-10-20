package main

import (
	"errors"
	"strconv"
	"strings"
)

// Pixel formats from pkg/input/camera.go.
const formatThermal = 1

var (
	errNotCamera = errors.New("not a camera line")
	errBadLine   = errors.New("bad camera line")
	errSum       = errors.New("camera line sum does not match")
)

// chunk is one line from firmware/camera-receiver.
type chunk struct {
	seq, format, width, height, index, count int
	pixels                                   []int
}

// parseLine reads "cam seq format width height chunk chunks count pixels... sum".
func parseLine(line string) (chunk, error) {
	fields := strings.Fields(line)
	if len(fields) == 0 || fields[0] != "cam" {
		return chunk{}, errNotCamera
	}
	if len(fields) < 9 {
		return chunk{}, errBadLine
	}
	v := make([]int, len(fields)-1)
	for i, f := range fields[1:] {
		n, err := strconv.Atoi(f)
		if err != nil {
			return chunk{}, errBadLine
		}
		v[i] = n
	}
	sum := 0
	for _, n := range v[:len(v)-1] {
		sum += n
	}
	if sum != v[len(v)-1] {
		return chunk{}, errSum
	}
	c := chunk{seq: v[0], format: v[1], width: v[2], height: v[3], index: v[4], count: v[5]}
	n := v[6]
	if len(v) != 8+n || c.count == 0 || c.index >= c.count {
		return chunk{}, errBadLine
	}
	c.pixels = v[7 : 7+n]
	return c, nil
}

// Frame is a whole image as the web page gets it. Pixels times Scale gives the value in Unit.
type Frame struct {
	Seq    int     `json:"seq"`
	Width  int     `json:"width"`
	Height int     `json:"height"`
	Scale  float64 `json:"scale"`
	Unit   string  `json:"unit"`
	Pixels []int   `json:"pixels"`
}

// chunkPixels matches CameraChunkPixels in pkg/input/camera.go.
const chunkPixels = 119

// assembler joins chunks into frames. A chunk from a new frame drops any frame left unfinished.
type assembler struct {
	frame Frame
	got   []bool
	left  int
}

// add returns the finished frame once its last chunk arrives.
func (a *assembler) add(c chunk) (Frame, bool) {
	if c.format != formatThermal {
		return Frame{}, false
	}
	size := c.width * c.height
	if a.got == nil || c.seq != a.frame.Seq || len(a.frame.Pixels) != size || len(a.got) != c.count {
		a.frame = Frame{Seq: c.seq, Width: c.width, Height: c.height, Scale: 0.25, Unit: "°C", Pixels: make([]int, size)}
		a.got = make([]bool, c.count)
		a.left = c.count
	}
	start := c.index * chunkPixels
	if a.got[c.index] || start+len(c.pixels) > size {
		return Frame{}, false
	}
	copy(a.frame.Pixels[start:], c.pixels)
	a.got[c.index] = true
	a.left--
	if a.left > 0 {
		return Frame{}, false
	}
	f := a.frame
	a.got = nil
	return f, true
}
