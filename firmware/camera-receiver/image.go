package main

import "github.com/hybridgroup/tinyflight/pkg/input"

// imageSize is the side of the square image on the display, in display pixels.
const imageSize = 64

// minSpan keeps the scale at least 8 °C wide in 0.25 °C units, like Auto in tools/cameraview.
const minSpan = 32

// bayer is a 4 by 4 ordered dither, so a one bit display can show shades.
var bayer = [4][4]uint8{
	{0, 8, 2, 10},
	{12, 4, 14, 6},
	{3, 11, 1, 9},
	{15, 7, 13, 5},
}

// thermalImage holds the newest thermal frame scaled to imageSize by imageSize.
type thermalImage struct {
	cells [imageSize][imageSize]int16
}

// add copies a frame chunk into the image and reports whether it was the last chunk.
func (m *thermalImage) add(f *input.CameraFrame) bool {
	w, h := int(f.Width), int(f.Height)
	if f.Format != input.CameraThermal || w == 0 || h == 0 {
		return false
	}
	start := int(f.Chunk) * input.CameraChunkPixels
	for i, v := range f.Pixels[:min(int(f.Count), input.CameraChunkPixels)] {
		sx, sy := (start+i)%w, (start+i)/w
		if sy >= h {
			break
		}
		for y := sy * imageSize / h; y < (sy+1)*imageSize/h; y++ {
			for x := sx * imageSize / w; x < (sx+1)*imageSize/w; x++ {
				m.cells[y][x] = v
			}
		}
	}
	return f.Chunk+1 >= f.Chunks
}

// span returns the coldest and hottest cells.
func (m *thermalImage) span() (lo, hi int16) {
	lo, hi = m.cells[0][0], m.cells[0][0]
	for y := range m.cells {
		for _, v := range m.cells[y] {
			lo, hi = min(lo, v), max(hi, v)
		}
	}
	return lo, hi
}

// lit reports whether the display pixel at x, y is on, with hot pixels lit.
func (m *thermalImage) lit(x, y int, lo, hi int16) bool {
	if d := int(hi) - int(lo); d < minSpan {
		mid := (int(lo) + int(hi)) / 2
		lo, hi = int16(mid-minSpan/2), int16(mid+minSpan/2)
	}
	level := (int(m.cells[y][x]) - int(lo)) * 16 / (int(hi) - int(lo))
	return level > int(bayer[y%4][x%4])
}

// center returns the middle cell.
func (m *thermalImage) center() int16 {
	return m.cells[imageSize/2][imageSize/2]
}

// appendTemp writes v in 0.25 °C units as degrees with one decimal.
func appendTemp(b []byte, v int16) []byte {
	t := int(v) * 10
	if t < 0 {
		b = append(b, '-')
		t = -t
	}
	t = (t + 2) / 4
	return append(appendInt(b, t/10), '.', byte('0'+t%10))
}

func appendInt(b []byte, n int) []byte {
	if n >= 10 {
		b = appendInt(b, n/10)
	}
	return append(b, byte('0'+n%10))
}
