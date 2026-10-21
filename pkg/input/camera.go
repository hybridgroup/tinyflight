package input

import (
	"encoding/binary"
	"errors"
	"strconv"
)

var ErrNotCamera = errors.New("input: not a camera packet")

// CameraPacketMax is the ESP-NOW payload limit, ESP_NOW_MAX_DATA_LEN in esp_now.h of ESP-IDF.
const CameraPacketMax = 250

const (
	cameraHeaderSize = 10
	// CameraChunkPixels is how many pixels fit in one packet after the header and checksum.
	CameraChunkPixels = (CameraPacketMax - cameraHeaderSize - 1) / 2
)

// Camera pixel formats.
const (
	// CameraThermal pixels are int16 in 0.25 °C units, the AMG88xx pixel register resolution.
	CameraThermal = 1
)

// The header starts "CM". The joystick only reads packets starting 88 88, from OnDataRecv in
// https://github.com/m5stack/Atom-JoyStick/blob/main/examples/StampFlyController/src/main.cpp
var cameraTag = [2]byte{'C', 'M'}

// CameraFrame is one chunk of a camera image. A frame with more than CameraChunkPixels pixels
// is sent as several chunks, and chunk n holds the pixels from n*CameraChunkPixels on.
type CameraFrame struct {
	Seq           uint16
	Format        uint8
	Width, Height uint8
	Chunk, Chunks uint8
	Count         uint8
	Pixels        [CameraChunkPixels]int16
}

// Encode fills p with the packet and returns its length.
func (f *CameraFrame) Encode(p *[CameraPacketMax]byte) int {
	n := min(int(f.Count), CameraChunkPixels)
	p[0], p[1] = cameraTag[0], cameraTag[1]
	binary.LittleEndian.PutUint16(p[2:], f.Seq)
	p[4], p[5], p[6], p[7], p[8], p[9] = f.Format, f.Width, f.Height, f.Chunk, f.Chunks, uint8(n)
	for i, v := range f.Pixels[:n] {
		binary.LittleEndian.PutUint16(p[cameraHeaderSize+2*i:], uint16(v))
	}
	end := cameraHeaderSize + 2*n
	p[end] = checksum(p[:end])
	return end + 1
}

// DecodeCamera reads a camera packet into f.
func DecodeCamera(data []byte, f *CameraFrame) error {
	if len(data) < cameraHeaderSize+1 {
		return ErrShort
	}
	if data[0] != cameraTag[0] || data[1] != cameraTag[1] {
		return ErrNotCamera
	}
	n := int(data[9])
	end := cameraHeaderSize + 2*n
	if n > CameraChunkPixels || len(data) < end+1 {
		return ErrShort
	}
	if checksum(data[:end]) != data[end] {
		return ErrChecksum
	}
	f.Seq = binary.LittleEndian.Uint16(data[2:])
	f.Format, f.Width, f.Height, f.Chunk, f.Chunks, f.Count = data[4], data[5], data[6], data[7], data[8], uint8(n)
	for i := range n {
		f.Pixels[i] = int16(binary.LittleEndian.Uint16(data[cameraHeaderSize+2*i:]))
	}
	return nil
}

// AppendCameraLine writes "cam seq format width height chunk chunks count pixels... sum" for
// tools/cameraview, where sum adds every value before it so the viewer can drop damaged lines.
func AppendCameraLine(b []byte, f *CameraFrame) []byte {
	b = append(b, "cam"...)
	var sum int64
	put := func(v int64) {
		b = append(b, ' ')
		b = strconv.AppendInt(b, v, 10)
		sum += v
	}
	put(int64(f.Seq))
	put(int64(f.Format))
	put(int64(f.Width))
	put(int64(f.Height))
	put(int64(f.Chunk))
	put(int64(f.Chunks))
	put(int64(f.Count))
	for _, p := range f.Pixels[:min(int(f.Count), CameraChunkPixels)] {
		put(int64(p))
	}
	b = append(b, ' ')
	b = strconv.AppendInt(b, sum, 10)
	return append(b, '\n')
}

func checksum(b []byte) byte {
	var sum byte
	for _, v := range b {
		sum += v
	}
	return sum
}
