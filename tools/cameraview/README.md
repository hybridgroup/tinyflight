# cameraview

Shows camera frames from a StampFly live in the browser. It reads the lines that `firmware/camera-receiver` or `firmware/stampfly-cameratest` print over USB serial and pushes each frame to the page.

```
StampFly --ESP-NOW--> camera-receiver --USB serial--> cameraview --> browser
```

See [docs/camera.md](../../docs/camera.md) for the hardware and firmware setup.

![cameraview](../../images/cameraview.png)

## Running

```
cd tools/cameraview
go run . -port /dev/ttyACM0
```

Then open http://localhost:8081. On macOS the port looks like `/dev/cu.usbmodem1101`, and on Windows like `COM3`.

| Flag | Default | Description |
| --- | --- | --- |
| `-port` | `/dev/ttyACM0` | Serial port of the camera receiver |
| `-addr` | `localhost:8081` | Address to serve the page on |

If the board is unplugged, cameraview keeps trying to open the port every second. Serial lines that are not camera frames are printed to the log.

## The page

The image fills as much of the window as it can and resizes with it. On narrow screens the color scale and readings move below the image.

- The readings show the coldest, hottest and center temperatures, the frame number and the frame rate.
- Hover over the image to read any spot.
- Range picks the color scale. Auto keeps the scale at least 8 °C wide, so an empty scene does not show the sensor noise.
- Smooth blends between the sensor pixels.
- Flip vertical and Flip horizontal match how the camera is mounted, and the browser remembers them.
- The status shows `live`, `no frames` when frames stop for 1.5 s, or `server gone` when cameraview stops.

## How it works

Each frame arrives as one or more lines in the form

```
cam seq format width height chunk chunks count pixels... sum
```

where `sum` is the sum of all the numbers before it. Lines with a bad sum are dropped. The chunks of a frame are joined, and a chunk from a new frame drops any frame left unfinished. Only the thermal format is shown, with each pixel in steps of 0.25 °C. See [docs/protocol.md](../../docs/protocol.md#camera-packet-drone-to-ground-receiver).

Whole frames go to every open page as server sent events on `/events`. A slow page skips frames rather than holding up the others. The page files in `web` are embedded in the binary.

## Tests

```
cd tools/cameraview
go test ./...
```
