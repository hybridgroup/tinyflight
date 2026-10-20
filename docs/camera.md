# Thermal camera

Tinyflight can stream low resolution images from an optional AMG8833 thermal sensor while you fly with the M5 Atom Joystick. The drone sends each frame over ESP-NOW. A second ESP32-C3 board plugged into the computer hears them, and `tools/cameraview` shows them live in the browser.

```
AMG8833 --Grove I2C--> StampFly --ESP-NOW--> ESP32-C3 --USB serial--> cameraview --> browser
                          ^
                          |  ESP-NOW
                     Atom Joystick
```

The AMG8833 gives 8 by 8 pixels from 0 to 80 °C at 10 frames per second. The packet and viewer are not tied to it, so a better camera can use them later. See [protocol.md](protocol.md#camera-packet-drone-to-ground-receiver).

## What you need

- An AMG8833 breakout with a Grove cable, such as the Seeed Grove or Adafruit board. The Grove port gives 5 V, so the breakout needs its own 3.3 V regulator. Most have one.
- An ESP32-C3 board, such as a Seeed XIAO ESP32-C3. A computer cannot hear ESP-NOW on its own.

## Wiring

Plug the sensor into the StampFly Grove port. It has its own I2C bus on GPIO13 SDA and GPIO15 SCL, so the camera never shares a bus with the flight sensors. Point the sensor forward or down and fix it so it cannot touch a propeller.

At power on the firmware looks for the sensor at I2C address 0x69, then 0x68. If it finds one, it prints `camera found` over USB. If not, the drone flies as before.

## Flashing

The drone needs the normal flight firmware. See the [README](../README.md#building-for-the-stampfly).

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly
```

Then flash the receiver onto the ESP32-C3. For another ESP32-C3 board, use its TinyGo target.

```
tinygo flash -target xiao-esp32c3 ./firmware/camera-receiver
```

## Viewing

Plug the receiver into the computer and run

```
cd tools/cameraview
go run . -port /dev/ttyACM0
```

Then open http://localhost:8081. On macOS the port looks like `/dev/cu.usbmodem1101`, and on Windows like `COM3`.

The page shows the image, the coldest, hottest and center temperatures, and the frame rate. Hover over the image to read any spot. Range picks the color scale, and Smooth blends between the 8 by 8 pixels.

## How it works

- The drone reads the sensor every 100 ms on core 0. The flight loop on core 1 is not touched.
- Frames are only sent once the joystick has paired, since the joystick checks every packet while pairing. After that it ignores camera packets.
- Each frame is one 139 byte broadcast, about 1.4 kB/s. The `cam_sent` count in the USB printout shows how many went out.
- The receiver only listens and never sends.
- The receiver prints each frame as a line `cam seq format width height chunk chunks count pixels... sum`. The viewer drops any line where the sum does not match.
