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

- An AMG8833 breakout, such as the Adafruit board.
- A small 3.3 V regulator that runs from 5 V, such as a Pololu D24V5F3 or an MCP1700-3302E with a 1 µF capacitor on each side. The sensor draws only a few mA.
- An ESP32-C3 board, such as a Seeed XIAO ESP32-C3. A computer cannot hear ESP-NOW on its own.

## Wiring

The sensor goes on the StampFly Grove port. It has its own I2C bus on GPIO13 SDA and GPIO15 SCL, so the camera never shares a bus with the flight sensors.

The Grove port gives 5 V, with 4.7 kΩ pull-ups on SDA and SCL to 3.3 V, from J3 in the StampFly schematic. The Adafruit board pulls SDA and SCL up to its Vin, so with Vin on 5 V the lines would sit near 3.8 V. That is over the 3.6 V limit of the ESP32-S3 pins, from section 5.1 of the ESP32-S3 datasheet. A regulator puts Vin on 3.3 V instead.

| From | To |
|---|---|
| Grove red, 5 V | Regulator IN |
| Grove black, GND | Regulator GND and Adafruit GND |
| Regulator OUT, 3.3 V | Adafruit Vin |
| Grove yellow, SCL | Adafruit SCL |
| Grove white, SDA | Adafruit SDA |

Before connecting SDA and SCL to the StampFly, power it up and check that regulator OUT and the Adafruit SDA and SCL pins all read about 3.3 V.

Point the sensor forward or down and fix it so it cannot touch a propeller.

At power on the firmware looks for the sensor at I2C address 0x69, then 0x68. If it finds one, it prints `camera found` over USB. If not, it prints `no camera` and flies without it.

## Flashing

The drone needs the camera flight firmware. It flies like `firmware/stampfly` and also sends the frames. See the [README](../README.md#building-for-the-stampfly).

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-camera
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

![cameraview](../images/cameraview.png)

The page shows the image, the coldest, hottest and center temperatures, and the frame rate. Hover over the image to read any spot. Range picks the color scale, and Smooth blends between the 8 by 8 pixels.

## Testing on the bench

To check the camera without flying, flash the bench firmware. It keeps the motors off, does not use the radio, and prints the frames over USB itself, so you need no joystick or receiver.

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-cameratest
```

Leave the StampFly plugged in and point the viewer at its port.

```
cd tools/cameraview
go run . -port /dev/ttyACM0
```

Then open http://localhost:8081. A hand in front of the sensor shows as a warm spot. If the sensor is not found, the StampFly prints `no camera` every few seconds.

## How it works

- The drone reads the sensor every 100 ms on core 0. The flight loop on core 1 is not touched.
- Frames are only sent once the joystick has paired, since the joystick checks every packet while pairing. After that it ignores camera packets.
- Each frame is one 139 byte broadcast, about 1.4 kB/s. The `cam_sent` and `cam_failed` counts in the USB printout show how many went out.
- The receiver only listens and never sends.
- The receiver prints each frame as a line `cam seq format width height chunk chunks count pixels... sum`. The viewer drops any line where the sum does not match.
