# camera-receiver

Ground receiver for the thermal camera. It runs on an ESP32-C3 or ESP32-S3 board plugged into the computer, not on the StampFly. It listens for camera frames from [firmware/stampfly-camera](../stampfly-camera/README.md) over ESP-NOW and prints each one over USB serial for [tools/cameraview](../../tools/cameraview/README.md). It never sends, so it cannot disturb the joystick.

```
StampFly --ESP-NOW--> camera-receiver --USB serial--> cameraview --> browser
```

See [docs/camera.md](../../docs/camera.md) for the full setup.

## Flashing

```
tinygo flash -target xiao-esp32c3 ./firmware/camera-receiver
```

For another board, use its TinyGo target, such as `m5stamp-s3a`.

At startup it prints `camera receiver ready on channel` with the ESP-NOW channel.

## Display

If an SSD1306 128 by 64 OLED is on the board's default I2C pins at address 0x3C or 0x3D, the receiver prints `display found` and shows the image on it. The left half shows the image with hot areas lit, and the right half shows the hottest, center and coldest temperatures in °C. On the XIAO the pins are D4 SDA and D5 SCL, and the Seeed Studio XIAO expansion board works. The `m5stamp-c3` target has no I2C in TinyGo, so it runs without the display.

To flip the display image to match how the camera is mounted, set either flip when you flash. Set both to turn it upside down.

```
tinygo flash -target xiao-esp32c3 -ldflags="-X main.FlipVertical=true -X main.FlipHorizontal=true" ./firmware/camera-receiver
```

## Output

Each frame is printed as a line in the form

```
cam seq format width height chunk chunks count pixels... sum
```

See [docs/protocol.md](../../docs/protocol.md#camera-packet-drone-to-ground-receiver).

## Tests

The image scaling for the display has host tests.

```
go test ./firmware/camera-receiver
```
