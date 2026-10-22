<img alt="tinyflight" src="images/tinyflight-banner.svg" width="400">

# Tinyflight

Flight control firmware written in Go and built with [TinyGo](https://tinygo.org). Supports the [M5Stack StampFly](https://docs.m5stack.com/en/app/StampFly_v1.1).

![m5stamp fly](./images/m5stampfly.gif)

## Features

- Flight modes: Acrobatic, Stabilized, Altitude Hold, and Position Hold
- Fly with the M5 Atom Joystick over ESP-NOW, using its stock firmware
- Arming checks, plus failsafes for lost pilot input, low battery, crashes and IMU failure
- Pure Go flight stack with full set of tests, including closed loop tests on a simulated quad
- tinyflightsim, a browser flight sim that runs the real firmware as WebAssembly, flown with an on-screen Atom Joystick
- Live images from an optional AMG8833 thermal camera, sent over ESP-NOW and shown in the browser
- Bench control with text lines over USB serial
- Optional blackbox flight log saved to flash

## How it works

- 400 Hz flight loop with no heap memory allocations after startup
- Supports multicore on the ESP32-S3, where the flight loop gets a core to itself
- Attitude estimation using Mahony filters
- Height and position estimates from Kalman filters using the barometer, ToF distance sensor and optical flow
- Cascaded position, attitude and rate controllers

See [docs/architecture.md](docs/architecture.md) for architectural details. See [docs/protocol.md](docs/protocol.md) for the radio protocol used with the M5 Atom Joystick. See [docs/joystick.md](docs/joystick.md) for how to fly with it. See [docs/simulator.md](docs/simulator.md) for the browser sim. See [docs/camera.md](docs/camera.md) for streaming images from an optional thermal camera.

## tinyflightsim

Fly the real firmware in a web browser before you fly the drone. See [Flying in the browser sim](#flying-in-the-browser-sim).

![tinyflightsim](images/tinyflightsim.png)

## Cameras

Stream live images from an optional AMG8833 thermal camera while you fly, and watch them in the browser with `tools/cameraview`. See [docs/camera.md](docs/camera.md).

![cameraview](images/cameraview.png)

## Layout

```
pkg/      flight stack written in pure Go with unit tests
devices/  adapters from TinyGo drivers to pkg/hal
boards/   board setup, such as boards/stampfly, and boards/sim for the browser sim
firmware/ firmware and bench tools to flash
tools/    host tools, such as the tinyflightsim browser sim and the cameraview image viewer
docs/     documentation
```

## Running the tests

```
go test ./...
```

This includes closed loop tests that fly a simulated quad.

## Flying in the browser sim

```
cd tools/tinyflightsim
go run .
```

Then open http://localhost:8080. The server builds `firmware/stampfly` with TinyGo to WebAssembly and runs it against a physics model of the StampFly. See [docs/simulator.md](docs/simulator.md).

## Building for the StampFly

The build needs a TinyGo built with this PR merged into `dev`:

| PR | Change |
|---|---|
| [#5934](https://github.com/tinygo-org/tinygo/pull/5934) | machine/esp32s3: add flash BlockDevice support |

Build with the cores scheduler, so the flight loop gets a core to itself:

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-sensors
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-motortest
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-camera
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-cameratest
```

### Height limit

In AltHold and PosHold the StampFly will not climb past 10 m above where it took off. Set a different limit in meters with `-ldflags`, or 0 for no limit:

```
tinygo flash -scheduler=cores -target m5stamp-fly -ldflags="-X github.com/hybridgroup/tinyflight/boards/stampfly.MaxHeight=2" ./firmware/stampfly
```

### Firmware

| Firmware | What it does |
|---|---|
| `stampfly-sensors` | Prints every sensor, the attitude, loop timing and heap use. The motors stay off. |
| `stampfly-motortest` | Spins each motor in turn when you press the button. Remove the propellers first. |
| `stampfly` | The flight firmware. For bench tests, send lines over USB serial in the form `rc roll pitch yaw throttle arm mode`. Mode is 0 for acro, 1 for stabilize, 2 for althold and 3 for poshold. Built with `-tags blackbox`, it saves the last 5 flights to flash and prints the latest over USB every 10 s while disarmed. See [docs/joystick.md](docs/joystick.md#reading-the-flight-log). |
| `stampfly-camera` | The flight firmware plus an AMG88xx thermal camera on the Grove port. It streams frames over ESP-NOW. See [docs/camera.md](docs/camera.md). |
| `stampfly-cameratest` | Bench tool for the thermal camera. The motors stay off and frames go over USB to `tools/cameraview`. See [docs/camera.md](docs/camera.md#testing-on-the-bench). |
| `camera-receiver` | For an ESP32-C3 on the computer, not the StampFly. Prints camera frames heard over ESP-NOW for `tools/cameraview`, and shows them on an SSD1306 display if one is attached. See [docs/camera.md](docs/camera.md). |
