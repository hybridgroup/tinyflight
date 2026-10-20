<img alt="tinyflight" src="images/tinyflight-banner.svg" width="400">

# Tinyflight

Flight control firmware written in Go and built with [TinyGo](https://tinygo.org). Supports the [M5Stack StampFly](https://docs.m5stack.com/en/app/StampFly_v1.1).

![m5stamp fly](./images/m5stampfly.gif)

## Features

- Flight modes: Acrobatic, Stabilized, Altitude Hold, and Position Hold
- Fly with the M5 Atom Joystick over ESP-NOW, using its stock firmware
- Arming checks, plus failsafes for lost pilot input, low battery, crashes and IMU failure
- Pure Go flight stack with full set of tests, including closed loop tests on a simulated quad
- Bench control with text lines over USB serial
- Optional blackbox flight log saved to flash

## How it works

- 400 Hz flight loop with no heap memory allocations after startup
- Supports multicore on the ESP32-S3, where the flight loop gets a core to itself
- Attitude estimation using Mahony filters
- Height and position estimates from Kalman filters using the barometer, ToF distance sensor and optical flow
- Cascaded position, attitude and rate controllers

See [docs/architecture.md](docs/architecture.md) for architectural details. See [docs/protocol.md](docs/protocol.md) for the radio protocol used with the M5 Atom Joystick. See [docs/joystick.md](docs/joystick.md) for how to fly with it.

## Layout

```
pkg/      flight stack written in pure Go with unit tests
devices/  adapters from TinyGo drivers to pkg/hal
boards/   board setup, such as boards/stampfly
firmware/ firmware and bench tools to flash
docs/     documentation
```

## Running the tests

```
go test ./...
```

This includes closed loop tests that fly a simulated quad.

## Building for the StampFly

Some pieces aren't merged upstream yet. Until they are, the build needs a TinyGo built with these PRs, and local checkouts next to this repo.

**TinyGo**, with these PRs merged into `dev`:

| PR | Change |
|---|---|
| [#5934](https://github.com/tinygo-org/tinygo/pull/5934) | machine/esp32s3: add flash BlockDevice support |

**`../drivers-tinyflight`**, a tinygo-org/drivers checkout with these PR branches merged:

| PR | Driver |
|---|---|
| [#912](https://github.com/tinygo-org/drivers/pull/912) | vl53l3cx |
| [#913](https://github.com/tinygo-org/drivers/pull/913) | bmm150 |
| [#915](https://github.com/tinygo-org/drivers/pull/915) | bmp280 |
| [#916](https://github.com/tinygo-org/drivers/pull/916) | ina3221 |
| [#917](https://github.com/tinygo-org/drivers/pull/917) | bmi270 |
| [#918](https://github.com/tinygo-org/drivers/pull/918) | pmw3901 |

**`../espradio`**, a tinygo-org/espradio checkout of the `espnow-noalloc` branch from [#102](https://github.com/tinygo-org/espradio/pull/102), which builds on [#101](https://github.com/tinygo-org/espradio/pull/101).

`go.mod` points at the two local checkouts with `replace` lines. They go away once the PRs merge.

Build with the cores scheduler, so the flight loop gets a core to itself:

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-sensors
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-motortest
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly
```

### Firmware

| Firmware | What it does |
|---|---|
| `stampfly-sensors` | Prints every sensor, the attitude, loop timing and heap use. The motors stay off. |
| `stampfly-motortest` | Spins each motor in turn when you press the button. Remove the propellers first. |
| `stampfly` | The flight firmware. For bench tests, send lines over USB serial in the form `rc roll pitch yaw throttle arm mode`. Mode is 0 for acro, 1 for stabilize, 2 for althold and 3 for poshold. Built with `-tags blackbox`, it saves the last 5 flights to flash and prints the latest over USB every 10 s while disarmed. See [docs/joystick.md](docs/joystick.md#reading-the-flight-log). |
