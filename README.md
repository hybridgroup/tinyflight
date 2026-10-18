<img alt="tinyflight" src="images/tinyflight-banner.svg" width="400">

# tinyflight

Flight control firmware written in Go and built with [TinyGo](https://tinygo.org). Supports the [M5Stack StampFly](https://docs.m5stack.com/en/app/StampFly_v1.1).

See [docs/architecture.md](docs/architecture.md) for how it works. See [docs/protocol.md](docs/protocol.md) for the radio protocol used with the M5 Atom Joystick.

## Layout

```
pkg/      flight stack, pure Go, runs and tests on a PC
devices/  adapters from TinyGo drivers to pkg/hal
boards/   board setup, such as boards/stampfly
firmware/ firmware and bench tools to flash
docs/     architecture
```

## Testing on a PC

```
go test ./...
```

This includes closed loop tests that fly a simulated quad.

## Building for the StampFly

Some pieces aren't merged upstream yet. Until they are, the build needs a TinyGo built with these PRs, and local checkouts next to this repo.

**TinyGo**, with these PRs merged into `dev`:

| PR | Change |
|---|---|
| [#5919](https://github.com/tinygo-org/tinygo/pull/5919) | cores scheduler wakes sleeping tasks on time |
| [#5920](https://github.com/tinygo-org/tinygo/pull/5920) | ESP32-S3 SPI on any pins |
| [#5921](https://github.com/tinygo-org/tinygo/pull/5921) | `m5stamp-fly` target |
| [#5922](https://github.com/tinygo-org/tinygo/pull/5922) | ESP32 I2C without allocations |

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
| `stampfly` | The flight firmware. For bench tests, send lines over USB serial in the form `rc roll pitch yaw throttle arm mode`. Mode is 0 for acro, 1 for stabilize and 2 for althold. |
