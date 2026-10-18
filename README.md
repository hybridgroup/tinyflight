# tinyflight

Flight control firmware written in Go and built with [TinyGo](https://tinygo.org). The first supported board is the [M5Stack StampFly](https://docs.m5stack.com/en/app/StampFly_v1.1).

See [docs/architecture.md](docs/architecture.md) for how it works.

## Layout

```
pkg/      flight stack, pure Go, runs and tests on a PC
devices/  adapters from TinyGo drivers to pkg/hal
boards/   board setup, such as boards/stampfly
cmd/      firmware and bench tools
docs/     architecture
```

## Testing on a PC

```
go test ./...
```

This includes closed loop tests that fly a simulated quad.

## Building for the StampFly

Some pieces aren't merged upstream yet:

- The `m5stamp-fly` target is on the `m5stamp-fly` branch of TinyGo.
- The sensor drivers are open PRs in tinygo-org/drivers. `go.mod` points at a local `../drivers-tinyflight` checkout that merges them.

```
tinygo flash -target m5stamp-fly ./cmd/stampfly-sensors
tinygo flash -target m5stamp-fly ./cmd/stampfly-motortest
tinygo flash -target m5stamp-fly ./cmd/stampfly
```

### Commands

| Command | What it does |
|---|---|
| `stampfly-sensors` | Prints every sensor, the attitude, loop timing and heap use. The motors stay off. |
| `stampfly-motortest` | Spins each motor in turn when you press the button. Remove the propellers first. |
| `stampfly` | The flight firmware. For bench tests, send lines over USB serial in the form `rc roll pitch yaw throttle arm mode`. Mode is 0 for acro, 1 for stabilize and 2 for althold. |
