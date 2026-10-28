# stampfly

The Tinyflight flight firmware for the M5Stack StampFly. Fly it with the M5 Atom Joystick over ESP-NOW using the stock joystick firmware, or on the bench with lines over USB serial.

See [docs/joystick.md](../../docs/joystick.md) for how to fly it and [docs/architecture.md](../../docs/architecture.md) for how it works.

## Flashing

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly
```

The cores scheduler puts the flight loop alone on core 1. Core 0 runs the radio, the slow sensors and the printout.

In AltHold and PosHold the drone will not climb past 10 m above where it took off. Set a different limit in meters, or 0 for no limit.

```
tinygo flash -scheduler=cores -target m5stamp-fly -ldflags="-X github.com/hybridgroup/tinyflight/boards/stampfly.MaxHeight=2" ./firmware/stampfly
```

## USB output

At startup it prints the Tinyflight version and the ESP-NOW MAC address. Then every 500 ms it prints the arming state, mode, failsafe, attitude, position, battery, loop timing, motor outputs, radio counts, pilot input and heap use.

```
tinygo monitor
```

## Bench control

Send lines over USB serial in the form

```
rc roll pitch yaw throttle arm mode
```

Roll, pitch and yaw go from -1 to 1 and throttle from 0 to 1. Arm is 0 or 1. Mode is 0 for acro, 1 for stabilize, 2 for althold and 3 for poshold. Remove the propellers before arming on the bench.

## Flight log

Build with the `blackbox` tag to save the last 5 flights to flash. While disarmed it prints the latest saved flight over USB every 10 s.

```
tinygo flash -scheduler=cores -tags blackbox -target m5stamp-fly ./firmware/stampfly
```

See [Reading the flight log](../../docs/joystick.md#reading-the-flight-log) for what each line holds.

## In the browser sim

The same firmware builds to WebAssembly for `tools/tinyflightsim`, where `board_sim.go` replaces the StampFly board, radio and cores.

```
tinygo build -target=wasip1 -tags=tinyflightsim -no-debug -o firmware.wasm ./firmware/stampfly
```

See [tools/tinyflightsim](../../tools/tinyflightsim/README.md).
