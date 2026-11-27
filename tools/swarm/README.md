# swarm

Flies several Tinyflight drones together from a computer, through [firmware/radio-bridge](../../firmware/radio-bridge/README.md) on USB serial. Each drone takes off, turns clockwise and back in place, and lands. See [docs/multi-drone.md](../../docs/multi-drone.md).

```
swarm --USB serial--> radio-bridge --ESP-NOW--> StampFly 1, StampFly 2, ...
```

## Running

Flash the radio bridge with the same `ChannelSetting` as the drones and plug it in. Then from this directory run

```
go run . -port /dev/ttyACM0
```

| Flag | Default | Does |
|---|---|---|
| `-port` | `/dev/ttyACM0` | Serial port of the radio bridge |
| `-count` | 0 | Number of drones to wait for. 0 flies all it finds in about 3 s. |
| `-height` | 50 | Takeoff height in cm |
| `-sim` | false | Fly the drone in [tinyflightsim](../../docs/simulator.md#flying-with-a-ground-station-program) instead of through a radio bridge |

Drones that already have an owner, such as one paired with a joystick, are skipped.

## Flying

1. Put the drones on flat ground at least 1 m apart, and power them on.
2. Do not pair a joystick with any of them.
3. Run swarm. It prints each drone it finds and every flying state change.

Ctrl+C lands every drone. If swarm or the bridge stops sending, each drone lands by itself after 2 s.

Every drone holds its own position, but nothing keeps them apart. Moves that are the same for all drones, like turning in place, keep their spacing.
