# ground-takeoff

Ground station example for the [command protocol](../../docs/command-protocol.md). It runs on an ESP32-C3 or ESP32-S3 board, not on the StampFly. It finds a drone, takes off to 50 cm, flies right, hovers, flies back left, hovers, and lands.

```
ground-takeoff --ESP-NOW--> StampFly
```

## Flashing

```
tinygo flash -target xiao-esp32c3 ./firmware/ground-takeoff
```

For another board, use its TinyGo target, such as `m5stamp-s3a`. Build it with the same `ChannelSetting` as the StampFly. See [Radio channel](../../README.md#radio-channel).

## Flying in the sim

The same program flies the drone in [tinyflightsim](../../docs/simulator.md#flying-with-a-ground-station-program). Start the sim and open the page, then from the root of the repo run

```
go run ./firmware/ground-takeoff
```

## Flying

1. Put the StampFly on flat ground with room around it, and power it on.
2. Do not pair a joystick with it. A drone takes commands from one controller at a time.
3. Reset the ground station board. It prints each step and every flying state change over USB serial.

If the ground station stops sending, the drone lands by itself after 2 s.

The flight uses `groundstation`, which has an API like [tinygo-minidrone](https://github.com/hybridgroup/tinygo-minidrone):

```go
station, _ := groundstation.Start()
infos, _ := station.Find(time.Second)
drone := station.Drone(infos[0].MAC)
drone.Start()
drone.TakeOff()
time.Sleep(5 * time.Second)
drone.Right(50)
time.Sleep(2 * time.Second)
drone.Hover()
time.Sleep(3 * time.Second)
drone.Land()
```
