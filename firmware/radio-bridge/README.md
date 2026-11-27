# radio-bridge

Radio bridge for flying several drones from a computer. It runs on an ESP32-C3 or ESP32-S3 board plugged into the computer, not on the StampFly. It forwards ESP-NOW packets between USB serial and the drones, and the program on the computer does the flying, such as [tools/swarm](../../tools/swarm/README.md).

```
swarm --USB serial--> radio-bridge --ESP-NOW--> StampFly 1, StampFly 2, ...
```

See [docs/multi-drone.md](../../docs/multi-drone.md) for the design.

## Flashing

```
tinygo flash -target xiao-esp32c3 ./firmware/radio-bridge
```

For another board, use its TinyGo target, such as `m5stamp-s3a`. Build it with the same `ChannelSetting` as the drones. See [Radio channel](../../README.md#radio-channel).

At startup it prints `radio bridge ready on channel` with the ESP-NOW channel.

## Serial lines

Each packet is one line with a tag, the 6 byte address in hex, and the payload in hex. The format is in `pkg/bridge`.

```
tx ffffffffffff 5446010101009d    computer to bridge, a Hello sent to every drone
rx 02544653494d 5446018101...     bridge to computer, from that address
```

The bridge sends every packet it hears on the channel, from any drone. Any other line is log text. A Go program can use `groundstation.StartBridge` with an open serial port to get a `Station` that flies through the bridge.

ESP-NOW allows about 20 unicast peers, so one bridge can fly up to about 20 drones.
