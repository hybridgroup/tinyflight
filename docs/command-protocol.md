# Command protocol design

Status: design, not built yet.

This is a higher level radio protocol so a program can fly a Tinyflight drone with commands such as take off, hover, go right, hover, land. It is modeled on the Parrot Minidrone protocol used by [tinygo-minidrone](https://github.com/hybridgroup/tinygo-minidrone), but runs over ESP-NOW. It shares the radio with the Atom Joystick protocol in [protocol.md](protocol.md), so the same firmware can be flown by either one.

## Goals

- Fly with the Atom Joystick and its stock firmware, exactly as today.
- Or fly from a TinyGo program on an ESP32 ground station, with an API close to tinygo-minidrone.
- No changes to the flight stack. Commands become `msg.ManualInput` in PosHold.
- Keep every existing failsafe, including landing when the link is lost.
- One ground station can fly several drones on the same channel. See [multi-drone.md](multi-drone.md).

## How it fits the flight stack

In PosHold the sticks already ask for a speed, not a tilt. `holdPosition` in `pkg/flight/flight.go` turns roll and pitch into a speed in the body frame of up to `Horizontal.MaxSpeed`, 0.6 m/s. The climb input asks for a climb rate of up to `MaxClimb`, 0.5 m/s. Centered sticks hold position and height. This is the same model as the Minidrone `Pcmd`, so a command such as right at 30% is the roll stick at 0.3 in PosHold.

The drone side turns commands into a `msg.ManualInput` each step. The commander, failsafes and controllers see it like any other pilot input.

## Coexisting with the Atom protocol

All packets use ESP-NOW on the same channel. The drone sorts them by content.

| Starts with | Length | Protocol |
|---|---|---|
| `54 46` "TF" and version 1 | Fixed per message type, 7 to 23 bytes | Command protocol, this page |
| `88 88` | 120 | Atom telemetry, drone to joystick |
| `43 4D` "CM" | Varies | Camera |
| Last 3 bytes of the drone MAC | 25 | Atom control |

No command packet is 25 bytes long, so an Atom control packet can never be read as a command packet, even when the drone MAC happens to end in `54 46 01`.

The pairing broadcast does not change, so the joystick pairs as it does now.

## Ownership

Only one controller flies a drone at a time.

- The first valid packet from either protocol makes its sender the owner.
- Packets from anyone else are ignored, apart from `Hello`.
- Telemetry goes only to the owner, as Atom telemetry for a joystick and as `State` for a ground station.
- Ownership ends when the drone is disarmed and the owner has sent nothing for 2 s.
- The drone keeps sending the pairing broadcast while it has no owner.

This changes one thing for the joystick. Today control packets from a second joystick are also accepted. With ownership they are ignored.

## Packet format

All multi-byte values are little endian.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | `54 46` | "TF" |
| 2 | 1 | Version | 1 |
| 3 | 1 | Message type | See below |
| 4 | 2 | Sequence, uint16 | Counts up by one per packet from that sender |
| 6 | n | Payload | Depends on the type |
| 6 + n | 1 | Checksum | Sum of all bytes before it, modulo 256 |

A packet with the wrong length for its type, or a bad checksum, is dropped.

### Ground station to drone

| Type | Name | Payload | Length |
|---|---|---|---|
| `01` | Hello | none | 7 |
| `02` | TakeOff | Height, uint16, cm. 0 means the default of 50 cm. | 9 |
| `03` | Land | none | 7 |
| `04` | Emergency | none | 7 |
| `05` | Pcmd | Right, forward, yaw, up. Each int8 from -100 to 100. | 11 |

**Hello** may be broadcast or sent to one drone. Every drone that hears it answers with `Info`, owned or not, so a ground station can find drones on the channel.

**Pcmd** is also the heartbeat. Once started, the ground station sends it 20 times per second, with zeros when it wants to hover, as tinygo-minidrone does with `StartPcmd`. If it stops, the commander lands the drone after `HoldTimeout`, 2 s.

**TakeOff, Land and Emergency** are one shot commands. The drone puts the sequence number of the last one it acted on in `State`. The ground station sends the command again every 100 ms until `State` shows it, for up to 1 s.

### Drone to ground station

| Type | Name | Payload | Length |
|---|---|---|---|
| `81` | Info | See below | 18 |
| `82` | State | See below | 23 |

**Info**, the answer to `Hello`:

| Offset | Size | Field |
|---|---|---|
| 6 | 1 | Highest command protocol version the drone speaks |
| 7 | 3 | Firmware version, major, minor, patch |
| 10 | 6 | Drone MAC |
| 16 | 1 | Flags. Bit 0 has an owner, bit 1 position hold available, bit 2 camera fitted. |

**State**, sent 10 times per second to the owner:

| Offset | Size | Field | Notes |
|---|---|---|---|
| 6 | 1 | Flying state | See below |
| 7 | 1 | Failsafe | `msg.Failsafe` value, 0 none |
| 8 | 1 | Flags | Bit 0 armed, bit 1 height valid, bit 2 position valid |
| 9 | 1 | Flight mode | `msg.FlightMode` value |
| 10 | 2 | Battery, uint16, mV | |
| 12 | 2 | Height, int16, cm | Above the takeoff point |
| 14 | 2 | Roll, int16, 0.01° | |
| 16 | 2 | Pitch, int16, 0.01° | |
| 18 | 2 | Yaw, int16, 0.01° | |
| 20 | 2 | Command ack, uint16 | Sequence of the last TakeOff, Land or Emergency acted on |

### Flying states

The values match `FlyingState` in tinygo-minidrone, so programs written for it read the same way.

| Value | State |
|---|---|
| 0 | Landed |
| 1 | Taking off |
| 2 | Hovering |
| 3 | Flying |
| 4 | Landing, from Land or a failsafe |
| 5 | Emergency |

## Drone side

### Turning commands into input

A state machine in `pkg/input/command.go` makes one `msg.ManualInput` per step. Mode is always PosHold, `ArmToggle` is false so `Arm` works like a switch, and `Throttle` is 0 so the arming checks pass.

| State | Arm | Climb | Sticks | Moves to |
|---|---|---|---|---|
| Landed | false | 0 | 0 | Taking off on TakeOff |
| Taking off | true | 0.5 | 0 | Hovering at the target height |
| Hovering | true | 0 | 0 | Flying on a Pcmd that is not all zeros |
| Flying | true | up / 100 | right / 100, -forward / 100, yaw / 100 | Hovering on an all zero Pcmd |
| Landing | true | -1 | 0 | Landed when the drone disarms |
| Emergency | false | 0 | 0 | Landed |

Notes:

- Pitch is negated since a positive pitch stick is nose up, which flies backward.
- Taking off needs the height estimate, which `hal.Input` does not give today. The board passes in a function that reads the height from the flight loop.
- Landing relies on the existing landed check, `LandedDisarm` in `pkg/flight`, which disarms after 2 s on the ground.
- A Pcmd while taking off or landing is used only as the heartbeat.
- If the commander disarms on its own, for example on a tilt failsafe, the state goes to Landed and `State` reports the failsafe.

### Radio

`devices/espnowinput` gets a dispatcher in `receive` that checks for the `TF` header before the Atom decoder, and tracks the owner. It sends `State` instead of Atom telemetry when the owner is a ground station. The command input joins the Atom input with `input.Merge`. With ownership only one of them ever has new input.

## Ground station side

The packet encoding lives in a pure Go package, `pkg/command`, shared by the drone and the ground station and tested with `go test`.

A client package wraps `espradio` with an API close to tinygo-minidrone:

```go
d, err := groundstation.Find(time.Second) // broadcast Hello and take the first Info
d.Start()                                  // start the 20 Hz Pcmd heartbeat
d.TakeOff()
time.Sleep(5 * time.Second)
d.Right(30)
time.Sleep(2 * time.Second)
d.Hover()
time.Sleep(3 * time.Second)
d.Land()
```

| Method | Does |
|---|---|
| `TakeOff()`, `Land()`, `Emergency()` | Send the one shot command and retry until acknowledged |
| `Up`, `Down`, `Forward`, `Backward`, `Right`, `Left`, `Clockwise`, `CounterClockwise` | Set one axis of the next Pcmd, 0 to 100 |
| `Hover()` | Set every axis to 0 |
| `FlyingStateChange(func(state int))` | Called when `State` shows a new flying state |
| `State()` | The last `State` received |

Each `Drone` holds its own peer and heartbeat, so one ground station can fly several drones. ESP-NOW allows about 20 unencrypted peers, and every drone must be on the same Wi-Fi channel.

## Testing

- Encode and decode round trips and bad packet cases in `pkg/command`.
- State machine tests in `pkg/input`, such as Pcmd ignored while taking off, and Emergency from every state.
- Closed loop tests in `boards/sim` that fly take off, right, hover, land on the simulated quad and check the height and position at each step.
- Lost link tests that stop the heartbeat in flight and check the drone lands.

## Plan

1. `pkg/command` packet encoding with tests.
2. Command state machine in `pkg/input` with tests, and closed loop tests in `boards/sim`.
3. Dispatcher and ownership in `devices/espnowinput`, and `State` telemetry.
4. Ground station client package and an example program under `firmware/`.
5. Update [protocol.md](protocol.md) with the final packet tables.

## Open questions

- **Moves by distance.** Tinyflight has a position estimate, so a later `MoveBy(dx, dy, dz)` could fly to a point relative to where the drone is. Version 1 has only timed moves like the Minidrone.
- **Security.** Like the Atom protocol, packets are not authenticated. ESP-NOW encryption would fix that, but lowers the number of peers per ground station.
- **Camera frames.** Camera packets are broadcast with no drone ID, so frames from several drones get mixed. A drone ID in the camera header would fix that.
- **Sim.** The browser sim could accept command packets too, so ground station programs can be tested before real flights.
