# Radio protocol

Tinyflight talks to the M5 Atom Joystick with the stock StampFly firmware over ESP-NOW. This page describes what goes over the air, so you can write another controller or a ground station.

The protocol comes from these sources:

- The drone side: [`rc.cpp`, `rc.hpp`, `telemetry.cpp` and `flight_control.hpp`](https://github.com/m5stack/M5StampFly/tree/main/src) in the M5StampFly firmware.
- The joystick side: [`main.cpp` and `app_lvgl.cpp`](https://github.com/m5stack/Atom-JoyStick/tree/main/examples/StampFlyController/src) in the Atom-JoyStick StampFlyController.

All multi-byte values are little endian. Floats are IEEE 754 float32.

## Radio settings

- **Transport:** ESP-NOW, unencrypted, on the station interface.
- **Wi-Fi channel:** 3, which is `CHANNEL` in `rc.hpp`.

## Pairing

The joystick does not know the drone's MAC address until they pair.

1. The drone broadcasts a pairing packet to `FF:FF:FF:FF:FF:FF` every 50 ms.
2. In pairing mode, the joystick switches through channels 1 to 14 until it hears that packet. It then saves the channel and the drone's MAC address.
3. From then on the joystick sends control packets to that MAC address on that channel.

The stock drone firmware sends the pairing packet only 50 times, at boot. Tinyflight keeps sending it until the first valid control packet arrives, so the joystick can pair at any time without resetting the drone.

### Pairing packet, drone to joystick, 11 bytes

| Offset | Size | Value |
|---|---|---|
| 0 | 1 | Wi-Fi channel, 3 |
| 1 | 6 | Drone MAC address |
| 7 | 4 | `AA 55 16 88` |

## Control packet, joystick to drone, 25 bytes

About 100 per second.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 3 | Last 3 bytes of the drone MAC | Packets for other drones are ignored |
| 3 | 4 | Yaw stick, float | -1 to 1 |
| 7 | 4 | Throttle stick, float | -1 to 1, center is 0 |
| 11 | 4 | Roll stick, float | -1 to 1 |
| 15 | 4 | Pitch stick, float | -1 to 1 |
| 19 | 1 | Arm button | 1 for about 20 packets after a press |
| 20 | 1 | Flip button | not used by Tinyflight |
| 21 | 1 | Control mode | 0 angle, 1 rate |
| 22 | 1 | Altitude mode | 4 altitude hold, 5 manual |
| 23 | 1 | Proactive flag | not used by Tinyflight |
| 24 | 1 | Checksum | Sum of bytes 0 to 23, modulo 256 |

### How Tinyflight reads it

This is done by `pkg/input/atom.go`.

**Sticks:**

| Stick | Tinyflight input |
|---|---|
| Roll, pitch, yaw | Roll, pitch and yaw, clamped to -1 to 1 |
| Throttle above 0.2 | Throttle, rescaled from 0 at 0.2 to 1 at full stick. Below 0.2 the throttle is 0, the same deadband as the stock firmware. |
| Throttle, whole range | Also gives the climb rate request for AltHold and PosHold, with a 0.2 deadband around center |

**Arming:** each time the arm button is released, arming toggles. The commander still runs its own arming checks.

**Modes:** control mode 1 gives Acro, otherwise Stabilize. Altitude mode 4 gives AltHold with control mode 0, and PosHold with control mode 1. On the joystick the Mode button toggles altitude mode and the Option button toggles control mode, from `check_alt_mode_change` and `check_control_mode_change` in [StampFlyController](https://github.com/m5stack/Atom-JoyStick/blob/main/examples/StampFlyController/src/main.cpp).

## Telemetry packet, drone to joystick, 120 bytes

Tinyflight broadcasts it 10 times per second once a joystick has paired. The joystick shows the drone attitude, battery and altitude. In altitude hold mode it also needs the flight mode, or its arm button does nothing.

**Header:** bytes 0 and 1 are both 88.

**Values:** the rest is a list of 4 byte slots, numbered from 1, starting at byte 2. Slot `n` starts at byte `2 + 4*(n-1)`. Tinyflight only fills the slots the joystick reads and leaves the rest at 0.

| Slot | Offset | Field | Tinyflight value |
|---|---|---|---|
| 3 | 10 | Roll, float, degrees | Attitude estimate |
| 4 | 14 | Pitch, float, degrees | Attitude estimate |
| 5 | 18 | Yaw, float, degrees | Attitude estimate |
| 15 | 58 | Battery voltage, float | INA3221 reading |
| 25 | 98 | Altitude, float, m | Height above the takeoff point |
| 28, byte 1 | 110 | Altitude flag | 1 when armed in AltHold or PosHold |
| 28, byte 2 | 111 | Flight mode | See below |
| 28, bytes 3 and 4 | 112 | Front ToF distance, uint16, mm | 0 for now |

The other slots in the stock firmware carry rates, setpoints, accelerometer readings and motor duty for logging. Tinyflight leaves them at 0.

### Flight mode values

These come from `flight_control.hpp`. Tinyflight sends only three of them.

| Value | Stock name | Tinyflight state |
|---|---|---|
| 2 | `FLIGHT_MODE` | Armed |
| 3 | `PARKING_MODE` | Disarmed |
| 5 | `AUTO_LANDING_MODE` | Failsafe landing |

The stock firmware also defines 0 init, 1 averaging, 4 log and 6 flip.
