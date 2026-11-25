# Multiple drone design

Status: design, not built yet.

This page covers how one ground station could fly several Tinyflight drones at the same time over ESP-NOW. It builds on the [command protocol](command-protocol.md), and also notes what works with only the [Atom protocol](protocol.md) today.

## What works today

The drone firmware already allows it.

- **Each drone has its own address.** Every Atom control packet starts with the last 3 bytes of the drone MAC, and a drone ignores packets for another one. A ground station can send a separate control stream to each drone.
- **Drones can be found.** A drone with no owner broadcasts its pairing packet, with its full MAC, every 50 ms (`announce` in `devices/espnowinput`). A ground station listens on the channel and collects the MACs.
- **Telemetry comes back per drone.** A drone sends telemetry to the sender of the first control packet it accepts (`joyAddr` in `devices/espnowinput`). The source MAC of each telemetry packet says which drone it came from.

With the command protocol, `Hello` sent as a broadcast gets an `Info` answer from every drone on the channel, and `State` replaces Atom telemetry.

## Ground station

The ground station is an ESP32 running TinyGo with `espradio`. The Atom Joystick and its stock firmware can only pair with one drone.

There are two ways to write the program.

- **All on the ESP32.** The flight program runs on the ground station, with one client per drone from the command protocol client package.
- **Radio bridge.** The ESP32 forwards packets between USB serial and the radio. Each frame on the serial side carries the drone MAC. A program on a computer does the flying. This is the simplest first step and the easiest to debug.

## Limits

| Limit | Why | What to do |
|---|---|---|
| One Wi-Fi channel | One ESP32 radio listens on one channel at a time | Build every drone with the same `ChannelSetting`, or use one ground radio per channel |
| About 20 drones per radio | ESP-NOW allows about 20 unencrypted unicast peers | Use more than one ground radio |
| Airtime | Each drone takes about 100 control and 20 telemetry packets per second with the Atom protocol, or 20 Pcmd and 10 State with the command protocol | Use the command protocol, and run `stampfly-channelscan` to pick a quiet channel |
| Lost packets | Too much traffic on the channel triggers the lost input failsafe | As above |

Send to each drone's own MAC, not to the broadcast address. A broadcast gets no acknowledgement or retries, so it is lost more often on a busy channel.

## Ownership

Today the drone locks telemetry to the first sender, but accepts control packets from anyone who knows its MAC. If a joystick pairs first, the ground station gets no telemetry from that drone, yet its control packets still fly it. That is a safety problem when several controllers share a channel.

The command protocol fixes this with an owner for each drone. Only the owner's packets are used, until the drone is disarmed and the owner has been quiet for 2 s. See [Ownership](command-protocol.md#ownership).

## Camera frames

Camera packets are broadcast with no drone ID, so frames from several camera drones get mixed in `tools/cameraview` and `firmware/camera-receiver`. Adding a drone ID, such as the last 3 bytes of the MAC, to the camera header would fix that. This changes the camera packet in [protocol.md](protocol.md#camera-packet-drone-to-ground-receiver), so the receivers need updating at the same time.

## Plan

1. Build the command protocol, including ownership.
2. Radio bridge firmware under `firmware/` that forwards between USB serial and any number of drones.
3. A host program that finds drones with `Hello` and flies them together.
4. Drone ID in the camera packet, with updated receivers.

## Open questions

- **Collisions.** Each drone holds its own position, but nothing stops two drones from flying into each other. The ground station has to keep them apart, and the position estimate is relative to each drone's own takeoff point.
- **Spacing.** Prop wash from a drone above pushes on the one below and upsets its height hold. Drones need to keep some distance, which needs testing to find.
- **Shared timing.** Commands to each drone are sent one after another, so moves start a few ms apart. That is fine for most uses, but a broadcast `Pcmd` for a group could be added if needed.
