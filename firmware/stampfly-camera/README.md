# stampfly-camera

The flight firmware for the M5Stack StampFly with an AMG8833 thermal camera on the Grove port. It flies like [firmware/stampfly](../stampfly/README.md) and also streams camera frames over ESP-NOW to [firmware/camera-receiver](../camera-receiver/README.md).

See [docs/camera.md](../../docs/camera.md) for the wiring, mounting and the full setup.

## Flashing

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-camera
tinygo monitor
```

At startup it prints `camera found` or `no camera`. Without a camera it still flies.

## Using it

Fly it with the M5 Atom Joystick as with `firmware/stampfly`. See [docs/joystick.md](../../docs/joystick.md). The bench control lines over USB serial work the same too.

The camera is read every 100 ms on core 0, so the flight loop on core 1 is not touched. Frames are only sent once the joystick has paired.

Every 500 ms the USB printout shows the same state as `firmware/stampfly`, plus `cam_sent` and `cam_failed` for the camera frames sent over ESP-NOW.

The flight log from the `blackbox` tag is only in `firmware/stampfly`.
