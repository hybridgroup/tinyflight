# stampfly-cameratest

Bench tool for the AMG8833 thermal camera on the M5Stack StampFly. The motors stay off and the radio is not used. It prints each frame over USB in the same format as [firmware/camera-receiver](../camera-receiver/README.md), so [tools/cameraview](../../tools/cameraview/README.md) can read the StampFly directly with no joystick or receiver.

See [docs/camera.md](../../docs/camera.md#testing-on-the-bench) for the wiring.

## Flashing

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-cameratest
```

## Viewing

Leave the StampFly plugged in and point the viewer at its port.

```
cd tools/cameraview
go run . -port /dev/ttyACM0
```

Then open http://localhost:8081. A hand in front of the sensor shows as a warm spot.

If the sensor is not found it prints `no camera` every second and keeps looking.
