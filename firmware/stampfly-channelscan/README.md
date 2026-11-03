# stampfly-channelscan

Bench tool that counts the Wi-Fi traffic on channels 1 to 13 and prints the quietest one to fly on. Use it at a busy venue such as a conference before flying. The motors stay off.

## Flashing

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-channelscan
tinygo monitor
```

It also runs on other ESP32-S3 and ESP32-C3 boards, such as the camera receiver board, with their own TinyGo target.

## Using it

Every 4 s it prints the packets heard on each channel with a bar, then the quietest channel. Busy channels next to a channel count against it too, since a Wi-Fi signal spreads over about 4 channels each side. Let it run for a minute while people are around, since the traffic changes a lot.

Then flash the flight firmware on that channel and pair the joystick again. See [Radio channel](../../README.md#radio-channel).
