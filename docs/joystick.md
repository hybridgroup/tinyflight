# Flying with the M5 Atom Joystick

Tinyflight works with the stock StampFlyController firmware on the M5 Atom Joystick. This page explains the controls. See [protocol.md](protocol.md) for what goes over the air.

The button behaviour comes from [`main.cpp`, `app_lvgl.cpp` and `lvgl_porting.cpp`](https://github.com/m5stack/Atom-JoyStick/tree/main/examples/StampFlyController/src) in StampFlyController. The stick handling comes from `pkg/input/atom.go`.

## Screens

The joystick firmware has two screens. The buttons do different things on each.

- **Setup screen.** Shown at power on. Choose the flight mode here. Altitude mode is on at power on, from `setup()` in `main.cpp`.
- **Running screen.** Fly from here.

## Buttons

| Button | Setup screen | Running screen |
|---|---|---|
| Screen, press the AtomS3 display | Go to the running screen | Arm or disarm. Each press toggles. |
| Left button | Toggle between angle (Stabilize) and rate (Acro) | Go back to the setup screen. If armed, this also disarms. |
| Right button | Toggle altitude mode, which gives AltHold or PosHold | Nothing |
| Stick presses | Nothing | Nothing |

The mode buttons only work on the setup screen, so choose the mode before going to the running screen. Each change beeps.

## Sticks

These are for stick mode 2, the stock setting.

| Stick | Up and down | Left and right |
|---|---|---|
| Left | Throttle | Yaw |
| Right | Pitch, forward flies forward | Roll |

The throttle stick centres at 0, which is not like a normal RC transmitter.

- **Stabilize and Acro.** Centre or below is no thrust. Above the deadband thrust rises smoothly, and the drone hovers with the left stick about halfway between centre and full up. Letting go drops the motors to idle.
- **AltHold and PosHold.** After arming the motors stay at idle until you push the left stick up past 20%. From then on, centre holds the height, up climbs and down descends, up to 0.5 m/s. The StampFly will not climb past 10 m above where it took off. See [Height limit](../README.md#height-limit) to change it.

There is a 20% deadband around centre, the same as the stock drone firmware.

## Modes

| Setup screen shows | Flight mode |
|---|---|
| Stable and Manual | Stabilize |
| Sport and Manual | Acro |
| Stable and Auto | AltHold |
| Sport and Auto | PosHold |

The left tile is the control mode, Stable in blue for angle or Sport in yellow for rate. The right tile is the altitude mode, Manual in pink for off or Auto in green for on.

Altitude mode is on at power on, so press the right button once on the setup screen to fly Stabilize. The right tile then shows Manual.

PosHold holds the spot over the ground and needs a textured surface below, between 8 cm and about 2 m. Elsewhere it flies like AltHold. See [architecture.md](architecture.md#flight-modes).

## Arming

Tinyflight only arms when

- the gyro has finished calibrating, so keep the drone still for a second after power on,
- the drone is close to level,
- the throttle is low, so keep the left stick at centre or below,
- the battery is not low,
- the joystick signal is fresh.

Once armed, the motors spin at idle. In AltHold and PosHold they stay at idle until you push the left stick up.

## Landing

There is no land button, and Tinyflight does not detect touchdown. Fly it down, then disarm.

**Stabilize and Acro**

1. Lower the left stick slowly below the hover point, about halfway up, so the drone sinks gently.
2. Keep easing it down until it touches.
3. Bring the left stick to centre. The motors drop to idle.
4. Press the screen to disarm.

Centring the stick in the air cuts all thrust, so only centre it on the ground or a few cm above.

**AltHold and PosHold**

1. Hold the left stick down. The drone descends at up to 0.4 m/s.
2. After touchdown, leave the left stick down or centred. The drone disarms by itself after 2 s on the ground.
3. You can also press the screen to disarm sooner.

**Emergency stop**

Press the screen to disarm, or press the left button to leave the running screen, which also disarms. The motors stop at once and the drone drops.

## Pairing

Pairing is needed the first time, or to fly a different drone.

1. Turn the drone on.
2. Hold the screen button while turning the joystick on.
3. Press the screen again to confirm.

The joystick then scans the channels until it finds the drone. Tinyflight keeps offering to pair until the first control packet arrives, so the drone does not need a restart.

## Failsafes

| Problem | What happens |
|---|---|
| Signal lost for 0.5 s | Level out and descend, then disarm |
| Battery low for 2 s | Level out and descend, then disarm |
| Tilt past about 70° in any mode but Acro | Stop motors at once |

The failsafe landing ramps thrust down to half over 4 s, then disarms whether the drone is down or not. From a low hop that is fine, but from higher up it drops the rest of the way.

## First flight in Stabilize

1. On the setup screen, press the right button once to turn altitude mode off. It is on at power on.
2. Press the screen to go to the running screen.
3. With the left stick at centre, press the screen to arm.
4. Raise the left stick slowly past 20% until the drone lifts.
5. To stop, press the screen to disarm, or bring the left stick back to centre.

## Tethered test

Do the first flights on a tether, so a bad setting cannot send the drone away. For example, tie a fishing line from below each motor to a corner of a small base, about 15 cm long.

### Before you start

- Put a textured surface under the drone, such as a rug or printed paper. The optical flow sensor needs texture and only works above 8 cm.
- Lift the drone by hand to full line length and check that no line, slack or taut, can reach a propeller.
- Check each propeller is on the right motor and turns the right way. `stampfly-motortest` spins each motor in turn.
- Charge the battery. The low battery failsafe starts below 3.34 V.
- Flash `stampfly`, unplug USB, and pair the joystick. Then press the right button once on the setup screen to turn altitude mode off for Stabilize. It is on at power on.

### Flights

Do these in order. If anything looks wrong, press the screen to disarm.

1. **Arm only.** On the running screen, with the left stick at centre, press the screen. All 4 motors spin at idle. Wait 5 s and press the screen again to stop them.
2. **Light on its feet.** Arm, then raise the left stick slowly past 20% until the drone gets light without lifting. Hold 3 s. It should not tip or turn. Lower the stick and disarm.
3. **Short hop.** Lift to 5 to 10 cm with the lines still slack and hold for 5 to 10 s. Nudge the right stick once each way. Right rolls right, forward tips the nose down. Land and disarm.
4. **Hover for the log.** Lift to about 10 cm and hover for 20 to 30 s with as few corrections as you can. Then make small slow moves forward, back, left and right, about 2 s each with pauses. Land and disarm.

Disarm at once if it tips the wrong way on a stick input, wobbles faster and faster, spins in yaw by itself, or a line touches a propeller. Keep it a little below line length, since pulling hard against the lines makes the controllers fight them.

### Reading the flight log

The flight log is only in firmware built with the `blackbox` tag. It needs `machine.Flash` for the ESP32-S3 from tinygo-org/tinygo#5934.

```
tinygo flash -scheduler=cores -tags blackbox -target m5stamp-fly ./firmware/stampfly
```

The firmware logs up to the first minute of each flight. One second after you disarm, it saves the flight to flash, which keeps the last 5 flights even with the battery off. Flashing new firmware with `tinygo flash` erases the whole flash chip, so read any saved flights first. While disarmed it prints the latest saved flight over USB every 10 s, between `saved log start` and `saved log end`.

The last 11 s of a flight are also kept in memory that survives a reset, such as a reboot after a crash. They are saved to flash at the next start, and the printout says the flight is from before a reset.

1. After the flight you want to check, disarm and wait a second or two before switching off, so it can save.
2. Plug in USB and read the serial output.
3. Note roughly when you made each move, counting from arming, so the log can be matched to what you saw.

Saving pauses the flight loop for about a second, so it only happens while disarmed, and it stops at once if you arm.

Each log line has the time since arming in ms, the position and speed estimate in mm and mm/s, roll, pitch and yaw in tenths of a degree, thrust from 0 to 1000, the raw range in mm, body Z accel and the height filter's accel bias in mm/s², flow quality, mode, and whether the position and height estimates are valid. The last columns show how each flow reading was used. They are the flow rate and the mean gyro rate over the same reading in mrad/s, the speed the estimator made of it in mm/s, and flags where 1 is sensor valid, 2 passed the height checks, 4 north accepted and 8 east accepted. Then come the stick inputs, from -1000 to 1000 with throttle from 0 to 1000, the vertical speed in mm/s, the height setpoint in mm, the battery in mV, and last the torque the roll, pitch and yaw rate integrators hold, in ten thousandths, which in a steady hover is the trim the drone needs.

The firmware also keeps what the height filter got and gave on every flight loop pass of the last 6 s, so the filter can be replayed off board. In AltHold or PosHold, from 3 s after arming and 1 s after the climb stick was last used, it freezes 2 s after the height first strays more than 25 cm from the setpoint, so it holds the start of a height swing. Otherwise it ends with the flight. It prints between `saved trace start` and `saved trace end`, and the start line says when it was triggered. Each line has the time since arming in µs, the vertical accel the filter got in mm/s², the step time in µs, any range reading in mm with the cosine of the tilt times 10000, any baro height in mm, the height and vertical speed after the pass, the height setpoint, the accel bias, thrust from 0 to 1000, and flags where 1 is a new range reading, 2 range accepted, 4 a new baro reading, 8 baro accepted and 16 on the ground.

USB serial output now and then loses a character, so every log and trace line ends with the sum of its values, counting true as 1. Drop any line where the sum does not match, and take it from the next print 10 s later.
