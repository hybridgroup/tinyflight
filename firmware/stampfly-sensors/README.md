# stampfly-sensors

Bench tool for the M5Stack StampFly. It runs the flight loop with the motors held off and prints every sensor once a second. Use it to check units, axis directions and loop timing.

## Flashing

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-sensors
tinygo monitor
```

## USB output

Each report is four lines followed by a blank line.

| Line | What it shows |
|---|---|
| 1 | Accel in mm/s², gyro in thousandths of a degree per second, and whether the gyro is calibrated |
| 2 | Roll, pitch and yaw in degrees, baro pressure in Pa, range in mm, and the height and position estimates in mm |
| 3 | Magnetometer in nT, optical flow counts and quality over the last second, the flow motion in mrad, and that motion with the turn of the drone removed. Then battery voltage and current. |
| 4 | Flight loop ticks, overruns, the longest step and jitter in µs, IMU and sensor errors, and heap allocations since the last report |

When the drone is still, `moved_mrad` should stay near 0 even while you turn it by hand. `mallocs` should stay at 0 once it is running.

If no IMU is found it prints `no IMU` and stops.
