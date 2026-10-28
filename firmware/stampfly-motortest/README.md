# stampfly-motortest

Bench tool for the M5Stack StampFly that spins each motor in turn. Remove the propellers first.

## Flashing

```
tinygo flash -scheduler=cores -target m5stamp-fly ./firmware/stampfly-motortest
tinygo monitor
```

## Using it

Press the button on the StampFly. Each motor spins at 20% power for 1 s, in the order front left, front right, rear left, rear right. The name of each motor is printed over USB as it spins.

Front right and rear left should spin counter clockwise seen from above, and front left and rear right clockwise. Press the button again to repeat.
