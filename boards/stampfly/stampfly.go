//go:build m5stamp_fly

// Package stampfly sets up the M5Stack StampFly and gives the flight stack its hardware and settings.
// Pins and sensor setup follow https://github.com/m5stack/M5StampFly
package stampfly

import (
	"errors"
	"machine"
	"time"

	"github.com/hybridgroup/tinyflight/devices/bmi270"
	"github.com/hybridgroup/tinyflight/devices/bmm150"
	"github.com/hybridgroup/tinyflight/devices/bmp280"
	"github.com/hybridgroup/tinyflight/devices/ina3221"
	"github.com/hybridgroup/tinyflight/devices/lineinput"
	"github.com/hybridgroup/tinyflight/devices/pmw3901"
	"github.com/hybridgroup/tinyflight/devices/pwmmotors"
	"github.com/hybridgroup/tinyflight/devices/vl53l3cx"
	"github.com/hybridgroup/tinyflight/pkg/flight"
	"github.com/hybridgroup/tinyflight/pkg/fmath"
	"github.com/hybridgroup/tinyflight/pkg/params"

	drvbmi270 "tinygo.org/x/drivers/bmi270"
	drvbmm150 "tinygo.org/x/drivers/bmm150"
	drvbmp280 "tinygo.org/x/drivers/bmp280"
	drvina3221 "tinygo.org/x/drivers/ina3221"
	drvpmw3901 "tinygo.org/x/drivers/pmw3901"
	drvvl53l3cx "tinygo.org/x/drivers/vl53l3cx"
)

const (
	frontToFAddress = 0x2A
	batteryChannel  = 2
	motorPWMHz      = 150_000
	spiHz           = 2_000_000 // fastest the PMW3901 allows
	i2cHz           = 400_000
	tofBootTime     = 100 * time.Millisecond
)

// Board holds every device. A device that failed to start is left nil and listed in Errors.
type Board struct {
	Hardware flight.Hardware
	Motors   *pwmmotors.Motors
	FrontToF *drvvl53l3cx.Device
	Errors   []error
}

// New starts every bus and sensor. The motors are stopped when it returns.
func New() *Board {
	b := &Board{}

	machine.BMI270_CS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	machine.BMI270_CS.High()
	machine.PMW3901_CS.Configure(machine.PinConfig{Mode: machine.PinOutput})
	machine.PMW3901_CS.High()

	b.Motors = b.setupMotors()
	b.Hardware.Motors = b.Motors

	spi := machine.SPI0
	b.check("spi", spi.Configure(machine.SPIConfig{
		Frequency: spiHz,
		SCK:       machine.SPI1_SCK_PIN,
		SDO:       machine.SPI1_MOSI_PIN,
		SDI:       machine.SPI1_MISO_PIN,
		Mode:      3,
	}))

	i2c := machine.I2C0
	b.check("i2c", i2c.Configure(machine.I2CConfig{Frequency: i2cHz, SDA: machine.SDA_PIN, SCL: machine.SCL_PIN}))

	imu := drvbmi270.NewSPI(spi, machine.BMI270_CS)
	if b.check("bmi270", imu.Configure(drvbmi270.Config{
		AccelRange: drvbmi270.AccelRange8G,
		AccelRate:  drvbmi270.AccelRate800Hz,
		GyroRange:  drvbmi270.GyroRange2000,
		GyroRate:   drvbmi270.GyroRate1600Hz,
	})) {
		b.Hardware.IMU = bmi270.New(&imu)
	}

	flow := drvpmw3901.New(spi, machine.PMW3901_CS)
	if b.check("pmw3901", flow.Configure()) {
		b.Hardware.Flow = pmw3901.New(&flow)
	}

	baro := drvbmp280.New(i2c)
	baro.Address = drvbmp280.AddressLow
	if baro.Connected() {
		baro.Configure(drvbmp280.STANDBY_1MS, drvbmp280.FILTER_4X, drvbmp280.SAMPLING_2X, drvbmp280.SAMPLING_16X, drvbmp280.MODE_NORMAL)
		b.Hardware.Baro = bmp280.New(&baro)
	} else {
		b.check("bmp280", errNotFound)
	}

	mag := drvbmm150.New(i2c)
	if b.check("bmm150", mag.Configure(drvbmm150.Config{DataRate: drvbmm150.DataRate20Hz})) {
		b.Hardware.Mag = bmm150.New(mag)
	}

	power := drvina3221.New(i2c)
	if b.check("ina3221", power.Configure(drvina3221.DefaultConfig)) {
		b.Hardware.Power = ina3221.New(&power, batteryChannel)
	}

	b.setupToF(i2c)

	b.Hardware.Input = lineinput.New(machine.Serial)
	return b
}

// Config returns the flight settings for the StampFly.
func Config() params.Config {
	cfg := params.Default()
	// IMU to body axes, from Accel_x_raw = acc_y and so on in sensor.cpp of the M5StampFly firmware.
	cfg.Sensors.Rotation = fmath.Mat3{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}
	cfg.Schedule.FlowHz = 0
	cfg.AttEst.UseMag = false
	return cfg
}

func (b *Board) setupMotors() *pwmmotors.Motors {
	pwm := machine.PWM0
	b.check("motor pwm", pwm.Configure(machine.PWMConfig{Period: uint64(time.Second) / motorPWMHz}))
	var ch [4]uint8
	for i, p := range []machine.Pin{
		machine.MOTOR_FRONT_LEFT,
		machine.MOTOR_FRONT_RIGHT,
		machine.MOTOR_REAR_LEFT,
		machine.MOTOR_REAR_RIGHT,
	} {
		c, err := pwm.Channel(p)
		b.check("motor pwm", err)
		ch[i] = c
	}
	m := pwmmotors.New(pwm, ch[:]...)
	m.Stop()
	return m
}

// setupToF moves the front sensor to a new address so both can share the bus.
// Timing follows tof_init in tof.cpp of the M5StampFly firmware.
func (b *Board) setupToF(i2c *machine.I2C) {
	for _, p := range []machine.Pin{machine.TOF_BOTTOM_XSHUT, machine.TOF_FRONT_XSHUT} {
		p.Configure(machine.PinConfig{Mode: machine.PinOutput})
		p.Low()
	}
	time.Sleep(10 * time.Millisecond)

	machine.TOF_FRONT_XSHUT.High()
	time.Sleep(tofBootTime)
	front := drvvl53l3cx.New(i2c)
	// Configure reloads the address from NVM, so the new address goes after it.
	if b.check("front tof", front.Configure(drvvl53l3cx.Config{Mode: drvvl53l3cx.Long})) &&
		b.check("front tof address", front.SetAddress(frontToFAddress)) {
		b.FrontToF = front
	}

	machine.TOF_BOTTOM_XSHUT.High()
	time.Sleep(tofBootTime)
	bottom := drvvl53l3cx.New(i2c)
	if b.check("bottom tof", bottom.Configure(drvvl53l3cx.Config{Mode: drvvl53l3cx.Medium, TimingBudget: 33000})) &&
		b.check("bottom tof start", bottom.StartRanging()) {
		b.Hardware.Range = vl53l3cx.New(bottom)
	}
}

func (b *Board) check(device string, err error) bool {
	if err != nil {
		b.Errors = append(b.Errors, SetupError{Device: device, Err: err})
		return false
	}
	return true
}

// SetupError says which device failed to start.
type SetupError struct {
	Device string
	Err    error
}

func (e SetupError) Error() string { return e.Device + ": " + e.Err.Error() }

var errNotFound = errors.New("not found")
