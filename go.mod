module github.com/hybridgroup/tinyflight

go 1.24.4

require tinygo.org/x/drivers v0.36.0

require github.com/soypat/lneto v0.4.0 // indirect

require (
	github.com/google/shlex v0.0.0-20191202100458-e7afc7fbc510 // indirect
	tinygo.org/x/espradio v0.0.0
)

replace tinygo.org/x/drivers => ../drivers-tinyflight

replace tinygo.org/x/espradio => ../espradio
