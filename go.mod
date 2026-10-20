module github.com/hybridgroup/tinyflight

go 1.24.4

require (
	tinygo.org/x/drivers v0.36.0
	tinygo.org/x/espradio v0.4.1-0.20261020142753-4e00dfa7867d
)

require (
	github.com/orsinium-labs/tinymath v1.2.1
	github.com/soypat/lneto v0.4.0 // indirect
)

require github.com/google/shlex v0.0.0-20191202100458-e7afc7fbc510 // indirect

replace tinygo.org/x/drivers => ../drivers-tinyflight
