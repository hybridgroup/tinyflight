//go:build m5stamp_fly && !blackbox

package main

import "github.com/hybridgroup/tinyflight/pkg/flight"

// startBlackbox does nothing without the blackbox build tag, which keeps the flight log out of RAM and flash.
func startBlackbox(f *flight.Flight) {}
