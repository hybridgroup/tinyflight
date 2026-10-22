//go:build tinygo && m5stamp_c3

package main

import "github.com/hybridgroup/tinyflight/pkg/input"

// The m5stamp-c3 target has no I2C in the TinyGo machine package.
type display struct{}

func newDisplay() *display { return nil }

func (d *display) show(f *input.CameraFrame) {}
