// Command swarm flies every drone it finds together through firmware/radio-bridge. Each one takes
// off, turns one way and back in place, and lands. See docs/multi-drone.md.
//
//	go run . -port /dev/ttyACM0
//
// With -sim it flies the drone in tinyflightsim instead.
package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"sync"
	"time"

	"github.com/hybridgroup/tinyflight/groundstation"
	"github.com/hybridgroup/tinyflight/pkg/command"
	"go.bug.st/serial"
)

func main() {
	port := flag.String("port", "/dev/ttyACM0", "serial port of the radio bridge")
	sim := flag.Bool("sim", false, "fly the drone in tinyflightsim instead of through a radio bridge")
	count := flag.Int("count", 0, "number of drones to wait for, 0 for all found")
	height := flag.Uint("height", 50, "takeoff height in cm")
	flag.Parse()

	station, err := start(*port, *sim)
	if err != nil {
		log.Fatal(err)
	}
	infos := find(station, *count)
	drones := make([]*groundstation.Drone, len(infos))
	for i, info := range infos {
		d := station.Drone(info.MAC)
		name := mac(info.MAC)
		d.FlyingStateChange(func(s int) { log.Printf("%s %s", name, groundstation.FlyingState(s)) })
		d.Start()
		drones[i] = d
	}

	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt)
	go func() {
		<-stop
		log.Print("interrupted, landing")
		land(drones)
	}()

	log.Printf("takeoff to %d cm", *height)
	if err := each(drones, func(d *groundstation.Drone) error { return d.TakeOffTo(uint16(*height)) }); err != nil {
		log.Printf("take off: %v, landing", err)
		land(drones)
	}
	time.Sleep(5 * time.Second)

	log.Print("clockwise")
	each(drones, func(d *groundstation.Drone) error { return d.Clockwise(50) })
	time.Sleep(2 * time.Second)

	log.Print("hover")
	each(drones, (*groundstation.Drone).Hover)
	time.Sleep(3 * time.Second)

	log.Print("counter clockwise")
	each(drones, func(d *groundstation.Drone) error { return d.CounterClockwise(50) })
	time.Sleep(2 * time.Second)

	log.Print("hover")
	each(drones, (*groundstation.Drone).Hover)
	time.Sleep(3 * time.Second)

	log.Print("land")
	land(drones)
}

func start(port string, sim bool) (*groundstation.Station, error) {
	if sim {
		log.Printf("flying in tinyflightsim at %s", groundstation.SimAddr)
		return groundstation.Start()
	}
	p, err := serial.Open(port, &serial.Mode{BaudRate: 115200})
	if err != nil {
		return nil, err
	}
	log.Printf("radio bridge on %s", port)
	return groundstation.StartBridge(p, func(line string) { log.Printf("bridge: %s", line) }), nil
}

// find looks for drones with no owner until it has count of them, or for a few seconds when count is 0.
// Hello is a broadcast, which can be lost, so it is sent several times.
func find(s *groundstation.Station, count int) []command.Info {
	var infos []command.Info
	seen := map[[6]byte]bool{}
	for tries := 0; ; tries++ {
		found, _ := s.Find(500 * time.Millisecond)
		for _, info := range found {
			if seen[info.MAC] {
				continue
			}
			seen[info.MAC] = true
			if info.Flags&command.InfoOwned != 0 {
				log.Printf("found %s, owned by another controller, skipping it", mac(info.MAC))
				continue
			}
			log.Printf("found %s firmware %d.%d.%d", mac(info.MAC), info.Firmware[0], info.Firmware[1], info.Firmware[2])
			infos = append(infos, info)
		}
		switch {
		case count > 0 && len(infos) >= count:
			return infos[:count]
		case count == 0 && len(infos) > 0 && tries >= 4:
			return infos
		case len(infos) == 0 || count > 0:
			log.Printf("looking for drones, found %d...", len(infos))
		}
	}
}

// each runs f on every drone at once and returns the first error.
func each(drones []*groundstation.Drone, f func(*groundstation.Drone) error) error {
	var wg sync.WaitGroup
	errs := make([]error, len(drones))
	for i, d := range drones {
		wg.Go(func() {
			if err := f(d); err != nil {
				errs[i] = fmt.Errorf("%s: %w", mac(d.Address()), err)
			}
		})
	}
	wg.Wait()
	for _, err := range errs {
		if err != nil {
			return err
		}
	}
	return nil
}

// land lands every drone, waits for them to come down, and exits.
func land(drones []*groundstation.Drone) {
	if err := each(drones, (*groundstation.Drone).Land); err != nil {
		log.Print(err)
	}
	time.Sleep(5 * time.Second)
	for _, d := range drones {
		d.Halt()
	}
	log.Print("done")
	os.Exit(0)
}

func mac(m [6]byte) string {
	return fmt.Sprintf("%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5])
}
