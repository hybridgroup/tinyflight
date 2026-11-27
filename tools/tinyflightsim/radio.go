package main

import (
	"encoding/hex"
	"io"
	"log"
	"net"
	"net/http"
	"sync"
)

// broadcast is the ESP-NOW broadcast address.
var broadcast = [6]byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}

// droneMAC is the address of the simulated drone, MAC in boards/sim/world.go.
var droneMAC = [6]byte{0x02, 0x54, 0x46, 0x53, 0x49, 0x4D}

// radio relays ESP-NOW packets between ground station programs on UDP and the drone on the page.
// A datagram is a 6 byte address and the payload, the destination from a station and the source to it.
// The page gets packets as server sent events and posts the drone's packets back.
type radio struct {
	conn     *net.UDPConn
	mu       sync.Mutex
	pages    map[chan []byte]struct{}
	stations map[[6]byte]*net.UDPAddr
}

func newRadio(conn *net.UDPConn) *radio {
	return &radio{conn: conn, pages: map[chan []byte]struct{}{}, stations: map[[6]byte]*net.UDPAddr{}}
}

// stationMAC makes up an address for a ground station from its UDP port, locally administered.
func stationMAC(a *net.UDPAddr) [6]byte {
	return [6]byte{0x02, 'G', 'S', 0, byte(a.Port >> 8), byte(a.Port)}
}

// listen relays each datagram from a ground station to every page, with the station's address as the source.
func (r *radio) listen() {
	var buf [256]byte
	for {
		n, from, err := r.conn.ReadFromUDP(buf[:])
		if err != nil {
			log.Printf("radio: %v", err)
			return
		}
		if n <= 6 {
			continue
		}
		src := stationMAC(from)
		r.mu.Lock()
		if _, ok := r.stations[src]; !ok {
			log.Printf("ground station %s connected", from)
		}
		r.stations[src] = from
		r.mu.Unlock()
		r.toPages(append(src[:], buf[6:n]...))
	}
}

func (r *radio) toPages(p []byte) {
	r.mu.Lock()
	defer r.mu.Unlock()
	for c := range r.pages {
		select {
		case c <- p:
		default:
		}
	}
}

// ServeHTTP streams packets for the drone to the page on GET, and sends the drone's packets to a ground station on POST.
func (r *radio) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	switch req.Method {
	case http.MethodGet:
		r.stream(w, req)
	case http.MethodPost:
		b, err := io.ReadAll(io.LimitReader(req.Body, 256))
		if err != nil || len(b) <= 6 {
			http.Error(w, "want a 6 byte address and a payload", http.StatusBadRequest)
			return
		}
		r.toStation([6]byte(b[:6]), b[6:])
		w.WriteHeader(http.StatusNoContent)
	default:
		http.Error(w, "GET or POST", http.StatusMethodNotAllowed)
	}
}

// toStation sends a packet from the drone to the station at dst, or to every station for a broadcast.
func (r *radio) toStation(dst [6]byte, data []byte) {
	out := append(droneMAC[:len(droneMAC):len(droneMAC)], data...)
	r.mu.Lock()
	defer r.mu.Unlock()
	for mac, a := range r.stations {
		if dst == mac || dst == broadcast {
			r.conn.WriteToUDP(out, a)
		}
	}
}

func (r *radio) stream(w http.ResponseWriter, req *http.Request) {
	flush, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming not supported", http.StatusInternalServerError)
		return
	}
	c := make(chan []byte, 32)
	r.mu.Lock()
	r.pages[c] = struct{}{}
	r.mu.Unlock()
	defer func() {
		r.mu.Lock()
		delete(r.pages, c)
		r.mu.Unlock()
	}()
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(http.StatusOK)
	flush.Flush()
	for {
		select {
		case <-req.Context().Done():
			return
		case p := <-c:
			io.WriteString(w, "data: "+hex.EncodeToString(p)+"\n\n")
			flush.Flush()
		}
	}
}
