// Command cameraview shows camera frames from a StampFly live in the browser. It reads the lines
// firmware/camera-receiver prints over USB serial and pushes each frame to the page.
package main

import (
	"bufio"
	"embed"
	"encoding/json"
	"flag"
	"io"
	"io/fs"
	"log"
	"net/http"
	"sync"
	"time"

	"go.bug.st/serial"
)

//go:embed web
var web embed.FS

func main() {
	addr := flag.String("addr", "localhost:8081", "address to serve on")
	port := flag.String("port", "/dev/ttyACM0", "serial port of the camera receiver")
	flag.Parse()

	h := &hub{subs: map[chan []byte]bool{}}
	go readPort(*port, h)

	static, _ := fs.Sub(web, "web")
	http.Handle("/events", h)
	http.Handle("/", http.FileServerFS(static))
	log.Printf("serving on http://%s", *addr)
	log.Fatal(http.ListenAndServe(*addr, nil))
}

// readPort reads the receiver forever, opening the port again after an unplug.
func readPort(name string, h *hub) {
	for {
		p, err := serial.Open(name, &serial.Mode{BaudRate: 115200})
		if err != nil {
			log.Printf("open %s: %v", name, err)
			time.Sleep(time.Second)
			continue
		}
		log.Printf("reading %s", name)
		err = readFrames(p, h.publish)
		p.Close()
		log.Printf("read %s: %v", name, err)
		time.Sleep(time.Second)
	}
}

// readFrames parses camera lines from r and passes on each whole frame. Other lines are logged.
func readFrames(r io.Reader, out func(Frame)) error {
	var a assembler
	s := bufio.NewScanner(r)
	for s.Scan() {
		c, err := parseLine(s.Text())
		switch err {
		case nil:
			if f, ok := a.add(c); ok {
				out(f)
			}
		case errNotCamera:
			if s.Text() != "" {
				log.Printf("receiver: %s", s.Text())
			}
		}
	}
	if s.Err() != nil {
		return s.Err()
	}
	return io.EOF
}

// hub sends each frame to every open page as a server sent event.
type hub struct {
	mu   sync.Mutex
	subs map[chan []byte]bool
}

func (h *hub) publish(f Frame) {
	b, err := json.Marshal(f)
	if err != nil {
		return
	}
	h.mu.Lock()
	defer h.mu.Unlock()
	for c := range h.subs {
		// A slow page skips frames rather than holding up the others.
		select {
		case c <- b:
		default:
		}
	}
}

func (h *hub) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "streaming not supported", http.StatusInternalServerError)
		return
	}
	c := make(chan []byte, 1)
	h.mu.Lock()
	h.subs[c] = true
	h.mu.Unlock()
	defer func() {
		h.mu.Lock()
		delete(h.subs, c)
		h.mu.Unlock()
	}()

	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-store")
	flusher.Flush()
	for {
		select {
		case <-r.Context().Done():
			return
		case b := <-c:
			if _, err := io.WriteString(w, "data: "+string(b)+"\n\n"); err != nil {
				return
			}
			flusher.Flush()
		}
	}
}
