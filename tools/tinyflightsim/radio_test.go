package main

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"net"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

func TestRadioRelay(t *testing.T) {
	udp, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer udp.Close()
	r := newRadio(udp)
	go r.listen()
	srv := httptest.NewServer(r)
	defer srv.Close()

	res, err := http.Get(srv.URL)
	if err != nil {
		t.Fatal(err)
	}
	defer res.Body.Close()
	events := bufio.NewReader(res.Body)

	station, err := net.DialUDP("udp", nil, udp.LocalAddr().(*net.UDPAddr))
	if err != nil {
		t.Fatal(err)
	}
	defer station.Close()
	station.SetDeadline(time.Now().Add(2 * time.Second))
	hello := []byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 'T', 'F', 1}
	if _, err := station.Write(hello); err != nil {
		t.Fatal(err)
	}
	line, err := events.ReadString('\n')
	if err != nil {
		t.Fatal(err)
	}
	got, err := hex.DecodeString(strings.TrimSpace(strings.TrimPrefix(line, "data: ")))
	if err != nil {
		t.Fatal(err)
	}
	mac := stationMAC(station.LocalAddr().(*net.UDPAddr))
	if !bytes.Equal(got, append(mac[:], 'T', 'F', 1)) {
		t.Fatalf("page got % x", got)
	}

	post := func(dst [6]byte) {
		res, err := http.Post(srv.URL, "application/octet-stream", bytes.NewReader(append(dst[:], 'T', 'F', 2)))
		if err != nil {
			t.Fatal(err)
		}
		res.Body.Close()
		if res.StatusCode != http.StatusNoContent {
			t.Fatalf("post status %d", res.StatusCode)
		}
	}
	other := mac
	other[5]++
	post(other)
	post(mac)
	var buf [64]byte
	n, err := station.Read(buf[:])
	if err != nil {
		t.Fatal(err)
	}
	if want := append(droneMAC[:len(droneMAC):len(droneMAC)], 'T', 'F', 2); !bytes.Equal(buf[:n], want) {
		t.Errorf("station got % x, want % x", buf[:n], want)
	}

	res2, _ := http.Post(srv.URL, "application/octet-stream", strings.NewReader("short"))
	if res2.StatusCode != http.StatusBadRequest {
		t.Errorf("short post status %d", res2.StatusCode)
	}
}
