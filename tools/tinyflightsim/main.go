// Command tinyflightsim serves a browser flight simulator. It builds the StampFly firmware
// with TinyGo to wasm and runs it against simulated physics, flown with an on-screen Atom Joystick.
package main

import (
	"compress/gzip"
	"embed"
	"flag"
	"io"
	"io/fs"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"
)

//go:embed web
var web embed.FS

func main() {
	addr := flag.String("addr", "localhost:8080", "address to serve on")
	repo := flag.String("repo", "../..", "root of the tinyflight module")
	tinygo := flag.String("tinygo", "tinygo", "tinygo command")
	pkg := flag.String("firmware", "./firmware/stampfly", "firmware package to build")
	flag.Parse()

	root, err := filepath.Abs(*repo)
	if err != nil {
		log.Fatal(err)
	}
	cache, err := os.UserCacheDir()
	if err != nil {
		log.Fatal(err)
	}
	b, err := newBuilder(root, *pkg, *tinygo, filepath.Join(cache, "tinyflightsim"))
	if err != nil {
		log.Fatal(err)
	}

	static, _ := fs.Sub(web, "web")
	http.Handle("/api/firmware.wasm", addHeaders(b))
	http.Handle("/", addHeaders(http.FileServerFS(static)))
	log.Printf("building %s in %s", *pkg, root)
	log.Printf("serving on http://%s", *addr)
	log.Fatal(http.ListenAndServe(*addr, nil))
}

// addHeaders allows SharedArrayBuffer and Atomics.wait, which the runner needs.
// From main.go of https://github.com/tinygo-org/playground
func addHeaders(h http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Cross-Origin-Opener-Policy", "same-origin")
		w.Header().Set("Cross-Origin-Embedder-Policy", "require-corp")
		h.ServeHTTP(w, r)
	})
}

// ServeHTTP sends the firmware, building it first if the sources changed.
// Build errors come back as plain text with status 500.
func (b *builder) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Cache-Control", "no-store")
	path, out, err := b.build(r.Context())
	if err != nil {
		w.Header().Set("Content-Type", "text/plain; charset=utf-8")
		w.WriteHeader(http.StatusInternalServerError)
		io.WriteString(w, err.Error()+"\n")
		w.Write(out)
		return
	}
	f, err := os.Open(path)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	defer f.Close()
	w.Header().Set("Content-Type", "application/wasm")
	w.Header().Set("X-Build", strings.TrimSuffix(filepath.Base(path), ".wasm"))
	if !strings.Contains(r.Header.Get("Accept-Encoding"), "gzip") {
		io.Copy(w, f)
		return
	}
	w.Header().Set("Content-Encoding", "gzip")
	gw := gzip.NewWriter(w)
	io.Copy(gw, f)
	gw.Close()
}
