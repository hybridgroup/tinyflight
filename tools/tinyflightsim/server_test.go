package main

import (
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// fakeTinygo writes a script that copies main.go to the output file, or fails when it holds "bad".
func fakeTinygo(t *testing.T, dir string) string {
	script := filepath.Join(dir, "tinygo")
	body := `#!/bin/sh
[ "$1" = version ] && { echo "tinygo version test"; exit 0; }
echo run >> "` + filepath.Join(dir, "runs") + `"
while [ "$1" != "-o" ]; do shift; done
grep -q bad main.go && { echo "main.go:1: bad code"; exit 1; }
cp main.go "$2"
`
	if err := os.WriteFile(script, []byte(body), 0o755); err != nil {
		t.Fatal(err)
	}
	return script
}

func runs(t *testing.T, dir string) int {
	b, _ := os.ReadFile(filepath.Join(dir, "runs"))
	return strings.Count(string(b), "run")
}

func get(t *testing.T, h http.Handler) (*http.Response, string) {
	rec := httptest.NewRecorder()
	h.ServeHTTP(rec, httptest.NewRequest("GET", "/api/firmware.wasm", nil))
	res := rec.Result()
	body, _ := io.ReadAll(res.Body)
	return res, string(body)
}

func TestBuildAndCache(t *testing.T) {
	tools, repo := t.TempDir(), t.TempDir()
	write := func(name, s string) {
		if err := os.WriteFile(filepath.Join(repo, name), []byte(s), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	write("main.go", "package main // v1")
	write("README.md", "docs")
	b, err := newBuilder(repo, ".", fakeTinygo(t, tools), t.TempDir())
	if err != nil {
		t.Fatal(err)
	}
	h := addHeaders(b)

	res, body := get(t, h)
	if res.StatusCode != 200 || body != "package main // v1" {
		t.Fatalf("status %d body %q", res.StatusCode, body)
	}
	if res.Header.Get("Content-Type") != "application/wasm" || res.Header.Get("Cross-Origin-Embedder-Policy") != "require-corp" {
		t.Errorf("headers %v", res.Header)
	}

	write("README.md", "more docs")
	get(t, h)
	if n := runs(t, tools); n != 1 {
		t.Errorf("%d builds after a docs change, want 1", n)
	}

	write("main.go", "package main // bad")
	res, body = get(t, h)
	if res.StatusCode != 500 || !strings.Contains(body, "bad code") {
		t.Errorf("status %d body %q", res.StatusCode, body)
	}

	write("main.go", "package main // v2")
	if _, body = get(t, h); body != "package main // v2" {
		t.Errorf("body %q after fix", body)
	}
}
