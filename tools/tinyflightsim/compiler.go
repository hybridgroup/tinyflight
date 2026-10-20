// The build queue and cache follow compiler.go of https://github.com/tinygo-org/playground
// Copyright The TinyGo Authors, BSD 3-clause license.

package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"io/fs"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strings"
	"sync"
)

const maxCacheSize = 20 * 1000 * 1000

// Tags for the sim build. See firmware/stampfly/board_sim.go.
var buildArgs = []string{"build", "-target=wasip1", "-tags=tinyflightsim", "-no-debug"}

type builder struct {
	repo, pkg, tinygo, cacheDir string
	version                     []byte
	mu                          sync.Mutex // one build at a time
}

func newBuilder(repo, pkg, tinygo, cacheDir string) (*builder, error) {
	if err := os.MkdirAll(cacheDir, 0o777); err != nil {
		return nil, err
	}
	version, err := exec.Command(tinygo, "version").Output()
	if err != nil {
		return nil, errors.New("tinygo not found: " + err.Error())
	}
	return &builder{repo: repo, pkg: pkg, tinygo: tinygo, cacheDir: cacheDir, version: version}, nil
}

// build returns the cached wasm for the current sources, or builds it.
// On a failed build it returns the compiler output.
func (b *builder) build(ctx context.Context) (string, []byte, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	hash, err := b.sourceHash()
	if err != nil {
		return "", nil, err
	}
	out := filepath.Join(b.cacheDir, "firmware-"+hash[:16]+".wasm")
	if _, err := os.Stat(out); err == nil {
		return out, nil, nil
	}
	tmp := out + ".tmp"
	defer os.Remove(tmp)
	args := append(append([]string{}, buildArgs...), "-o", tmp, b.pkg)
	cmd := exec.CommandContext(ctx, b.tinygo, args...)
	cmd.Dir = b.repo
	var buf bytes.Buffer
	cmd.Stdout, cmd.Stderr = &buf, &buf
	log.Printf("building %s", hash[:16])
	if err := cmd.Run(); err != nil {
		return "", buf.Bytes(), errors.New("build failed: " + err.Error())
	}
	if err := os.Rename(tmp, out); err != nil {
		return "", nil, err
	}
	b.cleanupCache()
	return out, nil, nil
}

// sourceHash covers every Go file and module file that could be in the build, and the tinygo version.
func (b *builder) sourceHash() (string, error) {
	h := sha256.New()
	h.Write(b.version)
	err := filepath.WalkDir(b.repo, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		name := d.Name()
		if d.IsDir() {
			// tools holds other modules, which are never in the firmware.
			if path != b.repo && (strings.HasPrefix(name, ".") || name == "tools" || name == "node_modules") {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(name, ".go") && name != "go.mod" && name != "go.sum" {
			return nil
		}
		f, err := os.Open(path)
		if err != nil {
			return err
		}
		defer f.Close()
		rel, _ := filepath.Rel(b.repo, path)
		io.WriteString(h, rel+"\x00")
		_, err = io.Copy(h, f)
		return err
	})
	if err != nil {
		return "", err
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}

// cleanupCache removes the oldest builds once the cache grows past maxCacheSize.
func (b *builder) cleanupCache() {
	entries, err := os.ReadDir(b.cacheDir)
	if err != nil {
		log.Println("read cache:", err)
		return
	}
	var files []fs.FileInfo
	var total int64
	for _, e := range entries {
		if fi, err := e.Info(); err == nil {
			files = append(files, fi)
			total += fi.Size()
		}
	}
	sort.Slice(files, func(i, j int) bool { return files[i].ModTime().Before(files[j].ModTime()) })
	for len(files) > 1 && total > maxCacheSize {
		if err := os.Remove(filepath.Join(b.cacheDir, files[0].Name())); err != nil {
			log.Println("clean cache:", err)
		}
		total -= files[0].Size()
		files = files[1:]
	}
}
