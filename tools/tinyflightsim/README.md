# tinyflightsim

A browser flight sim for Tinyflight. It builds the real [firmware/stampfly](../../firmware/stampfly/README.md) with TinyGo to WebAssembly and runs it against a physics model of the StampFly. Fly it with an on-screen M5 Atom Joystick, the keyboard or a gamepad.

![tinyflightsim](../../images/tinyflightsim.png)

See [docs/simulator.md](../../docs/simulator.md) for the controls, how it works and its limits.

## Running

You need Go and TinyGo.

```
cd tools/tinyflightsim
go run .
```

Then open http://localhost:8080. Open it from the server and not as a file, since the page needs cross origin isolation.

| Flag | Default | Description |
| --- | --- | --- |
| `-addr` | `localhost:8080` | Address to serve on |
| `-repo` | `../..` | Root of the tinyflight module |
| `-tinygo` | `tinygo` | TinyGo command |
| `-firmware` | `./firmware/stampfly` | Firmware package to build |

The server builds the firmware the first time the page asks for it. Builds are cached in the user cache directory on a hash of the Go sources and the TinyGo version. After you change the firmware or a setting, press **Restart** on the page to rebuild. Build errors show on the page.

To build with the newest TinyGo, run the server in the TinyGo dev container from the root of the repo.

```
docker run --rm -it -p 8080:8080 -e GOFLAGS=-buildvcs=false -v $(pwd):/src -w /src/tools/tinyflightsim tinygo/tinygo-dev:latest go run . -addr :8080
```

## Layout

The page files in `web` are embedded in the binary. `web/runner.js` runs the firmware in a web worker, and the page talks to it through a SharedArrayBuffer. The simulated drone itself is in `boards/sim` and `pkg/sim`.

## Tests

```
cd tools/tinyflightsim
go test ./...
```
