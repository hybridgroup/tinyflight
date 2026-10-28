# estimate

Host only checks of the on-board filters against [go-estimate](https://github.com/milosgajdos/go-estimate). It runs the altitude Kalman filter from `pkg/estimator/altitude` side by side with a go-estimate filter of the same model, and checks they give the same answers.

The on-board filters use float32 and fixed size arrays, since the ESP32-S3 has no hardware float64 and the flight loop must not allocate. go-estimate is built on gonum and float64, which is too heavy for the microcontroller. This folder has its own `go.mod`, so gonum never gets into the firmware.

See [Filters and go-estimate](../../docs/architecture.md#filters-and-go-estimate).

## Tests

```
cd tools/estimate
go test ./...
```
