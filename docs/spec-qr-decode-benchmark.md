# Spec: QR Decode Benchmark App

## Status

Future / not started. Will live in a separate repo (`qr-decode-bench`).

## Goal

Measure k_quirc decode performance across QR versions on ESP32-S3, ESP32-P4, and
future targets using synthetic (computer-generated) QR codes. Produce
reproducible, comparable numbers that predict real-world decode behavior.

## Why a separate repo

This project (esp-camera-pipeline) is a component library with no application
entry point. The benchmark is an application with target-specific sdkconfig,
partition tables, and no camera/display dependencies. It runs on bare devkits —
just CPU, PSRAM, and serial.

## Test design

### Difficulty axis: pixels per module (PPM)

Rather than artificial noise/skew (which doesn't correspond to real camera
output), the benchmark varies **pixels per module** — the number of image pixels
per QR module (black/white square). This directly simulates distance-from-camera,
which is the dominant real-world difficulty factor. Deterministic, reproducible,
and tells you at what version × resolution combination the hardware starts
struggling.

### Test matrix

| Axis | Values |
|---|---|
| QR version | 1 through 25 (hardcoded upstream maximum) |
| ECC level | L (fixed — matches our use case) |
| Pixels per module | 8, 5, 3, 2 |
| Preprocessing features | contrast stretch, adaptive threshold, bilinear threshold (on/off) |

Frame sizes match real deployment: 320×320 for S3, 480×480 for P4.

### What's NOT tested

- Noise/blur/skew — arbitrary, non-predictive
- RGB565→grayscale conversion — pipeline concern, not decoder performance
- Camera/display pipeline — isolated decoder benchmark only
- Multi-QR frames — edge case, not the bottleneck

## Image generation

**Build time** (host Python script): generate QR codes for each version, export
as compact module grids (1 bit per module). Embed as C arrays in flash. Total
storage is a few KB.

**Runtime** (on device, not timed): expand module grid to grayscale buffer at
the target PPM, center in frame with mid-gray surround, feed to k_quirc.

## Runtime flow

```
for each (version, ppm, feature_flags):
    render module grid → grayscale frame
    verify: one decode pass, assert success + correct payload
    warmup: N iterations (default 50)
    timed: M iterations (default 200)
        record per-iteration decode time via esp_timer_get_time()
    compute mean, min, max, stddev
    emit CSV line
```

## Output

CSV over serial:

```
chip,freq_mhz,psram_type,version,ppm,frame_w,frame_h,contrast_stretch,adaptive_thresh,bilinear,iterations,mean_us,min_us,max_us,stddev_us,decoded_ok
esp32s3,240,octal-spiram,5,8,320,320,1,1,1,200,2340,2280,2450,35,1
esp32p4,400,octal-spiram,5,8,480,480,1,1,1,200,980,950,1020,15,1
```

Host-side Python script parses CSV for tables and comparison charts.

## Project structure

```
qr-decode-bench/
  main/
    main.c                   # benchmark loop, serial output
    qr_test_images.c/.h      # generated module grids (build artifact)
    Kconfig.projbuild         # iteration counts, frame size, PPM list
  components/
    k_quirc/                 # submodule from kdmukAI-bot/k_quirc (seedsigner-dev branch)
  scripts/
    generate_qr_grids.py     # host: QR version → C module grid arrays
    analyze_results.py        # host: parse CSV, produce comparison charts
  sdkconfig.defaults
  sdkconfig.defaults.esp32s3
  sdkconfig.defaults.esp32p4
  CMakeLists.txt
```

## What this tells you

1. How does decode time scale with QR version on each chip?
2. At what version does each chip exceed the real-time frame budget?
3. What's the P4 speedup over S3 at each test point?
4. How much does each preprocessing feature cost per frame?
5. At what PPM does decode start failing (predicts effective camera range)?

### QR version producer guidelines

The max QR version that a scanner can reliably decode is limited by camera
resolution, not the decoder setting (hardcoded at 25). These are recommendations
for QR code generators targeting SeedSigner devices:

| Scanner frame size | Recommended max QR version | PPM at 90% fill |
|---|---|---|
| 320x320 (S3) | 12 | ~4.4 |
| 480x480 (P4) | 16-18 | ~4.9-4.1 |
| 640x640 | 25 | ~4.9 |

Beyond these limits, payloads should be split into animated frames (BBQr/UR).
A typical 2-of-3 multisig wallet descriptor (~430 bytes) fits in QR version 14,
well within all frame sizes above.
