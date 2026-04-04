# ESP Camera Pipeline

A triple-buffered camera-to-display pipeline engine for ESP32 with zero-copy frame access for external consumers. Designed as a reusable ESP-IDF component with abstract camera and display driver interfaces, so it can be wired up to different hardware (P4 CSI, S3 DVP, various displays) without modification.

## Overview

ESP Camera Pipeline handles the core plumbing of camera capture, live display preview, and frame consumer access. The app provides hardware-specific camera and display drivers through simple vtable interfaces; the engine handles everything else: buffer management, cropping, and display pushing. Consumers (QR decoders, entropy collectors, image analyzers) attach externally via the lock/release frame API.

**Key features:**

- **Abstract driver interfaces** for camera and display hardware -- write a driver once, swap hardware without touching the engine
- **Triple-buffered** with lock/release for safe zero-copy consumer access -- no stage blocks any other
- **Pre-allocated buffers** at create time; zero allocations in the hot loop
- **LVGL overlay support** -- apps can parent custom widgets on top of the live video feed
- **Per-stage debug metrics** (camera FPS, display FPS/skip rate, consumer FPS/timing) via Kconfig
- **Multi-target** -- currently supports ESP32-S3 and ESP32-P4 (with PPA hardware crop+scale+rotate), extensible to future ESP32 variants

The pipeline is a pure C ESP-IDF component usable from any ESP32 application -- whether native C/C++ or MicroPython.

### Acknowledgement

Extracted and evolved from [Kern](https://github.com/odudex/Kern) by [Odudex](https://github.com/odudex).

k_quirc is based on Espressif's [quirc component](https://components.espressif.com/components/espressif/quirc), substantially reworked by Odudex in Kern with adaptive/bilinear thresholding, performance optimizations, and ESP32 memory safeguards.

## Architecture

### Triple buffer design

Three buffers rotate through three roles:

| Buffer | Role | Who uses it |
|---|---|---|
| A | **Locked** by consumer | Consumer reading -- however long it takes |
| B | **Front** (display) | Display showing this frame |
| C | **Back** (writing) | Camera filling next frame |

When a new camera frame arrives, the frame callback picks whichever buffer is neither front nor locked as the next back buffer. The camera and display keep cycling at full frame rate while the consumer holds its lock indefinitely.

**Why not double buffer?** With two buffers, a slow consumer risks reading a buffer the camera is about to overwrite. On fast hardware the race window may be negligible, but on slower targets (e.g., ESP32-S3 with slower PSRAM bus) a consumer that takes 20-50ms makes the race window real. The third buffer eliminates it entirely.

### Three independent frame rates

| Stage | Rate | Governed by |
|---|---|---|
| **Camera** | Sensor native (e.g., 30fps) | Hardware -- the ceiling that drives everything |
| **Display** | Up to camera rate, skips if busy | Display driver responsiveness (e.g., LVGL lock availability) |
| **Consumer** | Whatever pace the consumer can sustain | Processing time (via lock/release) |

The camera is the only hard clock. None of the three stages block each other. A slow consumer doesn't stall the camera or the display. A slow display doesn't stall the camera or the consumer. The camera just keeps producing, and each downstream stage independently takes what it can and skips what it can't.

### Memory cost

The third buffer's memory cost is fixed at create time and modest relative to total pipeline allocation:

| Resolution | Per buffer (= triple buffer overhead) | Total pipeline |
|---|---|---|
| 320x320 (S3 typical) | 200KB | ~600KB |
| 480x480 | 450KB | ~1.4MB |
| 640x640 (P4 typical) | 800KB | ~2.4MB |

On an 8MB PSRAM module (standard for S3 dev boards running MicroPython + LVGL), the delta is negligible.

### Consumer access pattern

```c
// Lock the most recent complete frame (zero-copy, returns pointer)
const uint8_t *buf = cam_pipeline_lock_frame(handle);
if (buf) {
    // ... read from buf as long as needed, safe from camera writes ...
    cam_pipeline_release_frame(handle);
}
```

The included `cam_pipeline_qr` component uses this same interface, serving as a reference consumer implementation.

## API

### Lifecycle

```
create(config)  ->  handle               # allocate everything, begin streaming
lock_frame(handle)  ->  pointer          # zero-copy access to most recent frame
release_frame(handle)                    # release locked frame back to pool
pause_frame_access(handle)               # lock_frame returns NULL until resumed
resume_frame_access(handle)              # resume frame access
set_ae_target(handle, val)               # adjust exposure (anytime between create/destroy)
set_focus(handle, val)                   # adjust focus (anytime between create/destroy)
get_overlay_parent(handle)  ->  widget   # get LVGL parent for overlay UI
destroy(handle)                          # stop everything, free all resources
```

The pipeline itself is purely camera + display + frame access. Consumers (QR decoding, entropy collection, etc.) are separate components that attach via `lock_frame`/`release_frame`.

### Public header: `esp_cam_pipeline.h`

The main public API. See [esp_cam_pipeline.h](include/esp_cam_pipeline.h) for the full interface. Key types:

- `cam_pipeline_handle_t` -- opaque handle returned by `cam_pipeline_create()`
- `cam_pipeline_config_t` -- configuration struct (display dimensions, camera/display drivers)

### Camera driver interface: `cam_pipeline_camera_driver.h`

Implement this vtable to connect a camera backend. See [cam_pipeline_camera_driver.h](include/cam_pipeline_camera_driver.h).

```c
typedef struct {
    void *(*init)(const void *platform_config);
    esp_err_t (*start)(void *handle, cam_pipeline_frame_cb_t frame_cb,
                       void *user_ctx, int core_id);
    esp_err_t (*stop)(void *handle);
    void (*deinit)(void *handle);
    esp_err_t (*get_resolution)(void *handle, uint32_t *width, uint32_t *height);

    /* Optional camera controls -- NULL if not supported */
    esp_err_t (*set_ae_target)(void *handle, uint32_t level);
    esp_err_t (*set_focus)(void *handle, uint32_t position);
    bool (*has_focus_motor)(void *handle);
} cam_pipeline_camera_driver_t;
```

### Display driver interface: `cam_pipeline_display_driver.h`

Implement this vtable to connect a display backend. See [cam_pipeline_display_driver.h](include/cam_pipeline_display_driver.h).

```c
typedef struct {
    void *(*init)(void *parent, uint32_t width, uint32_t height,
                  const void *driver_config);
    bool (*push_frame)(void *handle, const uint8_t *rgb565_buf,
                       uint32_t width, uint32_t height);
    void (*deinit)(void *handle);
    void *(*get_overlay_parent)(void *handle);
} cam_pipeline_display_driver_t;
```

`push_frame` is called from camera streaming context at frame rate. It must be non-blocking -- dropping a frame is OK, stalling the camera is not.

## Configuration

### Kconfig options

| Option | Type | Default | Description |
|---|---|---|---|
| `CAM_PIPELINE_DEBUG` | bool | n | Enable per-stage FPS logging with timing breakdowns |
| `CAM_PIPELINE_DEBUG_LOG_INTERVAL_MS` | int | 2000 | Debug metrics log interval (ms) |

When `CAM_PIPELINE_DEBUG` is enabled, metrics are available through two channels:

1. **Serial console**: Logged via `ESP_LOGI` at the configured interval
2. **Public stats getter**: `cam_pipeline_get_debug_stats()` returns computed metrics the app can render into LVGL labels on the overlay parent

## Integration

### ESP-IDF component

Add `esp-camera-pipeline` as a component in your ESP-IDF project. The `CMakeLists.txt` registers it with dependencies on `freertos` and `esp_timer`. PPA support (P4 only) is detected automatically via SOC capability.

### MicroPython

Frame consumers (QR decoding, entropy hashing) are implemented in C and use the pipeline's `lock_frame`/`release_frame` API directly. MicroPython never handles raw frame data -- it receives end results (decoded QR bytes, final entropy hash) from the C consumers.

## Included Components

### cam_pipeline_qr

A reference frame consumer that decodes QR codes from pipeline frames. Located in `components/cam_pipeline_qr/`, it demonstrates the same pattern any external consumer would use: create a task, call `cam_pipeline_lock_frame()`/`release_frame()`, process the frame, repeat. Automatically center-crops non-square frames to a square region before passing to the QR decoder.

Uses k_quirc for QR detection and decoding. See [cam_pipeline_qr.h](components/cam_pipeline_qr/include/cam_pipeline_qr.h) for the API.

### k_quirc

QR detection and decoding library, evolved from Espressif's quirc component (`espressif_quirc`) through ~15 commits of significant changes: adaptive/bilinear thresholding, contrast stretch preprocessing for low-light conditions, 15%+ performance optimizations, ESP32 memory safeguards (heap-allocated flood-fill stack, SPIRAM-preferred allocations), version cap reduction, and debug visualization.

Currently vendored in `components/k_quirc/`; will eventually move to its own repo as a git submodule.

## Repo Structure

```
esp-camera-pipeline/
  CMakeLists.txt                     # Top-level ESP-IDF component registration
  Kconfig                            # CAM_PIPELINE_DEBUG, log interval
  README.md

  include/
    esp_cam_pipeline.h               # Pipeline public API
    cam_pipeline_camera_driver.h     # Abstract camera driver interface
    cam_pipeline_display_driver.h    # Abstract display driver interface

  src/
    esp_cam_pipeline.c               # Pipeline core (triple buffer, crop, display push, consumer access)

  components/
    cam_pipeline_qr/                 # Reference QR decode consumer
      CMakeLists.txt
      Kconfig
      include/cam_pipeline_qr.h
      src/cam_pipeline_qr.c

    k_quirc/                         # QR library (vendored; future: git submodule)
      CMakeLists.txt
      Kconfig
      include/k_quirc.h
      src/
        k_quirc.c
        k_quirc_decode.c
        k_quirc_identify.c
        k_quirc_internal.h
        k_quirc_version.c
```
