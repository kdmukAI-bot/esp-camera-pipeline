# ESP Camera Pipeline: Implementation Plan

## Context

The Kern project contains a tightly integrated camera → QR decode → live preview pipeline built for the ESP32-P4 with CSI camera and LVGL display. This pipeline is valuable beyond Kern — any ESP32-based project that needs camera capture with live preview could use it. The goal is to extract the platform-agnostic core into a standalone repo (`esp-camera-pipeline`) with abstract camera and display driver interfaces, so it can be wired up to different hardware (P4 CSI, S3 DVP, various displays) without modification.

The primary consumer is MicroPython on ESP32 — the pipeline is a C module that MicroPython code controls for use cases like QR scanning, entropy capture from camera images, or any future application that needs live camera frames.

## Design Decisions

- **Camera input** is abstracted through a driver interface (P4 CSI vs S3 DVP vs future hardware)
- **Display output** is abstracted through a driver interface (LVGL vs raw framebuffer vs future)
- The app can **overlay custom LVGL widgets** on top of the live video feed
- **Triple-buffered** with lock/release for safe zero-copy consumer access (see rationale below)
- All buffers are **pre-allocated at create**; zero allocations in the hot loop
- **QR decoding** is included as a built-in optional consumer (thin layer over k_quirc), but the pipeline is general-purpose — any frame consumer can use the same lock/release interface
- `QUIRC_MAX_VERSION` becomes a Kconfig option
- This is its **own repo**, not part of ESP Board Common — it's an application engine, not a hardware driver. ESP Board Common provides silicon-level drivers (camera sensors, display panels); this engine consumes them through abstract interfaces. The app snaps them together.

## Triple Buffer Architecture

### Why not double buffer (Kern's current design)

Kern uses two display buffers with no locking between the camera producer and QR decode consumer. The camera writes to the back buffer, swaps it to front, and passes the front buffer pointer to the QR decoder via a size-1 queue. The implicit safety assumption is that the decoder finishes before the camera wraps around to the same buffer:

```
Frame N:   write to B → swap front→B → queue B to decoder
Frame N+1: write to A → swap front→A → queue A to decoder
Frame N+2: write to B ← OVERWRITES B (decoder may still be reading B)
```

On the ESP32-P4, this race never triggers in practice because k_quirc decodes in ~3-5ms while the camera frame period is ~33ms. But on the ESP32-S3 — with slower CPU, slower PSRAM bus, and potentially the same or higher frame rate — QR decoding could take 20-50ms, making the race window real.

### Three independent frame rates

The triple buffer architecture enables three fully decoupled rates:

| Stage | Rate | Governed by |
|---|---|---|
| **Camera** | Sensor native (e.g., 30fps) | Hardware — the ceiling that drives everything |
| **Display** | Up to camera rate, skips if busy | Display driver responsiveness (e.g., LVGL lock availability) |
| **Consumer** | Whatever pace the consumer can sustain | Processing time (QR decode, entropy hash, etc.) |

The camera is the only hard clock. It produces frames at its native rate regardless of what the display or consumer are doing. The display takes what it can — if the LVGL lock can't be acquired non-blockingly, the frame callback skips the display push and moves on. The consumer runs at its own pace via lock/release — it locks the current front buffer when it's ready, processes for however long it needs, releases, and comes back for the next one whenever it's ready. It naturally gets the most recent complete frame at that moment.

None of the three stages block each other. A slow consumer doesn't stall the camera or the display. A slow display doesn't stall the camera or the consumer. The camera just keeps producing, and each downstream stage independently takes what it can and skips what it can't.

### Per-stage debug metrics

When `CONFIG_CAM_PIPELINE_DEBUG` is enabled, the pipeline tracks independent metrics for each stage:

| Metric | Stage | What it measures |
|---|---|---|
| Camera FPS | Camera | Frames delivered by camera driver per second |
| Display FPS | Display | Frames successfully pushed to display per second |
| Display skip rate | Display | Frames where display push was skipped (lock contention) |
| Consumer FPS | Consumer | Frames locked + processed per second |
| Consumer lock wait | Consumer | Average time to acquire frame lock |
| Consumer hold time | Consumer | Average time between lock and release |

These are tracked via atomic counters (safe across cores) and logged at a configurable interval (default 2 seconds, matching Kern's current `FPS_LOG_INTERVAL_MS`). For QR decode mode, additional k_quirc-specific metrics are included: grayscale conversion time, quirc detection time, and decode success rate.

The per-stage breakdown makes it immediately visible where a bottleneck lives — whether the camera is underperforming, the display is dropping frames due to LVGL contention, or the consumer is falling behind.

Metrics are available through two channels:

1. **Serial console**: The pipeline logs via `ESP_LOGI` at the configured interval (default 2 seconds) for headless debugging.
2. **Public stats getter**: The app can read computed stats at any time and render them into LVGL labels on the overlay parent — the pipeline exposes the numbers, the app owns the UI.

```c
#ifdef CONFIG_CAM_PIPELINE_DEBUG
typedef struct {
    float camera_fps;
    float display_fps;
    float display_skip_pct;
    float consumer_fps;
    float avg_consumer_lock_wait_ms;
    float avg_consumer_hold_time_ms;
    // QR-specific (zeroed if preview-only mode)
    float avg_grayscale_ms;
    float avg_quirc_ms;
    float qr_detections_per_sec;
} cam_pipeline_debug_stats_t;

esp_err_t cam_pipeline_get_debug_stats(cam_pipeline_handle_t handle,
                                            cam_pipeline_debug_stats_t *stats);
#endif
```

### Triple buffer with lock/release

Three buffers rotate through three roles:

| Buffer | Role | Who uses it |
|---|---|---|
| A | **Locked** by consumer | QR decode (or entropy hash, or any consumer) reading — however long it takes |
| B | **Front** (display) | Display showing this frame |
| C | **Back** (writing) | Camera filling next frame |

When a new camera frame arrives, the frame callback picks whichever buffer is neither front nor locked as the next back buffer. The camera and display keep cycling at full frame rate while the consumer holds its lock indefinitely.

When the consumer finishes and releases the lock, the buffer returns to the pool. The next lock request gets whichever buffer is the current front at that moment — the most recent complete frame.

### Memory cost

The third buffer's memory cost is fixed at create time and modest relative to total pipeline allocation:

| Resolution | Per buffer | 2→3 buffer delta | Total pipeline (QR mode) |
|---|---|---|---|
| 320×320 (S3 typical) | 200KB | +200KB | ~930KB |
| 480×480 | 450KB | +450KB | ~2MB |
| 640×640 (P4 typical) | 800KB | +800KB | ~3.4MB |

On an 8MB PSRAM module (standard for S3 dev boards running MicroPython + LVGL), the delta is negligible. The third buffer adds zero runtime overhead — the frame callback just selects from three buffers instead of two.

### Consumer access pattern

```c
// Lock the most recent complete frame (zero-copy, returns pointer)
const uint8_t *buf = cam_pipeline_lock_frame(handle);
if (buf) {
    // ... read from buf as long as needed, safe from camera writes ...
    cam_pipeline_release_frame(handle);
}
```

This replaces the size-1 queue used by QR decode in Kern. The built-in QR decode consumer uses lock/release internally, and external C consumers (e.g., an entropy hashing module) use the same interface. A `capture_frame()` convenience (lock → memcpy → release) could be added later if a use case arises.

## Lifecycle

```
create(config)  →  handle               # allocate everything, begin streaming
pause_decode(handle)                     # stop QR decoding, keep camera + preview live
resume_decode(handle)                    # resume QR decoding
lock_frame(handle)  →  pointer           # zero-copy access to most recent frame
release_frame(handle)                    # release locked frame back to pool
set_ae_target(handle, val)               # adjust exposure (anytime between create/destroy)
set_focus(handle, val)                   # adjust focus (anytime between create/destroy)
destroy(handle)                          # stop everything, free all resources
```

The C API uses create/destroy naming matching Kern's existing convention — both calls do full resource allocation/deallocation. MicroPython holds the config as Python attributes and passes them to `create()` each time. There is no lightweight init; the 64KB grayscale LUT (when QR mode is active) is rebuilt in ~1ms, not worth persisting.

**QR scanning mode** (`on_qr_decoded != NULL`): Pipeline allocates k_quirc, grayscale LUT, and spawns a decode task that uses lock/release internally to process frames.

**Preview-only mode** (`on_qr_decoded == NULL`): Just camera + display, no decode resources allocated. Used for entropy capture or any use case that only needs raw frames via lock/release or capture_frame.

**Pause/resume:** Between create/destroy, the user may want to adjust exposure or focus while seeing the live preview. `pause_decode()` stops the decode task from consuming frames while the camera and preview keep running — the user sees the effect of their adjustments in real-time without wasting decode cycles on transitional frames. Camera controls (`set_ae_target`, `set_focus`) work anytime between create/destroy regardless of pause state.

## Source Material (from Kern)

Files to extract/adapt:

| Kern path | Disposition | Notes |
|---|---|---|
| `components/k_quirc/` (all files) | Vendored copy (submodule later) | Already clean, no app coupling. Works on S3 and P4. See k_quirc section below |
| `main/qr/scanner.c` lines 464-521 | Extract | Buffer allocation, RGB565→grayscale LUT + conversion |
| `main/qr/scanner.c` lines 523-627 | Extract + simplify | `qr_decode_task()` — remove parser/progress/completion, replace with raw callback via lock/release |
| `main/qr/scanner.c` lines 629-711 | Extract | `qr_decoder_init()` / `qr_decoder_cleanup()` — remove `qr_parser` refs |
| `main/qr/scanner.c` lines 753-867 | Extract + abstract | `camera_video_frame_operation()`, `horizontal_crop_cam_to_display()` — replace double-buffer with triple-buffer, replace LVGL calls with display driver |
| `main/qr/scanner.c` lines 869-974 | **Do not extract** | `camera_init()` — P4/Waveshare-specific, replaced by camera driver interface |
| `main/qr/scanner.c` lines 984-1048 | **Do not extract** | `qr_scanner_page_create()` — app-specific UI, replaced by pipeline create |
| `main/qr/scanner.c` lines 1062-1149 | Extract + adapt | Destroy/cleanup sequence — replace app_video calls with driver calls |
| `main/qr/scanner.c` lines 110-210 | Extract | Perf debug metrics (`QR_PERF_DEBUG`), guarded by Kconfig |
| `components/video/` | Reference only | Informs P4 driver implementation but not copied directly |

**Excluded entirely:** `main/qr/parser.c`, `main/qr/parser.h`, `components/bbqr/`, `components/cUR/`, all progress indicator code (lines 213-328), settings overlay (lines 353-462), completion timer (lines 330-342), theme calls, app navigation.

## Repo Structure

```
esp-camera-pipeline/
  CMakeLists.txt                     # Top-level ESP-IDF component registration
  Kconfig                            # CAM_PIPELINE_DEBUG, task stack/priority, etc.
  LICENSE
  README.md

  include/
    esp_cam_pipeline.h               # Pipeline public API
    cam_pipeline_camera_driver.h  # Abstract camera driver interface
    cam_pipeline_display_driver.h # Abstract display driver interface

  src/
    esp_cam_pipeline.c               # Pipeline core (triple buffer, crop, display push, consumer access)
    qr_decode.c                      # Built-in QR decode consumer (thin layer over k_quirc)
    qr_decode.h                      # Internal header
    rgb565_lut.c                     # Grayscale LUT build + conversion
    rgb565_lut.h                     # Internal header

  components/
    k_quirc/                           # Vendored copy (future: git submodule)
      CMakeLists.txt
      Kconfig
      include/
        k_quirc.h
      src/
        k_quirc.c
        k_quirc_decode.c
        k_quirc_identify.c
        k_quirc_internal.h
        k_quirc_version.c

```

## Public API: `esp_cam_pipeline.h`

```c
#pragma once

#include "cam_pipeline_camera_driver.h"
#include "cam_pipeline_display_driver.h"
#include <esp_err.h>
#include <k_quirc.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct cam_pipeline *cam_pipeline_handle_t;

/**
 * Called from decode task for each successfully decoded QR code.
 * payload/len is the raw decoded data. metadata has version, ECC, data type.
 * Must return quickly — runs in decode task context.
 */
typedef void (*cam_pipeline_qr_cb_t)(const uint8_t *payload, size_t len,
                                          const k_quirc_data_t *metadata,
                                          void *user_ctx);

typedef struct {
    uint32_t display_width;              // Cropped frame width for display + consumers
    uint32_t display_height;             // Cropped frame height for display + consumers

    const cam_pipeline_camera_driver_t *camera_driver;
    const void *camera_config;           // Opaque, passed to camera_driver->init()

    const cam_pipeline_display_driver_t *display_driver;
    const void *display_config;          // Opaque, passed to display_driver->init()
    void *display_parent;                // e.g. lv_obj_t* for LVGL, NULL for raw

    // Built-in QR decode consumer (optional)
    cam_pipeline_qr_cb_t on_qr_decoded;  // NULL = no QR decoding (preview-only)
    void *user_ctx;                           // Passed to QR callback
} cam_pipeline_config_t;

/**
 * Allocate all resources (triple buffer, display surface, camera hardware),
 * begin streaming. If on_qr_decoded is non-NULL, also creates QR decode
 * task + k_quirc + grayscale LUT.
 * Returns handle on success, NULL on failure.
 */
cam_pipeline_handle_t cam_pipeline_create(const cam_pipeline_config_t *config);

/**
 * Stop streaming, tear down all tasks, free all resources.
 * Handle is invalid after this call.
 */
void cam_pipeline_destroy(cam_pipeline_handle_t handle);

/* --- Frame access for external consumers --- */

/**
 * Lock the most recent complete frame buffer. Returns a direct pointer
 * to the RGB565 pixel data — no copy. The buffer will not be overwritten
 * by the camera while locked. Returns NULL if no frame is available yet.
 *
 * The caller MUST call release_frame() when done. Holding the lock too
 * long does not cause corruption — the camera and display continue using
 * the other two buffers — but it reduces the buffer pool and may cause
 * the camera to drop frames if both remaining buffers are also in use.
 */
const uint8_t *cam_pipeline_lock_frame(cam_pipeline_handle_t handle);

/**
 * Release a previously locked frame buffer back to the pool.
 */
void cam_pipeline_release_frame(cam_pipeline_handle_t handle);

/* --- Camera control (callable anytime between create/destroy) --- */

esp_err_t cam_pipeline_set_ae_target(cam_pipeline_handle_t handle, uint8_t target);
esp_err_t cam_pipeline_set_focus(cam_pipeline_handle_t handle, uint16_t position);
bool      cam_pipeline_has_focus_motor(cam_pipeline_handle_t handle);

/* --- QR decode control (no-op if on_qr_decoded was NULL) --- */

/**
 * Pause QR decoding. Camera streaming and display preview continue.
 * Useful while adjusting exposure/focus so decode doesn't waste cycles
 * on transitional frames.
 */
void cam_pipeline_pause_decode(cam_pipeline_handle_t handle);

/**
 * Resume QR decoding after pause.
 */
void cam_pipeline_resume_decode(cam_pipeline_handle_t handle);

/* --- Display overlay --- */

/**
 * Get overlay parent for app UI widgets (display-driver-specific).
 * Returns lv_obj_t* for LVGL driver, NULL for raw framebuffer drivers.
 * App creates children on this object — they render on top of video feed.
 */
void *cam_pipeline_get_overlay_parent(cam_pipeline_handle_t handle);

/* --- Debug stats (only available when CONFIG_CAM_PIPELINE_DEBUG is set) --- */

#ifdef CONFIG_CAM_PIPELINE_DEBUG
typedef struct {
    float camera_fps;
    float display_fps;
    float display_skip_pct;
    float consumer_fps;
    float avg_consumer_lock_wait_ms;
    float avg_consumer_hold_time_ms;
    float avg_grayscale_ms;        // 0 if preview-only mode
    float avg_quirc_ms;            // 0 if preview-only mode
    float qr_detections_per_sec;   // 0 if preview-only mode
} cam_pipeline_debug_stats_t;

/**
 * Read current debug metrics. Stats are computed from counters since
 * the last internal log interval reset. App can poll this to render
 * metrics into LVGL labels on the overlay parent.
 */
esp_err_t cam_pipeline_get_debug_stats(cam_pipeline_handle_t handle,
                                            cam_pipeline_debug_stats_t *stats);
#endif
```

## Camera Driver Interface: `cam_pipeline_camera_driver.h`

```c
#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Frame callback signature. The driver calls this for each captured frame.
 * rgb565_buf is valid only for the duration of the call.
 */
typedef void (*cam_pipeline_frame_cb_t)(uint8_t *rgb565_buf,
                                             uint32_t width, uint32_t height,
                                             void *user_ctx);

typedef struct {
    void *(*init)(const void *platform_config);
    esp_err_t (*start)(void *handle, cam_pipeline_frame_cb_t frame_cb,
                       void *user_ctx, int core_id);
    esp_err_t (*stop)(void *handle);
    void (*deinit)(void *handle);
    esp_err_t (*get_resolution)(void *handle, uint32_t *width, uint32_t *height);

    /* Optional camera controls — NULL if not supported */
    esp_err_t (*set_ae_target)(void *handle, uint32_t level);
    esp_err_t (*set_focus)(void *handle, uint32_t position);
    bool (*has_focus_motor)(void *handle);
} cam_pipeline_camera_driver_t;
```

## Display Driver Interface: `cam_pipeline_display_driver.h`

```c
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    void *(*init)(void *parent, uint32_t width, uint32_t height,
                  const void *driver_config);

    /**
     * Push a frame to the display. Called from camera streaming context
     * at frame rate. Must be non-blocking — dropping a frame is OK,
     * stalling the camera is not.
     */
    bool (*push_frame)(void *handle, const uint8_t *rgb565_buf,
                       uint32_t width, uint32_t height);

    void (*deinit)(void *handle);

    /**
     * Return an object the app can parent overlay widgets to.
     * For LVGL: returns lv_obj_t*. For raw framebuffer: returns NULL.
     */
    void *(*get_overlay_parent)(void *handle);
} cam_pipeline_display_driver_t;
```

## Engine Core: `esp_cam_pipeline.c`

### Pipeline struct

```c
struct cam_pipeline {
    // Config
    uint32_t display_width;
    uint32_t display_height;

    // Drivers
    const cam_pipeline_camera_driver_t *camera_driver;
    void *camera_handle;
    const cam_pipeline_display_driver_t *display_driver;
    void *display_handle;

    // Triple-buffered frame pool
    uint8_t *buffers[3];
    uint8_t *front_buffer;         // currently displayed
    uint8_t *back_buffer;          // being written by camera
    uint8_t *locked_buffer;        // held by consumer (NULL if none)
    SemaphoreHandle_t buffer_mutex; // protects buffer role assignments

    // QR decode consumer (optional, only when on_qr_decoded != NULL)
    cam_pipeline_qr_cb_t on_qr_decoded;
    void *user_ctx;
    k_quirc_t *qr_decoder;
    uint8_t *rgb565_gray_lut;
    TaskHandle_t qr_decode_task_handle;
    SemaphoreHandle_t qr_task_done_sem;
    volatile bool decode_paused;

    // Lifecycle control
    EventGroupHandle_t event_group;
    volatile bool closing;
    volatile bool destruction_in_progress;
    volatile bool is_initialized;
    volatile int active_frame_operations;

    // PPA rotation (P4 only, compiled conditionally)
#if SOC_PPA_SUPPORTED
    ppa_client_handle_t ppa_client;
    uint8_t *ppa_buffer;
    size_t ppa_buffer_size;
    ppa_srm_rotation_angle_t ppa_angle;
#endif

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    // Per-stage metrics (atomic counters, safe across cores)
    volatile uint32_t camera_frames;       // frames delivered by camera
    volatile uint32_t display_frames;      // frames pushed to display
    volatile uint32_t display_skips;       // display pushes skipped (lock contention)
    volatile uint32_t consumer_frames;     // frames locked by consumer
    volatile uint64_t consumer_lock_wait_us;  // cumulative lock acquisition time
    volatile uint64_t consumer_hold_time_us;  // cumulative lock hold time
    // QR-specific (only when on_qr_decoded != NULL)
    volatile uint64_t grayscale_time_us;
    volatile uint64_t quirc_time_us;
    volatile uint32_t qr_detections;
    int64_t last_log_time;
#endif
};
```

### `cam_pipeline_create()` sequence

1. Allocate pipeline struct
2. Allocate three frame buffers (display_width × display_height × 2 each, SPIRAM)
3. Create buffer mutex, event group
4. Call `display_driver->init()` → display handle
5. Call `camera_driver->init()` → camera handle
6. Conditionally set up PPA counter-rotation (`#if SOC_PPA_SUPPORTED`)
7. **If `on_qr_decoded != NULL` (QR decode mode):**
   - Build RGB565→grayscale LUT (64KB SPIRAM)
   - Create k_quirc decoder, resize to display_width × display_height
   - Create done semaphore
   - Spawn QR decode task pinned to Core 1
8. Call `camera_driver->start()` with internal `frame_operation_cb` on Core 0
9. Mark `is_initialized = true`
10. Return handle

### `frame_operation_cb()` (internal, called by camera driver from Core 0)

1. Atomic increment `active_frame_operations`
2. Guard: check `closing`, `destruction_in_progress`, `is_initialized`
3. Acquire buffer mutex
4. Select back buffer: whichever of the 3 buffers is neither front nor locked
5. Release buffer mutex
6. `horizontal_crop()` camera buffer → back buffer
7. Conditionally PPA counter-rotate
8. Call `display_driver->push_frame()` with the (possibly rotated) buffer
9. Acquire buffer mutex
10. Promote back → front (swap pointers)
11. Release buffer mutex
12. Atomic decrement `active_frame_operations`

### `qr_decode_task()` (Core 1, only exists when QR callback provided)

The built-in QR decode consumer uses the same lock/release interface as external consumers:

1. Call `cam_pipeline_lock_frame()` (blocks briefly on mutex, gets front buffer pointer)
2. Skip if `decode_paused` → release and retry
3. `rgb565_to_grayscale()` via LUT into k_quirc's internal grayscale buffer
4. `k_quirc_end()` → detect patterns
5. For each detected code: `k_quirc_decode()` → if valid, call `on_qr_decoded(payload, len, metadata, user_ctx)`
6. `cam_pipeline_release_frame()`
7. Loop until `closing` or delete event

This makes QR decode a reference consumer — it demonstrates exactly the same pattern any external consumer would use.

### `cam_pipeline_lock_frame()` / `release_frame()`

```c
const uint8_t *cam_pipeline_lock_frame(cam_pipeline_handle_t handle) {
    xSemaphoreTake(handle->buffer_mutex, portMAX_DELAY);
    if (!handle->front_buffer || handle->locked_buffer) {
        xSemaphoreGive(handle->buffer_mutex);
        return NULL;  // no frame yet, or already locked
    }
    handle->locked_buffer = handle->front_buffer;
    xSemaphoreGive(handle->buffer_mutex);
    return handle->locked_buffer;
}

void cam_pipeline_release_frame(cam_pipeline_handle_t handle) {
    xSemaphoreTake(handle->buffer_mutex, portMAX_DELAY);
    handle->locked_buffer = NULL;
    xSemaphoreGive(handle->buffer_mutex);
}
```

### `cam_pipeline_destroy()` sequence

1. Set `destruction_in_progress = true`, `closing = true`
2. Clear task-run event bit, set delete event bit
3. Spin-wait for `active_frame_operations` to drain (300ms max)
4. `camera_driver->stop()` then `camera_driver->deinit()`
5. **If QR decode mode:** wait for decode task semaphore, delete task, free LUT, k_quirc decoder
6. `display_driver->deinit()`
7. Free: all three frame buffers, PPA buffer, buffer mutex, event group
8. Free pipeline struct (handle is now invalid)

## Kconfig

```kconfig
menu "Camera Pipeline"

    config CAM_PIPELINE_DEBUG
        bool "Enable pipeline performance metrics"
        default n
        help
            Enables per-stage FPS logging (camera, display, consumer)
            with timing breakdowns. Logged at the configured interval.

    config CAM_PIPELINE_DEBUG_LOG_INTERVAL_MS
        int "Debug metrics log interval (ms)"
        default 2000
        depends on CAM_PIPELINE_DEBUG

    config CAM_PIPELINE_QR_DECODE_TASK_STACK_SIZE
        int "QR decode task stack size"
        default 32768

    config CAM_PIPELINE_QR_DECODE_TASK_PRIORITY
        int "QR decode task priority"
        default 5

endmenu
```

k_quirc gets its own Kconfig:

```kconfig
menu "K-Quirc QR Decoder"

    config K_QUIRC_MAX_VERSION
        int "Maximum QR code version (1-40)"
        default 25
        range 1 40
        help
            Higher versions support larger QR codes but use more memory.
            Version 25 = 117x117 modules. Version 15 = 77x77 modules.
            Most Bitcoin/crypto QR codes fit within version 15.

    config K_QUIRC_ADAPTIVE_THRESHOLD
        bool "Enable adaptive threshold"
        default y

    config K_QUIRC_BILINEAR_THRESHOLD
        bool "Enable bilinear threshold interpolation"
        default y

    config K_QUIRC_DEBUG
        bool "Enable QR decoder debug visualization"
        default n

endmenu
```

k_quirc_internal.h changes: `QUIRC_MAX_VERSION` becomes `CONFIG_K_QUIRC_MAX_VERSION`. k_quirc.h derives `K_QUIRC_MAX_BITMAP` and `K_QUIRC_MAX_PAYLOAD` from the version constant at compile time.

## k_quirc as a Separate Repo

k_quirc doesn't exist as a standalone repo or published component. It was created in-tree in Kern, starting from Espressif's quirc component (`espressif_quirc`) and evolved through ~15 commits of significant changes: adaptive/bilinear thresholding, 15%+ performance optimizations, ESP32 memory safeguards (heap-allocated flood-fill stack, SPIRAM-preferred allocations), version cap reduction, and debug visualization.

These modifications make it a distinct library, not a trivial fork. It should eventually get its own repo (`k-quirc`) so that:
- It has its own version history and can be updated independently
- esp-camera-pipeline references it as a git submodule at a pinned commit
- Kern can also switch to submodule reference (eliminating the in-tree copy)
- Other projects can use k_quirc directly without pulling in the pipeline engine
- It could optionally be published to the ESP Component Registry

**Sequencing:** For now, copy k_quirc into esp-camera-pipeline as a vendored component. The Kern author will publish it as a standalone repo, at which point esp-camera-pipeline switches `components/k_quirc/` to a git submodule and Kern does the same.

## Build System

### `CMakeLists.txt` (top-level component)

```cmake
idf_component_register(
    SRCS
        "src/esp_cam_pipeline.c"
        "src/rgb565_lut.c"
        "src/qr_decode.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES k_quirc freertos esp_timer
    PRIV_REQUIRES esp_hw_support
)

# PPA support (P4 only) — detected automatically
if(CONFIG_SOC_PPA_SUPPORTED)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE CAM_PIPELINE_PPA_SUPPORTED)
    idf_component_get_property(ppa_lib esp_driver_ppa COMPONENT_LIB)
    target_link_libraries(${COMPONENT_LIB} PRIVATE ${ppa_lib})
endif()
```

## Changes to Kern After Extraction

Once esp-camera-pipeline exists, Kern's `main/qr/scanner.c` would be refactored to:

1. Include `esp_cam_pipeline.h`
2. In the existing `camera_init()` / `qr_scanner_page_create()`, call `cam_pipeline_create()` with:
   - P4 CSI camera driver + Waveshare BSP I2C handle
   - LVGL display driver + app's parent widget
   - An `on_qr_decoded` callback that feeds into the existing parser.c logic
3. Build app-specific UI (title, settings button/overlay, progress indicators) as overlays via `cam_pipeline_get_overlay_parent()`
4. Wire settings sliders to `cam_pipeline_set_ae_target()` / `cam_pipeline_set_focus()`
5. Remove all the code that moved into the engine (~600 lines)
6. Keep parser.c, progress indicators, completion detection, settings overlay — all app-level

## MicroPython Integration Pattern

MicroPython is the command-and-control layer. Frame consumers (QR decoding, entropy hashing) are implemented in C and use the pipeline's `lock_frame`/`release_frame` API directly. MicroPython never handles raw frame data — it receives end results (decoded QR bytes, final entropy hash) from the C consumers.

```python
import cam_pipeline

# QR scanning — C-level decode task calls back with raw QR bytes
cam = cam_pipeline.create(config, on_qr_decoded=my_qr_handler)
# ... my_qr_handler receives decoded QR payload in MicroPython ...
cam_pipeline.destroy(cam)

# Entropy capture — C-level entropy module uses lock_frame internally,
# MicroPython gets the final hash
cam = cam_pipeline.create(config)  # preview-only, no QR decode
entropy_hash = entropy_module.collect(cam, num_frames=50)
cam_pipeline.destroy(cam)
```

## Verification

1. **Pipeline core compiles**: Verify the pipeline core compiles for both ESP32-P4 and ESP32-S3 targets (PPA code conditionally excluded on S3)
2. **k_quirc Kconfig**: Set `K_QUIRC_MAX_VERSION` to 15, verify it compiles and the max payload/bitmap constants shrink accordingly
3. **Preview-only mode**: Create with `on_qr_decoded=NULL`, verify camera streams and display updates without allocating k_quirc/LUT/decode task
4. **Triple buffer safety**: Verify that a consumer with unpredictable timing — both slow holds and wildly varying request rates — does not corrupt frames or stall the camera
5. **Lock/release from decode task**: Verify the built-in QR decode consumer correctly locks, processes, and releases frames
6. **Kern integration**: Replace Kern's scanner internals with esp-camera-pipeline, verify identical QR scanning behavior with the existing parser.c on top
7. **Decode callback**: Verify the `on_qr_decoded` callback fires with correct payload bytes matching what parser.c currently receives at Kern's scanner.c:578-580
