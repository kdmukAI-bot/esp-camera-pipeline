# Spec: PPA Hardware Scaling in frame_cb

## Context

The esp-camera-pipeline is being adopted by esp-board-common as the camera
abstraction layer for all apps (viewfinder, QR scanner, etc.). On ESP32-P4
boards, the MIPI-CSI sensor outputs at a fixed resolution (e.g., 800x1280)
that is significantly larger than the display (e.g., 480x800). The pipeline
needs to scale the camera frame down to display dimensions.

## Current State

`frame_cb` in `esp_cam_pipeline.c` currently does:

1. **Software center-crop** (`horizontal_crop`) — copies the center
   `display_width x display_height` pixels from the camera frame at 1:1.
   Requires `camera >= display` in both dimensions.
2. **PPA rotation** (P4 only) — rotates the cropped frame by a configured
   angle using the PPA SRM engine. No scaling.

This works when camera and display resolutions are close (e.g., S3 DVP
configured to output at 320x320 for a 320x480 display). But when the
camera is much larger than the display (800x1280 → 480x800), the 1:1 crop
discards most of the sensor's field of view.

## What Needs to Change

On P4 (where `SOC_PPA_SUPPORTED` is true), replace the two-step
software-crop + PPA-rotate with a **single PPA SRM operation** that does
crop + scale + rotate in one hardware pass. The PPA SRM engine supports all
three simultaneously.

### Scaling Strategy: Uniform Fill

Use the same strategy as the reference implementation (see below):

- Compute a **uniform scale factor**: `scale = max(display_w / camera_w, display_h / camera_h)`
- This "fill" approach ensures the output completely covers the display with
  no black bars, at the cost of cropping some of the input
- Aspect ratio is preserved (same scale for X and Y)
- The input crop region is centered on the camera frame

### Computation (from reference)

```c
float sx = (float)display_w / camera_w;
float sy = (float)display_h / camera_h;
float scale = (sx > sy) ? sx : sy;       // fill, not fit

uint32_t in_w = (uint32_t)(display_w / scale);
uint32_t in_h = (uint32_t)(display_h / scale);
uint32_t off_x = (camera_w - in_w) / 2;
uint32_t off_y = (camera_h - in_h) / 2;
```

### PPA SRM Configuration

```c
ppa_srm_oper_config_t srm_cfg = {
    .in = {
        .buffer = camera_buf,
        .pic_w = camera_w,
        .pic_h = camera_h,
        .block_w = in_w,           // cropped input width
        .block_h = in_h,           // cropped input height
        .block_offset_x = off_x,  // center offset
        .block_offset_y = off_y,
        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
    },
    .out = {
        .buffer = back_buffer,
        .buffer_size = buffer_size,
        .pic_w = display_w,
        .pic_h = display_h,
        .block_offset_x = 0,
        .block_offset_y = 0,
        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
    },
    .rotation_angle = p->ppa_angle,  // existing rotation config
    .scale_x = scale,
    .scale_y = scale,
    .mode = PPA_TRANS_MODE_BLOCKING,
};
ppa_do_scale_rotate_mirror(p->ppa_client, &srm_cfg);
```

## Changes Required

### 1. frame_cb — P4 path (SOC_PPA_SUPPORTED)

When PPA is available and the camera resolution differs from the display
resolution, use the PPA SRM operation to do crop + scale + rotate into
the back buffer directly. This replaces both the `horizontal_crop` call
and the separate PPA rotation block.

The PPA output goes directly into the back buffer (the pipeline's triple-
buffered pool), not into the separate `ppa_buffer`. This eliminates
`ppa_buffer` entirely — the PPA writes straight to the destination.

The existing `horizontal_crop` + separate PPA rotate path can be kept as
a fallback for the case where camera == display dimensions (scale = 1.0),
or the unified PPA path can handle that case too (PPA with scale 1.0 and
no crop offset is effectively a copy + rotate).

### 2. frame_cb — non-PPA path

No change needed. On S3, the camera driver is expected to output at (or
near) the display resolution. The existing `horizontal_crop` handles any
minor dimension mismatch via center-crop at 1:1.

### 3. ppa_buffer removal

The current pipeline allocates a separate `ppa_buffer` as a rotation
staging area: crop into `back_buffer`, then PPA rotate from `back_buffer`
into `ppa_buffer`, then `display_src = ppa_buffer`.

With the unified PPA path, PPA writes directly into `back_buffer`:
camera frame → PPA crop+scale+rotate → `back_buffer`. The separate
`ppa_buffer` and its allocation/cleanup can be removed.

**However**: verify that PPA SRM supports in-place or same-destination
operation correctly, and that the back buffer meets PPA DMA alignment
requirements. The pipeline's `allocate_buffer` uses `heap_caps_malloc`
without alignment — this may need to change to `heap_caps_aligned_alloc`
with the cache line size the PPA requires (typically 64 or 128 bytes).

### 4. Pipeline struct changes (esp_cam_pipeline_internal.h)

In the `#if SOC_PPA_SUPPORTED` block:

- **Remove**: `ppa_buffer`, `ppa_buffer_size` (no longer needed)
- **Keep**: `ppa_client`, `ppa_angle`

### 5. cam_pipeline_create — P4 path

- Remove `ppa_buffer` allocation
- Ensure triple-buffer allocation uses cache-aligned alloc for PPA DMA
  compatibility (e.g., `heap_caps_aligned_alloc(128, size, MALLOC_CAP_SPIRAM)`)
- PPA client registration stays the same

### 6. cam_pipeline_destroy — P4 path

- Remove `ppa_buffer` free

## What NOT to Change

- The public API (`esp_cam_pipeline.h`) — no changes needed. The config
  struct already has `display_width`/`display_height`, and the pipeline
  gets the camera resolution from `camera_driver->get_resolution()`.
- The camera driver interface — drivers continue to deliver frames at
  whatever resolution the sensor produces.
- The display driver interface — continues to receive display-sized frames.
- The consumer API — `lock_frame` / `release_frame` still returns
  display-sized RGB565 frames.
- The non-PPA code path — `horizontal_crop` stays for S3 and any future
  non-P4 targets.

## Reference Implementation

The working PPA scaling code is in the esp-board-common camera viewfinder
app. This is the code being replaced by the pipeline:

**File**: `esp-board-common/apps/camera_viewfinder/main/main.c`, lines 66-106

```c
/* Hardware-accelerated scale via PPA SRM engine.
 * Use uniform scaling to maintain aspect ratio and fill the screen.
 * Pick the larger scale factor so the image covers the display
 * completely, then center-crop the input to match exactly. */
float sx = (float)VIEW_W / src_w;
float sy = (float)VIEW_H / src_h;
float scale = (sx > sy) ? sx : sy;  /* fill (not fit) */
uint32_t in_w = (uint32_t)(VIEW_W / scale);
uint32_t in_h = (uint32_t)(VIEW_H / scale);
uint32_t off_x = (src_w - in_w) / 2;
uint32_t off_y = (src_h - in_h) / 2;

ppa_srm_oper_config_t srm_cfg = {
    .in = {
        .buffer = frame.buf,
        .pic_w = src_w,
        .pic_h = src_h,
        .block_w = in_w,
        .block_h = in_h,
        .block_offset_x = off_x,
        .block_offset_y = off_y,
        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
    },
    .out = {
        .buffer = cam_buf,
        .buffer_size = VIEW_SIZE,
        .pic_w = VIEW_W,
        .pic_h = VIEW_H,
        .block_offset_x = 0,
        .block_offset_y = 0,
        .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
    },
    .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
    .scale_x = scale,
    .scale_y = scale,
    .mode = PPA_TRANS_MODE_BLOCKING,
};
ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_cfg);
```

## Testing

- Build for an ESP32-P4 target and verify PPA scaling produces correct
  output at various camera/display resolution combinations
- Verify that rotation + scaling work together in a single PPA pass
- Verify no visual artifacts at the edges (alignment/rounding)
- Build for an ESP32-S3 target and verify the non-PPA center-crop path
  still compiles and works unchanged
- Run with `CONFIG_CAM_PIPELINE_DEBUG=y` and verify frame rate metrics
  are comparable to (or better than) the previous two-step approach
