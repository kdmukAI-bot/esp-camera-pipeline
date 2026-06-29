/*
 * Camera Pipeline QR Decode Consumer
 *
 * A reference frame consumer that uses the pipeline's public lock/release
 * interface to decode QR codes from camera frames. Demonstrates the same
 * pattern any external consumer would use.
 */

#pragma once

#include "esp_cam_pipeline.h"
#include <k_quirc.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct cam_pipeline_qr *cam_pipeline_qr_handle_t;

/**
 * Called from decode task for each successfully decoded QR code.
 * payload/len is the raw decoded data. metadata has version, ECC, data type.
 * Must return quickly -- runs in decode task context.
 */
typedef void (*cam_pipeline_qr_cb_t)(const uint8_t *payload, size_t len,
                                     const k_quirc_data_t *metadata,
                                     void *user_ctx);

typedef struct {
    cam_pipeline_handle_t pipeline; // Pipeline to consume frames from
    uint32_t frame_width;           // Frame dimensions (for QR decoder sizing)
    uint32_t frame_height;
    cam_pipeline_qr_cb_t on_decoded; // Callback for decoded QR results
    void *user_ctx;                  // Passed to callback
} cam_pipeline_qr_config_t;

/**
 * Create the QR decode consumer. Allocates k_quirc, grayscale LUT,
 * and spawns a decode task that locks frames from the pipeline,
 * converts to grayscale, and runs QR detection.
 * Returns handle on success, NULL on failure.
 */
cam_pipeline_qr_handle_t
cam_pipeline_qr_create(const cam_pipeline_qr_config_t *config);

/**
 * Stop the decode task and free all QR resources.
 * The pipeline itself is not affected.
 */
void cam_pipeline_qr_destroy(cam_pipeline_qr_handle_t handle);

/**
 * Decode reliability + timing snapshot, refreshed once per debug-log interval.
 * Populated only when CONFIG_CAM_PIPELINE_QR_DEBUG is enabled.
 */
typedef struct {
    float decode_fps;          /* frames processed per second */
    float gray_ms;             /* avg grayscale conversion time */
    float quirc_ms;            /* avg quirc identify+decode time */
    float detections_per_sec;  /* valid decodes per second */
    float identify_pct;        /* % of frames where a QR was located */
    float decode_pct;          /* % of frames that fully decoded (reliability) */
    uint32_t total_decodes;    /* monotonic count of valid decodes since create */
    float last_px_per_module;  /* measured px/module of the latest decode (HUD) */
    uint16_t last_modules;     /* module count of the latest decode */
} cam_pipeline_qr_debug_stats_t;

/**
 * Copy the latest reliability/timing snapshot into *out.
 * Returns true if stats are available (debug build + at least one interval
 * elapsed), false otherwise (out is zeroed).
 */
bool cam_pipeline_qr_get_debug_stats(cam_pipeline_qr_handle_t handle,
                                     cam_pipeline_qr_debug_stats_t *out);
