/*
 * Built-in QR decode consumer — internal header
 *
 * The QR decode task is a reference consumer that uses the pipeline's
 * lock_frame/release_frame interface, exactly as any external consumer would.
 */

#pragma once

/* Forward declaration — full definition in esp_cam_pipeline_internal.h */
struct cam_pipeline;

/**
 * Initialize QR decode resources and spawn the decode task.
 * Called from cam_pipeline_create() when on_qr_decoded is non-NULL.
 */
bool qr_decode_init(struct cam_pipeline *pipeline);

/**
 * Stop the decode task and free all QR resources.
 * Called from cam_pipeline_destroy().
 */
void qr_decode_cleanup(struct cam_pipeline *pipeline);
