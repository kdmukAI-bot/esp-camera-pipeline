/*
 * ESP Camera Pipeline — Abstract Camera Driver Interface
 *
 * Implement this interface to connect a camera hardware backend
 * (e.g., P4 CSI via V4L2, S3 DVP via esp_camera) to the pipeline engine.
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

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
    esp_err_t (*get_resolution)(void *handle, uint32_t *width,
                                uint32_t *height);

    /* Optional camera controls — NULL if not supported */
    esp_err_t (*set_ae_target)(void *handle, uint32_t level);
    esp_err_t (*set_focus)(void *handle, uint32_t position);
    bool (*has_focus_motor)(void *handle);
} cam_pipeline_camera_driver_t;
