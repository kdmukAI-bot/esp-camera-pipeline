/*
 * ESP Camera Pipeline — Abstract Display Driver Interface
 *
 * Implement this interface to connect a display backend
 * (e.g., LVGL image widget, raw framebuffer) to the pipeline engine.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

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
