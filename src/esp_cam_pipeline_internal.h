/*
 * ESP Camera Pipeline — Internal Header
 *
 * Contains the full pipeline struct definition, shared between
 * esp_cam_pipeline.c and qr_decode.c. Not part of the public API.
 */

#pragma once

#include "esp_cam_pipeline.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <k_quirc.h>

#if SOC_PPA_SUPPORTED
#include <driver/ppa.h>
#endif

#define EVENT_TASK_RUN BIT(0)
#define EVENT_DELETE   BIT(1)

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
    size_t buffer_size;
    uint8_t *front_buffer;  // most recently completed frame (display source)
    uint8_t *back_buffer;   // being written by camera crop
    uint8_t *locked_buffer; // held by consumer (NULL if none)
    SemaphoreHandle_t buffer_mutex;

    // QR decode consumer (optional)
    cam_pipeline_qr_cb_t on_qr_decoded;
    void *user_ctx;
    k_quirc_t *qr_decoder;
    uint8_t *rgb565_gray_lut;
    TaskHandle_t qr_decode_task_handle;
    SemaphoreHandle_t qr_task_done_sem;

    // Frame access control
    volatile bool frame_access_paused;

    // Lifecycle control
    EventGroupHandle_t event_group;
    volatile bool closing;
    volatile bool destruction_in_progress;
    volatile bool is_initialized;
    volatile int active_frame_operations;

    // PPA rotation (P4 only)
#if SOC_PPA_SUPPORTED
    ppa_client_handle_t ppa_client;
    uint8_t *ppa_buffer;
    size_t ppa_buffer_size;
    ppa_srm_rotation_angle_t ppa_angle;
#endif

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    // Per-stage metrics (atomic counters, safe across cores)
    volatile uint32_t camera_frames;
    volatile uint32_t display_frames;
    volatile uint32_t display_skips;
    volatile uint32_t consumer_frames;
    volatile uint64_t consumer_lock_wait_us;
    volatile uint64_t consumer_hold_time_us;
    // QR-specific (only meaningful when on_qr_decoded != NULL)
    volatile uint64_t grayscale_time_us;
    volatile uint64_t quirc_time_us;
    volatile uint32_t qr_detections;
    int64_t last_log_time;
#endif
};

#ifdef CONFIG_CAM_PIPELINE_DEBUG
/* Periodic debug metrics logging — called from decode task */
void log_debug_metrics(struct cam_pipeline *p);
#endif
