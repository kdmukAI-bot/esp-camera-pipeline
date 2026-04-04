/*
 * Camera Pipeline QR Decode Consumer
 *
 * Uses the pipeline's public lock_frame/release_frame interface --
 * exactly the same pattern any external consumer would use.
 */

#include "cam_pipeline_qr.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "cam_pipeline_qr";

/* --- RGB565 to grayscale LUT --- */

static uint8_t *rgb565_lut_build(void) {
    uint8_t *lut = heap_caps_malloc(65536, MALLOC_CAP_SPIRAM);
    if (!lut) {
        ESP_LOGW(TAG, "Failed to allocate RGB565 grayscale LUT");
        return NULL;
    }

    for (uint32_t i = 0; i < 65536; i++) {
        uint8_t r5 = (i >> 11) & 0x1F;
        uint8_t g6 = (i >> 5) & 0x3F;
        uint8_t b5 = i & 0x1F;
        uint8_t r8 = (r5 * 255 + 15) / 31;
        uint8_t g8 = (g6 * 255 + 31) / 63;
        uint8_t b8 = (b5 * 255 + 15) / 31;
        lut[i] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
    }

    return lut;
}

static void rgb565_to_grayscale(const uint8_t *rgb565_data, uint8_t *gray_data,
                                uint32_t width, uint32_t height,
                                const uint8_t *lut) {
    const uint16_t *pixels = (const uint16_t *)rgb565_data;
    uint32_t total = width * height;

    if (lut) {
        for (uint32_t i = 0; i < total; i++) {
            gray_data[i] = lut[pixels[i]];
        }
    } else {
        for (uint32_t i = 0; i < total; i++) {
            uint16_t pixel = pixels[i];
            uint8_t r5 = (pixel >> 11) & 0x1F;
            uint8_t g6 = (pixel >> 5) & 0x3F;
            uint8_t b5 = pixel & 0x1F;
            uint8_t r8 = (r5 * 255 + 15) / 31;
            uint8_t g8 = (g6 * 255 + 31) / 63;
            uint8_t b8 = (b5 * 255 + 15) / 31;
            gray_data[i] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
        }
    }
}

/**
 * Convert a centered square crop of an RGB565 frame to grayscale.
 * Only processes the crop region, writing crop_size * crop_size pixels.
 */
static void rgb565_to_grayscale_cropped(const uint8_t *rgb565_data,
                                        uint8_t *gray_data,
                                        uint32_t frame_width,
                                        uint32_t crop_size,
                                        uint32_t offset_x,
                                        uint32_t offset_y,
                                        const uint8_t *lut) {
    const uint16_t *pixels = (const uint16_t *)rgb565_data;

    for (uint32_t y = 0; y < crop_size; y++) {
        const uint16_t *src_row = pixels + (y + offset_y) * frame_width + offset_x;
        uint8_t *dst_row = gray_data + y * crop_size;

        if (lut) {
            for (uint32_t x = 0; x < crop_size; x++) {
                dst_row[x] = lut[src_row[x]];
            }
        } else {
            for (uint32_t x = 0; x < crop_size; x++) {
                uint16_t pixel = src_row[x];
                uint8_t r5 = (pixel >> 11) & 0x1F;
                uint8_t g6 = (pixel >> 5) & 0x3F;
                uint8_t b5 = pixel & 0x1F;
                uint8_t r8 = (r5 * 255 + 15) / 31;
                uint8_t g8 = (g6 * 255 + 31) / 63;
                uint8_t b8 = (b5 * 255 + 15) / 31;
                dst_row[x] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
            }
        }
    }
}

/* --- Internal state --- */

struct cam_pipeline_qr {
    cam_pipeline_handle_t pipeline;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t crop_size;      /* square crop dimension (min of w, h) */
    uint32_t crop_offset_x;  /* horizontal offset to center crop */
    uint32_t crop_offset_y;  /* vertical offset to center crop */
    cam_pipeline_qr_cb_t on_decoded;
    void *user_ctx;

    k_quirc_t *qr_decoder;
    uint8_t *rgb565_gray_lut;
    TaskHandle_t task_handle;
    SemaphoreHandle_t task_done_sem;
    volatile bool closing;

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    volatile uint32_t consumer_frames;
    volatile uint64_t grayscale_time_us;
    volatile uint64_t quirc_time_us;
    volatile uint32_t qr_detections;
    int64_t last_log_time;
#endif
};

/* --- Debug logging --- */

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
static void log_debug_metrics(struct cam_pipeline_qr *qr) {
    int64_t now = esp_timer_get_time();
    int64_t elapsed_us = now - qr->last_log_time;

    if (elapsed_us < (CONFIG_CAM_PIPELINE_QR_DEBUG_LOG_INTERVAL_MS * 1000)) {
        return;
    }

    float elapsed_sec = elapsed_us / 1000000.0f;
    float consumer_fps = qr->consumer_frames / elapsed_sec;
    float avg_gray_ms = 0;
    float avg_quirc_ms = 0;
    float detections_per_sec = qr->qr_detections / elapsed_sec;

    if (qr->consumer_frames > 0) {
        avg_gray_ms =
            (qr->grayscale_time_us / qr->consumer_frames) / 1000.0f;
        avg_quirc_ms =
            (qr->quirc_time_us / qr->consumer_frames) / 1000.0f;
    }

    ESP_LOGI(TAG,
             "decode=%.1ffps(gray=%.1fms quirc=%.1fms) det/s=%.1f",
             consumer_fps, avg_gray_ms, avg_quirc_ms, detections_per_sec);

    qr->consumer_frames = 0;
    qr->grayscale_time_us = 0;
    qr->quirc_time_us = 0;
    qr->qr_detections = 0;
    qr->last_log_time = now;
}
#endif

/* --- Decode task --- */

static void qr_decode_task(void *pvParameters) {
    struct cam_pipeline_qr *qr = (struct cam_pipeline_qr *)pvParameters;
    k_quirc_result_t qr_result;

    while (!qr->closing) {

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
        log_debug_metrics(qr);
#endif

        const uint8_t *frame = cam_pipeline_lock_frame(qr->pipeline);
        if (!frame) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
        int64_t gray_start, gray_end, quirc_start, quirc_end;
#endif

        uint8_t *qr_buf = k_quirc_begin(qr->qr_decoder, NULL, NULL);
        if (qr_buf) {
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            gray_start = esp_timer_get_time();
#endif
            rgb565_to_grayscale_cropped(frame, qr_buf,
                                       qr->frame_width,
                                       qr->crop_size,
                                       qr->crop_offset_x,
                                       qr->crop_offset_y,
                                       qr->rgb565_gray_lut);
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            gray_end = esp_timer_get_time();
            quirc_start = esp_timer_get_time();
#endif
            k_quirc_end(qr->qr_decoder, false);
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            quirc_end = esp_timer_get_time();
#endif

            int num_codes = k_quirc_count(qr->qr_decoder);
            for (int i = 0; i < num_codes; i++) {
                if (qr->closing) {
                    break;
                }

                k_quirc_error_t err =
                    k_quirc_decode(qr->qr_decoder, i, &qr_result);
                if (err == K_QUIRC_SUCCESS && qr_result.valid) {
#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
                    __atomic_add_fetch(&qr->qr_detections, 1,
                                       __ATOMIC_RELAXED);
#endif
                    qr->on_decoded(qr_result.data.payload,
                                   qr_result.data.payload_len,
                                   &qr_result.data, qr->user_ctx);
                }
            }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
            __atomic_add_fetch(&qr->consumer_frames, 1, __ATOMIC_RELAXED);
            __atomic_add_fetch(&qr->grayscale_time_us,
                               (uint64_t)(gray_end - gray_start),
                               __ATOMIC_RELAXED);
            __atomic_add_fetch(&qr->quirc_time_us,
                               (uint64_t)(quirc_end - quirc_start),
                               __ATOMIC_RELAXED);
#endif
        }

        cam_pipeline_release_frame(qr->pipeline);
    }

    if (qr->task_done_sem) {
        xSemaphoreGive(qr->task_done_sem);
    }
    vTaskSuspend(NULL);
}

/* --- Public API --- */

cam_pipeline_qr_handle_t
cam_pipeline_qr_create(const cam_pipeline_qr_config_t *config) {
    if (!config || !config->pipeline || !config->on_decoded) {
        ESP_LOGE(TAG, "Invalid config: pipeline and callback required");
        return NULL;
    }

    struct cam_pipeline_qr *qr = calloc(1, sizeof(struct cam_pipeline_qr));
    if (!qr) {
        ESP_LOGE(TAG, "Failed to allocate QR consumer struct");
        return NULL;
    }

    qr->pipeline = config->pipeline;
    qr->frame_width = config->frame_width;
    qr->frame_height = config->frame_height;
    qr->on_decoded = config->on_decoded;
    qr->user_ctx = config->user_ctx;

    /* Compute centered square crop (use shorter dimension) */
    uint32_t w = config->frame_width;
    uint32_t h = config->frame_height;
    qr->crop_size = (w < h) ? w : h;
    qr->crop_offset_x = (w - qr->crop_size) / 2;
    qr->crop_offset_y = (h - qr->crop_size) / 2;

    // Build RGB565->grayscale LUT (64KB, SPIRAM)
    qr->rgb565_gray_lut = rgb565_lut_build();
    // Non-fatal if LUT fails -- rgb565_to_grayscale falls back to per-pixel

    qr->qr_decoder = k_quirc_new();
    if (!qr->qr_decoder) {
        ESP_LOGE(TAG, "Failed to create QR decoder");
        goto error;
    }

    if (k_quirc_resize(qr->qr_decoder, qr->crop_size,
                       qr->crop_size) < 0) {
        ESP_LOGE(TAG, "Failed to resize QR decoder");
        goto error;
    }

    qr->task_done_sem = xSemaphoreCreateBinary();
    if (!qr->task_done_sem) {
        ESP_LOGE(TAG, "Failed to create task done semaphore");
        goto error;
    }

#ifdef CONFIG_CAM_PIPELINE_QR_DEBUG
    qr->last_log_time = esp_timer_get_time();
#endif

    // Pin decode task to Core 1 to avoid competing with camera on Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        qr_decode_task, "qr_decode",
        CONFIG_CAM_PIPELINE_QR_TASK_STACK_SIZE, qr,
        CONFIG_CAM_PIPELINE_QR_TASK_PRIORITY,
        &qr->task_handle, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create QR decode task");
        goto error;
    }

    ESP_LOGI(TAG, "QR consumer created: %" PRIu32 "x%" PRIu32
             " (crop %" PRIu32 "x%" PRIu32 " at +%" PRIu32 ",+%" PRIu32 ")",
             config->frame_width, config->frame_height,
             qr->crop_size, qr->crop_size,
             qr->crop_offset_x, qr->crop_offset_y);

    return qr;

error:
    cam_pipeline_qr_destroy(qr);
    return NULL;
}

void cam_pipeline_qr_destroy(cam_pipeline_qr_handle_t handle) {
    if (!handle) {
        return;
    }

    struct cam_pipeline_qr *qr = handle;
    qr->closing = true;

    if (qr->task_handle && qr->task_done_sem) {
        if (xSemaphoreTake(qr->task_done_sem, pdMS_TO_TICKS(500)) !=
            pdTRUE) {
            ESP_LOGW(TAG, "Timeout waiting for QR decode task");
        }
        vTaskDelete(qr->task_handle);
        qr->task_handle = NULL;
    }

    if (qr->task_done_sem) {
        vSemaphoreDelete(qr->task_done_sem);
        qr->task_done_sem = NULL;
    }

    if (qr->qr_decoder) {
        k_quirc_destroy(qr->qr_decoder);
        qr->qr_decoder = NULL;
    }

    if (qr->rgb565_gray_lut) {
        heap_caps_free(qr->rgb565_gray_lut);
        qr->rgb565_gray_lut = NULL;
    }

    free(qr);
    ESP_LOGI(TAG, "QR consumer destroyed");
}
