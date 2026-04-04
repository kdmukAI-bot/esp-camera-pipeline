/*
 * Built-in QR decode consumer
 *
 * Uses the pipeline's lock_frame/release_frame interface — exactly the
 * same pattern any external consumer would use. This serves as a
 * reference consumer implementation.
 *
 * Extracted and adapted from Kern main/qr/scanner.c qr_decode_task().
 */

#include "esp_cam_pipeline_internal.h"
#include "qr_decode.h"
#include "rgb565_lut.h"
#include <esp_log.h>
#include <esp_timer.h>

static const char *TAG = "cam_pipeline_qr";

/*
 * QR decode task — runs on Core 1, loops until pipeline is closing.
 *
 * Each iteration:
 *   1. Lock the most recent frame (returns NULL if paused or no frame yet)
 *   2. Convert RGB565 → grayscale via LUT into k_quirc's internal buffer
 *   3. Run QR detection + decode
 *   4. Fire callback for each valid QR code
 *   5. Release the frame
 */
static void qr_decode_task(void *pvParameters) {
    struct cam_pipeline *p = (struct cam_pipeline *)pvParameters;
    k_quirc_result_t qr_result;

    while (!p->closing && !p->destruction_in_progress) {

#ifdef CONFIG_CAM_PIPELINE_DEBUG
        log_debug_metrics(p);
#endif

        const uint8_t *frame =
            cam_pipeline_lock_frame((cam_pipeline_handle_t)p);
        if (!frame) {
            // No frame available (not yet, or frame access paused)
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
        int64_t gray_start, gray_end, quirc_start, quirc_end;
#endif

        uint8_t *qr_buf = k_quirc_begin(p->qr_decoder, NULL, NULL);
        if (qr_buf) {
#ifdef CONFIG_CAM_PIPELINE_DEBUG
            gray_start = esp_timer_get_time();
#endif
            rgb565_to_grayscale(frame, qr_buf,
                                p->display_width, p->display_height,
                                p->rgb565_gray_lut);
#ifdef CONFIG_CAM_PIPELINE_DEBUG
            gray_end = esp_timer_get_time();
            quirc_start = esp_timer_get_time();
#endif
            k_quirc_end(p->qr_decoder, false);
#ifdef CONFIG_CAM_PIPELINE_DEBUG
            quirc_end = esp_timer_get_time();
#endif

            int num_codes = k_quirc_count(p->qr_decoder);
            for (int i = 0; i < num_codes; i++) {
                if (p->closing || p->destruction_in_progress) {
                    break;
                }

                k_quirc_error_t err =
                    k_quirc_decode(p->qr_decoder, i, &qr_result);
                if (err == K_QUIRC_SUCCESS && qr_result.valid) {
#ifdef CONFIG_CAM_PIPELINE_DEBUG
                    __atomic_add_fetch(&p->qr_detections, 1,
                                       __ATOMIC_RELAXED);
#endif
                    // Fire the raw callback — app handles parsing/progress
                    p->on_qr_decoded(qr_result.data.payload,
                                     qr_result.data.payload_len,
                                     &qr_result.data, p->user_ctx);
                }
            }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
            __atomic_add_fetch(&p->grayscale_time_us,
                               (uint64_t)(gray_end - gray_start),
                               __ATOMIC_RELAXED);
            __atomic_add_fetch(&p->quirc_time_us,
                               (uint64_t)(quirc_end - quirc_start),
                               __ATOMIC_RELAXED);
#endif
        }

        cam_pipeline_release_frame((cam_pipeline_handle_t)p);
    }

    // Signal that the task has finished
    if (p->qr_task_done_sem) {
        xSemaphoreGive(p->qr_task_done_sem);
    }
    vTaskSuspend(NULL);
}

bool qr_decode_init(struct cam_pipeline *p) {
    // Build RGB565→grayscale LUT (64KB, SPIRAM)
    p->rgb565_gray_lut = rgb565_lut_build();
    // Non-fatal if LUT fails — rgb565_to_grayscale falls back to per-pixel

    p->qr_decoder = k_quirc_new();
    if (!p->qr_decoder) {
        ESP_LOGE(TAG, "Failed to create QR decoder");
        goto error;
    }

    if (k_quirc_resize(p->qr_decoder, p->display_width,
                       p->display_height) < 0) {
        ESP_LOGE(TAG, "Failed to resize QR decoder");
        goto error;
    }

    p->qr_task_done_sem = xSemaphoreCreateBinary();
    if (!p->qr_task_done_sem) {
        ESP_LOGE(TAG, "Failed to create QR task done semaphore");
        goto error;
    }

    // Pin decode task to Core 1 to avoid competing with camera on Core 0
    BaseType_t result = xTaskCreatePinnedToCore(
        qr_decode_task, "qr_decode",
        CONFIG_CAM_PIPELINE_QR_DECODE_TASK_STACK_SIZE, p,
        CONFIG_CAM_PIPELINE_QR_DECODE_TASK_PRIORITY,
        &p->qr_decode_task_handle, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create QR decode task");
        goto error;
    }

    return true;

error:
    qr_decode_cleanup(p);
    return false;
}

void qr_decode_cleanup(struct cam_pipeline *p) {
    if (!p) {
        return;
    }

    // Wait for decode task to exit
    if (p->qr_decode_task_handle && p->qr_task_done_sem) {
        if (xSemaphoreTake(p->qr_task_done_sem, pdMS_TO_TICKS(500)) !=
            pdTRUE) {
            ESP_LOGW(TAG, "Timeout waiting for QR decode task");
        }
        vTaskDelete(p->qr_decode_task_handle);
        p->qr_decode_task_handle = NULL;
    }

    if (p->qr_task_done_sem) {
        vSemaphoreDelete(p->qr_task_done_sem);
        p->qr_task_done_sem = NULL;
    }

    if (p->qr_decoder) {
        k_quirc_destroy(p->qr_decoder);
        p->qr_decoder = NULL;
    }

    if (p->rgb565_gray_lut) {
        rgb565_lut_free(p->rgb565_gray_lut);
        p->rgb565_gray_lut = NULL;
    }
}
