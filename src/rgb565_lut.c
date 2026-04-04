/*
 * RGB565 → Grayscale LUT
 * Extracted from Kern main/qr/scanner.c
 */

#include "rgb565_lut.h"
#include <esp_heap_caps.h>
#include <esp_log.h>

static const char *TAG = "cam_pipeline_lut";

uint8_t *rgb565_lut_build(void) {
    uint8_t *lut = heap_caps_malloc(65536, MALLOC_CAP_SPIRAM);
    if (!lut) {
        ESP_LOGW(TAG, "Failed to allocate RGB565 grayscale LUT");
        return NULL;
    }

    for (uint32_t i = 0; i < 65536; i++) {
        uint8_t r5 = (i >> 11) & 0x1F;
        uint8_t g6 = (i >> 5) & 0x3F;
        uint8_t b5 = i & 0x1F;
        // BT.601 luma with full 8-bit precision:
        // expand RGB565 to 8-bit, then Y = (77*R + 150*G + 29*B) >> 8
        uint8_t r8 = (r5 * 255 + 15) / 31;
        uint8_t g8 = (g6 * 255 + 31) / 63;
        uint8_t b8 = (b5 * 255 + 15) / 31;
        lut[i] = (uint8_t)((77 * r8 + 150 * g8 + 29 * b8) >> 8);
    }

    return lut;
}

void rgb565_lut_free(uint8_t *lut) {
    if (lut) {
        heap_caps_free(lut);
    }
}

void rgb565_to_grayscale(const uint8_t *rgb565_data, uint8_t *gray_data,
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
