/*
 * RGB565 → Grayscale conversion via pre-built LUT
 */

#pragma once

#include <stdint.h>

/**
 * Build the 64KB RGB565→grayscale lookup table in SPIRAM.
 * Returns the LUT pointer, or NULL on allocation failure.
 * Uses BT.601 luma: Y = (77*R + 150*G + 29*B) >> 8
 */
uint8_t *rgb565_lut_build(void);

/**
 * Free a previously built LUT.
 */
void rgb565_lut_free(uint8_t *lut);

/**
 * Convert RGB565 image to grayscale using the LUT.
 * Falls back to per-pixel computation if lut is NULL.
 */
void rgb565_to_grayscale(const uint8_t *rgb565_data, uint8_t *gray_data,
                         uint32_t width, uint32_t height,
                         const uint8_t *lut);
