/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

//
// Frame Buffer
//

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "configuration.h"

#define BITS_PER_PIXEL 2 // the current implementation only supports 2.
#define BITS_PER_BYTE 8

#define FRAME_BUFFER_LINE_SIZE ((PIXEL_COUNT / BITS_PER_BYTE) * BITS_PER_PIXEL)

void frameBuffer_eraseInit(void);
void frameBuffer_erase(uint8_t *frameBuffer);

uint8_t *frameBuffer_getBuffer(uint8_t index);

//
// Drawing
//

// FIXME currently using the same format as the pixel buffer (i.e. black inverted), but this should probably be normalized for maintenance reasons now.

#define FRAME_WHITE_ON 1
#define FRAME_WHITE_OFF 0
#define FRAME_BLACK_ON 0
#define FRAME_BLACK_OFF 1

#define FRAME_BLACK_BIT_OFFSET 0
#define FRAME_WHITE_BIT_OFFSET 1

#define FRAME_PIXEL_WHITE       ((FRAME_WHITE_ON  << FRAME_WHITE_BIT_OFFSET) | (FRAME_BLACK_OFF << FRAME_BLACK_BIT_OFFSET))
#define FRAME_PIXEL_BLACK       ((FRAME_WHITE_OFF << FRAME_WHITE_BIT_OFFSET) | (FRAME_BLACK_ON  << FRAME_BLACK_BIT_OFFSET))
#define FRAME_PIXEL_GREY        ((FRAME_WHITE_ON  << FRAME_WHITE_BIT_OFFSET) | (FRAME_BLACK_ON  << FRAME_BLACK_BIT_OFFSET))
#define FRAME_PIXEL_TRANSPARENT ((FRAME_WHITE_OFF << FRAME_WHITE_BIT_OFFSET) | (FRAME_BLACK_OFF << FRAME_BLACK_BIT_OFFSET))

// Use FRAME_PIXEL_* for 'mode` arguments below.

void frameBuffer_slowWriteString(uint8_t *frameBuffer, uint16_t x, uint16_t y, const uint8_t *message, uint8_t messageLength);
void frameBuffer_slowWriteCharacter(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t characterIndex);
void framebuffer_drawVerticalLine(uint8_t *frameBuffer, uint16_t x, uint16_t y0, uint16_t y1, uint8_t mode);
void framebuffer_drawLine(uint8_t *frameBuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t mode);
void framebuffer_drawRectangle(uint8_t *frameBuffer, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t mode);
