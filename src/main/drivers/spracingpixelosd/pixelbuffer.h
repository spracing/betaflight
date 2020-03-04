/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#pragma once

#include "configuration.h"

//
// FrameBuffer to PixelBuffer
//

void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t *frameBuffer, uint16_t lineIndex);

//
// PixelBuffer Debug
//

void pixelBuffer_createTestPattern1(uint8_t *destinationPixelBuffer, uint8_t bands);

