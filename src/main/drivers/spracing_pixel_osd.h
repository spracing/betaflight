/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Author: Dominic Clifton
 */

#pragma once

//
// General
//


typedef enum {
    MODE_UNKNOWN = 0,
    MODE_PAL,
    MODE_NTSC
} videoMode_t;

extern volatile videoMode_t detectedVideoMode; // unstable value

struct vcdProfile_s;
struct spracingPixelOSDConfig_s;
bool    spracingPixelOSDInit(const struct spracingPixelOSDConfig_s *spracingPixelOSDConfig, const struct vcdProfile_s *vcdProfile);

//
// Pixel Line buffer
//

extern uint8_t *fillPixelBuffer;

//
// Frame Buffer
//

extern volatile bool fillLineNow;
extern volatile bool frameFlag;
extern volatile uint16_t fillLineIndex;

extern volatile bool videoModeChanged;

void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex);

uint8_t *frameBuffer_getBuffer(uint8_t index);
uint16_t frameBuffer_getCounter(void);

//
// Drawing
//

void frameBuffer_slowWriteString(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t *message, uint8_t messageLength);
void frameBuffer_slowWriteCharacter(uint8_t *frameBuffer, uint16_t x, uint16_t y, uint8_t characterIndex);
