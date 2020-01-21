/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#include "platform.h"

#include "framebuffer.h"

#include "pixelbuffer.h"

#define PIXEL_WHITE_ON 1
#define PIXEL_WHITE_OFF 0

#define PIXEL_MASK_ON 1
#define PIXEL_MASK_OFF 0

// black is inverted (open drain)
#define PIXEL_BLACK_ON 0
#define PIXEL_BLACK_OFF 1

#define PIXEL_WHITE       ((PIXEL_WHITE_ON  << PIXEL_WHITE_BIT) | (PIXEL_BLACK_OFF << PIXEL_BLACK_BIT))
#define PIXEL_BLACK       ((PIXEL_WHITE_OFF << PIXEL_WHITE_BIT) | (PIXEL_BLACK_ON  << PIXEL_BLACK_BIT))
#define PIXEL_GREY        ((PIXEL_WHITE_ON  << PIXEL_WHITE_BIT) | (PIXEL_BLACK_ON  << PIXEL_BLACK_BIT))
#define PIXEL_TRANSPARENT ((PIXEL_WHITE_OFF << PIXEL_WHITE_BIT) | (PIXEL_BLACK_OFF << PIXEL_BLACK_BIT))

#define PIXEL_WITH_MASK     (PIXEL_MASK_ON << PIXEL_MASK_ENABLE_BIT)
#define PIXEL_WITHOUT_MASK  (PIXEL_MASK_OFF << PIXEL_MASK_ENABLE_BIT)

#if 0
void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex)
{
    // This method only works for BITS_PER_PIXEL == 2
    // And only when PIXEL_BLACK_BIT == WHITE_BLACK_BIT - 1 (adjacent GPIO pins)

#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif
    uint8_t *frameBuffer = frameBuffer_getBuffer(frameBufferIndex);
    uint8_t *frameBufferLine = frameBuffer + (FRAME_BUFFER_LINE_SIZE * lineIndex);
#if USE_SLOW_PIXEL_BUFFER_FILL_METHOD
    // XXX - After compilier optimization this is slower than the implementation below.
    uint8_t *pixel = destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t frameBlock = *(frameBufferLine + i);

        uint8_t mask = (1 << 7) | ( 1 << 6);
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 3) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 2) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 1) << PIXEL_BLACK_BIT;

        mask = mask >> BITS_PER_PIXEL;
        *pixel++ = (frameBlock & mask) >> (BITS_PER_PIXEL * 0) << PIXEL_BLACK_BIT;
    }
#else
    uint32_t *pixels = (uint32_t *)destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t frameBlock = *(frameBufferLine + i);

        *pixels++ = (
            ((frameBlock & (0x03 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x03 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x03 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x03 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        ) << PIXEL_BLACK_BIT;
    }
#endif
    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT; // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif
}
#else


void pixelBuffer_fillFromFrameBuffer(uint8_t *destinationPixelBuffer, uint8_t frameBufferIndex, uint16_t lineIndex)
{
    // Rev B has 4 IO lines for White Source, Black, Mask and White, black and white are NOT adjacent so the bits cannot be copied and shifted together...
#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif

    uint8_t *frameBuffer = frameBuffer_getBuffer(frameBufferIndex);

    uint8_t *frameBufferLine = frameBuffer + (FRAME_BUFFER_LINE_SIZE * lineIndex);

    uint32_t *pixels = (uint32_t *)destinationPixelBuffer;
    for (int i = 0; i < FRAME_BUFFER_LINE_SIZE; i++) {
        uint8_t frameBlock = *(frameBufferLine + i);

#if 0 // old
        uint32_t blackBits = (
            ((frameBlock & (0x01 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x01 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x01 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x01 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        );

        uint32_t whiteBits = (
            ((frameBlock & (0x02 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x02 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x02 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x02 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        );

        *pixels++ = blackBits << (PIXEL_BLACK_BIT - FRAME_BLACK_BIT_OFFSET)
            | whiteBits << (PIXEL_WHITE_BIT - FRAME_WHITE_BIT_OFFSET)
            | whiteBits << (PIXEL_MASK_ENABLE_BIT - FRAME_WHITE_BIT_OFFSET);

        /*
        uint32_t gpioBits = 0;
        gpioBits |= blackBits << (PIXEL_BLACK_BIT - FRAME_BLACK_BIT_OFFSET);
        gpioBits |= whiteBits << (PIXEL_WHITE_BIT - FRAME_WHITE_BIT_OFFSET);
        gpioBits |= whiteBits << (PIXEL_MASK_ENABLE_BIT - FRAME_WHITE_BIT_OFFSET);
        */
#else

        uint32_t frameBlockBits = (
            ((frameBlock & (0x03 << 0)) >> (BITS_PER_PIXEL * 0) << 24) |
            ((frameBlock & (0x03 << 2)) >> (BITS_PER_PIXEL * 1) << 16) |
            ((frameBlock & (0x03 << 4)) >> (BITS_PER_PIXEL * 2) << 8) |
            ((frameBlock & (0x03 << 6)) >> (BITS_PER_PIXEL * 3) << 0)
        );

        uint32_t blackGpioBitMask  = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_BLACK_BIT;
        uint32_t whiteGpioBitMask  = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_WHITE_BIT;
        uint32_t whiteSourceSelectGpioBitMask  = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_WHITE_SOURCE_SELECT_BIT;
        uint32_t maskGpioBitMask   = ((1 << 24) | (1 << 16) | (1 << 8) | (1 << 0)) << PIXEL_MASK_ENABLE_BIT;


        // gpio/frame level for black is inverted, so 0 = ON, 1 = OFF.

        uint32_t gpioBlackBits = (frameBlockBits << (PIXEL_BLACK_BIT - FRAME_BLACK_BIT_OFFSET)) & blackGpioBitMask;
        uint32_t gpioWhiteBits = (frameBlockBits << (PIXEL_WHITE_BIT - FRAME_WHITE_BIT_OFFSET)) & whiteGpioBitMask;
        uint32_t gpioWhiteSourceSelectBits = (frameBlockBits << (PIXEL_WHITE_SOURCE_SELECT_BIT - FRAME_WHITE_BIT_OFFSET)) & whiteSourceSelectGpioBitMask;

        uint32_t gpioNotBlackBits = ~(gpioBlackBits) & blackGpioBitMask; // now 1 = ON, 0 = OFF, for each black bit.

        uint32_t frameMaskOnBlackBits    = gpioBlackBits >> (PIXEL_BLACK_BIT);
        uint32_t frameMaskOnNotBlackBits = gpioNotBlackBits >> (PIXEL_BLACK_BIT);
        uint32_t frameMaskOnWhiteBits    = gpioWhiteBits >> (PIXEL_WHITE_BIT);

        uint32_t gpioMaskOnBlackBits = (frameMaskOnBlackBits << PIXEL_MASK_ENABLE_BIT) & maskGpioBitMask;
        uint32_t gpioMaskOnNotBlackBits = (frameMaskOnNotBlackBits << PIXEL_MASK_ENABLE_BIT) & maskGpioBitMask;
        uint32_t gpioMaskOnWhiteBits = (frameMaskOnWhiteBits << PIXEL_MASK_ENABLE_BIT) & maskGpioBitMask;

        uint32_t gpioWhiteBitsForEachBlackOn = (frameMaskOnNotBlackBits << PIXEL_WHITE_BIT) & whiteGpioBitMask;

        uint32_t gpioWhiteSourceSelectBitsForEachBlackOn = (frameMaskOnNotBlackBits << PIXEL_WHITE_SOURCE_SELECT_BIT) & whiteSourceSelectGpioBitMask;

        uint32_t gpioBlackBitsForEachWhiteOn = ~(frameMaskOnWhiteBits << PIXEL_BLACK_BIT) & blackGpioBitMask;

        //
        // GOOD
        //

        // Black = unmasked,unmasked White = fixed,unmasked.
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits; // works fine

        // Black = fixed, masked, White = fixed,unmasked
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnNotBlackBits | gpioWhiteBitsForEachBlackOn; // works

        // Black = fixed, masked, White = fixed,masked
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits | gpioWhiteBitsForEachBlackOn; // works

        // Black = DAC, masked, White = fixed,masked
        uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits | gpioWhiteSourceSelectBitsForEachBlackOn;

        // Black = white,Masked, White = black,fixed,masked
        //uint32_t gpioBits = gpioBlackBitsForEachWhiteOn | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits | gpioWhiteBitsForEachBlackOn; // works, produces blacks that are light grey and whites that are dark grey.

        // Black = unmasked, White = DAC,unmasked.
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteSourceSelectBits; // works

        // Black = fixed,masked, White = DAC,masked.
        //uint32_t gpioBits = gpioBlackBits | gpioMaskOnWhiteBits | gpioWhiteSourceSelectBits | gpioMaskOnNotBlackBits | gpioWhiteBitsForEachBlackOn; // inverted black/white

        // Black = DAC,masked, White = DAC,masked.
        //uint32_t gpioBits = gpioBlackBits | gpioMaskOnWhiteBits | gpioWhiteSourceSelectBits | gpioMaskOnNotBlackBits | gpioWhiteSourceSelectBitsForEachBlackOn; // works

        //
        // BAD
        //
        // Black = unmasked, White = fixed, masked.
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits; // works but white pixels are a bit dark, visible black shadow on right hand side of white pixels as mask is turned off, voltage after whites goes quite low.
        //uint32_t gpioBits = gpioBlackBitsForEachWhiteOn | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits; // works but blacks are 0v.


        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnNotBlackBits; // doesn't work, why? - because voltage goes to 0 and comparator triggers!
        //uint32_t gpioBits = gpioBlackBits | gpioMaskOnWhiteBits; // doesn't work.  odd frame is legible during v line level detection.
        //uint32_t gpioBits = gpioBlackBits | gpioMaskOnNotBlackBits; // doesn't work.
        //uint32_t gpioBits = BLOCK_TRANSPARENT | gpioMaskOnNotBlackBits; // doesn't work.
        //uint32_t gpioBits = BLOCK_TRANSPARENT | gpioMaskOnWhiteBits; // doesn't work.
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnNotBlackBits; // doesn't work
        //uint32_t gpioBits = gpioBlackBits | gpioWhiteBits | gpioMaskOnWhiteBits | gpioMaskOnBlackBits; // doesn't work,

#ifdef DEBUG_PATTERN_BARS
        const int lineOffset = 32;
        const int linesPerPattern = 4;
        const int patternCount = 16; // 4 IO lines = 16 combinations
        if (lineIndex < lineOffset || lineIndex >= lineOffset + (patternCount * linesPerPattern)) {
            *pixels++ = gpioBits;
        } else {

            uint8_t pattern = (((lineIndex - lineOffset) / linesPerPattern) % patternCount);

            bool patternCauses0VSignal = (pattern == 4 || pattern == 6);
            if (patternCauses0VSignal) {
                pattern = BLOCK_FILL;
            }

            uint32_t patternBits = (pattern << 24) | (pattern << 16) | (pattern << 8) | (pattern << 0);
            uint32_t gpioPatternBits = patternBits << PIXEL_CONTROL_FIRST_BIT;
            *pixels++ = gpioPatternBits;
        }
#else
        *pixels++ = gpioBits;
#endif // DEBUG_PATTERN_BARS

#endif

    }
    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT & ~(PIXEL_MASK_ON << PIXEL_MASK_ENABLE_BIT); // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
#ifdef DEBUG_PIXEL_BUFFER_FILL
    pixelDebug2Toggle();
#endif
}
#endif

#ifdef DEBUG
void pixelBuffer_createTestPattern1(uint8_t *destinationPixelBuffer, uint8_t bands)
{
    uint8_t pattern = 0;
    uint8_t patterns = 8;
    uint8_t bandWidth = PIXEL_COUNT / bands;
    for (int i = 0; i < PIXEL_COUNT; i++) {
        uint8_t band = i / bandWidth;

        pattern = band % patterns;

        uint8_t pixelValue = 0x00;

        if (pattern == 0) {
            pixelValue = PIXEL_BLACK;
            if (i & 1) {
                pixelValue = PIXEL_WHITE;
            }
        } else if (pattern == 1) {
            pixelValue = PIXEL_WHITE;
        } else if (pattern == 2) {
            pixelValue = PIXEL_TRANSPARENT;
            if (i & 1) {
                pixelValue = PIXEL_BLACK;
            }
        } else if (pattern == 3) {
            pixelValue = PIXEL_TRANSPARENT;
        } else if (pattern == 4) {
            pixelValue = PIXEL_GREY;
        } else if (pattern == 5) {
            pixelValue = PIXEL_TRANSPARENT;
            if (i & 1) {
                pixelValue = PIXEL_WHITE;
            }
        } else if (pattern == 6) {
            pixelValue = PIXEL_WHITE;
            if (i & 1) {
                pixelValue = PIXEL_BLACK;
            }
        } else if (pattern == 7){
            pixelValue = PIXEL_BLACK;
        }

        destinationPixelBuffer[i] = pixelValue;
    }
    destinationPixelBuffer[PIXEL_COUNT] = PIXEL_TRANSPARENT; // IMPORTANT!  The white source/black sink must be disabled before the SYNC signal, otherwise we change the sync voltage level.
}
#endif
