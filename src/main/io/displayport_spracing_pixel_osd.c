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
 * Author: Dominic Clifton / Seriously Pro Racing
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPRACING_PIXEL_OSD

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/spracing_pixel_osd.h"

#include "config/config.h"

#include "io/displayport_spracing_pixel_osd.h"

#include "osd/font_max7456_12x18.h"
#include "osd/osd.h"

#include "pg/spracing_pixel_osd.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/vcd.h"

displayPort_t spracingPixelOSDDisplayPort;

PG_REGISTER_WITH_RESET_FN(displayPortProfile_t, displayPortProfileSPRacingPixelOSD, PG_DISPLAY_PORT_SPRACING_PIXEL_OSD_CONFIG, 0);

void pgResetFn_displayPortProfileSPRacingPixelOSD(displayPortProfile_t *displayPortProfile)
{
    displayPortProfile->colAdjust = 0;
    displayPortProfile->rowAdjust = 0;

    displayPortProfile->invert = false;         // UNSUPPORTED
    displayPortProfile->blackBrightness = 0;    // UNSUPPORTED
    displayPortProfile->whiteBrightness = 0;    // UNSUPPORTED
}

static int grab(displayPort_t *displayPort)
{
    // FIXME this should probably not have a dependency on the OSD or OSD slave code
    UNUSED(displayPort);
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif

    return 0;
}

static int release(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    return 0;
}

static int clearScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    uint8_t *frameBuffer = frameBuffer_getBuffer(0);
    frameBuffer_erase(frameBuffer);

    return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    // TODO

    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return (16 * 30); // TODO
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, const char *s)
{
    UNUSED(displayPort);

    uint8_t *frameBuffer = frameBuffer_getBuffer(0);

    frameBuffer_slowWriteString(frameBuffer, x * FONT_MAX7456_WIDTH, y * FONT_MAX7456_HEIGHT, (uint8_t *)s, strlen(s));

    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c)
{
    UNUSED(displayPort);

    uint8_t *frameBuffer = frameBuffer_getBuffer(0);
    frameBuffer_slowWriteCharacter(frameBuffer, x * FONT_MAX7456_WIDTH, y * FONT_MAX7456_HEIGHT, c);

    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);

    // TODO
    return false;
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);

    // TODO
    return true;
}

static void resync(displayPort_t *displayPort)
{
    UNUSED(displayPort);

    displayPort->rows = 16; // FIXME hardcoded to PAL.
    displayPort->cols = 30;

    // TODO
}

static int heartbeat(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return 0;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return UINT32_MAX;
}

static const displayPortVTable_t spracingPixelOSDVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
};

displayPort_t *spracingPixelOSDDisplayPortInit(const vcdProfile_t *vcdProfile)
{
    if (
        !spracingPixelOSDInit(spracingPixelOSDConfig(), vcdProfile)
    ) {
        return NULL;
    }

    displayInit(&spracingPixelOSDDisplayPort, &spracingPixelOSDVTable);

    resync(&spracingPixelOSDDisplayPort);
    return &spracingPixelOSDDisplayPort;
}
#endif // USE_SPRACING_PIXEL_OSD
