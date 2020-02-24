/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working OSD system.
 */

#include <stdbool.h>

#include "platform.h"

#include "configuration.h"

#include "videotiming.h"
#include "videosystem.h"

static const videoTimings_t ntscVideoTimings = {
  .lineNs = VIDEO_NTSC_LINE_LEN,
  .syncShortNs = VIDEO_NTSC_SYNC_SHORT,
  .syncHSyncNs = VIDEO_NTSC_SYNC_HSYNC,
//  .blankingNs = VIDEO_NTSC_BLANKING,
  .frontPorchNs = VIDEO_NTSC_FRONT_PORCH,
  .backPorchNs = VIDEO_NTSC_BACK_PORCH,

  .lineCount = NTSC_LINES,
};

static const videoTimings_t palVideoTimings = {
  .lineNs = VIDEO_PAL_LINE_LEN,
  .syncShortNs = VIDEO_PAL_SYNC_SHORT,
  .syncHSyncNs = VIDEO_PAL_SYNC_HSYNC,
//  .blankingNs = VIDEO_PAL_BLANKING,
  .frontPorchNs = VIDEO_PAL_FRONT_PORCH,
  .backPorchNs = VIDEO_PAL_BACK_PORCH,

  .lineCount = PAL_LINES,
};

static videoPulseTimings_t localVideoPulseTimings = {0};
videoPulseTimings_t *videoPulseTimings = &localVideoPulseTimings;

const videoTimings_t *videoTimings;

static void recalculatePulseTimings(const videoTimings_t *vt, videoPulseTimings_t *vp)
{
  //#define VIDEO_SYNC_LO_BROAD       ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC)
  vp->lowBroad.periodNs = (vt->lineNs / 2) - vt->syncHSyncNs;

  //#define VIDEO_SYNC_LO_BROAD_MIN   _US_TO_CLOCKS(VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC/2.0)
  //#define VIDEO_SYNC_LO_BROAD_MAX   _US_TO_CLOCKS(VIDEO_SYNC_LO_BROAD + VIDEO_SYNC_HSYNC/2.0)
  vp->lowBroad.minNs = vp->lowBroad.periodNs - (vt->syncHSyncNs / 2);
  vp->lowBroad.maxNs = vp->lowBroad.periodNs + (vt->syncHSyncNs / 2);

  vp->lowBroad.minClocks = _NS_TO_CLOCKS(vp->lowBroad.minNs);
  vp->lowBroad.maxClocks = _NS_TO_CLOCKS(vp->lowBroad.maxNs);


  //#define VIDEO_SYNC_HI_BROAD     (VIDEO_SYNC_HSYNC)
  vp->highBroad.periodNs = vt->syncHSyncNs;

  //#define VIDEO_SYNC_HI_VSYNC     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC)
  vp->highVSync.periodNs = (vt->lineNs / 2) - vt->syncHSyncNs;
  //#define VIDEO_SYNC_HI_SHORT     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_SHORT)
  vp->highShort.periodNs = (vt->lineNs / 2) - vt->syncShortNs;

  //#define VIDEO_SYNC_VSYNC_MIN        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC - (VIDEO_SYNC_HI_VSYNC - VIDEO_SYNC_HI_BROAD)/2.0)
  //#define VIDEO_SYNC_VSYNC_MAX        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC + (VIDEO_SYNC_HI_SHORT - VIDEO_SYNC_HI_VSYNC)/2.0)
  vp->highVSync.minNs = vp->highVSync.periodNs - (vp->highVSync.periodNs - vp->highBroad.periodNs) / 2;
  vp->highVSync.maxNs = vp->highVSync.periodNs + (vp->highShort.periodNs - vp->highVSync.periodNs) / 2;

  vp->highVSync.minClocks = _NS_TO_CLOCKS(vp->highVSync.minNs);
  vp->highVSync.maxClocks = _NS_TO_CLOCKS(vp->highVSync.maxNs);

  //#define VIDEO_SYNC_SHORT_MIN    _US_TO_CLOCKS(VIDEO_SYNC_SHORT / 2.0)
  //#define VIDEO_SYNC_SHORT_MAX    _US_TO_CLOCKS(VIDEO_SYNC_SHORT +  (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
  vp->lowShort.minNs = vt->syncShortNs / 2;
  vp->lowShort.maxNs = vt->syncShortNs + (vt->syncHSyncNs - vt->syncShortNs) / 2;

  vp->lowShort.minClocks = _NS_TO_CLOCKS(vp->lowShort.minNs);
  vp->lowShort.maxClocks = _NS_TO_CLOCKS(vp->lowShort.maxNs);

  //#define VIDEO_SYNC_HSYNC_MIN    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC - (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
  //#define VIDEO_SYNC_HSYNC_MAX    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC + (VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC)/2.0)
  vp->lowVSync.minNs = vt->syncHSyncNs - (vt->syncHSyncNs - vt->syncShortNs) / 2;
  vp->lowVSync.maxNs = vt->syncHSyncNs + (vp->lowBroad.periodNs - vt->syncHSyncNs) / 2;

  vp->lowVSync.minClocks = _NS_TO_CLOCKS(vp->lowVSync.minNs);
  vp->lowVSync.maxClocks = _NS_TO_CLOCKS(vp->lowVSync.maxNs);
}

void refreshVideoTimings(videoSystem_t videoSystem)
{
  switch (videoSystem) {
  default:
  case VIDEO_SYSTEM_PAL:
    videoTimings = &palVideoTimings;
    break;
  case VIDEO_SYSTEM_NTSC:
    videoTimings = &ntscVideoTimings;
    break;
  }

  recalculatePulseTimings(videoTimings, videoPulseTimings);
}
