
//
// **GPL** FIXME Some of the time definitions in this file come from TinyOSD which is GPL code, replace prior to release.
//

//
// Video Format
//

#define PAL_LINES 625
#define NTSC_LINES 525

//
// Video Timing
//

#ifdef USE_NTSC
#define VIDEO_LINE_LEN            63.556  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#define VIDEO_BLANKING            10.900  // us
#define VIDEO_FRONT_PORCH          1.500  // us
#else
#define VIDEO_LINE_LEN            64.000  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#define VIDEO_BLANKING            12.000  // us
#define VIDEO_FRONT_PORCH          1.650  // us
#endif
#define VIDEO_FIELD_ODD            1
#define VIDEO_FIELD_EVEN           (1-VIDEO_FIELD_ODD)
#define VIDEO_FIRST_FIELD          VIDEO_FIELD_ODD     // ODD (NTSC)
#define VIDEO_SECOND_FIELD         (1-(VIDEO_FIRST_FIELD))

// timing for high level (from shortest to longest period)
// (1) [HI] BROAD SYNC: t ~ VIDEO_SYNC_HSYNC
// (2) [HI] VSYNC+DATA: t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_HSYNC
// (3) [HI] SHORT SYNC: t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_SHORT
// (4) [HI] VIDEO DATA: t ~ VIDEO_LINE_LEN - VIDEO_SYNC_HSYNC
//
#define VIDEO_SYNC_HI_BROAD     (VIDEO_SYNC_HSYNC)
#define VIDEO_SYNC_HI_VSYNC     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC)
#define VIDEO_SYNC_HI_SHORT     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_SHORT)
#define VIDEO_ACTIVE            (VIDEO_LINE_LEN - VIDEO_BLANKING)
#define VIDEO_SYNC_HI_DATA      (VIDEO_LINE_LEN)
//
// -> valid vsync = .... (1)---[xxx(2)xxx]---(3)------(4)
//
#define VIDEO_SYNC_VSYNC_MIN        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC - (VIDEO_SYNC_HI_VSYNC - VIDEO_SYNC_HI_BROAD)/2.0)
#define VIDEO_SYNC_VSYNC_MAX        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC + (VIDEO_SYNC_HI_SHORT - VIDEO_SYNC_HI_VSYNC)/2.0)

// timing for low level (from shortest to longest period)
// (1) [LO] SHORT SYNC: t ~ 2.0us
// (2) [LO] HSYNC     : t ~ 4.7us
// (3) [LO] BROAD     : t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_HSYNC
//
//
// short sync =  (1)xxx]---(2)------(3)
//
#define VIDEO_SYNC_SHORT_MIN    _US_TO_CLOCKS(VIDEO_SYNC_SHORT / 2.0)
#define VIDEO_SYNC_SHORT_MAX    _US_TO_CLOCKS(VIDEO_SYNC_SHORT +  (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
//
// hsync      =  (1)---[xxx(2)xxx]---(3)
//
#define VIDEO_SYNC_HSYNC_MIN    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC - (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
#define VIDEO_SYNC_HSYNC_MAX    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC + (VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC)/2.0)
//
// broad      = (1)------(2)---[xxx(3)]
//
#define VIDEO_SYNC_LO_BROAD       (VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC
#define VIDEO_SYNC_LO_BROAD_MIN   _US_TO_CLOCKS(VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC/2.0)
#define VIDEO_SYNC_LO_BROAD_MAX   _US_TO_CLOCKS(VIDEO_SYNC_LO_BROAD + VIDEO_SYNC_HSYNC/2.0)
