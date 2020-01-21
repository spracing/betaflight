
//
// Horizontal
//

#define HORIZONTAL_RESOLUTION 720
#define RESOLUTION_SCALE 2

#define PIXEL_COUNT (HORIZONTAL_RESOLUTION / RESOLUTION_SCALE)

//
// Vertical
//

#define PAL_VISIBLE_LINES 288  // MAX7456 (16 rows * 18 character height)
#define NTSC_VISIBLE_LINES 234 // MAX7456 (13 rows * 18 character height)

//
// Timing
//

#define TIMER_BUS_CLOCK   200000000
#define TIMER_CLOCK  100000000

#define TIMER_CLOCKS_PER_US                      (TIMER_CLOCK / 1000000)
#define _US_TO_CLOCKS(__us)                      ((uint32_t)((__us) * TIMER_CLOCKS_PER_US))


//
// DEBUG
//

#if 1
#define DEBUG_PULSE_STATISTICS
#define DEBUG_PULSE_ERRORS      // debug led 2
#define DEBUG_OSD_EVENTS
#define DEBUG_PIXEL_DMA         // debug led 1
#define DEBUG_BLANKING          // signal on M8
#define DEBUG_GATING            // signal on M7
#else
#define DEBUG_COMP_TRIGGER      // debug led 2
#define DEBUG_PATTERN_BARS
#define DEBUG_PIXEL_BUFFER_FILL
#define DEBUG_LAST_HALF_LINE
#define DEBUG_PIXEL_BUFFER
#define DEBUG_SYNC_PWM
#define DEBUG_FIELD_START
#define DEBUG_SHORT_PULSE
#define DEBUG_FIRST_SYNC_PULSE
#endif
