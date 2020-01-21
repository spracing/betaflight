
#include "configuration.h"

typedef struct spracingPixelOSDIO_s {
    IO_t blackPin;
    IO_t whitePin;
    IO_t syncInPin;
    IO_t debug1Pin;
    IO_t debug2Pin;
#ifdef DEBUG_BLANKING
    IO_t blankingDebugPin;
#endif
#ifdef DEBUG_GATING
    IO_t gatingDebugPin;
#endif
    IO_t whiteSourceSelectPin;
    IO_t maskEnablePin;
} spracingPixelOSDIO_t;

void spracingPixelOSD_initIO(void);

//
// OSD Pin Debugging
//
void pixelDebug1Set(bool state);
void pixelDebug2Set(bool state);
void pixelDebug1Low(void);
void pixelDebug2Low(void);
void pixelDebug1High(void);
void pixelDebug2High(void);
void pixelDebug1Toggle(void);
void pixelDebug2Toggle(void);

