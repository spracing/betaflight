
//
// Implementation only
//

extern TIM_HandleTypeDef htim1;   // Video Timer
extern TIM_HandleTypeDef htim2;   // Pulse timer
extern TIM_HandleTypeDef htim15;  // Pixel generation
extern COMP_HandleTypeDef hcomp2; // Sync comparator
extern DAC_HandleTypeDef hdac1;   // Sync threshold and White level

extern uint8_t *fillPixelBuffer;
extern uint8_t *outputPixelBuffer;

void spracingPixelOSDPause(void);
void spracingPixelOSDRestart(void);

void setVideoSourceVoltageMv(uint32_t whiteMv);
