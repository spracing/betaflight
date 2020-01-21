
void pixelInit(void);

void pixelGateAndBlankStart(void);

void pixelOutputDisable(void);

void pixelConfigureDMAForNextField(void);
void pixelXferCpltCallback(struct __DMA_HandleTypeDef *hdma);

void pixelStartDMA(void);
void pixelStopDMA(void);
