void SDCARD_ReadBlock(uint32_t addr, uint8_t sdCardBuffer[]);
void SDCARD_WriteBlock(uint32_t addr, uint8_t sdCardBuffer[]);
void SDCARD_HCInitialize(uint8_t verbose);
void SDCARD_Initialize(uint8_t verbose);
void SDCARD_SetIdle(uint8_t verbose);
uint8_t SDCARD_SetBlockLength(void);
uint8_t SDCARD_PollWriteComplete(void);

void hexDumpBuffer(uint8_t sdCardBuffer[]);

#define WRITE_NOT_COMPLETE  0xFF

