#include "mcc_generated_files/mcc.h"
#include "sdCard.h"

#pragma warning disable 373

//--------------------------------------------------------------
// http://elm-chan.org/docs/mmc/mmc_e.html
//--------------------------------------------------------------
#define CMD_SEND_STATUS  13
#define CMD_READ_BLOCK   17
#define CMD_WRITE_BLOCK  24
#define START_TOKEN      0xFE
#define SPI1_DUMMY_DATA  0xFF

//--------------------------------------------------------------
//--------------------------------------------------------------
void SDCARD_ReadBlock(uint32_t addr, uint8_t sdCardBuffer[]) {
    
    uint16_t i=0;
        
    CS_SetLow();
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0x40 | CMD_READ_BLOCK);
    SPI2_ExchangeByte((uint8_t)((addr >> 24) & 0xFF));
    SPI2_ExchangeByte((uint8_t)((addr >> 16) & 0xFF));
    SPI2_ExchangeByte((uint8_t)((addr >> 8) & 0xFF));
    SPI2_ExchangeByte((uint8_t)(addr & 0xFF));
    SPI2_ExchangeByte(0xFF);

    // Wait for R1 response
    while(SPI2_ExchangeByte(0xFF) == 0xFF);
 
    // Wait for start token at the start of the data packet
    while(SPI2_ExchangeByte(0xFF) == 0xFF);
    
    // Read in one block, 512 bytes
    for(i = 0; i < 512; i++)
        sdCardBuffer[i] = SPI2_ExchangeByte(0xFF);    
    
    // Finally chew up the 2-byte CRC
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0xFF);
    
    CS_SetHigh();

}


//--------------------------------------------------------------
//--------------------------------------------------------------
void SDCARD_WriteBlock(uint32_t addr, uint8_t sdCardBuffer[]) {
         
    uint16_t i = 0;
    
    CS_SetLow();
    
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0x40 | CMD_WRITE_BLOCK);
    SPI2_ExchangeByte((uint8_t)((addr >> 24) & 0xFF));
    SPI2_ExchangeByte((uint8_t)((addr >> 16) & 0xFF));
    SPI2_ExchangeByte((uint8_t)((addr >> 8) & 0xFF));
    SPI2_ExchangeByte((uint8_t)(addr & 0xFF));
    SPI2_ExchangeByte(0xFF);
    
    // Wait for R1 response
    while(SPI2_ExchangeByte(0xFF) == 0xFF);

    // Send at least one byte to buffer data packet
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0xFF);
    
    // Start data packet with a 1 byte data token
    SPI2_ExchangeByte(START_TOKEN);
    
    // followed by a 512 byte data block
    for(i = 0; i < 512; i++) 
        SPI2_ExchangeByte(sdCardBuffer[i]);    
            
    CS_SetHigh();
  
}


//--------------------------------------------------------------
// 
//--------------------------------------------------------------
uint8_t SDCARD_PollWriteComplete(void) {
    
    uint8_t status;
    
    CS_SetLow();        
    status = SPI2_ExchangeByte(CMD_SEND_STATUS);
    CS_SetHigh();
    
    if (status == 0xFF) {
        return (WRITE_NOT_COMPLETE);
    } else {
        // REad out all 32-bits
        (void) SPI2_ExchangeByte(0xFF);
        (void) SPI2_ExchangeByte(0xFF);
        (void) SPI2_ExchangeByte(0xFF);
        return(status);
    }

}


//--------------------------------------------------------------
// 
//--------------------------------------------------------------
void SDCARD_Initialize(uint8_t verbose) {
    
    uint8_t response;
    
    SDCARD_SetIdle(verbose);
    
    do {
        CS_SetLow();
        SPI2_ExchangeByte(0xFF);
        SPI2_ExchangeByte(0x41);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0xFF);
        SPI2_ExchangeByte(0xFF);
        response = SPI2_ExchangeByte(0xFF);
        CS_SetHigh();
    } while(response != 0);
    
    if (verbose == true) printf("CMD1, Init Response: %x\r\n", response);
    
    response = SDCARD_SetBlockLength();
    if (verbose == true) printf("Block Length Response: %x\r\n", response);
}


//--------------------------------------------------------------
// 
//--------------------------------------------------------------
void SDCARD_HCInitialize(uint8_t verbose) {
    
    uint8_t response;
    
    SDCARD_SetIdle(verbose);

    // Send check voltage command, required for SD
    CS_SetLow();  
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0x48);
    SPI2_ExchangeByte(0x00);
    SPI2_ExchangeByte(0x00);
    SPI2_ExchangeByte(0x01);
    SPI2_ExchangeByte(0xAA);
    SPI2_ExchangeByte(0x87);
    SPI2_ExchangeByte(0xFF);
    response = SPI2_ExchangeByte(0xFF);

    // Get echo of Arg
    for(int i = 0; i < 4; i++) { 
        printf("%x", SPI2_ExchangeByte(SPI1_DUMMY_DATA));
    }
        printf("\r\n");
    CS_SetHigh();

    printf("CMD 8 Response: %x\r\n", response);
    
    // Send ACMD41, initialize SD card
    // Takes a few ms, so repeat until result is 0
    do {
        CS_SetLow();
        
        // CMD55
        SPI2_ExchangeByte(0xFF);
        SPI2_ExchangeByte(0x77);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0xFF);
        response = SPI2_ExchangeByte(0xFF);
        printf("CMD 55 Response: %x\r\n", response);
        
        // CMD 41
        SPI2_ExchangeByte(0x69);
        SPI2_ExchangeByte(0x40);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0xFF);
        response = SPI2_ExchangeByte(0xFF);
        
        CS_SetHigh();
        
        printf("ACMD 41 Response: %x\r\n", response);
    } while(response == 0x01);
    
    if(response == 0) {
        printf("Successfully Initialized SD Card\r\n");

    } else {
        printf("Error Initializing SD Card\r\n");
    }
    
    //    setBlockLength(512);
    //SDCARD_SetBlockLength();
    
}


//--------------------------------------------------------------
// 
//--------------------------------------------------------------
void SDCARD_SetIdle(uint8_t verbose) {
    uint8_t response;
    
    // Send at least 74 clock cycles to SD card
    for(int i = 0; i < 10; i++) {
        SPI2_ExchangeByte(0xFF);
    }
    
    // Send idle command
    CS_SetLow();
    SPI2_ExchangeByte(0xFF);
    SPI2_ExchangeByte(0x40);
    SPI2_ExchangeByte(0x00);
    SPI2_ExchangeByte(0x00);
    SPI2_ExchangeByte(0x00);
    SPI2_ExchangeByte(0x00);
    SPI2_ExchangeByte(0x95);
    SPI2_ExchangeByte(0xFF);
    response = SPI2_ExchangeByte(0xFF);
    CS_SetHigh();
    
    if (verbose == true) printf("CMD0, Reset Response: %x\r\n", response);
}



//--------------------------------------------------------------
// Needed during initialization
//--------------------------------------------------------------
uint8_t SDCARD_SetBlockLength(void) {
    
    uint8_t response;
    
    do {
        CS_SetLow();
        SPI2_ExchangeByte(0xFF);
        SPI2_ExchangeByte(0x50);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0x02);
        SPI2_ExchangeByte(0x00);
        SPI2_ExchangeByte(0xFF);
        SPI2_ExchangeByte(0xFF);
        response = SPI2_ExchangeByte(0xFF);
        CS_SetHigh();
    } while(response == 0xFF);
    
    return(response);
    
}





//--------------------------------------------------------------
// 
//--------------------------------------------------------------
void hexDumpBuffer(uint8_t sdCardBuffer[]) {
        
    uint16_t i;
    
    printf("\r\n\n");           
    
    for(i = 0; i < 512; i++) {
      if(i != 0 && i % 8 == 0) printf(" ");
      if(i != 0 && i % 16 == 0)  {
          printf("  ");
          for(int j = i - 16; j < i; j++) {
              if(sdCardBuffer[j] < 32) {
                printf(".");  
              } else {
                printf("%c", sdCardBuffer[j]);
              }
          }          
          printf("\r\n");
      }
      
      printf("%02x ", sdCardBuffer[i]);   
    }
    
    printf("   ");
    for(int j = i - 16; j < i; j++) {
        if(sdCardBuffer[j] < 32) {
          printf(".");  
        } else {
          printf("%c", sdCardBuffer[j]);
        }
    }
    printf("\r\n");
}


