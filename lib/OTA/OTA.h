#ifndef H_OTA
#define H_OTA

#include <stdint.h>

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> msp data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
#define RC_DATA_PACKET  0b00
#define MSP_DATA_PACKET 0b01
#define SYNC_PACKET     0b10
#define TLM_PACKET      0b11
#define RC_HIRES_DATA   0b11  // NB Same as TLM_PACKET since we don't use that value for TX -> RX packets

#ifdef USE_HIRES_DATA

#define OTA_PACKET_LENGTH 9

#else

#define OTA_PACKET_LENGTH 8

#endif // USE_HIRES_DATA

// XXX replace with a IS_RX flag
#ifdef ESPC3

#include "CRSF.h"

#if defined HYBRID_SWITCHES_8 or defined UNIT_TEST


void UnpackChannelDataHybridSwitches8(volatile uint8_t* Buffer, CRSF *crsf);
void UnpackHiResChannelData(volatile uint8_t* Buffer, CRSF *crsf);


#endif // HYBRID_SWITCHES_8

#if defined SEQ_SWITCHES or defined UNIT_TEST

void ICACHE_RAM_ATTR GenerateChannelDataSeqSwitch(volatile uint8_t* Buffer, CRSF *crsf, uint8_t addr);
void UnpackChannelDataSeqSwitches(volatile uint8_t* Buffer, CRSF *crsf);

#endif // SEQ_SWITCHES

#else
// void GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, CRSF *crsf, uint8_t addr);
void GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, const uint16_t scaledADC[], 
                                                      const uint8_t currentSwitches[], const uint8_t nextSwitchIndex, uint8_t addr);

#endif // ESPC3

#endif // H_OTA
