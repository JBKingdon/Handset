#pragma once

// #ifndef H_OTA
// #define H_OTA

#include <stdint.h>
#include "../../src/user_config.h"

#define PACKED __attribute__((packed))


typedef struct Pwm6Payload_s {
    unsigned int header : 2;
    unsigned int ch0 : 11;
    unsigned int ch1 : 11;
    unsigned int ch2 : 11;
    unsigned int ch3 : 11;
    unsigned int ch4 : 11;
    unsigned int ch5 : 11; // 66 + 2 = 68
    unsigned int sw0 : 1;
    unsigned int sw1 : 1;
    unsigned int sw2 : 1;
    unsigned int sw3 : 1;
    unsigned int sw4 : 1;
    unsigned int sw5 : 1; // 6 + 68 = 74
    unsigned int crc : 14; // 14 + 74 = 88 = 11 bytes with 0 bits spare
} PACKED Pwm6Payload_t;



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

#elif defined(USE_PWM6)

#define OTA_PACKET_LENGTH 11

#else

#define OTA_PACKET_LENGTH 8

#endif // USE_HIRES_DATA

// XXX replace with a IS_RX flag
// #ifdef ESPC3

#include "CRSF.h"

#if defined HYBRID_SWITCHES_8 or defined UNIT_TEST

void UnpackChannelDataHybridSwitches8(volatile uint8_t* Buffer, CRSF *crsf);
void UnpackHiResChannelData(volatile uint8_t* Buffer, CRSF *crsf);

void UnpackChannelDataPWM6(Pwm6Payload_t* Buffer, CRSF *crsf);

#endif // HYBRID_SWITCHES_8

#if defined SEQ_SWITCHES or defined UNIT_TEST

void ICACHE_RAM_ATTR GenerateChannelDataSeqSwitch(volatile uint8_t* Buffer, CRSF *crsf, uint8_t addr);
void UnpackChannelDataSeqSwitches(volatile uint8_t* Buffer, CRSF *crsf);

#endif // SEQ_SWITCHES

// #else // not the ESPC3 (RX)
// void GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, CRSF *crsf, uint8_t addr);
void GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, const uint16_t scaledADC[], 
                                                      const uint8_t currentSwitches[], const uint8_t nextSwitchIndex, uint8_t addr);

void GenerateChannelDataPWM6(Pwm6Payload_t* outputBuffer, const uint16_t scaledADC[], 
                             const uint8_t currentSwitches[]);

// #endif // ESPC3

// #endif // H_OTA
