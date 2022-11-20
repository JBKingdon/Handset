#pragma once

// #ifndef H_OTA
// #define H_OTA

#include <stdint.h>
#include "../../src/config.h"

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

// Dual band packet formats



// 915 uplink packets. Max 18 bytes to work with
typedef struct DB915Packet_s {
    unsigned int crc : 16;       // 16 bits
    unsigned int nonce : 16;     // 16 bits

    unsigned int ch0 : 10;
    unsigned int ch1 : 10;
    unsigned int ch2 : 10;
    unsigned int ch3 : 10;       // 40 bits

    // Extra full res channels spread across packets with odd/even nonce
    // odd: chA = ch4, chB = ch5
    // even: chA = ch6, chB = ch7
    unsigned int chA : 12;
    unsigned int chB : 12;       // 24 bits

    // Switches spread across packets with odd/even nonce
    // odd: channels 8 through 11
    // even: channels 12 through 15
    unsigned int swA : 2;
    unsigned int swB : 2;
    unsigned int swC : 2;
    unsigned int swD : 2;
                                // 8 bits

    unsigned int txPower : 8;    // 8 bits

    unsigned int rateIndex : 2; // 2 bits
    unsigned int armed : 1;     // 1 bit

} PACKED DB915Packet_t;          // total 15 bytes

typedef struct DB915Telem_s {
    unsigned int crc : 16;       // 16 bits (only 14 used, can be shrunk and moved to the end)

    int rssi915 : 8;
    unsigned int lq915 : 8;

    int rssi2G4 : 8;
    unsigned int lq2G4 : 8;     // 8 bits

    int snr2G4 : 8;         // signed value in tenths, so  -12.8 to +12.7 

    uint8_t serialData[7];

    unsigned int serialDataLength : 4;

    unsigned int packetType : 2;

} PACKED DB915Telem_t;  // total 15 bytes

// NB FLRC only sends the first 6 bytes, so doesn't include 'armed'. It's sent on every 915 packet,
// so can probably be removed from here - but make sure that failsafe works properly if the 915 goes down
typedef struct DB2G4Packet_s {
    unsigned int ch0 : 12;
    unsigned int ch1 : 12;
    unsigned int ch2 : 12;
    unsigned int ch3 : 12;
    unsigned int armed : 1;
    unsigned int crc : 14;

} PACKED DB2G4Packet_t;       // total 8 bytes

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

#elif defined(USE_DB_PACKETS)

#define OTA_PACKET_LENGTH_2G4 8
#define OTA_PACKET_LENGTH_2G4_FLRC 6
#define OTA_PACKET_LENGTH_915 15

#define OTA_PACKET_LENGTH_TELEM 15

#else

#define OTA_PACKET_LENGTH_915 8

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
