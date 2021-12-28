#pragma once

// #include "../../src/targets.h"
#include "SX1262_Regs.h"
#include "SX1262_hal.h"
#include "../../src/user_config.h"
// #include "crsf_protocol.h"
#include "../OTA/OTA.h"

#include <stdint.h>

#if defined(USE_HIRES_DATA) && defined(ELRS_OG_COMPATIBILITY)
#error "Can't use hires data in compatibility mode (yet)"
#endif


#ifdef GD32
#define ICACHE_RAM_ATTR
#endif

//void ICACHE_RAM_ATTR TXnbISR(); // why is this here? Can we get rid of it?

// XXX used?
// enum InterruptAssignment_
// {
//     NONE,
//     RX_DONE,
//     TX_DONE
// };

class SX1262Driver
{
private:
    uint32_t rxTimeout = 0xFFFFFF; // default to continuous mode as that's what earlier impls used
    void setupLora();
    void clearRxTimeout();
    void optimizeIQ(bool isInverted);

public:
    ///////Callback Function Pointers/////
    static void ICACHE_RAM_ATTR nullCallback(void);

    void (*RXdoneCallback)() = &nullCallback; //function pointer for callback
    void (*TXdoneCallback)() = &nullCallback; //function pointer for callback

    static void (*TXtimeout)(); //function pointer for callback
    static void (*RXtimeout)(); //function pointer for callback

    // InterruptAssignment_ InterruptAssignment = NONE;
    /////////////////////////////

    ///////////Radio Variables////////

    // TODO reconsider use of public variables

    // Do these need to be so big?
    // XXX just get rid of them
    volatile uint8_t TXdataBuffer[256];
    volatile uint8_t RXdataBuffer[256];

    uint8_t TXbuffLen;  // XXX check usage
    uint8_t RXbuffLen;

    static uint8_t _syncWord;

    SX1262_RadioLoRaBandwidths_t currBW = SX1262_LORA_BW_500;       // Are these initialisation values used? Do they have to match the settings in common.cpp? WTF?
    SX1262_RadioLoRaSpreadingFactors_t currSF = SX1262_LORA_SF6;
    SX1262_RadioLoRaCodingRates_t currCR = SX1262_LORA_CR_4_7;
    uint32_t currFreq = 915500000;
    // SX1262_RadioOperatingModes_t currOpmode = SX1262_MODE_SLEEP;

    int8_t currPWR;
    // static uint8_t maxPWR;

    ///////////////////////////////////

    /////////////Packet Stats//////////
    int8_t LastPacketRSSI = 0;
    int8_t LastPacketSNR = 0;
    volatile uint8_t NonceTX = 0;
    volatile uint8_t NonceRX = 0;
    static uint32_t TotalTime;
    static uint32_t TimeOnAir;
    static uint32_t TXstartMicros;
    static uint32_t TXspiTime;
    static uint32_t HeadRoom;
    static uint32_t TXdoneMicros;
    
    
    ////////////////Configuration Functions/////////////
    SX1262Driver();
    static SX1262Driver *instance;  // XXX get rid of this
    void Begin();
    void End();
    void SetMode(SX1262_RadioOperatingModes_t OPmode);
    void Config(SX1262_RadioLoRaBandwidths_t bw, SX1262_RadioLoRaSpreadingFactors_t sf, SX1262_RadioLoRaCodingRates_t cr, uint32_t freq, 
                uint8_t PreambleLength, const bool invertIQ = false);

    void ConfigModParams(SX1262_RadioLoRaBandwidths_t bw, SX1262_RadioLoRaSpreadingFactors_t sf, SX1262_RadioLoRaCodingRates_t cr);
    void SetPacketParams(uint8_t PreambleLength, SX1262_RadioLoRaPacketLengthsModes_t HeaderType, uint8_t PayloadLength, SX1262_RadioLoRaCrcModes_t crc, SX1262_RadioLoRaIQModes_t InvertIQ);
    void ICACHE_RAM_ATTR SetFrequency(uint32_t freq);
    void ICACHE_RAM_ATTR SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    void SetOutputPower(int8_t power);
    void setPacketLength(uint8_t packetLen);


    uint16_t convertPowerToMw(int power);
    uint16_t getPowerMw();

    void setRxTimeout(uint32_t newTimeout);

    static void ICACHE_RAM_ATTR TXnb(volatile uint8_t *data, uint8_t length);
    static void ICACHE_RAM_ATTR TXnbISR(); //ISR for non-blocking TX routine

    static void ICACHE_RAM_ATTR RXnb();
    static void ICACHE_RAM_ATTR RXnbISR(); //ISR for non-blocking RC routine
    
    void readRXData();

    void ICACHE_RAM_ATTR ClearIrqStatus(uint16_t irqMask);

    void ICACHE_RAM_ATTR GetStatus();

    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);


    uint8_t ICACHE_RAM_ATTR GetRxBufferAddr();

    int8_t ICACHE_RAM_ATTR GetLastPacketRSSI();
    int8_t ICACHE_RAM_ATTR GetLastPacketSNR();
    static uint16_t ICACHE_RAM_ATTR GetIrqStatus();
};