#pragma once

// #include "../../src/targets.h"
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include "../../src/user_config.h"

#include <stdint.h>

#if defined(USE_HIRES_DATA) && defined(ELRS_OG_COMPATIBILITY)
#error "Can't use hires data in compatibility mode (yet)"
#endif


void ICACHE_RAM_ATTR TXnbISR();

// enum InterruptAssignment_
// {
//     NONE,
//     RX_DONE,
//     TX_DONE
// };

class SX1280Driver
{
private:
    SX1280Hal * hal = nullptr; // something tries to use the hal before it is created. But when adding debug to try and track it, the problem goes away
    bool isPrimary;
    uint8_t timeoutHigh = 0, timeoutLow = 0;

    void setupLora();
    void setupFLRC();
    void SetPacketParamsFLRC();
    void setHighSensitivity();


public:
    ///////Callback Function Pointers/////
    // static void ICACHE_RAM_ATTR nullCallback(void);

    // void (*RXdoneCallback)() = &nullCallback; //function pointer for callback
    // void (*TXdoneCallback)() = &nullCallback; //function pointer for callback

    // static void (*TXtimeout)(); //function pointer for callback
    // static void (*RXtimeout)(); //function pointer for callback

    // InterruptAssignment_ InterruptAssignment = NONE;
    /////////////////////////////

    ///////////Radio Variables////////
    volatile uint8_t TXdataBuffer[256]; // XXX get rid of these
    volatile uint8_t RXdataBuffer[256];

    uint8_t TXbuffLen;
    uint8_t RXbuffLen;

    // static uint8_t _syncWord; // XXX used? remove or make non-static

    SX1280_RadioLoRaBandwidths_t currBW = SX1280_LORA_BW_0800;
    SX1280_RadioLoRaSpreadingFactors_t currSF = SX1280_LORA_SF6;
    SX1280_RadioLoRaCodingRates_t currCR = SX1280_LORA_CR_4_7;
    uint32_t currFreq = 2400000000;
    SX1280_RadioOperatingModes_t currOpmode = SX1280_MODE_SLEEP;

    int8_t currPWR;
    // static uint8_t maxPWR;

    ///////////////////////////////////

    /////////////Packet Stats//////////
    int8_t LastPacketRSSI = 0;
    int8_t LastPacketSNR = 0;
    // volatile uint8_t NonceTX = 0;
    // volatile uint8_t NonceRX = 0;
    // static uint32_t TotalTime;
    // static uint32_t TimeOnAir;
    // static uint32_t TXstartMicros;
    // static uint32_t TXspiTime;
    // static uint32_t HeadRoom;
    // static uint32_t TXdoneMicros;

    ////////////////Configuration Functions/////////////
    SX1280Driver();
    SX1280Driver(const uint32_t cssPin);

    void hardwareInit();
    void reset();

    void checkVersion();

    int32_t Begin();
    int32_t Begin(const bool usePreamble);
    void End();
    void SetMode(SX1280_RadioOperatingModes_t OPmode);
    void Config(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr, uint32_t freq, 
                uint8_t PreambleLength, const bool invertIQ = false);
    void ICACHE_RAM_ATTR ConfigFLRC(uint32_t freq);

    void ConfigModParams(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr);
    void ConfigModParamsFLRC(SX1280_RadioFLRCBandwidths_t bw, SX1280_RadioFLRCCodingRates_t cr, SX1280_RadioFLRCBTFilter_t bt);
    void SetPacketParams(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType, uint8_t PayloadLength, SX1280_RadioLoRaCrcModes_t crc, SX1280_RadioLoRaIQModes_t InvertIQ);
    void ICACHE_RAM_ATTR SetFrequency(uint32_t freq);
    void ICACHE_RAM_ATTR SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    void SetOutputPower(int8_t power);
    void setRxTimeout(uint32_t t);

    uint16_t convertPowerToMw(int power);
    uint16_t getPowerMw();

    int32_t ICACHE_RAM_ATTR GetFrequencyError();

    void ICACHE_RAM_ATTR TXnb(volatile uint8_t *data, uint8_t length);
    // static void ICACHE_RAM_ATTR TXnbISR(); //ISR for non-blocking TX routine

    void ICACHE_RAM_ATTR RXnb();
    // static void ICACHE_RAM_ATTR RXnbISR(); //ISR for non-blocking RC routine
    
    void readRXData();

    void ICACHE_RAM_ATTR ClearIrqStatus(uint16_t irqMask);

    void ICACHE_RAM_ATTR GetStatus();

    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);

    // uint8_t SetBandwidth(SX1280_RadioLoRaBandwidths_t bw);
    // uint32_t getCurrBandwidth();
    // uint8_t SetOutputPower(uint8_t Power);
    // uint8_t SetPreambleLength(uint8_t PreambleLen);
    // uint8_t SetSpreadingFactor(SX1280_RadioLoRaSpreadingFactors_t sf);
    // uint8_t SetCodingRate(SX1280_RadioLoRaCodingRates_t cr);

    bool ICACHE_RAM_ATTR GetFrequencyErrorbool();
    void ICACHE_RAM_ATTR SetPPMoffsetReg(int32_t offset);
    uint8_t ICACHE_RAM_ATTR GetRxBufferAddr();

    int8_t ICACHE_RAM_ATTR GetLastPacketRSSI();
    int8_t ICACHE_RAM_ATTR GetLastPacketSNR();
    uint16_t ICACHE_RAM_ATTR GetIrqStatus();
};