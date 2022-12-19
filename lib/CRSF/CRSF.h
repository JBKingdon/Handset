#pragma once

// #define FEATURE_OPENTX_SYNC_AUTOTUNE


#include "../../src/config.h"

// XXX TODO what a mess, move this to config? And sort out a better debug vs production control so it's all in one place
#if defined(DUAL_BAND_PROTOTYPE) || defined(DB_PCB_V1) || defined(DB_TX_V1)

#ifdef IS_RECEIVER

// The receiver PCB needs to share uart0 with the normal debug output

#define CRSF_PORT_NUM 0

#else

// The transmitter can use uart1 and keep debug on uart0
// XXX but isn't for now - needs to be 0 for use in the handset, and 1 for stand alone dev. Can't remember why
#define CRSF_PORT_NUM 0
// #define CRSF_PORT_NUM 1

#endif // IS_RECEIVER

#elif defined(DUAL_BAND_BREADBOARD)

// The breadboard prototype can use uart1 with pin 12 for crossfire s.port

// As currently setup, use port 0 for normal operation and port 1 for dev with keyboard input to change modes

#ifdef DEV_MODE
#define CRSF_PORT_NUM 1
#else
#define CRSF_PORT_NUM 0 
#endif

#else

#error("Must define boardtype")

#endif // DUAL_BAND_PROTOTYPE


#ifdef IS_RECEIVER
#define CRSF_RX_MODULE
#endif
#ifdef IS_TRANSMITTER
#define CRSF_TX_MODULE
#endif


// #include "targets.h"
#include "crsf_protocol.h"
#ifndef TARGET_NATIVE
// #include "HardwareSerial.h" // Arduino serial library. We're going to need a HAL for that
#endif
// #include "msp.h"
// #include "msptypes.h"
// #include "LowPassFilter.h"
#include "../CRC/crc.h"
// #include "telemetry_protocol.h"

#ifdef ESPC3
#include "driver/uart.h"
#endif

#ifdef PLATFORM_ESP32
#include "esp32-hal-uart.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#endif


class CRSF
{
private:
    // Stream *_dev; // XXX more arduino-ness to clean up

    static volatile uint8_t SerialInPacketLen;                   // length of the CRSF packet as measured
    static volatile uint8_t SerialInPacketPtr;                   // index where we are reading/writing

    static volatile inBuffer_U inBuffer;

    static volatile bool CRSFframeActive;  //since we get a copy of the serial data use this flag to know when to ignore it

    #if defined(CRSF_TX_MODULE)
    /// OpenTX mixer sync ///
    static volatile uint32_t OpenTXsyncLastSent;
    static uint32_t RequestedRCpacketInterval;
    static volatile uint32_t RCdataLastRecv;
    static volatile uint32_t dataLastRecv;
    static volatile int32_t OpenTXsyncOffset;
    static volatile int32_t OpenTXsyncWindow;
    static volatile int32_t OpenTXsyncWindowSize;
    static uint32_t OpenTXsyncOffsetSafeMargin;
    static uint8_t CRSFoutBuffer[CRSF_MAX_PACKET_LEN];

    static bool    newLinkstatsDataAvailable;
    static uint8_t linkstatsBuffer[LinkStatisticsFrameLength+4];


    #ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
    static uint32_t SyncWaitPeriodCounter;
    #endif

    /// UART Handling ///
    static uint32_t GoodPktsCount;
    static uint32_t BadPktsCount;
    static uint32_t UARTwdtLastChecked;
    static uint32_t UARTcurrentBaud;
    static bool CRSFisConnected;
    // static uint8_t MspData[ELRS_MSP_BUFFER];
    static uint8_t MspDataLength;
    #if defined(PLATFORM_ESP32) || defined(ESPC3)
    static void ESP32uartTask(void *pvParameters);
    static void ESP32syncPacketTask(void *pvParameters);
    #endif

    static void duplex_set_RX();
    static void duplex_set_TX();
    static bool ProcessPacket();
    static void handleUARTout();
    static bool UARTwdt();
    #endif // CRSF_TX_MODULE

    static void flush_port_input(void);
    static void ICACHE_RAM_ATTR sendLinkstatsPacketToTX();

    static bool ICACHE_RAM_ATTR syncPacketRequired();
    static bool ICACHE_RAM_ATTR linkstatsPacketRequired();

public:

    static volatile uint16_t ChannelDataIn[16];
    static volatile uint16_t ChannelDataOut[16];

    // current and sent switch values
    #define N_SWITCHES 8

    static uint8_t currentSwitches[N_SWITCHES];
    static uint8_t sentSwitches[N_SWITCHES];
    // which switch should be sent in the next rc packet
    static uint8_t nextSwitchIndex;

    static void (*disconnected)();
    static void (*connected)();

    static void (*RecvParameterUpdate)();

    static volatile uint8_t ParameterUpdateData[2];

    /////Variables/////

    // #ifdef USE_ELRS_CRSF_EXTENSIONS
    static volatile crsf_elrs_channels_s elrsPackedRCdataOut;
    static volatile crsf_elrs_channels_hiRes_s elrsPackedHiResRCdataOut;
    static volatile crsf_elrs_channels_DB_t elrsPackedDBDataOut;
    static volatile elrsPayloadLinkstatistics_s elrsLinkStatistics; // Link statisitics stored as struct
    static volatile elrsLinkStatistics_DB_t elrsLinkStatsDB;        // extended version for dual band

    // #else
    static volatile crsfPayloadLinkstatistics_s LinkStatistics;     // Link Statisitics stored as Struct
    static volatile crsf_channels_s PackedRCdataOut;                // RC data in packed format for output.
    // #endif

    
    static volatile crsf_sensor_battery_s TLMbattSensor;



    /// UART Handling ///
    static uint32_t GoodPktsCountResult; // need to latch the results
    static uint32_t BadPktsCountResult; // need to latch the results

    static void Begin(); //setup timers etc
    static void End(); //stop timers etc

    CRSF();

    void ICACHE_RAM_ATTR sendRCFrameToFC();

    #if defined(USE_HIRES_DATA)
    void ICACHE_RAM_ATTR sendHiResRCFrameToFC();
    #elif defined(USE_DB_PACKETS)
    void ICACHE_RAM_ATTR sendDBRCFrameToFC();
    #endif

    // void ICACHE_RAM_ATTR sendMSPFrameToFC(uint8_t* data);
    void sendLinkStatisticsToFC();
    void sendLinkStatsDBtoFC();

    // Provide new linkstats data from basic telemetry for sending to the handset
    static void ICACHE_RAM_ATTR updateLinkStatistics();

    // void ICACHE_RAM_ATTR sendTelemetryToTX(uint8_t *data);

    // void sendLUAresponse(uint8_t val[], uint8_t len);

    // static void ICACHE_RAM_ATTR sendSetVTXchannel(uint8_t band, uint8_t channel);

    uint8_t ICACHE_RAM_ATTR getNextSwitchIndex();
    void ICACHE_RAM_ATTR setSentSwitch(uint8_t index, uint8_t value);

///// Variables for OpenTX Syncing //////////////////////////
    // #define OpenTXsyncPacketInterval 200 // in ms. Default is 200 for 5Hz, but if we send much more often we can keep the delta smaller
    #define OpenTXsyncPacketInterval 10 // in ms
    static void ICACHE_RAM_ATTR setSyncParams(uint32_t PacketInterval);
    // void ICACHE_RAM_ATTR setSyncParams(uint32_t PacketInterval);

    static void ICACHE_RAM_ATTR JustSentRFpacket();
    static void ICACHE_RAM_ATTR sendSyncPacketToTX();

    /////////////////////////////////////////////////////////////

    static void ICACHE_RAM_ATTR GetChannelDataIn();
    static uint32_t ICACHE_RAM_ATTR GetRCdataLastRecv();
    static void ICACHE_RAM_ATTR updateSwitchValues();

    static void inline nullCallback(void);

    static void handleUARTin();
    bool RXhandleUARTout();

    #if defined(CRSF_TX_MODULE)
    static uint8_t* GetMspMessage();
    static void UnlockMspMessage();
    static void AddMspMessage(const uint8_t length, volatile uint8_t* data);
    // static void AddMspMessage(mspPacket_t* packet);
    static void ResetMspQueue();
    #endif
};


