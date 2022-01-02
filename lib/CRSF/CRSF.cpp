#ifdef ESPC3

#include "CRSF.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "FIFO.h"
#include "../../src/LowPassFilter.h"

#include "../../src/utils.h"

#include "freertos/FreeRTOS.h"

// #include "../../lib/FIFO/FIFO.h"
// #include "telemetry_protocol.h"

// #define DEBUG_CRSF_NO_OUTPUT // debug, don't send RC msgs over UART

#if defined(PLATFORM_ESP32) || defined(ESPC3)
// HardwareSerial SerialPort(1);
// HardwareSerial CRSF::Port = SerialPort;
portMUX_TYPE FIFOmux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t xHandleOpenTXsync = NULL;
TaskHandle_t xESP32uartTask = NULL;
#elif CRSF_TX_MODULE_STM32
HardwareSerial CRSF::Port(GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX);
#if defined(STM32F3) || defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"
#elif defined(STM32F1) || defined(STM32F1xx)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#endif
#endif

GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);

///Out FIFO to buffer messages///
FIFO SerialOutFIFO;
// FIFO MspWriteFIFO;

volatile bool CRSF::CRSFframeActive = false; //since we get a copy of the serial data use this flag to know when to ignore it

void inline CRSF::nullCallback(void) {}

void (*CRSF::disconnected)() = &nullCallback; // called when CRSF stream is lost
void (*CRSF::connected)() = &nullCallback;    // called when CRSF stream is regained

void (*CRSF::RecvParameterUpdate)() = &nullCallback; // called when recv parameter update req, ie from LUA

/// UART Handling ///
uint32_t CRSF::GoodPktsCountResult = 0;
uint32_t CRSF::BadPktsCountResult = 0;

volatile uint8_t CRSF::SerialInPacketLen = 0; // length of the CRSF packet as measured
volatile uint8_t CRSF::SerialInPacketPtr = 0; // index where we are reading/writing

volatile uint16_t CRSF::ChannelDataIn[16] = {0};

volatile inBuffer_U CRSF::inBuffer;

// current and sent switch values, used for prioritising sequential switch transmission
uint8_t CRSF::currentSwitches[N_SWITCHES] = {0};
uint8_t CRSF::sentSwitches[N_SWITCHES] = {0};

uint8_t CRSF::nextSwitchIndex = 0; // for round-robin sequential switches

volatile uint8_t CRSF::ParameterUpdateData[2] = {0};


// #ifdef USE_ELRS_CRSF_EXTENSIONS
volatile crsf_elrs_channels_s CRSF::elrsPackedRCdataOut;
volatile crsf_elrs_channels_hiRes_s CRSF::elrsPackedHiResRCdataOut;
volatile elrsPayloadLinkstatistics_s CRSF::elrsLinkStatistics;
// #else
volatile crsf_channels_s CRSF::PackedRCdataOut;
volatile crsfPayloadLinkstatistics_s CRSF::LinkStatistics;
volatile crsf_sensor_battery_s CRSF::TLMbattSensor;
// #endif

#ifdef CRSF_TX_MODULE
/// OpenTX mixer sync ///
volatile uint32_t CRSF::OpenTXsyncLastSent = 0;
uint32_t CRSF::RequestedRCpacketInterval = 5000; // default to 200hz as per 'normal'
volatile uint32_t CRSF::RCdataLastRecv = 0;
volatile int32_t CRSF::OpenTXsyncOffset = 0;

static uint8_t baudrateIndex = 1;

#define MAX_BYTES_SENT_IN_UART_OUT 32
uint8_t CRSF::CRSFoutBuffer[CRSF_MAX_PACKET_LEN] = {0};

#ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
#define AutoSyncWaitPeriod 2000
uint32_t CRSF::OpenTXsyncOffsetSafeMargin = 1000;
static LPF LPF_OPENTX_SYNC_MARGIN(3);
static LPF LPF_OPENTX_SYNC_OFFSET(3);
uint32_t CRSF::SyncWaitPeriodCounter = 0;
#else
uint32_t CRSF::OpenTXsyncOffsetSafeMargin = 4000; // 400us
#endif

/// UART Handling ///
uint32_t CRSF::GoodPktsCount = 0;
uint32_t CRSF::BadPktsCount = 0;
uint32_t CRSF::UARTwdtLastChecked;
uint32_t CRSF::UARTcurrentBaud;
bool CRSF::CRSFstate = false;

// for the UART wdt, every 1000ms we change bauds when connect is lost
#define UARTwdtInterval 1000

// uint8_t CRSF::MspData[ELRS_MSP_BUFFER] = {0};
uint8_t CRSF::MspDataLength = 0;
#endif // CRSF_TX_MODULE


void CRSF::Begin()
{
    // Serial.println("About to start CRSF task...");
    printf("About to start CRSF task...\n");

    #ifdef CRSF_TX_PIN

    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    #ifdef CRSF_TX_MODULE
    uart_config.baud_rate = OPENTX_BAUDS[baudrateIndex];
    #else
    uart_config.baud_rate = CRSF_RX_BAUDRATE;
    #endif

    int intr_alloc_flags = 0;

// #if CONFIG_UART_ISR_IN_IRAM
//     intr_alloc_flags = ESP_INTR_FLAG_IRAM;
// #endif

    // XXX we need to set this up so that multiple inbound packets aren't buffered

    // Need to use a zero sized tx buffer so that we can control when data is sent on the bidir s.port wire
    const uint32_t rxBufferSize = 512;
    const uint32_t txBufferSize = 0;
    ESP_ERROR_CHECK(uart_driver_install(CRSF_PORT_NUM, rxBufferSize, txBufferSize, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(CRSF_PORT_NUM, &uart_config));
    uart_set_rx_full_threshold(CRSF_PORT_NUM, 2);

    #ifdef CRSF_TX_MODULE
    // s.port, rx and tx share the same pin
    ESP_ERROR_CHECK(uart_set_pin(CRSF_PORT_NUM, CRSF_TX_PIN, CRSF_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    #else
    // full uart with separate rx and tx pins
    // XXX add RX pin
    ESP_ERROR_CHECK(uart_set_pin(CRSF_PORT_NUM, CRSF_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    #endif // CRSF_TX_MODULE

    // Configure a temporary buffer for the incoming data
    // uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    #endif // CRSF_TX_PIN

#ifdef CRSF_TX_MODULE
    UARTcurrentBaud = OPENTX_BAUDS[baudrateIndex];
    UARTwdtLastChecked = millis() + UARTwdtInterval; // allows a delay before the first time the UARTwdt() function is called

#if defined(PLATFORM_ESP32) || defined(ESPC3)
    // disableCore0WDT();
    xTaskCreatePinnedToCore(ESP32uartTask, "ESP32uartTask", 3000, NULL, 5, &xESP32uartTask, 0);


#elif defined(PLATFORM_STM32)
    Serial.println("Start STM32 R9M TX CRSF UART");

    #if defined(GPIO_PIN_BUFFER_OE) && (GPIO_PIN_BUFFER_OE != UNDEF_PIN)
    pinMode(GPIO_PIN_BUFFER_OE, OUTPUT);
    digitalWrite(GPIO_PIN_BUFFER_OE, LOW ^ GPIO_PIN_BUFFER_OE_INVERTED); // RX mode default
    #endif

    CRSF::Port.setTx(GPIO_PIN_RCSIGNAL_TX);
    CRSF::Port.setRx(GPIO_PIN_RCSIGNAL_RX);
    CRSF::Port.begin(CRSF_OPENTX_FAST_BAUDRATE);

#if defined(TARGET_TX_GHOST)
    USART1->CR1 &= ~USART_CR1_UE;
    USART1->CR3 |= USART_CR3_HDSEL;
    USART1->CR2 |= USART_CR2_RXINV | USART_CR2_TXINV | USART_CR2_SWAP; //inv
    USART1->CR1 |= USART_CR1_UE;
#endif
    Serial.println("STM32 CRSF UART LISTEN TASK STARTED");
    CRSF::Port.flush();
#endif

    flush_port_input();
#endif // CRSF_TX_MODULE

    //The master module requires that the serial communication is bidirectional
    //The Reciever uses seperate rx and tx pins
}

void CRSF::End()
{
#ifdef CRSF_TX_MODULE
#ifdef PLATFORM_ESP32
    if (xESP32uartTask != NULL)
    {
        vTaskDelete(xESP32uartTask);
    }
#endif
    // uint32_t startTime = millis();
    #define timeout 2000
    // while (SerialOutFIFO.peek() > 0)
    // {
    //     handleUARTin();
    //     if (millis() - startTime > 1000)
    //     {
    //         break;
    //     }
    // }
    //CRSF::Port.end(); // don't call seria.end(), it causes some sort of issue with the 900mhz hardware using gpio2 for serial 
    std::cout << "CRSF UART END";
#endif // CRSF_TX_MODULE

    uart_driver_delete(CRSF_PORT_NUM);

}

void CRSF::flush_port_input(void)
{
    uart_flush_input(CRSF_PORT_NUM);
    // Make sure there is no garbage on the UART at the start
    // while (CRSF::Port.available())
    // {
    //     CRSF::Port.read();
    // }
}


#ifdef CRSF_TX_MODULE

/**
 * Determine which switch to send next.
 * If any switch has changed since last sent, we send the lowest index changed switch
 * and set nextSwitchIndex to that value + 1.
 * If no switches have changed then we send nextSwitchIndex and increment the value.
 * For pure sequential switches, all 8 switches are part of the round-robin sequence.
 * For hybrid switches, switch 0 is sent with every packet and the rest of the switches
 * are in the round-robin.
 */
uint8_t ICACHE_RAM_ATTR CRSF::getNextSwitchIndex()
{
    int firstSwitch = 0; // sequential switches includes switch 0

#if defined HYBRID_SWITCHES_8
    firstSwitch = 1; // skip 0 since it is sent on every packet
#endif

    // look for a changed switch
    int i;
    for (i = firstSwitch; i < N_SWITCHES; i++)
    {
        if (currentSwitches[i] != sentSwitches[i])
            break;
    }
    // if we didn't find a changed switch, we get here with i==N_SWITCHES
    if (i == N_SWITCHES)
    {
        i = nextSwitchIndex;
    }

    // keep track of which switch to send next if there are no changed switches
    // during the next call.
    nextSwitchIndex = (i + 1) % 8;

#ifdef HYBRID_SWITCHES_8
    // for hydrid switches 0 is sent on every packet, skip it in round-robin
    if (nextSwitchIndex == 0)
    {
        nextSwitchIndex = 1;
    }
#endif

    return i;
}

/**
 * Record the value of a switch that was sent to the rx
 */
void ICACHE_RAM_ATTR CRSF::setSentSwitch(uint8_t index, uint8_t value)
{
    sentSwitches[index] = value;
}


void ICACHE_RAM_ATTR CRSF::sendLinkStatisticsToTX()
{
    if (!CRSF::CRSFstate)
    {
        return;
    }

    uint8_t outBuffer[LinkStatisticsFrameLength + 4] = {0};

    outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[1] = LinkStatisticsFrameLength + 2;
    outBuffer[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

    memcpy(outBuffer + 3, (uint8_t *)&LinkStatistics, LinkStatisticsFrameLength);

    uint8_t crc = crsf_crc.calc(&outBuffer[2], LinkStatisticsFrameLength + 1);

    outBuffer[LinkStatisticsFrameLength + 3] = crc;

#ifdef PLATFORM_ESP32
    portENTER_CRITICAL(&FIFOmux);
#endif
    SerialOutFIFO.push(LinkStatisticsFrameLength + 4); // length
    SerialOutFIFO.pushBytes(outBuffer, LinkStatisticsFrameLength + 4);
#ifdef PLATFORM_ESP32
    portEXIT_CRITICAL(&FIFOmux);
#endif
}

// void CRSF::sendLUAresponse(uint8_t val[], uint8_t len)
// {
//     if (!CRSF::CRSFstate)
//     {
//         return;
//     }

//     uint8_t LUArespLength = len + 2;
//     uint8_t outBuffer[LUArespLength + 5] = {0};

//     outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
//     outBuffer[1] = LUArespLength + 2;
//     outBuffer[2] = CRSF_FRAMETYPE_PARAMETER_WRITE;

//     outBuffer[3] = CRSF_ADDRESS_RADIO_TRANSMITTER;
//     outBuffer[4] = CRSF_ADDRESS_CRSF_TRANSMITTER;

//     for (uint8_t i = 0; i < len; ++i)
//     {
//         outBuffer[5 + i] = val[i];
//     }

//     uint8_t crc = crsf_crc.calc(&outBuffer[2], LUArespLength + 1);

//     outBuffer[LUArespLength + 3] = crc;

// #ifdef PLATFORM_ESP32
//     portENTER_CRITICAL(&FIFOmux);
// #endif
//     SerialOutFIFO.push(LUArespLength + 4); // length
//     SerialOutFIFO.pushBytes(outBuffer, LUArespLength + 4);
// #ifdef PLATFORM_ESP32
//     portEXIT_CRITICAL(&FIFOmux);
// #endif
// }

void ICACHE_RAM_ATTR CRSF::sendTelemetryToTX(uint8_t *data)
{
    if (data[CRSF_TELEMETRY_LENGTH_INDEX] > CRSF_PAYLOAD_SIZE_MAX)
    {
        std::cout << "too large\n";
        return;
    }

    if (CRSF::CRSFstate)
    {
        data[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
#ifdef PLATFORM_ESP32
        portENTER_CRITICAL(&FIFOmux);
#endif
        SerialOutFIFO.push(CRSF_FRAME_SIZE(data[CRSF_TELEMETRY_LENGTH_INDEX])); // length
        SerialOutFIFO.pushBytes(data, CRSF_FRAME_SIZE(data[CRSF_TELEMETRY_LENGTH_INDEX]));
#ifdef PLATFORM_ESP32
        portEXIT_CRITICAL(&FIFOmux);
#endif
    }
}

void ICACHE_RAM_ATTR CRSF::setSyncParams(uint32_t PacketInterval)
{
    CRSF::RequestedRCpacketInterval = PacketInterval;
#ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
    CRSF::SyncWaitPeriodCounter = millis();
    CRSF::OpenTXsyncOffsetSafeMargin = 1000;
    LPF_OPENTX_SYNC_OFFSET.init(0);
    LPF_OPENTX_SYNC_MARGIN.init(0);
#endif
}

uint32_t ICACHE_RAM_ATTR CRSF::GetRCdataLastRecv()
{
    return CRSF::RCdataLastRecv;
}

void ICACHE_RAM_ATTR CRSF::JustSentRFpacket()
{
    CRSF::OpenTXsyncOffset = micros() - CRSF::RCdataLastRecv;

    if (CRSF::OpenTXsyncOffset > (int32_t)CRSF::RequestedRCpacketInterval) // detect overrun case when the packet arrives too late and caculate negative offsets.
    {
        CRSF::OpenTXsyncOffset = -(CRSF::OpenTXsyncOffset % CRSF::RequestedRCpacketInterval);
#ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
        // wait until we stablize after changing pkt rate
        if (millis() > (CRSF::SyncWaitPeriodCounter + AutoSyncWaitPeriod))
        {
            CRSF::OpenTXsyncOffsetSafeMargin = LPF_OPENTX_SYNC_MARGIN.update((CRSF::OpenTXsyncOffsetSafeMargin - OpenTXsyncOffset) + 100); // take worst case plus 50us
        }
#endif
    }

#ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
    if (CRSF::OpenTXsyncOffsetSafeMargin > 4000)
    {
        CRSF::OpenTXsyncOffsetSafeMargin = 4000; // hard limit at no tune default
    }
    else if (CRSF::OpenTXsyncOffsetSafeMargin < 1000)
    {
        CRSF::OpenTXsyncOffsetSafeMargin = 1000; // hard limit at no tune default
    }
#endif
    //Serial.print(CRSF::OpenTXsyncOffset);
    // Serial.print(",");
    // Serial.println(CRSF::OpenTXsyncOffsetSafeMargin / 10);
}


void ICACHE_RAM_ATTR CRSF::sendSyncPacketToTX() // in values in us. ??? JBK: Say what?
{
    uint32_t now = millis();
    if (CRSF::CRSFstate && now >= (OpenTXsyncLastSent + OpenTXsyncPacketInterval))
    {
        uint32_t packetRate;
        if (CRSF::UARTcurrentBaud == 115200 && CRSF::RequestedRCpacketInterval == 2000)
        {
            packetRate = 40000; //constrain to 250hz max (not a rate, it's an interval expressed in 10ths of a us)
        }
        else
        {
            packetRate = CRSF::RequestedRCpacketInterval * 10; //convert from us to right format
        }

        int32_t offset = CRSF::OpenTXsyncOffset * 10 - CRSF::OpenTXsyncOffsetSafeMargin; // + 400us offset so that opentx always has some headroom

        uint8_t outBuffer[OpenTXsyncFrameLength + 4] = {0};

        outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER; //0xEA
        outBuffer[1] = OpenTXsyncFrameLength + 2;      // equals 13?
        outBuffer[2] = CRSF_FRAMETYPE_RADIO_ID;        // 0x3A

        outBuffer[3] = CRSF_ADDRESS_RADIO_TRANSMITTER; //0XEA
        outBuffer[4] = 0x00;                           //??? not sure doesn't seem to matter
        outBuffer[5] = CRSF_FRAMETYPE_OPENTX_SYNC;     //0X10

        outBuffer[6] = (packetRate & 0xFF000000) >> 24; // XXX check the endianness and see if we can just use 32 bit write
        outBuffer[7] = (packetRate & 0x00FF0000) >> 16;
        outBuffer[8] = (packetRate & 0x0000FF00) >> 8;
        outBuffer[9] = (packetRate & 0x000000FF) >> 0;

        outBuffer[10] = (offset & 0xFF000000) >> 24;
        outBuffer[11] = (offset & 0x00FF0000) >> 16;
        outBuffer[12] = (offset & 0x0000FF00) >> 8;
        outBuffer[13] = (offset & 0x000000FF) >> 0;

        uint8_t crc = crsf_crc.calc(&outBuffer[2], OpenTXsyncFrameLength + 1);

        outBuffer[OpenTXsyncFrameLength + 3] = crc;

#ifdef PLATFORM_ESP32
        portENTER_CRITICAL(&FIFOmux);
#endif
        SerialOutFIFO.push(OpenTXsyncFrameLength + 4); // length
        SerialOutFIFO.pushBytes(outBuffer, OpenTXsyncFrameLength + 4);
#ifdef PLATFORM_ESP32
        portEXIT_CRITICAL(&FIFOmux);
#endif
        OpenTXsyncLastSent = now;
    }
}



bool ICACHE_RAM_ATTR CRSF::ProcessPacket()
{
    if (CRSFstate == false)
    {
        CRSFstate = true;
        std::cout << "CRSF UART Connected\n";

#ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
        SyncWaitPeriodCounter = millis(); // set to begin wait for auto sync offset calculation
        LPF_OPENTX_SYNC_MARGIN.init(0);
        LPF_OPENTX_SYNC_OFFSET.init(0);
#endif // FEATURE_OPENTX_SYNC_AUTOTUNE
        connected();
    }

    const uint8_t packetType = CRSF::inBuffer.asRCPacket_t.header.type;

    if (packetType == CRSF_FRAMETYPE_PARAMETER_WRITE)
    {
        const volatile uint8_t *SerialInBuffer = CRSF::inBuffer.asUint8_t;
        if (SerialInBuffer[3] == CRSF_ADDRESS_CRSF_TRANSMITTER &&
            SerialInBuffer[4] == CRSF_ADDRESS_RADIO_TRANSMITTER)
        {
            ParameterUpdateData[0] = SerialInBuffer[5];
            ParameterUpdateData[1] = SerialInBuffer[6];
            RecvParameterUpdate();
            return true;
        }
        std::cout << "Got Other Packet\n";
    }
    else if (packetType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        CRSF::RCdataLastRecv = micros();
        GoodPktsCount++;
        GetChannelDataIn();
        // std::cout << "got data\n";
        return true;
    }
    // else if (packetType == CRSF_FRAMETYPE_MSP_REQ || packetType == CRSF_FRAMETYPE_MSP_WRITE)
    // {
    //     volatile uint8_t *SerialInBuffer = CRSF::inBuffer.asUint8_t;
    //     const uint8_t length = CRSF::inBuffer.asRCPacket_t.header.frame_size + 2;
    //     AddMspMessage(length, SerialInBuffer);
    //     return true;
    // }
    return false;
}

// uint8_t* ICACHE_RAM_ATTR CRSF::GetMspMessage()
// {
//     if (MspDataLength > 0)
//     {
//         return MspData;
//     }
//     return NULL;
// }

// void ICACHE_RAM_ATTR CRSF::ResetMspQueue()
// {
//     MspWriteFIFO.flush();
//     MspDataLength = 0;
//     memset(MspData, 0, ELRS_MSP_BUFFER);
// }

// void ICACHE_RAM_ATTR CRSF::UnlockMspMessage()
// {
//     // current msp message is sent so restore next buffered write
//     if (MspWriteFIFO.peek() > 0)
//     {
//         uint8_t length = MspWriteFIFO.pop();
//         MspDataLength = length;
//         MspWriteFIFO.popBytes(MspData, length);
//     }
//     else
//     {
//         // no msp message is ready to send currently
//         MspDataLength = 0;
//         memset(MspData, 0, ELRS_MSP_BUFFER);
//     }
// }

// void ICACHE_RAM_ATTR CRSF::AddMspMessage(mspPacket_t* packet)
// {
//     if (packet->payloadSize > ENCAPSULATED_MSP_PAYLOAD_SIZE)
//     {
//         return;
//     }

//     const uint8_t totalBufferLen = ENCAPSULATED_MSP_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_NOT_COUNTED_BYTES;
//     uint8_t outBuffer[totalBufferLen] = {0};

//     // CRSF extended frame header
//     outBuffer[0] = CRSF_ADDRESS_BROADCAST;                                      // address
//     outBuffer[1] = ENCAPSULATED_MSP_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // length
//     outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;                                    // packet type
//     outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER;                              // destination
//     outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;                              // origin

//     // Encapsulated MSP payload
//     outBuffer[5] = 0x30;                // header
//     outBuffer[6] = packet->payloadSize; // mspPayloadSize
//     outBuffer[7] = packet->function;    // packet->cmd
//     for (uint8_t i = 0; i < ENCAPSULATED_MSP_PAYLOAD_SIZE; ++i)
//     {
//         // copy packet payload into outBuffer and pad with zeros where required
//         outBuffer[8 + i] = i < packet->payloadSize ? packet->payload[i] : 0;
//     }
//     // Encapsulated MSP crc
//     outBuffer[totalBufferLen - 2] = CalcCRCMsp(&outBuffer[6], ENCAPSULATED_MSP_FRAME_LEN - 2);

//     // CRSF frame crc
//     outBuffer[totalBufferLen - 1] = crsf_crc.calc(&outBuffer[2], ENCAPSULATED_MSP_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC - 1);
//     AddMspMessage(totalBufferLen, outBuffer);
// }

// void ICACHE_RAM_ATTR CRSF::AddMspMessage(const uint8_t length, volatile uint8_t* data)
// {
//     if (length > ELRS_MSP_BUFFER)
//     {
//         return;
//     }

//     // store next msp message
//     if (MspDataLength == 0)
//     {
//         for (uint8_t i = 0; i < length; i++)
//         {
//             MspData[i] = data[i];
//         }
//         MspDataLength = length;
//     }
//     // store all write requests since an update does send multiple writes
//     else
//     {
//         MspWriteFIFO.push(length);
//         for (uint8_t i = 0; i < length; i++)
//         {
//             MspWriteFIFO.push(data[i]);
//         }
//     }
// }

/**
 * This is the main input routine for edgeTx -> module serial traffic
 * 
 * The current implementation isn't a great fit. We should probably rewrite this
 * with HAL or LL and handle the FIFO interrupts ourselves.
 * 
 */
void ICACHE_RAM_ATTR CRSF::handleUARTin()
{
    volatile uint8_t *SerialInBuffer = CRSF::inBuffer.asUint8_t;

    if (UARTwdt())
    {
        return;
    }

    // uint8_t inByte;
    char inChar;
    int nRead = 1;

    while (nRead > 0)
    {
        nRead = uart_read_bytes(CRSF_PORT_NUM, &inChar, 1, 1);
        if (nRead != 1) break;

        if (CRSFframeActive == false)
        {
            // stage 1 wait for sync byte //
            if (inChar == CRSF_ADDRESS_CRSF_TRANSMITTER ||
                inChar == CRSF_SYNC_BYTE)
            {
                // we got sync, reset write pointer
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
                CRSFframeActive = true;
                SerialInBuffer[SerialInPacketPtr] = inChar;
                SerialInPacketPtr++;
            }
        }
        else // frame is active so we do the processing
        {
            // first if things have gone wrong //
            if (SerialInPacketPtr > CRSF_MAX_PACKET_LEN - 1)
            {
                // we reached the maximum allowable packet length, so start again because shit fucked up hey.
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
                CRSFframeActive = false;
                return;
            }

            // special case where we save the expected pkt len to buffer //
            if (SerialInPacketPtr == 1)
            {
                if (inChar <= CRSF_MAX_PACKET_LEN)
                {
                    SerialInPacketLen = inChar;
                }
                else
                {
                    SerialInPacketPtr = 0;
                    SerialInPacketLen = 0;
                    CRSFframeActive = false;
                    return;
                }
            }

            SerialInBuffer[SerialInPacketPtr] = inChar;
            SerialInPacketPtr++;

            if (SerialInPacketPtr >= (SerialInPacketLen + 2)) // plus 2 because the packlen is referenced from the start of the 'type' flag, IE there are an extra 2 bytes.
            {
                char CalculatedCRC = crsf_crc.calc((uint8_t *)SerialInBuffer + 2, SerialInPacketPtr - 3);
                if (CalculatedCRC == inChar)
                {
                    if (ProcessPacket())
                    {
                        handleUARTout();
                    }                        // esp_rom_delay_us(300);
                }
                else
                {
                    std::cout << "UART CRC failure\n";
                    // cleanup input buffer
                    flush_port_input();
                    BadPktsCount++;
                }
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
                break; // exit the loop and give wdt a chance to run
            }
        } // if (frame active or not)
    } // while (something was read from the uart)
}

void ICACHE_RAM_ATTR CRSF::handleUARTout()
{
    // both static to split up larger packages
    static uint8_t packageLength = 0;
    static uint8_t sendingOffset = 0;
    uint8_t writeLength = 0;

    // calculate mixer sync packet if needed
    sendSyncPacketToTX();

    // check if we have data in the output FIFO that needs to be written or a large package was split up and we need to send the second part
    if (sendingOffset > 0 || SerialOutFIFO.peek() > 0) 
    {
        duplex_set_TX();

        // keep trying until the fifo is empty
        // XXX add a time budget to this
        while(sendingOffset > 0 || SerialOutFIFO.peek() > 0)
        {
            #ifdef PLATFORM_ESP32
            portENTER_CRITICAL(&FIFOmux); // stops other tasks from writing to the FIFO when we want to read it
            #endif
            // no package is in transit so get new data from the fifo
            if (sendingOffset == 0) {
                packageLength = SerialOutFIFO.pop();
                SerialOutFIFO.popBytes(CRSFoutBuffer, packageLength);
            }

            // if the package is long we need to split it up so it fits in the sending interval
            if (packageLength > MAX_BYTES_SENT_IN_UART_OUT) {
                writeLength = MAX_BYTES_SENT_IN_UART_OUT;
            } else {
                writeLength = packageLength;
            }

            #ifdef PLATFORM_ESP32
            portEXIT_CRITICAL(&FIFOmux); // stops other tasks from writing to the FIFO when we want to read it
            #endif

            // write the packet out, if it's a large package the offset holds the starting position
            // Needs to be a blocking write - see Begin() for setup of port
            uart_write_bytes(CRSF_PORT_NUM, CRSFoutBuffer + sendingOffset, writeLength);
            uart_wait_tx_done(CRSF_PORT_NUM, 1);

            sendingOffset += writeLength;
            packageLength -= writeLength;

            // after everything was writen reset the offset so a new package can be fetched from the fifo
            if (packageLength == 0) {
                sendingOffset = 0;
            }

        } // while (still data to send)

        duplex_set_RX();

        // make sure there is no garbage on the UART left over
        // This seems dangerous, so far not much sign of downside of not having it
        // flush_port_input();

    } // if (something to send)
}


void ICACHE_RAM_ATTR CRSF::duplex_set_RX()
{
    #if defined(PLATFORM_ESP32) || defined(ESPC3)
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)CRSF_TX_PIN, GPIO_MODE_INPUT));
    // gpio_matrix_in((gpio_num_t)CRSF_TX_PIN, U1RXD_IN_IDX, true);

    #ifdef UART_INVERTED
    gpio_matrix_in((gpio_num_t)CRSF_TX_PIN, U1RXD_IN_IDX, true);
    gpio_pulldown_en((gpio_num_t)CRSF_TX_PIN);
    gpio_pullup_dis((gpio_num_t)CRSF_TX_PIN);
    #else
    gpio_matrix_in((gpio_num_t)CRSF_TX_PIN, U1RXD_IN_IDX, false);
    gpio_pullup_en((gpio_num_t)CRSF_TX_PIN);
    gpio_pulldown_dis((gpio_num_t)CRSF_TX_PIN);
    #endif

    #elif defined(GPIO_PIN_BUFFER_OE) && (GPIO_PIN_BUFFER_OE != UNDEF_PIN)

    digitalWrite(GPIO_PIN_BUFFER_OE, LOW ^ GPIO_PIN_BUFFER_OE_INVERTED);

    #endif
}

void ICACHE_RAM_ATTR CRSF::duplex_set_TX()
{
    #if defined(PLATFORM_ESP32) || defined(ESPC3)

    gpio_matrix_in((gpio_num_t)-1, U1RXD_IN_IDX, false);
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)CRSF_TX_PIN, GPIO_FLOATING));
    // ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)CRSF_TX_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CRSF_TX_PIN, 0));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)CRSF_TX_PIN, GPIO_MODE_OUTPUT));

    #ifdef UART_INVERTED
    gpio_matrix_out((gpio_num_t)CRSF_TX_PIN, U1TXD_OUT_IDX, true, false);
    #else
    gpio_matrix_out((gpio_num_t)CRSF_TX_PIN, U1TXD_OUT_IDX, false, false);
    #endif
    
    #elif defined(GPIO_PIN_BUFFER_OE) && (GPIO_PIN_BUFFER_OE != UNDEF_PIN)
    
    digitalWrite(GPIO_PIN_BUFFER_OE, HIGH ^ GPIO_PIN_BUFFER_OE_INVERTED);
    
    #endif // ESP32 or ESPC3
}


/**
 * Check packets received and cycle baud rate if necessary
 */
bool CRSF::UARTwdt()
{
    uint32_t now = millis();
    bool retval = false;
    if (now >= (UARTwdtLastChecked + UARTwdtInterval))
    {
        if (BadPktsCount >= GoodPktsCount)
        {
            std::cout << "Too many bad UART RX packets!\n";

            if (CRSFstate == true)
            {
                std::cout << "CRSF UART Disconnected\n";
                #ifdef FEATURE_OPENTX_SYNC_AUTOTUNE
                SyncWaitPeriodCounter = now; // set to begin wait for auto sync offset calculation
                CRSF::OpenTXsyncOffsetSafeMargin = 1000;
                CRSF::OpenTXsyncOffset = 0;
                CRSF::OpenTXsyncLastSent = 0;
                #endif
                disconnected();
                CRSFstate = false;
            }

            // uint32_t UARTrequestedBaud = (UARTcurrentBaud == CRSF_OPENTX_FAST_BAUDRATE) ?
            //     CRSF_OPENTX_SLOW_BAUDRATE : CRSF_OPENTX_FAST_BAUDRATE;

            baudrateIndex = (baudrateIndex+1) % OPENTX_N_BAUDS;
            uint32_t UARTrequestedBaud = OPENTX_BAUDS[baudrateIndex];

            std::cout << "UART WDT: Switch to: ";
            std::cout << UARTrequestedBaud;
            std::cout << " baud\n";

            SerialOutFIFO.flush();
            #if defined(PLATFORM_ESP32) || defined (ESPC3)
            uart_flush(CRSF_PORT_NUM);
            uart_set_baudrate(CRSF_PORT_NUM, UARTrequestedBaud);
            // CRSF::Port.flush();
            // CRSF::Port.updateBaudRate(UARTrequestedBaud);
            #else
            CRSF::Port.begin(UARTrequestedBaud);
            #if defined(TARGET_TX_GHOST)
            USART1->CR1 &= ~USART_CR1_UE;
            USART1->CR3 |= USART_CR3_HDSEL;
            USART1->CR2 |= USART_CR2_RXINV | USART_CR2_TXINV | USART_CR2_SWAP; //inv
            USART1->CR1 |= USART_CR1_UE;
            #endif
            #endif
            UARTcurrentBaud = UARTrequestedBaud;
            duplex_set_RX();
            // cleanup input buffer
            flush_port_input();

            retval = true;

        } // if (more bad than good)

        printf("UART STATS Bad:Good = %u:%u\n", BadPktsCount, GoodPktsCount);
        // std::cout << "UART STATS Bad:Good = ";
        // std::cout << BadPktsCount;
        // std::cout << ':';
        // std::cout << GoodPktsCount;
        // std::cout << '\n';

        UARTwdtLastChecked = now;
        GoodPktsCountResult = GoodPktsCount;
        BadPktsCountResult = BadPktsCount;
        BadPktsCount = 0;
        GoodPktsCount = 0;
    }
    return retval;
}

#if defined(PLATFORM_ESP32) || defined(ESPC3)
//RTOS task to read and write CRSF packets to the serial port
void ICACHE_RAM_ATTR CRSF::ESP32uartTask(void *pvParameters)
{
    (void)pvParameters;

    std::cout << "ESP32 CRSF UART LISTEN TASK STARTED\n";
    // CRSF::duplex_set_TX();
    // CRSF::Port.begin(CRSF_OPENTX_FAST_BAUDRATE, SERIAL_8N1,
    //                  GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX,
    //                  false, 500);
    CRSF::duplex_set_RX();
    vTaskDelay(500);
    flush_port_input();
    // XXX might as well just move the loop into handleUARTin()
    for (;;)
    {
        handleUARTin();
    }
}
#endif // PLATFORM_ESP32

#elif defined(CRSF_RX_MODULE) // !CRSF_TX_MODULE
// bool CRSF::RXhandleUARTout()
// {
//     uint8_t peekVal = SerialOutFIFO.peek(); // check if we have data in the output FIFO that needs to be written
//     if (peekVal > 0)
//     {
//         if (SerialOutFIFO.size() > (peekVal))
//         {
//             noInterrupts();
//             uint8_t OutPktLen = SerialOutFIFO.pop();
//             uint8_t OutData[OutPktLen];
//             SerialOutFIFO.popBytes(OutData, OutPktLen);
//             interrupts();
//             this->_dev->write(OutData, OutPktLen); // write the packet out
//             return true;
//         }
//     }
//     return false;
// }

// void ICACHE_RAM_ATTR CRSF::sendLinkStatisticsToFC()
// {
//     uint8_t outBuffer[LinkStatisticsFrameLength + 4];

//     outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
//     outBuffer[1] = LinkStatisticsFrameLength + 2;
//     outBuffer[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

//     memcpy(outBuffer + 3, (uint8_t *)&LinkStatistics, LinkStatisticsFrameLength);

//     uint8_t crc = crsf_crc.calc(&outBuffer[2], LinkStatisticsFrameLength + 1);

//     outBuffer[LinkStatisticsFrameLength + 3] = crc;
// #if !defined(DEBUG_CRSF_NO_OUTPUT) && defined(CRSF_TX_PIN)
//     // SerialOutFIFO.push(LinkStatisticsFrameLength + 4);
//     // SerialOutFIFO.pushBytes(outBuffer, LinkStatisticsFrameLength + 4);
//     //this->_dev->write(outBuffer, LinkStatisticsFrameLength + 4);
//     uart_write_bytes(CRSF_PORT_NUM, outBuffer, LinkStatisticsFrameLength + 4);
// #endif
// }

void ICACHE_RAM_ATTR CRSF::sendLinkStatisticsToFC()
{
    uint8_t outBuffer[LinkStatisticsFrameLength + 4] = {0};

    outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    outBuffer[1] = LinkStatisticsFrameLength + 2;
    #ifdef USE_ELRS_CRSF_EXTENSIONS
    outBuffer[2] = CRSF_FRAMETYPE_LINK_STATISTICS_ELRS;
    #else
    outBuffer[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    #endif

    memcpy(outBuffer + 3, (uint8_t *)&LinkStatistics, LinkStatisticsFrameLength);

    uint8_t crc = crsf_crc.calc(&outBuffer[2], LinkStatisticsFrameLength + 1);

    outBuffer[LinkStatisticsFrameLength + 3] = crc;

    #if !defined(DEBUG_CRSF_NO_OUTPUT) && defined(CRSF_TX_PIN)
    uart_write_bytes(CRSF_PORT_NUM, outBuffer, LinkStatisticsFrameLength + 4);
    #endif
}

/**
 * Send an original RCFrame to the FC using data in PackedRCdataOut
 */
void ICACHE_RAM_ATTR CRSF::sendRCFrameToFC()
{
    // uint8_t outBuffer[RCframeLength + 4] = {0}; // Don't think the zero init is necessary
    uint8_t outBuffer[RCframeLength + 4];

    outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    outBuffer[1] = RCframeLength + 2;

    #ifdef USE_ELRS_CRSF_EXTENSIONS

    outBuffer[2] = CRSF_FRAMETYPE_RC_ELRS;

    #else

    outBuffer[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    #endif

    memcpy(outBuffer + 3, (uint8_t *)&PackedRCdataOut, RCframeLength);

    uint8_t crc = crsf_crc.calc(&outBuffer[2], RCframeLength + 1);

    outBuffer[RCframeLength + 3] = crc;

#ifndef DEBUG_CRSF_NO_OUTPUT
    uart_write_bytes(CRSF_PORT_NUM, outBuffer, RCframeLength + 4);
#endif
}

#if defined(USE_HIRES_DATA) || defined(USE_DB_PACKETS)
void ICACHE_RAM_ATTR CRSF::sendHiResRCFrameToFC()
{
    uint8_t outBuffer[RCHiResframeLength + 4] = {0};

    // std::cout << "S";

    outBuffer[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    outBuffer[1] = RCHiResframeLength + 2;
    outBuffer[2] = CRSF_FRAMETYPE_RC_ELRS_HIRES;

    memcpy(outBuffer + 3, (void *)&elrsPackedHiResRCdataOut, RCHiResframeLength);

    uint8_t crc = CalcCRC(&outBuffer[2], RCHiResframeLength + 1);

    outBuffer[RCHiResframeLength + 3] = crc;

    #ifndef DEBUG_CRSF_NO_OUTPUT
    uart_write_bytes(CRSF_PORT_NUM, outBuffer, RCHiResframeLength + 4);
    #endif
}
#endif // use_hires_data or use_db_packets


// void ICACHE_RAM_ATTR CRSF::sendMSPFrameToFC(uint8_t* data)
// {
//     const uint8_t totalBufferLen = 14;

//     // SerialOutFIFO.push(totalBufferLen);
//     // SerialOutFIFO.pushBytes(outBuffer, totalBufferLen);
//     this->_dev->write(data, totalBufferLen);
// }
#endif // CRSF_TX_MODULE


/**
 * Convert the rc data corresponding to switches to 3 bit values.
 * The output is mapped evenly across 6 output values (0-5)
 * With a special value 7 indicating the middle so it works
 * with switches with a middle position as well as 6-position
 */
void ICACHE_RAM_ATTR CRSF::updateSwitchValues()
{
    // AUX1 is arm switch, one bit
    currentSwitches[0] = CRSF_to_BIT(ChannelDataIn[4]);

    // AUX2-(N-1) are Low Resolution, "7pos" (6+center)
    const uint16_t CHANNEL_BIN_COUNT = 6;
    const uint16_t CHANNEL_BIN_SIZE = CRSF_CHANNEL_VALUE_SPAN / CHANNEL_BIN_COUNT;
    for (int i = 1; i < N_SWITCHES-1; i++)
    {
        uint16_t ch = ChannelDataIn[i + 4];
        // If channel is within 1/4 a BIN of being in the middle use special value 7
        if (ch < (CRSF_CHANNEL_VALUE_MID-CHANNEL_BIN_SIZE/4)
            || ch > (CRSF_CHANNEL_VALUE_MID+CHANNEL_BIN_SIZE/4))
            currentSwitches[i] = CRSF_to_N(ch, CHANNEL_BIN_COUNT) & 0b111;
        else
            currentSwitches[i] = 7;
    } // for N_SWITCHES

    // AUXx is High Resolution 16-pos (4-bit)
    currentSwitches[N_SWITCHES-1] = CRSF_to_N(ChannelDataIn[N_SWITCHES-1 + 4], 16) & 0b1111;
}

void ICACHE_RAM_ATTR CRSF::GetChannelDataIn() // data is packed as 11 bits per channel
{
    const volatile crsf_channels_t *rcChannels = &CRSF::inBuffer.asRCPacket_t.channels;
    ChannelDataIn[0] = (rcChannels->ch0);
    ChannelDataIn[1] = (rcChannels->ch1);
    ChannelDataIn[2] = (rcChannels->ch2);
    ChannelDataIn[3] = (rcChannels->ch3);
    ChannelDataIn[4] = (rcChannels->ch4);
    ChannelDataIn[5] = (rcChannels->ch5);
    ChannelDataIn[6] = (rcChannels->ch6);
    ChannelDataIn[7] = (rcChannels->ch7);
    ChannelDataIn[8] = (rcChannels->ch8);
    ChannelDataIn[9] = (rcChannels->ch9);
    ChannelDataIn[10] = (rcChannels->ch10);
    ChannelDataIn[11] = (rcChannels->ch11);
    ChannelDataIn[12] = (rcChannels->ch12);
    ChannelDataIn[13] = (rcChannels->ch13);
    ChannelDataIn[14] = (rcChannels->ch14);
    ChannelDataIn[15] = (rcChannels->ch15);

    updateSwitchValues();
}

#endif // ESPC3