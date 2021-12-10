/**
 * Initial test code for C3 based receivers
 * 
 * TODO:
 *   Fix SPI collisions
 *   Add code to detect if a modem stops talking to us. There should always be one of txdone, rxdone or timeout in every frame
 *   Add linkstatus packets -> FC
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "freertos/queue.h"

#include "user_config.h"
#include "config.h"

#include "common.h"
#include "FHSS.h"
#include "SX1262Driver.h"
#include "HwTimer.h"
#include "CRSF.h"
#include "PFD.h"

#include "LowPassFilter.h"
#include "LQCALC.h"
#include "OTA.h"

#include "driver/rmt.h"
#include "led_strip.h"

#define LED_BRIGHTNESS 32

// #define PRINT_RX_SCOREBOARD 1


#ifdef USE_PWM6

// PWM constants

#define PWM_FREQ 50 // The cheap 9G servos become unstable at > 50Hz (even 75 or 100Hz is too much)
#define PWM_PERIOD (1000000/PWM_FREQ)

#define PWM_TICKS 16384

// standard 1000 to 2000us pulse range, gives about 90 degrees of travel
#define PWM_MIN (PWM_TICKS * 1000 / PWM_PERIOD)
#define PWM_MAX (PWM_TICKS * 2000 / PWM_PERIOD)
#define PWM_RANGE (PWM_MAX - PWM_MIN)

const int8_t PwmPins[] = {PWM_CH1_PIN, PWM_CH2_PIN, PWM_CH3_PIN, PWM_CH4_PIN, PWM_CH5_PIN, PWM_CH6_PIN};

#endif


#ifdef RADIO_E22

SX1262Driver radio1;

#elif defined (RADIO_E28_12) || defined(RADIO_E28_20) || defined(RADIO_E28_27)

// XXX Having one of these as a pointer and one not is going to be really annoying. Fix
SX1280Driver radio1;
SX1280Driver * radio2 = NULL;

#else
error("must define a radio module")

#endif // RADIO_*

enum DynamicRateStates
{
    Normal,
    ShiftUp,
    ShiftDown    
};

enum RadioSelection
{
    first,
    second,
    both
};

enum SpiTaskID
{
    RxDone,
    SetFrequency,
    SetStandby,
    StartRx,
    StartTx,
    ClearIRQs
};

typedef struct 
{
    SpiTaskID id;
    uint32_t radioID;
    union {
        uint32_t frequency;
        uint32_t irqMask;
    };

} SpiTaskInfo;

DynamicRateStates drState = DynamicRateStates::Normal;

CRSF crsf;
GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);
PFD pfdLoop;

uint8_t ExpressLRS_nextAirRateIndex = 0;
uint32_t timeout1Counter = 0;
uint32_t timeout2Counter = 0;
uint32_t mismatchCounter = 0;
uint32_t totalPackets = 0;
uint32_t totalRX1Events = 0;
uint32_t totalRX2Events = 0;
uint32_t totalMatches = 0;
uint32_t totalRX2 = 0;
int32_t cumulativeRSSI = 0;
uint32_t crc1Counter = 0;
uint32_t crc2Counter = 0;
uint32_t errCounter = 0;

uint32_t r1LastIRQms = 0;
uint32_t r2LastIRQms = 0;

static xQueueHandle rx_evt_queue = NULL;
static xQueueHandle tx_evt_queue = NULL;
static xQueueHandle tx2_evt_queue = NULL;

volatile uint8_t NonceRX = 0; // nonce that we THINK we are up to.

uint16_t timerNonce = 0;   // incremented every time the hwtimer fires, and scaled by rfModeDivisor to get the NonceTX
// XXX divisor would be more efficient if specified as a bit shift
uint32_t rfModeDivisor = 1; // Since the hwTimer now runs at a fixed 1kHz we need to a factor to scale to the packet rate

bool alreadyFHSS = false;
bool alreadyTLMresp = false;
volatile bool packetReceived = false;

bool radio1Timedout = false;
bool radio2Timedout = false;

// uint32_t beginProcessing;
// uint32_t doneProcessing;

volatile uint32_t tPacketR1 = 0;
volatile uint32_t tPacketR2 = 0;
volatile uint32_t lastValidPacket = 0;
volatile uint32_t lastSyncPacket = 0;

uint32_t linkStatstoFCLastSent = 0;

uint32_t pfdDebugCounter = 0; // when greater than 0 updatePFD will print the offset

int32_t RawOffset;
int32_t prevRawOffset;
int32_t Offset;
int32_t OffsetDx;
int32_t prevOffset;
RXtimerState_e RXtimerState = tim_disconnected;
uint32_t GotConnectionMillis = 0;
const uint32_t ConsiderConnGoodMillis = 1000; // minimum time before we can consider a connection to be 'good'

LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);


// uint32_t cycleInterval; // in ms
uint32_t RFmodeLastCycled = 0;
#define RFmodeCycleMultiplierSlow 10
uint8_t RFmodeCycleMultiplier;
bool LockRFmode = false;

// Track how long the timer callbacks have been running. 0 = not running
static uint32_t tickStartMs = 0;
static uint32_t tockStartMs = 0;


// LPF LPF_UplinkRSSI(5);
LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
LPF LPF_UplinkRSSI1(5);

LPF LPF_rcvInterval(4);

// filter for LQ for use with dynamic rates
LPF lpfLq(3);

/// LQ Calculation //////////
LQCALC<100> lqRadio1, lqRadio2, lqEffective;
uint8_t uplinkLQ;

uint8_t scanIndex = RATE_DEFAULT; // used when cycling through RF modes

uint8_t antenna = 0;    // which antenna is currently in use

bool isRXconnected = false;

#if defined(PRINT_RX_SCOREBOARD)
static bool lastPacketCrcError;
#endif

led_strip_t *strip = nullptr;

// forward refs
void setLedColour(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
void liveUpdateRFLinkRate(uint8_t index);


// XXX TODO move this somewhere sensible
void ICACHE_RAM_ATTR delay(uint32_t millis)
{
    if (millis < 30) {
        esp_rom_delay_us(millis*1000);
    } else {
        vTaskDelay(millis / portTICK_PERIOD_MS);
    }
}

void ICACHE_RAM_ATTR getRFlinkInfo()
{

    int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
    int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;

    int8_t t1, t2;
    switch (antenna) {
        case 0:
            t1 = radio1.GetLastPacketRSSI();
            rssiDBM0 = LPF_UplinkRSSI0.update(t1);
            // printf("update0 %d %d\n", t1, rssiDBM0);
            break;
        case 1:
            // rssiDBM1 = LPF_UplinkRSSI1.update(radio1.GetLastPacketRSSI());
            t2 = radio2->GetLastPacketRSSI();
            rssiDBM1 = LPF_UplinkRSSI1.update(t2);
            // printf("update1 %d %d\n", t2, rssiDBM1);

            break;
    }

    // int32_t rssiDBM = (antenna == 0) ? rssiDBM0 : rssiDBM1;
    // crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(constrain(rssiDBM, ExpressLRS_currAirRate_RFperfParams->RXsensitivity, -50),
    //                                            ExpressLRS_currAirRate_RFperfParams->RXsensitivity, -50, 0, 1023));
    // crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(uplinkLQ, 0, 100, 0, 1023));

    if (rssiDBM0 > 0) rssiDBM0 = 0;
    if (rssiDBM1 > 0) rssiDBM1 = 0;

    // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
    crsf.LinkStatistics.uplink_RSSI_1 = -rssiDBM0;
    crsf.LinkStatistics.uplink_RSSI_2 = -rssiDBM1;
    crsf.LinkStatistics.active_antenna = antenna;
    crsf.LinkStatistics.uplink_SNR = radio1.LastPacketSNR;
    crsf.LinkStatistics.uplink_Link_quality = uplinkLQ;
    crsf.LinkStatistics.rf_Mode = (uint8_t)RATE_LAST - (uint8_t)ExpressLRS_currAirRate_Modparams->enum_rate;
    //Serial.println(crsf.LinkStatistics.uplink_RSSI_1);
}

void TentativeConnection()
{
    pfdLoop.reset();
    connectionStatePrev = connectionState;
    connectionState = tentative;
    RXtimerState = tim_disconnected;
    // printf("tentative conn\n");
    std::cout << "TC";
    FreqCorrection = 0;
    Offset = 0;
    prevOffset = 0;
    LPF_Offset.init(0);
    RFmodeLastCycled = millis(); // reset the clock for cycling the mode

    LPF_rcvInterval.init(ExpressLRS_currAirRate_Modparams->interval);

    ExpressLRS_nextAirRateIndex = ExpressLRS_currAirRate_Modparams->index;

    // set timeout based on interval?

    radio1.setRxTimeout(20000);
    radio2->setRxTimeout(20000);


    #ifdef LED_STATUS_INDEX
    setLedColour(LED_STATUS_INDEX, 40, 20, 0);
    #endif

    // The caller MUST call hwTimer.resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}

/** Update the PWM output values from the channel data
 * 
 */
void updatePWM()
{
    #ifdef USE_PWM6
    for(int i=0; i<sizeof(PwmPins); i++) {
        if (PwmPins[i] != -1) {
            int32_t channelValue;
            switch (i) {
                case 0:
                channelValue = crsf.PackedRCdataOut.ch0;
                break;

                case 1:
                channelValue = crsf.PackedRCdataOut.ch1;
                break;

                case 2:
                channelValue = crsf.PackedRCdataOut.ch2;
                break;

                case 3:
                channelValue = crsf.PackedRCdataOut.ch3;
                break;

                case 4:
                channelValue = crsf.PackedRCdataOut.ch4;
                break;

                case 5:
                channelValue = crsf.PackedRCdataOut.ch5;
                break;
            }
            // TODO this will be different if using a native packet format instead of the reduced crsf range
            uint32_t newDuty = (channelValue - CRSF_CHANNEL_VALUE_1000) * PWM_RANGE / CRSF_CHANNEL_VALUE_SPAN + PWM_MIN;
            // printf("duty %lu\n", newDuty);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, newDuty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
        }
    }
    #endif // USE_PWM6
}

bool hasValidCRC(uint8_t *rxBuffer)
{
    const uint8_t type = rxBuffer[0] & 0b11;
    const uint16_t inCRC = ( ( (uint16_t)(rxBuffer[0] & 0b11111100) ) << 6 ) | rxBuffer[OTA_PACKET_LENGTH-1];

    rxBuffer[0] = type; // NB this destroys the original CRC data, so you can only call this function once per buffer

    uint16_t calculatedCRC = ota_crc.calc(rxBuffer, OTA_PACKET_LENGTH-1);

    return (inCRC == calculatedCRC);
}

/**
 * @return currently always returns 1
 */
uint8_t ICACHE_RAM_ATTR ProcessRFPacket(uint8_t *rxBuffer, uint32_t tPacketReceived)
{
    uint8_t type;

    // uint32_t beginProcessing;
    // uint32_t doneProcessing;

    // beginProcessing = micros();

    #ifdef USE_PWM6

    // XXX this code needs review and updating for dual modem

    type = ((Pwm6Payload_t*) radio1.RXdataBuffer)->header;
    inCRC = ((Pwm6Payload_t*) radio1.RXdataBuffer)->crc;
    ((Pwm6Payload_t*) radio1.RXdataBuffer)->crc = 0; // zero out the crc bits to match the sender

    uint16_t calculatedCRC = ota_crc.calc(radio1.RXdataBuffer, OTA_PACKET_LENGTH-1);

    #else

    type = rxBuffer[0] & 0b11;

    #endif // USE_PWM6

    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", rxBuffer[i]);
    // printf(", inCRC %04X calcCRC %04X\n", inCRC, calculatedCRC);


    // try this again?
    // Use the difference between interval and TOA to figure out how long after the packet is received until the
    // start of the frame
    // pfdLoop.extEvent(tPacketReceived + (ExpressLRS_currAirRate_Modparams->interval * 75 / 100));

    // TODO since the crc checking was moved out of this function does it make sense to move the pfd extEvent() call
    // out as well?

    // get the per mode offset
    uint32_t pfdOffset = ExpressLRS_currAirRate_RFperfParams->pfdOffset;

    pfdLoop.extEvent(tPacketReceived + pfdOffset);

#ifdef HYBRID_SWITCHES_8
    const uint8_t SwitchEncModeExpected = 0b01;
#else
    const uint8_t SwitchEncModeExpected = 0b00;
#endif
    uint8_t SwitchEncMode;
    uint8_t indexIN;
    uint8_t TLMrateIn;
    #if defined(ENABLE_TELEMETRY) && defined(HYBRID_SWITCHES_8)
    bool telemetryConfirmValue;
    #endif
    // bool currentMspConfirmValue;
    bool doStartTimer = false;

    lastValidPacket = millis();

    switch (type)
    {
    case RC_DATA_PACKET: //Standard RC Data Packet
        #ifdef USE_HIRES_DATA
        UnpackHiResChannelData(rxBuffer, &crsf);
        #elif defined(USE_PWM6)
        UnpackChannelDataPWM6((Pwm6Payload_t*)rxBuffer, &crsf);

        #else
        UnpackChannelDataHybridSwitches8(rxBuffer, &crsf);
        #endif

        #ifdef ENABLE_TELEMETRY
        telemetryConfirmValue = rxBuffer[6] & (1 << 7);
        TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);
        #endif

        if (connectionState != disconnected)
        {
            #ifdef CRSF_TX_PIN
            crsf.sendRCFrameToFC();
            #endif

            updatePWM();

        }
        break;

    case TLM_PACKET: //telemetry packet from master

        // not implimented yet
        break;

    case SYNC_PACKET: //sync packet from master

        #ifdef ELRS_OG_COMPATIBILITY
        indexIN = (rxBuffer[3] & 0b11000000) >> 6;
        TLMrateIn = (rxBuffer[3] & 0b00111000) >> 3;
        SwitchEncMode = (rxBuffer[3] & 0b00000110) >> 1;

        if (SwitchEncModeExpected == SwitchEncMode && rxBuffer[4] == UID[3] && rxBuffer[5] == UID[4] && rxBuffer[6] == UID[5])
        #else
        indexIN = ((rxBuffer[3] & 0b11100000) >> 5);
        TLMrateIn = ((rxBuffer[3] & 0b00011100) >> 2);
        // SwitchEncMode = (rxBuffer[3] & 0b00000110) >> 1;
        SwitchEncMode = HYBRID_SWITCHES_8; // fixed when not running in compatibility mode

        if (rxBuffer[5] == UID[4] && rxBuffer[6] == UID[5])
        #endif
        {
            lastSyncPacket = millis();
            #if defined(PRINT_RX_SCOREBOARD)
            // printf("s");
            std::cout << 's';
            #endif

            if (ExpressLRS_currAirRate_Modparams->TLMinterval != (expresslrs_tlm_ratio_e)TLMrateIn)
            { // change link parameters if required
                #ifndef DEBUG_SUPPRESS
                printf("New TLMrate: %u\n", TLMrateIn);
                #endif
                ExpressLRS_currAirRate_Modparams->TLMinterval = (expresslrs_tlm_ratio_e)TLMrateIn;
                //  telemBurstValid = false;
            }

            #ifdef ELRS_OG_COMPATIBILITY

            if (connectionState == disconnected
                || NonceRX != rxBuffer[2]
                || FHSSgetCurrIndex() != rxBuffer[1]-1) // XXX don't forget this one when fixing the offsets
            {
                //Serial.print(NonceRX, DEC); Serial.write('x'); Serial.println(rxBuffer[2], DEC);
                FHSSsetCurrIndex(rxBuffer[1]-1); // XXX take the +1 off the transmitter and stop messing around
                NonceRX = rxBuffer[2];
                TentativeConnection();
                doStartTimer = true;
            }

            #else // not ELRS_OG_COMPATIBILITY

            uint16_t newTimerNonce = (rxBuffer[1] << 8) + rxBuffer[2] + 0;
            const uint8_t nonceOffsets[] = { 0, 1, 3, 6}; // n(n+1)/2 - is this going to be stable?

            if (connectionState == disconnected)
            {
                // FHSSsetCurrIndex(rxBuffer[1]);
                // NonceRX = rxBuffer[2] + 1;

                timerNonce = 2 * (newTimerNonce + rfModeDivisor); // rfModeDivisor approximates how many increments happened on the tx while the packet was on the air

                NonceRX = timerNonce / (2 * rfModeDivisor);

                // printf("tN %u, cI %u fI %u\n", timerNonce, currentIndex, newFhssIndex);

                TentativeConnection();
                doStartTimer = true;

            } else { // not disconnected

                uint16_t expectedNonce = (newTimerNonce + nonceOffsets[ExpressLRS_currAirRate_Modparams->index]) & 0x7FFF; // XXX should the tx run the nonce at 2kHz as well?
                uint16_t currentNonce = timerNonce / 2;
                if (currentNonce != expectedNonce)
                {
                    printf("nonce slip: current %u, expected %u\n", currentNonce, expectedNonce);
                    timerNonce = 2 * (newTimerNonce + rfModeDivisor);
                    NonceRX = timerNonce / (2 * rfModeDivisor);
                }

            }
            #endif // not ELRS_OG_COMPATIBILITY

            // if (ExpressLRS_currAirRate_Modparams->index != (expresslrs_tlm_ratio_e)indexIN)
            if (ExpressLRS_currAirRate_Modparams->index != indexIN)
            {
                // change link parameters
                ExpressLRS_nextAirRateIndex = indexIN;
                liveUpdateRFLinkRate(ExpressLRS_nextAirRateIndex);

                // printf("changing rate index from %u to %u\n", ExpressLRS_currAirRate_Modparams->index, indexIN);
                uint32_t packetHz = 1000 / (1 << indexIN);
                printf("\nrate change: %u Hz\n", packetHz);
            }


        } else {
            printf("rejected sync pkt:");
            if (SwitchEncModeExpected != SwitchEncMode) printf(" switch mode");
            #ifdef ELRS_OG_COMPATIBILITY
            if (rxBuffer[4] != UID[3]) printf(", uid3");
            #endif
            if (rxBuffer[6] != UID[5]) printf(", uid5");
            printf("\n");
        }
        break;

    default:
        break;
    }

    // LQCalc.add(); // Received a packet, that's the definition of LQ
    // Extend cycle duration since we've received a packet at this rate
    // but do not extend it indefinitely
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

    // doneProcessing = micros();
    // #if defined(PRINT_RX_SCOREBOARD)
    // if (type != SYNC_PACKET) printf("R");
    // #endif

    if (doStartTimer)
        HwTimer::resume(); // will throw an interrupt immediately (JBK: not sure if this is true across all impls)

    return 1;
}

bool ICACHE_RAM_ATTR HandleFHSS()
{
    static uint16_t lastTimerNonce = 0;

    // Optimise for the case where we've already done the hop or will never hop - we want to get out as quickly as possible
    // These conditions will have short circuit evaluation, left to right.
    if (alreadyFHSS || (ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || (connectionState == disconnected))
    {
        // if (alreadyFHSS) std::cout << "H-already\n";
        // if (connectionState == disconnected) std::cout << "H-disc\n";
        return false;
    }

    // ok, so we might hop if we're on the right nonce, time to do the calculation
    // uint8_t modresultFHSS = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    uint8_t modresultFHSS = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    if (modresultFHSS != 0)
    {
        // std::cout << "H-nonce\n";
        return false;
    }

    // printf("fhss...\n");

    alreadyFHSS = true;

    // privatize so that the debug is consistent with what was used in the calculation
    uint16_t localTimerNonce = timerNonce;

    // We need to make sure that we're not calculating the index right on the boundary of the change,
    // so this offset moves the calculation into the middle of the range.
    uint16_t roundingOffset = rfModeDivisor * ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    uint8_t newIndex = (localTimerNonce - roundingOffset) / (2 * rfModeDivisor * ExpressLRS_currAirRate_Modparams->FHSShopInterval);

    // XXX debug
    uint8_t currentIndex = FHSSgetCurrIndex();
    uint8_t expectedIndex = currentIndex + 1;
    if (newIndex != expectedIndex) {
        printf("!!! fhss current %u, new %u, lastTN %u, current TN %u\n", currentIndex, newIndex, lastTimerNonce, localTimerNonce);
    }
    lastTimerNonce = localTimerNonce;

    FHSSsetCurrIndex(newIndex);

    uint32_t freq = FHSSgetCurrFreq();

    SpiTaskInfo taskInfo = {SpiTaskID::SetFrequency, 0, freq}; // 0 for both radios, 
    xQueueSend(rx_evt_queue, &taskInfo, 1000);


    // radio1.SetFrequency(freq);

    // #ifdef USE_SECOND_RADIO
    // radio2->SetFrequency(freq);
    // #endif

    // std::cout << 'H';

    // printf("done\n");

    // printf("f %u\n", freq);

    // XXX do we need this rxnb call?

    // uint8_t modresultTLM = (NonceRX + 1) % (TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval));
    // if (modresultTLM != 0 || ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) // if we are about to send a tlm response don't bother going back to rx
    // {
    //     radio1.RXnb();
    // }

    return true;
}

bool ICACHE_RAM_ATTR isTelemetryFrame()
{
    if (ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM)
    {
        return false;
    }

    #ifdef ELRS_OG_COMPATIBILITY
    const uint8_t modresult = (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    #else
    // const uint8_t modresult = (NonceRX + 0) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    const uint8_t modresult = (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    #endif

    return (modresult == 0);
}

char * telemWhere = (char *)"not set";

bool ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    #ifdef ENABLE_TELEMETRY
    uint8_t *data;
    uint8_t maxLength;
    uint8_t packageIndex;
    #endif

    // short circuit evaluation, so put the most likely conditions for the critical path first
    if (alreadyTLMresp || (connectionState == disconnected) || (ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM))
    {
        return false;
    }

    if (!isTelemetryFrame())
    {
        return false;
    }

    // printf("telem...\n");

    // XXX Use the radio with the best LQ for sending telem
    SX1280Driver *sendingRadio = &radio1;
    // SX1280Driver *otherRadio = radio2;

    telemWhere = (char *)"FS";

    // turn off the other radio so it doesn't report our own telem packet

    // SpiTaskInfo taskInfo;
    // taskInfo.id = SpiTaskID::SetStandby;
    // taskInfo.radioID = 2;

    // XXX put taskInfo onto the queue instead of calling SetMode directly
    // xQueueSend(rx_evt_queue, &taskInfo, 1000);

    // otherRadio->SetMode(SX1280_MODE_FS);

    alreadyTLMresp = true;
    sendingRadio->TXdataBuffer[0] = 0b11; // tlm packet

    #ifdef ELRS_OG_COMPATIBILITY
    radio1.TXdataBuffer[1] = 1; // ELRS_TELEMETRY_TYPE_LINK; XXX 
    #else
    sendingRadio->TXdataBuffer[1] = CRSF_FRAMETYPE_LINK_STATISTICS;
    #endif

    // OpenTX RSSI as -dBm is fine and supports +dBm values as well
    // but the value in linkstatistics is "positivized" (inverted polarity)
    sendingRadio->TXdataBuffer[2] = -crsf.LinkStatistics.uplink_RSSI_1;
    sendingRadio->TXdataBuffer[3] = -crsf.LinkStatistics.uplink_RSSI_2;
    sendingRadio->TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
    sendingRadio->TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;

    // XXX add the active antenna indicator

    telemWhere = (char *)"crc";

    // XXX hard coded packet length - does it matter?
    uint16_t crc = ota_crc.calc(sendingRadio->TXdataBuffer, 7);
    sendingRadio->TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    sendingRadio->TXdataBuffer[7] = crc & 0xFF;

    telemWhere = (char *)"tx";

    SpiTaskInfo taskInfo;
    taskInfo.id = SpiTaskID::StartTx;
    taskInfo.radioID = 1;

    xQueueSend(rx_evt_queue, &taskInfo, 1000);


    // sendingRadio->TXnb(sendingRadio->TXdataBuffer, 8);
    // printf("telem sent ");
    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", radio1.TXdataBuffer[i]);
    // printf("crc %04X\n", crc);

    // printf("...done\n");

    telemWhere = (char *)"end";
    return true;
}

/** Handle non-critical path interrupts
 * 
 * This was originally just for tx_done but has been expanded to handle all
 * the interrupts other than rx_done (rx_done being on the most critcal path for a receiver)
 * 
 * XXX rename the function
 */
static void ICACHE_RAM_ATTR tx_task(void* arg)
{
    SpiTaskInfo taskInfo;

    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(tx_evt_queue, &dummyData, portMAX_DELAY))
        {
            r1LastIRQms = millis();

            uint16_t irqs = radio1.GetIrqStatus(); // XXX how can we do this safely?

            // Sometimes we get preamble events even when not configured, so this needs to be guarded
            // if (irqs & SX1280_IRQ_PREAMBLE_DETECTED)
            // {
            //     // store the packet time here for use in PFD to reduce jitter on CRC failover
            //     tPacketR1 = esp_timer_get_time();
            //     radio1.ClearIrqStatus(SX1280_IRQ_PREAMBLE_DETECTED);
            //     // printf("p1");
            //     std::cout << "p1";

            // } else if (irqs & SX1280_IRQ_TX_DONE) {
            if (irqs & SX1280_IRQ_TX_DONE) {
                // give fhss a chance to run before tock()
                HandleFHSS();

                // start the next receive
                // XXX should this test if the next frame is for telem?
                taskInfo.id = SpiTaskID::StartRx;
                taskInfo.radioID = 0; // 0 for both radios
                xQueueSend(rx_evt_queue, &taskInfo, 1000);

                // radio1.RXnb();
                // radio2->RXnb();

            } else if (irqs & SX1280_IRQ_RX_TX_TIMEOUT) {
                // printf("r1 timeout\n");
                timeout1Counter++;
                // If we're connected it's safer to restart the receive from tock(), but
                // if we're not connected then the hwtimer isn't running and tock won't be called
                if (HwTimer::isRunning()) {
                    radio1Timedout = true;

                    taskInfo.id = SpiTaskID::ClearIRQs;
                    taskInfo.radioID = 1;
                    taskInfo.irqMask = SX1280_IRQ_RX_TX_TIMEOUT;
                    xQueueSend(rx_evt_queue, &taskInfo, 1000);

                    // radio1.ClearIrqStatus(SX1280_IRQ_RX_TX_TIMEOUT); // XXX replace with a new rxTask event
                    // std::cout << "DT1";
                } else {
                    // radio1.RXnb(); // clears all IRQs
                    taskInfo.id = SpiTaskID::StartRx;
                    taskInfo.radioID = 1;
                    xQueueSend(rx_evt_queue, &taskInfo, 1000);

                    // std::cout << "T1";
                }
            } else {
                printf("!!! tx_task irqs %04X\n", irqs);
            }
        }
    }
}

static void ICACHE_RAM_ATTR tx2_task(void* arg)
{
    SpiTaskInfo taskInfo;

    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(tx2_evt_queue, &dummyData, portMAX_DELAY))
        {
            r2LastIRQms = millis();

            uint16_t irqs = radio2->GetIrqStatus();

            // TODO Sometimes we get preamble events even when not configured, so this needs to be guarded
            // Since it's not currently used, just comment it out

            // if (irqs & SX1280_IRQ_PREAMBLE_DETECTED)
            // {
            //     // store the packet time here for use in PFD to reduce jitter on CRC failover
            //     tPacketR2 = esp_timer_get_time();
            //     radio2->ClearIrqStatus(SX1280_IRQ_PREAMBLE_DETECTED);
            //     // printf("p2");
            //     std::cout << "p2";

            // } else if (irqs & SX1280_IRQ_TX_DONE) {

            if (irqs & SX1280_IRQ_TX_DONE) {
                // printf("tx2!\n");            
                // give fhss a chance to run before tock()
                HandleFHSS();

                // start the next receive
                taskInfo.id = SpiTaskID::StartRx;
                taskInfo.radioID = 0; // 0 for both radios
                xQueueSend(rx_evt_queue, &taskInfo, 1000);

                // radio1.RXnb();
                // radio2->RXnb();

            } else if (irqs & SX1280_IRQ_RX_TX_TIMEOUT) {
                // printf("r2 timeout\n");
                timeout2Counter++;
                if (HwTimer::isRunning()) {
                    radio2Timedout = true;
                    taskInfo.id = SpiTaskID::ClearIRQs;
                    taskInfo.radioID = 2;
                    taskInfo.irqMask = SX1280_IRQ_RX_TX_TIMEOUT;
                    xQueueSend(rx_evt_queue, &taskInfo, 1000);
                    // radio2->ClearIrqStatus(SX1280_IRQ_RX_TX_TIMEOUT);
                    // std::cout << "DT2";
                } else {
                    // radio2->RXnb(); // implicit clear irqs
                    taskInfo.id = SpiTaskID::StartRx;
                    taskInfo.radioID = 2;
                    xQueueSend(rx_evt_queue, &taskInfo, 1000);

                    // std::cout << "T2";
                }

            } else {
                printf("!!! tx2_task irqs %04X\n", irqs);
            }
        }
    }
}


static void doReceive(const int radioID)
{

    // unsigned long tRcv = micros();

    // timeouts for sx1280 are currently routed into the tx_tasks
    // XXX do the same for E22 (sx1262)
    #ifdef RADIO_E22

    uint16_t irqS = radio1.GetIrqStatus();

    if (irqS & SX1262_IRQ_RX_TX_TIMEOUT)    // TODO move this to the tx task so that our mainline rx path is shorter
    {
        timeoutCounter++;
        // need to start another rx
        radio1.RXnb();

        continue;
    }

    #endif // RADIO_E22

    // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // rx has finished, so clear the debug pin
    // uint8_t processResult = 0;

    switch (radioID) {
        case 1:

            r1LastIRQms = millis();

            radio1.readRXData(); // get the data from the radio chip
            if (hasValidCRC((uint8_t*)radio1.RXdataBuffer))
            {
                if (!packetReceived)
                {
                    ProcessRFPacket((uint8_t*)radio1.RXdataBuffer, tPacketR1);
                    totalRX1Events++;
                    packetReceived = true;
                    HandleFHSS();
                    lqEffective.add();
                    #if defined(PRINT_RX_SCOREBOARD)
                    // printf("1");
                    std::cout << '1';
                    #endif

                } else {
                    // Skip at most 1 packet
                    // During startup the timer isn't running, so we can't rely on packetReceived being cleared in tick()
                    packetReceived = false;
                }

                lqRadio1.add();
                antenna = 0;
                getRFlinkInfo();

            } else {
                // failed CRC check
                crc1Counter++;
                #ifdef PRINT_RX_SCOREBOARD
                lastPacketCrcError = true;
                #endif
            }
            
            // start the next receive
            if (connectionState != connected || !isTelemetryFrame()) {
                radio1.RXnb();  // includes clearing the irqs
            } else {
                radio1.ClearIrqStatus(SX1280_IRQ_RX_DONE);
            }

            break;

        case 2:

            r2LastIRQms = millis();

            radio2->readRXData(); // get the data from the radio chip
            if (hasValidCRC((uint8_t*)radio2->RXdataBuffer))
            {
                if (!packetReceived)
                {
                    ProcessRFPacket((uint8_t*)radio2->RXdataBuffer, tPacketR2); 
                    totalRX2Events++;
                    packetReceived = true;
                    HandleFHSS();
                    lqEffective.add();                            
                    #if defined(PRINT_RX_SCOREBOARD)
                    // printf("2");
                    std::cout << '2';
                    #endif
                } else {
                    // Skip at most 1 packet
                    packetReceived = false;
                }

                lqRadio2.add();
                antenna = 1;
                getRFlinkInfo();

            } else {
                // failed CRC check
                crc2Counter++;
                #ifdef PRINT_RX_SCOREBOARD
                lastPacketCrcError = true;
                #endif
            }
            
            if (connectionState != connected || !isTelemetryFrame()) {
                radio2->RXnb();  // includes clearing the irqs
            } else {
                radio2->ClearIrqStatus(SX1280_IRQ_RX_DONE);
            }

            break;

        default:
            printf("unexpected radioID %u\n", radioID);
    }

    // check for variaton in the intervals between sending packets
    // static unsigned long tRcvLast = 0;
    // unsigned long delta = tRcv - tRcvLast;
    // int32_t avgDelta = LPF_rcvInterval.update(delta);
    // int32_t variance = delta - avgDelta;
    // if (variance < 0) variance = -variance;
    // if (variance > 10) {
    //     printf("fhhsMod %u delta %lu avg %ld, variance %ld\n", NonceRX % ExpressLRS_currAirRate_Modparams->FHSShopInterval, delta, avgDelta, variance);
    // }
    // tRcvLast = tRcv;
}




/** Handle received packets
 * 
 * 
 * Instead of passing the radioID via the queue, use an eventDescriptor instead?
 * 
 * Each item on the queue represents an spi transaction, so if an activity requires multiple
 * spi calls (without something else jumping in between) then that should be represented as a single
 * queue message.
 * 
 * Events that will need to be handled:
 *   packet received (from either modem)
 *   set frequency
 *   start receive (for either or both modems)
 *   start tx
 * 
 */
static void ICACHE_RAM_ATTR rx_task(void* arg)
{
    // uint32_t radioID;
    SpiTaskInfo taskInfo;
    for(;;) {
        if(xQueueReceive(rx_evt_queue, &taskInfo, portMAX_DELAY))
        {
            switch (taskInfo.id)
            {
                case SpiTaskID::RxDone:
                    doReceive(taskInfo.radioID);
                    break;

                // case SpiTaskID::SetStandby: // not used, remove?
                //     switch(taskInfo.radioID)
                //     {
                //         case 1:
                //             radio1.SetMode(SX1280_MODE_FS);
                //             break;
                //         case 2:
                //             radio2->SetMode(SX1280_MODE_FS);
                //             break;
                //         default:
                //             printf("unexpected radioID for standby %d\n", taskInfo.radioID);
                //     }
                //     break;

                case SpiTaskID::StartTx:
                    // send a packet which has already been placed in the radio instance's txbuffer
                    switch(taskInfo.radioID)
                    {
                        case 1:
                            // turn off the 'other' radio
                            radio2->SetMode(SX1280_MODE_FS);
                            // start the transmission
                            radio1.TXnb(radio1.TXdataBuffer, 8); // Currently only used to send telem packets which are fixed at 8 bytes
                            break;
                        case 2:
                            // turn off the 'other' radio
                            radio1.SetMode(SX1280_MODE_FS);
                            // start the transmission
                            radio2->TXnb(radio2->TXdataBuffer, 8); // Currently only used to send telem packets which are fixed at 8 bytes
                            break;
                        default:
                            printf("unexpected radioID for StartTx %d\n", taskInfo.radioID);
                    }
                    break;

                case SpiTaskID::SetFrequency:
                    switch(taskInfo.radioID)
                    {
                        case 0: // change on both radios
                            radio1.SetFrequency(taskInfo.frequency);
                            radio2->SetFrequency(taskInfo.frequency);
                            break;
                        case 1:
                            radio1.SetFrequency(taskInfo.frequency);
                            break;
                        case 2:
                            radio2->SetFrequency(taskInfo.frequency);
                            break;
                        default:
                            printf("unexpected radioID for StartRx %d\n", taskInfo.radioID);
                    }
                    break;

                case SpiTaskID::StartRx:
                    switch(taskInfo.radioID)
                    {
                        case 0: // start rx on both radios
                            radio1.RXnb();
                            radio2->RXnb();
                            break;
                        case 1:
                            radio1.RXnb();
                            break;
                        case 2:
                            radio2->RXnb();
                            break;
                        default:
                            printf("unexpected radioID for StartRx %d\n", taskInfo.radioID);
                    }
                    break;

                case SpiTaskID::ClearIRQs:
                    switch(taskInfo.radioID)
                    {
                        case 1:
                            radio1.ClearIrqStatus(taskInfo.irqMask);
                            break;
                        case 2:
                            radio2->ClearIrqStatus(taskInfo.irqMask);
                            break;
                        default:
                            printf("unexpected radioID for ClearIRQs %d\n", taskInfo.radioID);
                    }
                    break;


                default:
                    printf("unhandled SpiTaskID %u\n", taskInfo.id);
            }
        }
    }
}


/** dio1 interrupt when packet is received or preamble detected
 */
static void IRAM_ATTR dio1_isr_handler(void* arg)
{
    static const SpiTaskInfo taskInfo = {SpiTaskID::RxDone, 1, 0};

    BaseType_t taskWoken = 0;
    // const uint32_t radioID = 1;
    // store the packet time here for use in PFD to reduce jitter on CRC failover
    tPacketR1 = esp_timer_get_time();

    xQueueSendFromISR(rx_evt_queue, &taskInfo, &taskWoken);
    if (taskWoken) portYIELD_FROM_ISR();
}

/** dio1 interrupt when packet is received or preamble detected
*/
static void IRAM_ATTR r2_dio1_isr_handler(void* arg)
{
    static const SpiTaskInfo taskInfo = {SpiTaskID::RxDone, 2, 0};

    BaseType_t taskWoken = 0;
    // const uint32_t radioID = 2;
    // store the packet time here for use in PFD to reduce jitter on CRC failover
    tPacketR2 = esp_timer_get_time();

    // send to the same queue as radio1
    xQueueSendFromISR(rx_evt_queue, &taskInfo, &taskWoken);
    if (taskWoken) portYIELD_FROM_ISR();
}


static void IRAM_ATTR dio2_isr_handler(void* arg)
{
    BaseType_t taskWoken = 0;

    xQueueGiveFromISR(tx_evt_queue, &taskWoken); 

    if (taskWoken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR r2_dio2_isr_handler(void* arg)
{
    BaseType_t taskWoken = 0;

    xQueueGiveFromISR(tx2_evt_queue, &taskWoken); 

    if (taskWoken) portYIELD_FROM_ISR();
}


void setRadioParams(uint8_t index, RadioSelection targetRadio)
{
    bool invertIQ = false;

    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);

    switch(targetRadio)
    {
        case RadioSelection::first:
            radio1.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            break;
        case RadioSelection::second:
            radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            break;
        case RadioSelection::both:
            radio1.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            break;
    }
}

/** Update the radio and timer for the specified air rate
*   This version is for a live update without stopping timers or breaking the fhss
*/
void liveUpdateRFLinkRate(uint8_t index)
{
    bool invertIQ = false;

#if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
    invertIQ = (bool)(UID[5] & 0x01);
    printf("invertIQ %d\n", invertIQ);
#endif

    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    rfModeDivisor = ModParams->interval / 1000;

    // uint8_t newIndex = ((((timerNonce + 0) / (2 * rfModeDivisor))) / ExpressLRS_currAirRate_Modparams->FHSShopInterval);

    // uint8_t currentIndex = FHSSgetCurrIndex();
    // printf("fhss current %u, new %u\n", currentIndex, newIndex);
    // FHSSsetCurrIndex(newIndex);

    // uint32_t freq = FHSSgetCurrFreq();

    #ifdef USE_FLRC
    if (index == 0)
    { // special case FLRC for testing
        radio1.ConfigFLRC(GetInitialFreq());
    }
    else
    #endif
    {
        radio1.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
        #ifdef USE_SECOND_RADIO
        radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
        #endif
    }

    // HwTimer::setInterval(ModParams->interval);

    // Set timeout based on the interval
    radio1.setRxTimeout(ModParams->interval * 5 / 2);
    radio2->setRxTimeout(ModParams->interval * 5 / 2);

    radio1.RXnb();
    radio2->RXnb();

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    // pfdDebugCounter = 10; // trigger pfd debug
}



// Update the radio and timer for the specified air rate
void SetRFLinkRate(uint8_t index)
{
    bool invertIQ = false;

#if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
    invertIQ = (bool)(UID[5] & 0x01);
    printf("invertIQ %d\n", invertIQ);
#endif

    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

#ifdef USE_FLRC
    if (index == 0)
    { // special case FLRC for testing
        radio1.ConfigFLRC(GetInitialFreq());
    }
    else
#endif
    {
        radio1.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
        #ifdef USE_SECOND_RADIO
        radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
        #endif
    }

    // HwTimer::setInterval(ModParams->interval);
    rfModeDivisor = ModParams->interval / 1000;

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    isRXconnected = false;

    // if (UpdateRFparamReq)
    //    UpdateRFparamReq = false;
}


// Why isn't this part of PFD?
void ICACHE_RAM_ATTR updatePhaseLock()
{
    if (connectionState != disconnected)
    {
        RawOffset = pfdLoop.calcResult();
        pfdLoop.reset();
        // RawOffset = pfdLoop.getResult();

        const int32_t max = (int32_t)ExpressLRS_currAirRate_Modparams->interval * 2;
        const int32_t min = -max;

        // ignore values that are out of reasonable range
        // for reasons unknown, combining these two tests causes the < min clause to always evaluate true
        if (RawOffset > max)
        {
            printf("ign %d, max = %d\n", RawOffset, max);
            return;
        }

        if (RawOffset < min)
        {
            printf("ign %d, min = %d\n", RawOffset, min);
            return;
        }


        Offset = LPF_Offset.update(RawOffset);
        OffsetDx = LPF_OffsetDx.update(RawOffset - prevRawOffset);

        if (Offset > 1000000 || Offset < -1000000) {
            printf("raw %d off %d\n", RawOffset, Offset);
        }


        if (RXtimerState == tim_locked && (lqRadio1.currentIsSet() || lqRadio2.currentIsSet()))
        {
            if (NonceRX % 8 == 0) //limit rate of freq offset adjustment slightly
            {
                if (Offset > 0)
                {
                    HwTimer::incFreqOffset();
                }
                else if (Offset < 0)
                {
                    HwTimer::decFreqOffset();
                }
            }
        }

        if (connectionState != connected)
        {
            HwTimer::setPhaseShift(RawOffset >> 1);
        } else {
            HwTimer::setPhaseShift(Offset >> 2);
        }

        prevOffset = Offset;
        prevRawOffset = RawOffset;

        if (pfdDebugCounter > 0) {
            printf("Offset: %d\n", Offset);
            pfdDebugCounter--;
        }
    }

#ifndef DEBUG_SUPPRESS
    // printf("%ld %ld %ld %ld %u\n", Offset, RawOffset, OffsetDx, 0, uplinkLQ);
    // Serial.print(Offset);
    // Serial.print(":");
    // Serial.print(RawOffset);
    // Serial.print(":");
    // Serial.print(OffsetDx);
    // Serial.print(":");
    // Serial.print(HwTimer::getFreqOffset());
    // Serial.print(":");
    // Serial.println(uplinkLQ);
#endif
}


/** Aligned with (approximately) the midpoint of the radio interval (not necessarily the midpoint of the radio signal since duty cycle < 1)
 * 
 *  Assume that the radio modem is busy doing either rx or tx during this call, so we can only do non-radio related housekeeping in here.
 */
void ICACHE_RAM_ATTR tick()
{
    tickStartMs = millis();

    // printf("tickIn\n");
    // static unsigned long last = 0;
    // unsigned long now = micros();

    // if (last != 0) {
    //     printf("tick %lu\n", (now-last));
    // }
    // last = now;

    // timerNonce++; // moved to tickTock

    // uint32_t halfStep = rfModeDivisor/2;
    // // if (halfStep>0) halfStep--;

    // if (((timerNonce + halfStep) % rfModeDivisor) == 0)
    {
        // #ifdef DEBUG_PIN
        // gpio_set_level(DEBUG_PIN, 1);
        // #endif

        updatePhaseLock();
        NonceRX = timerNonce / (2 * rfModeDivisor);

        // reset the flag for dual radio diversity reception
        packetReceived = false;

        // Save the LQ value before the inc() reduces it by 1
        uplinkLQ = lqEffective.getLQ();
        lpfLq.update(uplinkLQ);

        // Only advance the LQI period counter if we didn't send Telemetry this period
        if (!alreadyTLMresp) {
            lqRadio1.inc();
            lqRadio2.inc();
            lqEffective.inc();
        }

        alreadyTLMresp = false;
        alreadyFHSS = false;
    }
    // printf("tickOut\n");
    tickStartMs = 0;

}

char *tockWhere = (char *)"not set";

/** Aligned with (approximately) the start of the radio signal
 * 
 */
void ICACHE_RAM_ATTR tock()
{
    tockStartMs = millis();

    // printf("tockIn\n");
    // std::cout << "O";

    tockWhere = (char *)"start";

    // if (timerNonce % rfModeDivisor == 0)
    {
        pfdLoop.intEvent(esp_timer_get_time());

        // #ifdef DEBUG_PIN
        // gpio_set_level(DEBUG_PIN, 0);
        // #endif

        // updateDiversity();
        tockWhere = (char *)"FHSS";
        HandleFHSS();
        tockWhere = (char *)"telem";
        HandleSendTelemetryResponse(); // XXX hangs in here

        // This is a hail-Mary check for if the radios haven't been talking and kick off new RXnb calls.
        // But if it's here then it will only active when the timer is running. Maybe we need two levels of hail-Mary

        tockWhere = (char *)"HM";


        // privatize the *Last times so they can't change after we assign 'now' and cause false positives
        uint32_t p1 = r1LastIRQms, p2 = r2LastIRQms;
        uint32_t now = millis();
        if ((now - p1) > 40000)
        {
            radio1.RXnb(); // XXX convert to event messages
            radio1Timedout = false;
            std::cout << "HM1";
            r1LastIRQms = now; // stop it spamming
        }

        if ((now - p2) > 40000)
        {
            radio2->RXnb();
            radio2Timedout = false;
            std::cout << "HM2";
            r2LastIRQms = now; // stop it spamming
        }

        tockWhere = (char *)"dTs";

        // deferred receives from rx timeouts
        if (!alreadyTLMresp) // if we're doing telem then the receivers will be restarted after anyway.
        {
            SpiTaskInfo taskInfo;
            taskInfo.id = SpiTaskID::StartRx;

            if (radio1Timedout) {
                taskInfo.radioID = 1;
                xQueueSend(rx_evt_queue, &taskInfo, 1000);

                // radio1.RXnb();
                // radio1Timedout = false;
            }

            if (radio2Timedout) {
                taskInfo.radioID = 2;
                xQueueSend(rx_evt_queue, &taskInfo, 1000);

                // radio2->RXnb();
                // radio2Timedout = false;
            }
        }


        #if defined(PRINT_RX_SCOREBOARD)
        static bool lastPacketWasTelemetry = false; // NB static for perstence across calls
        if (!(lqRadio1.currentIsSet() || lqRadio2.currentIsSet()) && !lastPacketWasTelemetry)
            // printf(lastPacketCrcError ? "X" : "_");
            std::cout << (lastPacketCrcError ? 'X' : '_');

        lastPacketCrcError = false;
        lastPacketWasTelemetry = alreadyTLMresp;
        #endif
        // printf("tockOut\n");

        static unsigned long tDrStart; // NB static for perstence across calls

        tockWhere = (char *)"DR";

        // Dynamic Rate state transitions
        // XXX disabled for testing
        if (false && connectionState == connected) // don't run dynamic rates if we're not connected
        {
            uint8_t currentIndex = ExpressLRS_currAirRate_Modparams->index;
            switch(drState)
            {
                case DynamicRateStates::Normal:
                    if ((lpfLq.SmoothDataINT < 90) && (currentIndex < (RATE_MAX-1)))
                    {
                        printf("shiftDown start\n");
                        setRadioParams(currentIndex+1, RadioSelection::second); // XXX choose the radio with the weakest signal
                        radio2->RXnb();
                        drState = DynamicRateStates::ShiftDown;
                        tDrStart = millis();
                    } // else check for shift up case
                    break;
                case DynamicRateStates::ShiftDown:
                    if (lqRadio1.getLQ() < lqRadio2.getLQ())
                    {
                        printf("shiftDown complete\n");
                        SetRFLinkRate(currentIndex+1);
                        radio1.RXnb();
                        radio2->RXnb();
                        drState = DynamicRateStates::Normal;
                    } else if ((millis() - tDrStart) > 500) {
                        // change didn't happen, reset the sniffer back to the current mode
                        printf("shiftDown timedout\n");
                        setRadioParams(currentIndex, RadioSelection::second);
                        radio2->RXnb();
                        drState = DynamicRateStates::Normal;
                    }

                    break;
                default:
                    printf("unhandled case %d\n", drState);
            }
        } // if (connected)

    } // if (timerNonce mod divisor is zero)
    tockWhere = (char *)"end";

    tockStartMs = 0;
}

/** 
 * Ok, so this is a silly way to implement this, but it's a quick and easy way for testing
 * 
 * This routine will be used for both tick and tock callbacks from the hwTimer. It increments
 * a counter and calls either tick or tock according to the value and the mode divisor. The idea
 * is to be able to get tick on the midpoint, as opposed to the previous impl where tick was always
 * 1 timer period off centre.
 * 
 */
void ICACHE_RAM_ATTR tickTock()
{
    // std::cout << 'O';

    timerNonce++;
    // if (timerNonce == 0) std::cout << 'W';

    if (timerNonce % (2 * rfModeDivisor) == 0)
    {
        tock();
    } else if ((timerNonce + rfModeDivisor) % (2 * rfModeDivisor) == 0) {
        tick();
    }

}


/** docs?
 */
void lostConnection()
{
    // TODO pretty sure FreqCorrection isn't used
    printf("lost conn fc=%d fo=%d\n", FreqCorrection, HwTimer::getFreqOffset());
    // printf(" fo="); Serial.println(HwTimer::getFreqOffset, DEC);

    RFmodeCycleMultiplier = 1;
    connectionStatePrev = connectionState;
    connectionState = disconnected; //set lost connection
    RXtimerState = tim_disconnected;
    HwTimer::resetFreqOffset();
    FreqCorrection = 0;

    Offset = 0;
    OffsetDx = 0;
    RawOffset = 0;
    prevOffset = 0;
    GotConnectionMillis = 0;
    uplinkLQ = 0;
    lqRadio1.reset();
    lqRadio2.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    alreadyTLMresp = false;
    alreadyFHSS = false;
    // LED = false; // Make first LED cycle turn it on

    // XXX is this guaranteed to complete? 
    // At the moment it can hang. Need to make sure that tock() runs at a higher priority than whatever calls this function.
    // This only gets called from the event loop which should be running at min priority, so how does this hang?
    // printf("waiting for timer sync\n");fflush(stdout);
    // while(micros() - pfdLoop.getIntEventTime() > 250); // time it just after the tock()
    HwTimer::stop();
    // printf("timer sync done\n");

    // XXX These two are potential SPI collisions if there might still be activity on the rx/tx tasks
    SetRFLinkRate(ExpressLRS_nextAirRateIndex); // also sets to initialFreq 

    // set timeouts to cover the entire time we're going to wait for sync in this mode
    radio1.setRxTimeout(ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval * 1000);
    radio2->setRxTimeout(ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval * 1000);

    radio1.RXnb();
    #ifdef USE_SECOND_RADIO
    radio2->RXnb();
    #endif

    #ifdef LED_STATUS_INDEX
    setLedColour(LED_STATUS_INDEX, LED_BRIGHTNESS, 0, 0);
    #endif


// #ifdef GPIO_PIN_LED_GREEN
//     digitalWrite(GPIO_PIN_LED_GREEN, LOW ^ GPIO_LED_GREEN_INVERTED);
// #endif

// #ifdef GPIO_PIN_LED_RED
//     digitalWrite(GPIO_PIN_LED_RED, LOW ^ GPIO_LED_RED_INVERTED);
// #endif

// #ifdef GPIO_PIN_LED
//     digitalWrite(GPIO_PIN_LED, LOW ^ GPIO_LED_RED_INVERTED); // turn off led
// #endif
}

/**
 * doc?
 */
void gotConnection()
{
    if (connectionState == connected)
    {
        return; // Already connected
    }

#ifdef LOCK_ON_FIRST_CONNECTION
    LockRFmode = true;
#endif

    connectionStatePrev = connectionState;
    connectionState = connected; //we got a packet, therefore no lost connection
    // disableWebServer = true;
    RXtimerState = tim_tentative;
    GotConnectionMillis = millis();

    printf("got conn\n");

    #ifdef LED_STATUS_INDEX
    setLedColour(LED_STATUS_INDEX, 0, LED_BRIGHTNESS, 0);
    #endif


#ifdef GPIO_PIN_LED_GREEN
    digitalWrite(GPIO_PIN_LED_GREEN, HIGH ^ GPIO_LED_GREEN_INVERTED);
#endif

#ifdef GPIO_PIN_LED_RED
    digitalWrite(GPIO_PIN_LED_RED, HIGH ^ GPIO_LED_RED_INVERTED);
#endif

#ifdef GPIO_PIN_LED
    digitalWrite(GPIO_PIN_LED, HIGH ^ GPIO_LED_RED_INVERTED); // turn on led
#endif
}

static uint8_t minLqForChaos()
{
    // Determine the most number of CRC-passing packets we could receive on
    // a single channel out of 100 packets that fill the LQcalc span.
    // The LQ must be GREATER THAN this value, not >=
    // The amount of time we coexist on the same channel is
    // 100 divided by the total number of packets in a FHSS loop (rounded up)
    // and there would be 4x packets received each time it passes by so
    // FHSShopInterval * ceil(100 / FHSShopInterval * NR_FHSS_ENTRIES) or
    // FHSShopInterval * trunc((100 + (FHSShopInterval * NR_FHSS_ENTRIES) - 1) / (FHSShopInterval * NR_FHSS_ENTRIES))
    // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
    uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    return interval * ((interval * NR_FHSS_ENTRIES + 99) / (interval * NR_FHSS_ENTRIES));
}

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
static void cycleRfMode()
{
    // statics for persistence accross calls
    static uint32_t LEDLastUpdate = 0; 
    static bool LED = false; 

    const uint32_t LED_INTERVAL_DISCONNECTED = 1000;

    if (connectionState != disconnected) // || InBindingMode || webUpdateMode)
        return;

    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if (LockRFmode == false && (millis() - RFmodeLastCycled) > (ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval * RFmodeCycleMultiplier))
    {
        printf("cycling\n");
        unsigned long now = millis();
        RFmodeLastCycled = now;
        lastSyncPacket = now;           // reset this variable
        SetRFLinkRate(scanIndex % RATE_MAX); // switch between rates
        linkStatstoFCLastSent = now;
        lqRadio1.reset();
        lqRadio2.reset();
        printf("%u\n", ExpressLRS_currAirRate_Modparams->interval);
        scanIndex++;
        getRFlinkInfo();
        crsf.sendLinkStatisticsToFC();
        // delay(100);
        // crsf.sendLinkStatisticsToFC(); // need to send twice, not sure why, seems like a BF bug?


        radio1.RXnb();
        #ifdef USE_SECOND_RADIO
        radio2->RXnb();
        #endif

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode

    // Always blink the LED at a steady rate when not connected, independent of the cycle status
    if (millis() - LEDLastUpdate > LED_INTERVAL_DISCONNECTED)
    {
        #ifdef GPIO_PIN_LED
            digitalWrite(GPIO_PIN_LED, LED ^ GPIO_LED_RED_INVERTED);
        #elif GPIO_PIN_LED_GREEN
            digitalWrite(GPIO_PIN_LED_GREEN, LED ^ GPIO_LED_GREEN_INVERTED);
        #endif

        #ifdef LED_STATUS_INDEX
        setLedColour(LED_STATUS_INDEX, LED ? LED_BRIGHTNESS : 0, 0, 0);
        #endif

        LED = !LED;
        LEDLastUpdate = millis();
    } // if cycle LED


}

void setupPWM()
{
    #ifdef USE_PWM6
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.duty_resolution  = LEDC_TIMER_14_BIT;
    ledc_timer.freq_hz          = PWM_FREQ;  // Set output frequency in Hz
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel;
    memset(&ledc_channel, 0, sizeof(ledc_channel));

    for(int i=0; i<sizeof(PwmPins); i++)
    {
        if (PwmPins[i] != -1) {
            ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
            ledc_channel.channel        = (ledc_channel_t) i;
            ledc_channel.timer_sel      = LEDC_TIMER_0;
            ledc_channel.intr_type      = LEDC_INTR_DISABLE;
            ledc_channel.gpio_num       = PwmPins[i];
            ledc_channel.duty           = PWM_RANGE/2 + PWM_MIN; // Set duty to mid position
            ledc_channel.hpoint         = 0;

            ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        }
    }
    #endif // USE_PWM6
}

void initLeds()
{
    // const uint32_t brightness = 64;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED2812_PIN, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;
    config.tx_config.loop_count = 0;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    const uint8_t N_LEDS = 3;
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(N_LEDS, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        printf("install WS2812 driver failed\n");
        return;
    }

    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    // gratuitous startup test

    // ramp red
    for(int i=0; i<256; i++)
    {
        strip->set_pixel(strip, 0, i, 0, 0);
        strip->set_pixel(strip, 1, i, 0, 0);
        strip->set_pixel(strip, 2, i, 0, 0);

        strip->refresh(strip, 1);
        delay(1);
    }

    // transition to green
    for(int i=0; i<256; i++)
    {
        strip->set_pixel(strip, 0, 255-i, i, 0);
        strip->set_pixel(strip, 1, 255-i, i, 0);
        strip->set_pixel(strip, 2, 255-i, i, 0);
        strip->refresh(strip, 1);
        delay(1);
    }

    // transition to blue
    for(int i=0; i<256; i++)
    {
        strip->set_pixel(strip, 0, 0, 255-i, i);
        strip->set_pixel(strip, 1, 0, 255-i, i);
        strip->set_pixel(strip, 2, 0, 255-i, i);
        strip->refresh(strip, 1);
        delay(1);
    }

    // ramp down
    for(int i=0; i<256; i++)
    {
        strip->set_pixel(strip, 0, 0, 0, 255-i);
        strip->set_pixel(strip, 1, 0, 0, 255-i);
        strip->set_pixel(strip, 2, 0, 0, 255-i);
        strip->refresh(strip, 1);
        delay(2);
    }

}

void setLedColour(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    strip->set_pixel(strip, index, red, green, blue);
    strip->refresh(strip, 100);
}

extern "C"
{

extern char * wcWhere;


void app_main()
{
    // uart_set_baudrate(UART_NUM_0, 921600);
    uart_set_baudrate(UART_NUM_0, 2000000);
    printf("ESP32-C3 FreeRTOS RX\n");

    #ifdef DEBUG_PIN

    // Set the pin as an output
    gpio_reset_pin(DEBUG_PIN);
    gpio_set_direction(DEBUG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_PIN, 0);

    #endif // DEBUG_PIN


    setupPWM(); // does nothing if pwm not being used

    initLeds();

    // The HwTimer is using priority 15 - should it be higher or lower than the rx task?
    HwTimer::init();
    HwTimer::stop();
    HwTimer::setInterval(1000); // timer always runs at 1kHz (actually 2kHz since it calls both tick and tock)

    // HwTimer::setCallbackTick(tick);
    // HwTimer::setCallbackTock(tock);
    HwTimer::setCallbackTick(tickTock);
    HwTimer::setCallbackTock(tickTock);

    radio2 = new SX1280Driver(RADIO2_NSS_PIN);

    radio1.currFreq = GetInitialFreq();
    radio2->currFreq = GetInitialFreq();

    const bool usePreambleDetect = false; // this didn't work out, but maybe worth another look someday?

    // XXX retry if either radio doesn't initialise properly

    if (radio1.Begin(usePreambleDetect) == 0) {
        // set led green
        setLedColour(LED_RADIO1_INDEX, 0, LED_BRIGHTNESS, 0);
    } else {
        // set led red
        setLedColour(LED_RADIO1_INDEX, LED_BRIGHTNESS, 0, 0);
    }

    if (radio2->Begin(usePreambleDetect) == 0) {
        // set led green
        setLedColour(LED_RADIO2_INDEX, 0, LED_BRIGHTNESS, 0);
    } else {
        // set led red
        setLedColour(LED_RADIO2_INDEX, LED_BRIGHTNESS, 0, 0);
    }

    radio1.SetOutputPower(DISARM_POWER);
    radio2->SetOutputPower(DISARM_POWER);

    //create a queue to handle gpio event from isr
    rx_evt_queue = xQueueCreate(10, sizeof(SpiTaskInfo));
    // rx2_evt_queue = xQueueCreate(10, 0);
    tx_evt_queue = xQueueCreate(10, 0);
    tx2_evt_queue = xQueueCreate(10, 0);
    //start tasks
    xTaskCreate(rx_task, "rx_task", 2048, NULL, 16, NULL); // Task priority 1=min, max is ??? Default main task is pri 1
    // xTaskCreate(rx2_task, "rx2_task", 2048, NULL, 17, NULL);
    xTaskCreate(tx_task, "tx_task", 2048, NULL,  5, NULL);
    xTaskCreate(tx2_task, "tx2_task", 2048, NULL,  5, NULL);


    // need to attach an ISR handlers to DIO pins

    gpio_set_intr_type(RADIO_DIO1_PIN, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(RADIO2_DIO1_PIN, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(RADIO_DIO2_PIN, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(RADIO2_DIO2_PIN, GPIO_INTR_POSEDGE);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    //hook isr handler for specific gpio pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO1_PIN, dio1_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO2_DIO1_PIN, r2_dio1_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO2_PIN, dio2_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO2_DIO2_PIN, r2_dio2_isr_handler, NULL));

    uint8_t linkRateIndex = 0;
    SetRFLinkRate(linkRateIndex);

    // delay(50);
    // printf("setting FS\n\r");
    // radio.SetMode(SX1262_MODE_FS);
    // delay(50);
    // radio.GetStatus();

    // XXX What's a reasonable timeout?
    radio1.setRxTimeout(20000);
    radio2->setRxTimeout(20000);

    FHSSrandomiseFHSSsequence();

    radio1.RXnb();
    radio2->RXnb();
    crsf.Begin();

    delay(500);
    strip->clear(strip, 10);

    unsigned long lastDebug = 0;
    unsigned long lastLEDUpdate = 0;
    unsigned long prevTime = 0;

    uint32_t lastRX1Events = 0, lastRX2Events = 0;

    // loop
    while(true) {
        // things that don't have to happen very quickly
        //delay(100);   // idle watchdog has to be disabled
        unsigned long now = millis();
        if (now - prevTime > 200) {
            printf("big gap %u to %u\n", prevTime, now);
        }
        prevTime = now;

        if (lastLEDUpdate + 100 < now)
        {
            lastLEDUpdate = now;

            uint32_t delta1 = totalRX1Events - lastRX1Events;
            uint32_t delta2 = totalRX2Events - lastRX2Events;

            lastRX1Events = totalRX1Events;
            lastRX2Events = totalRX2Events;

            uint32_t totalPackets = delta1 + delta2 + timeout1Counter + timeout2Counter + crc1Counter + crc2Counter + 1;

            uint32_t brightness1 = delta1 * LED_BRIGHTNESS / totalPackets;
            uint32_t brightness2 = delta2 * LED_BRIGHTNESS / totalPackets;

            uint32_t green1 = lqRadio1.getLQ() * brightness1 / 100;
            uint32_t green2 = lqRadio2.getLQ() * brightness2 / 100;

            // uint32_t red1 = (crc1Counter * brightness1 / totalPackets) + (brightness1 - green1);
            // uint32_t red2 = (crc2Counter * brightness2 / totalPackets) + (brightness2 - green2);
            uint32_t red1 = (brightness1 - green1);
            uint32_t red2 = (brightness2 - green2);

            uint32_t blue1 = timeout1Counter * LED_BRIGHTNESS / totalPackets;
            uint32_t blue2 = timeout2Counter * LED_BRIGHTNESS / totalPackets;

            strip->set_pixel(strip, LED_RADIO1_INDEX, red1, green1, blue1);
            strip->set_pixel(strip, LED_RADIO2_INDEX, red2, green2, blue2);
            strip->refresh(strip, 10);

            // printf("delta %u bright %u green %u\n", delta1, brightness1, green1);
        }

        if (lastDebug + 1000 < now)
        {
            uint32_t elapsedT = now - lastDebug;
            lastDebug = now;
            #if defined(PRINT_RX_SCOREBOARD)
            // printf("\n");
            std::cout << '\n';
            #endif

            // check for hung callbacks
            if (tickStartMs && ((now - tickStartMs) > 100))
            {
                std::cout << "XXX tick hung\n";
            }
            if (tockStartMs && ((now - tockStartMs) > 100))
            {
                printf("XXX tock hung at tock %s, telem %s, wc %s\n", tockWhere, telemWhere, wcWhere);
            }

            printf("rx1Events %u, rx2Events %u, LQ1 %3u, LQ2 %3u, LQC %3u, crc1Errors/s %2u, crc2Errors/s %2u ", 
                    totalRX1Events, totalRX2Events, lqRadio1.getLQ(), lqRadio2.getLQ(), lqEffective.getLQ(),
                    crc1Counter*1000/elapsedT, crc2Counter*1000/elapsedT);

            int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
            int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;
            printf("rss1 %4d, rssi2 %4d ", rssiDBM0, rssiDBM1);
            // printf("tN %u ", timerNonce);
            printf("Offset %4d DX %4d freqOffset %d\n", Offset, OffsetDx, HwTimer::getFreqOffset());

            // totalPackets = 0;
            timeout1Counter = 0;
            crc1Counter = 0;
            timeout2Counter = 0;
            crc2Counter = 0;

            if (connectionState == tentative)
            {
                printf("tentative:");
                if (abs(OffsetDx) > 10) printf(" offsetDx %d", OffsetDx);
                if (Offset >= 100) printf(" offset %d", Offset);
                if (uplinkLQ <= minLqForChaos()) printf(" lq %u", uplinkLQ);
                printf("\n");
            }
        }


        // This is potentially dangerous if rx may still be in progress
        // Fail a tentative connection if we don't get a sync packet in RFmodeCycleAddtionalTime
        if (connectionState == tentative)
        {
            const uint32_t localLastSyncPacket = lastSyncPacket;
            const uint32_t tBS = millis();
            if ((tBS - localLastSyncPacket) > ExpressLRS_currAirRate_RFperfParams->RFmodeCycleAddtionalTime)
            {
                printf("Bad sync, aborting\n");
                lostConnection();
                RFmodeLastCycled = now;
                lastSyncPacket = now;
            }
        }

        // check if we lost conn.
        const uint32_t localLastValidPacket = lastValidPacket; // Required to prevent race condition due to LastValidPacket getting updated from ISR
        const uint32_t tLost = millis();
        if ((connectionState == connected) && ((int32_t)ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval < (int32_t)(tLost - localLastValidPacket)))
        {
            printf("loop: connection lost, localLVP %lu, tLost %lu, LQ %u\n", localLastValidPacket, tLost, uplinkLQ);
            lostConnection();
        }

        // detects when we are connected
        if ((connectionState == tentative) && (abs(OffsetDx) <= 10) && (Offset < 100) && (uplinkLQ > minLqForChaos()))
        {
            printf("loop: connected\n");
            gotConnection();
        }

        // If the link is looking stable we can move the timer to locked mode (and start doing loop freq compensation)
        if ((RXtimerState == tim_tentative) && ((millis() - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(OffsetDx) <= 5))
        {
            // #ifndef DEBUG_SUPPRESS
            printf("Timer Locked\n");
            // #endif
            RXtimerState = tim_locked;
        }

        if (RATE_MAX > 1) cycleRfMode();

    } // while true

}

} // extern "C"