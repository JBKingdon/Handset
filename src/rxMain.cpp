/**
 * Initial test code for C3 based receivers and transmitters
 * 
 * TODO:
 *   Fix SPI collisions
 *   Add code to detect if a modem stops talking to us. There should always be one of txdone, rxdone or timeout in every frame
 *      add code to try harder when the modem stops talking
 *   Figure out why the radios stop talking
 */

const bool TRANSMITTER = false;

#define INITIAL_LINK_RATE_INDEX 0

// recompile both rx & tx when changing this:
// XXX doesn't work, needs updating after change to conventional telemetry
// #define DISABLE_TELEM

// only need to recompile tx:
// #define DISABLE_2G4

// only need to recompile rx:
// #define PFD_CALIBRATION

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
#include "SX1280Driver.h"
#include "HwTimer.h"
#include "CRSF.h"
#include "PFD.h"

#include "LowPassFilter.h"
#include "LQCALC.h"
#include "OTA.h"

#include "driver/rmt.h"
#include "led_strip.h"

// #define DEBUG_SUPPRESS
// #define PRINT_RX_SCOREBOARD 1

#define LED_BRIGHTNESS 32

#define SEND_LINK_STATS_TO_FC_INTERVAL 50    // normal rate

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

SX1262Driver * radio1 = NULL;


#elif defined (RADIO_E28_12) || defined(RADIO_E28_20) || defined(RADIO_E28_27)


SX1280Driver * radio1 = NULL;

#else
error("must define a radio module")

#endif // RADIO_*

#ifdef USE_SECOND_RADIO

SX1280Driver * radio2 = NULL;

#endif

enum DynamicRateStates
{
    Normal,
    ShiftUp,
    ShiftDown    
};

enum RadioSelection
{
    both,
    first,
    second
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
        uint32_t unused;
    };

} SpiTaskInfo;

static DynamicRateStates drState = DynamicRateStates::Normal;
// static unsigned long tDrStart;
// static uint8_t dynamicRateTargetIndex;

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

static uint32_t r1LastIRQms = 0;
static uint32_t r2LastIRQms = 0;

static xQueueHandle rx_evt_queue = NULL;
static xQueueHandle rx2G4_evt_queue = NULL;
static xQueueHandle tx_evt_queue = NULL;
static xQueueHandle tx2_evt_queue = NULL;

static volatile uint32_t nonceRX915 = 0;    // XXX check if these need to be volatile - it seems unlikely
static volatile uint32_t nonceRX2G4 = 0;
static uint32_t nonceTX915 = 0;
static uint32_t nonceTX2G4 = 0;

static uint16_t timerNonce = 0;   // incremented every time the hwtimer fires, and scaled by rfModeDivisor to get the nonce

// XXX divisor would be more efficient if specified as a bit shift
uint32_t rfModeDivisor915 = 1; // Since the hwTimer now runs at a fixed 1kHz we need a factor to scale to the packet rate
uint32_t rfModeDivisor2G4 = 1; // for each radio

static bool alreadyFHSS915 = false;
static bool alreadyFHSS2G4 = false;
static bool alreadyTLMresp = false;
static volatile bool packetReceived = false;

static bool radio1Timedout = false;
static bool radio2Timedout = false;

// uint32_t beginProcessing;
// uint32_t doneProcessing;

volatile uint32_t tPacketR1 = 0;
volatile uint32_t tPacketR2 = 0;
volatile uint32_t lastValidPacket = 0;
volatile uint32_t lastSyncPacket = 0;

uint32_t linkStatstoFCLastSent = 0;

uint32_t pfdDebugCounter = 0; // when greater than 0 updatePFD will print the offset

int32_t RawOffset915;
int32_t prevRawOffset915;
int32_t Offset;
int32_t OffsetDx;
int32_t prevOffset;

int32_t RawOffset2G4;
int32_t prevRawOffset2G4;

RXtimerState_e RXtimerState = tim_disconnected;
uint32_t GotConnectionMillis = 0;
const uint32_t ConsiderConnGoodMillis = 1000; // minimum time before we can consider a connection to be 'good'

// XXX rename to *915
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);

LPF LPF_Offset2G4(2);
LPF LPF_OffsetDx2G4(4);

// uint32_t cycleInterval; // in ms
uint32_t RFmodeLastCycled = 0;
#define RFmodeCycleMultiplierSlow 10
uint8_t RFmodeCycleMultiplier;
bool LockRFmode = false;

// Track how long the timer callbacks have been running. 0 = not running
static uint32_t tickStartMs = 0;
static uint32_t tockStartMs = 0;

static DB915Telem_t telemData;

// LPF LPF_UplinkRSSI(5);
LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
LPF LPF_UplinkRSSI1(5);

LPF LPF_rcvInterval(4);

// filter for LQ for use with dynamic rates
LPF lpfLq(3);

/// LQ Calculation //////////
LQCALC<99> lqRadio1, lqRadio2, lqEffective;
uint8_t uplinkLQ;

uint8_t scanIndex = RATE_DEFAULT; // used when cycling through RF modes

uint8_t antenna = 0;    // which antenna is currently in use

bool isRXconnected = false;

#if defined(PRINT_RX_SCOREBOARD)
static bool lastPacketCrcError;
#endif

#ifdef LED2812_PIN
led_strip_t *strip = nullptr;
#endif

// forward refs
void setLedColour(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
void liveUpdateRFLinkRate(uint8_t index);
void SetRFLinkRate(uint8_t index);



// XXX TODO move this somewhere sensible
void ICACHE_RAM_ATTR delay(uint32_t millis)
{
    if (millis < 30) {
        esp_rom_delay_us(millis*1000);
    } else {
        vTaskDelay(millis / portTICK_PERIOD_MS);
    }
}
/**
 * XXX Need new linkstats format for dual band
 */
void ICACHE_RAM_ATTR getRFlinkInfo()
{

    int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
    int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;

    int8_t t1, t2;
    switch (antenna) {
        case 0:
            t1 = radio1->GetLastPacketRSSI();
            rssiDBM0 = LPF_UplinkRSSI0.update(t1);
            // printf("update0 %d %d\n", t1, rssiDBM0);
            break;
        case 1:
            #ifdef USE_SECOND_RADIO
            t2 = radio2->GetLastPacketRSSI();
            rssiDBM1 = LPF_UplinkRSSI1.update(t2);
            // printf("update1 %d %d\n", t2, rssiDBM1);
            #endif

            break;
    }

    // int32_t rssiDBM = (antenna == 0) ? rssiDBM0 : rssiDBM1;
    // crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(constrain(rssiDBM, ExpressLRS_currAirRate_RFperfParams->RXsensitivity, -50),
    //                                            ExpressLRS_currAirRate_RFperfParams->RXsensitivity, -50, 0, 1023));
    // crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(uplinkLQ, 0, 100, 0, 1023));

    if (rssiDBM0 > 0) rssiDBM0 = 0;
    if (rssiDBM1 > 0) rssiDBM1 = 0;

    #ifdef USE_ELRS_CRSF_EXTENSIONS
    crsf.LinkStatistics.rssi0 = -rssiDBM0; // negate to match BF
    crsf.LinkStatistics.rssi1 = -rssiDBM1;

    // crsf.LinkStatistics.snr = Radio.LastPacketSNR; // * 10; Swapped out snr for rssi1
    crsf.LinkStatistics.link_quality = uplinkLQ | (antenna << 7); // carry the current antenna info in the top bit of lq

    // crsf.LinkStatistics.rf_Mode = RATE_MAX - ExpressLRS_currAirRate_Modparams->index;
    crsf.LinkStatistics.rf_Mode = RATE_MAX;

    #else // standard crsf link stats packet

    // BetaFlight/iNav expect positive values for -dBm (e.g. -80dBm -> sent as 80)
    crsf.LinkStatistics.uplink_RSSI_1 = -rssiDBM0;
    crsf.LinkStatistics.uplink_RSSI_2 = -rssiDBM1;
    crsf.LinkStatistics.active_antenna = antenna;
    crsf.LinkStatistics.uplink_SNR = radio1->LastPacketSNR;
    crsf.LinkStatistics.uplink_Link_quality = uplinkLQ;
    crsf.LinkStatistics.rf_Mode = (uint8_t)RATE_LAST - (uint8_t)ExpressLRS_currAirRate_Modparams->enum_rate;
    //Serial.println(crsf.LinkStatistics.uplink_RSSI_1);

    #endif // elrs vs crsf format
}

void TentativeConnection()
{
    pfdLoop.reset915();
    pfdLoop.reset2G4();
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

    LPF_rcvInterval.init(airRateConfig915.interval);

    // ExpressLRS_nextAirRateIndex = ExpressLRS_currAirRate_Modparams->index;

    // set timeout based on interval?

    radio1->setRxTimeout(20000);

    #ifdef USE_SECOND_RADIO
    radio2->setRxTimeout(20000);
    #endif

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

#ifndef USE_DB_PACKETS

/** Do CRC check for pre-DB packet shape
 * 
 */
bool hasValidCRC(uint8_t *rxBuffer, const uint8_t packetLength)
{
    const uint8_t type = rxBuffer[0] & 0b11;
    const uint16_t inCRC = ( ( (uint16_t)(rxBuffer[0] & 0b11111100) ) << 6 ) | rxBuffer[packetLength-1];

    rxBuffer[0] = type; // NB this destroys the original CRC data, so you can only call this function once per buffer

    uint16_t calculatedCRC = ota_crc.calc(rxBuffer, packetLength-1);

    return (inCRC == calculatedCRC);
}

#endif // not USE_DB_PACKETS

/** Do CRC check for DB 915 packets
 * 
 */
bool hasValidCRC915DB(DB915Packet_t *packet)
{
    uint8_t * asBytes = (uint8_t *)packet;

    const uint16_t inCRC = packet->crc;

    packet->crc = 0;

    uint16_t calculatedCRC = ota_crc.calc(asBytes, OTA_PACKET_LENGTH_915);

    packet->crc = inCRC;

    return (inCRC == calculatedCRC);
}

bool hasValidCRC915DB(DB915Telem_t *packet)
{
    uint8_t * asBytes = (uint8_t *)packet;

    const uint16_t inCRC = packet->crc;

    packet->crc = 0;

    uint16_t calculatedCRC = ota_crc.calc(asBytes, OTA_PACKET_LENGTH_TELEM);

    packet->crc = inCRC;

    return (inCRC == calculatedCRC);
}


/** Do CRC check for DB 2G4 packets
 * 
 */
bool hasValidCRC2G4DB(DB2G4Packet_t *packet)
{
    uint8_t * asBytes = (uint8_t *)packet;

    const uint16_t inCRC = packet->crc;

    packet->crc = 0; // clear the crc bits before doing the calculation

    uint16_t calculatedCRC = ota_crc.calc(asBytes, OTA_PACKET_LENGTH_2G4);

    packet->crc = inCRC; // replace the crc bits in case we want to use them again

    return (inCRC == calculatedCRC);
}


void setRadioParams(uint8_t index, RadioSelection targetRadio)
{
    bool invertIQ = false;

    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);

    switch(targetRadio)
    {
        case RadioSelection::first:
            std::cout << "setRadioParams not supported for radio1\n";
            // radio1->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            break;
        case RadioSelection::second:
            #ifdef USE_SECOND_RADIO
            radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq2G4(), ModParams->PreambleLen, invertIQ);
            #endif
            break;
        case RadioSelection::both:
            std::cout << "setRadioParams not supported for both\n";
            // radio1->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            // radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
            break;
    }
}

/** 
 * 
 */
void GenerateSyncPacketData()
{
   radio1->TXdataBuffer[0] = SYNC_PACKET;  // XXX New packet format needed

   // radio.TXdataBuffer[2] = NonceTX;
   radio1->TXdataBuffer[1] = (timerNonce >> 8) & 0xFF;
   radio1->TXdataBuffer[2] = (timerNonce >> 0) & 0xFF;

   // send either the current or next index if using sync spamming on rate change
   uint8_t rateIndex;
   #ifdef USE_SYNC_SPAM
   if (syncSpamCounter != 0 && nextLinkRate != -1) {
      // Notify the RX that we're going to change packet rate
      rateIndex = (nextLinkRate & 0b11);
      syncSpamCounter--;
   } else {
      // Send the current packet rate
      rateIndex = (ExpressLRS_currAirRate_Modparams->index & 0b11);
   }
   #else
   // Send the current packet rate
//    rateIndex = (ExpressLRS_currAirRate_Modparams->index & 0b11);
    // XXX this will be the rate of the 2G4 link eventually
   rateIndex = 0;
   #endif

   radio1->TXdataBuffer[3] = ((rateIndex & 0b111) << 5) + ((airRateConfig915.TLMinterval & 0b111) << 2);


   int8_t adjustedDBM = radio1->currPWR;
   #if defined(RADIO_E28_27)    // XXX make the radio instance know what sort of radio it is and query it at runtime?
   // for e28-27, PA output is +27dBm of the pre-PA setting, up to a max of 0 input.
   adjustedDBM += 27;
   #elif defined(RADIO_E28_20)
   // for e28-20, PA output is +22dBm of the pre-PA setting, up to a max of -2 input.
   adjustedDBM += 22;
   #endif

   radio1->TXdataBuffer[4] = adjustedDBM;

   radio1->TXdataBuffer[5] = UID[4];
   radio1->TXdataBuffer[6] = UID[5];
}

/**
 * Test out the new DB specific packet format
 */
void ICACHE_RAM_ATTR sendRCdataToRF915_DB()
{
    // printf("%u\n", nonceTX915);

    // XXX temporary data for testing
    uint16_t scaledADC[4];
    scaledADC[0] = 750;
    scaledADC[1] = 750;
    scaledADC[2] = 750;
    scaledADC[3] = 750;

    // uint8_t currentSwitches[4] = {0};
    // GenerateChannelDataHybridSwitch8(radio1->TXdataBuffer, scaledADC, currentSwitches, nextSwitchIndex, DeviceAddr);

    // defend size while developing
    #ifndef DEBUG_SUPPRESS
    if (sizeof(DB915Packet_t) != OTA_PACKET_LENGTH_915)
    {
        std::cout << "915 OTA_LENGTH ERROR!\n";
        while (true) {};
    }
    #endif

    DB915Packet_t * packet = (DB915Packet_t *)(radio1->TXdataBuffer);

    packet->nonce = timerNonce;
    packet->armed = 0;
    packet->txPower = 1;
    packet->rateIndex = ExpressLRS_currAirRate_Modparams->index;

    packet->ch0 = scaledADC[0];
    packet->ch1 = scaledADC[1];
    packet->ch2 = scaledADC[2];
    packet->ch3 = scaledADC[3];

    // these will eventually be set with different channels depending on whether we're on odd or even (nonce / rfModeDivisor915)
    packet->chA = 0;
    packet->chB = 0;
    packet->swA = 0;
    packet->swB = 0;
    packet->swC = 0;
    packet->swD = 0;


    // Calculate the CRC

    // clear the old crc before calculating the new one as there may be bit overlap across the byte boundaries
    packet->crc = 0;

    // calculate over the entire packet rather than trying to double guess the compiler's packing strategy
    uint16_t crc = ota_crc.calc(radio1->TXdataBuffer, OTA_PACKET_LENGTH_915);

    packet->crc = crc;

    // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // clear the rx debug pin as we're definitely not listening now

    // debug
    // if (radio1->currFreq == GetInitialFreq915())
    // {
    //     for(int i=0; i<OTA_PACKET_LENGTH_915; i++) printf("%02X ", radio1->TXdataBuffer[i]);
    //     printf(", crc %04X\n", crc);
    // }

    // std::cout << "sendRC TXnb\n";
    // radio1->setPacketLength(OTA_PACKET_LENGTH_915); // XXX make this automatic as part of the tx/rx calls
    radio1->TXnb(radio1->TXdataBuffer, OTA_PACKET_LENGTH_915);
}

/** OG send data with seperate sync packets
 * 
 */
void ICACHE_RAM_ATTR sendRCdataToRF915()
{
    // static to persist across calls
    static uint8_t syncPacketIndex = 1;

    // printf("n %u\n\r", nonceTX915);

    // for(int i=0; i<10; i++) radio.TXdataBuffer[i] = i;

    uint32_t SyncInterval;
    // XXX testing
    if (false && isRXconnected)
    {
        SyncInterval = airRateRFPerf915.SyncPktIntervalConnected;
    }
    else
    {
        SyncInterval = airRateRFPerf915.SyncPktIntervalDisconnected;
    }

    // safety check to make sure that if the mode is changed to one with a smaller fhss interval we don't get
    // stuck with syncPacketIndex > the new interval
    // Also sets the index to 1 if we're trying to establish a new connection as it seems to help avoid an initial 'slip' connection
    if (syncPacketIndex >= airRateConfig915.FHSShopInterval || !isRXconnected)
    {
        syncPacketIndex = 1;
    }

    // XXX temp hacking
    const int syncSpamCounter = 0;
    const int lastLinkRateChangeTime = 0;

    static unsigned long SyncPacketLastSent = 0; // static for persistence between calls

    //   if ((syncWhenArmed || !isArmed()) &&    // send sync packets when not armed, or always if syncWhenArmed is true
    if (
        ((syncSpamCounter != 0 && (millis() - lastLinkRateChangeTime > 300)) || // if we're trying to notify the rx of changing packet rate, just do it, otherwise...
         (
             ((millis() - SyncPacketLastSent) > SyncInterval) && // ...has enough time passed?
             (radio1->currFreq == GetInitialFreq915())                // we're on the sync frequency
             // sync just after we changed freqs (helps with hwTimer.init() being in sync from the get go)
             && ((nonceTX915 % airRateConfig915.FHSShopInterval) == syncPacketIndex))))
    { // Send a sync packet

        GenerateSyncPacketData();
        SyncPacketLastSent = millis();

        // rotate the sync packet through the packets in each fhss interval
        // if (isRXconnected) {
        //    syncPacketIndex = (syncPacketIndex + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
        //    if (syncPacketIndex == 0) syncPacketIndex = 1; // skip index 0 as it collides with telemetry
        // }

        std::cout << "sync\n";
    }
    else
    { // Send a normal RC packet
        // printf("%u\n", nonceTX915);

        // debug lack of sync packets
        // if ((millis() - SyncPacketLastSent) > SyncInterval) {
        //    // we're overdue for a sync packet, why didn't we send it?
        //    if (radio.currFreq != GetInitialFreq()) {
        //       printf("not freq\n\r");
        //    } else if ((nonceTX915 % ExpressLRS_currAirRate_Modparams->FHSShopInterval) != syncPacketIndex) {
        //       printf("not index\n\r");
        //    }
        // }

        // XXX temporary data for testing
        uint16_t scaledADC[4];
        scaledADC[0] = 750;
        scaledADC[1] = 750;
        scaledADC[2] = 750;
        scaledADC[3] = 750;

        #ifdef USE_HIRES_DATA
        GenerateHiResChannelData(radio.TXdataBuffer, scaledADC, DeviceAddr);
        #elif defined USE_PWM6
        GenerateChannelDataPWM6((Pwm6Payload_t *)radio.TXdataBuffer, scaledADC, currentSwitches);
        #else
        // XXX more temporary hacking
        // uint8_t nextSwitchIndex = getNextSwitchIndex();
        // uint8_t nextSwitchValue = currentSwitches[nextSwitchIndex];
        // setSentSwitch(nextSwitchIndex, nextSwitchValue);
        uint8_t nextSwitchIndex = 1;
        uint8_t currentSwitches[4] = {0};
        GenerateChannelDataHybridSwitch8(radio1->TXdataBuffer, scaledADC, currentSwitches, nextSwitchIndex, DeviceAddr);
        #endif // USE_HIRES_DATA
    }

    ///// Next, Calculate the CRC and put it into the buffer /////


    // need to clear the crc bits of the first byte before calculating the crc
    radio1->TXdataBuffer[0] &= 0b11;

    uint16_t crc = ota_crc.calc(radio1->TXdataBuffer, OTA_PACKET_LENGTH_915 - 1);
    radio1->TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    radio1->TXdataBuffer[OTA_PACKET_LENGTH_915 - 1] = crc & 0xFF;

    // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // clear the rx debug pin as we're definitely not listening now

    // debug
    // if (radio1->currFreq == GetInitialFreq915())
    // {
    //     for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", radio1->TXdataBuffer[i]);
    //     printf(", crc %04X\n", crc);
    // }

    // std::cout << "sendRC TXnb\n";
    radio1->TXnb(radio1->TXdataBuffer, OTA_PACKET_LENGTH_915);

    // // check for variaton in the intervals between sending packets
    // static LPF LPF_sentInterval(4);
    // static unsigned long tSentLast = 0;
    // unsigned long tSent = micros();
    // unsigned long delta = tSent - tSentLast;
    // int32_t avgDelta = LPF_sentInterval.update(delta);
    // int32_t variance = delta - avgDelta;
    // if (variance < 0) variance = -variance;
    // if (variance > 10) {
    //    printf("delta %lu avg %ld\n", delta, avgDelta);
    // }
    // tSentLast = tSent;
}

#ifndef USE_DB_PACKETS
/**
 * Pre dual band packet format
 */
void ICACHE_RAM_ATTR sendRCdataToRF2G4()
{
    // XXX temporary data for testing
    uint16_t scaledADC[4];
    scaledADC[0] = 750;
    scaledADC[1] = 750;
    scaledADC[2] = 750;
    scaledADC[3] = 750;

    #ifdef USE_HIRES_DATA
    GenerateHiResChannelData(radio.TXdataBuffer, scaledADC, DeviceAddr);
    #elif defined USE_PWM6
    GenerateChannelDataPWM6((Pwm6Payload_t *)radio.TXdataBuffer, scaledADC, currentSwitches);
    #else
    // XXX more temporary hacking
    // uint8_t nextSwitchIndex = getNextSwitchIndex();
    // uint8_t nextSwitchValue = currentSwitches[nextSwitchIndex];
    // setSentSwitch(nextSwitchIndex, nextSwitchValue);
    uint8_t nextSwitchIndex = 1;
    uint8_t currentSwitches[4] = {0};
    GenerateChannelDataHybridSwitch8(radio2->TXdataBuffer, scaledADC, currentSwitches, nextSwitchIndex, DeviceAddr);
    #endif // USE_HIRES_DATA

    ///// Next, Calculate the CRC and put it into the buffer /////

    // need to clear the crc bits of the first byte before calculating the crc
    radio2->TXdataBuffer[0] &= 0b11;

    uint16_t crc = ota_crc.calc(radio2->TXdataBuffer, OTA_PACKET_LENGTH_2G4 - 1);
    radio2->TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    radio2->TXdataBuffer[OTA_PACKET_LENGTH_2G4 - 1] = crc & 0xFF;

    // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // clear the rx debug pin as we're definitely not listening now

    // std::cout << "sendRC TXnb\n";
    radio2->TXnb(radio2->TXdataBuffer, OTA_PACKET_LENGTH_2G4);


    // // check for variaton in the intervals between sending packets
    // static LPF LPF_sentInterval(4);
    // static unsigned long tSentLast = 0;
    // unsigned long tSent = micros();
    // unsigned long delta = tSent - tSentLast;
    // int32_t avgDelta = LPF_sentInterval.update(delta);
    // int32_t variance = delta - avgDelta;
    // if (variance < 0) variance = -variance;
    // if (variance > 10) {
    //    printf("delta %lu avg %ld\n", delta, avgDelta);
    // }
    // tSentLast = tSent;
}
#endif // not USE_DB_PACKETS

/**
 * Dual band packet format
 */
void ICACHE_RAM_ATTR sendRCdataToRF2G4DB()
{
    DB2G4Packet_t * packet = (DB2G4Packet_t *) radio2->TXdataBuffer;

    // XXX temporary data for testing

    packet->ch0 = 3071;
    packet->ch1 = 3071;
    packet->ch2 = 3071;
    packet->ch3 = 3071;

    packet->armed = 0;

    packet->crc = 0;

    uint16_t crc = ota_crc.calc(radio2->TXdataBuffer, OTA_PACKET_LENGTH_2G4);
    
    packet->crc = crc;

    // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // clear the rx debug pin as we're definitely not listening now

    // std::cout << "sendRC TXnb\n";
    radio2->TXnb(radio2->TXdataBuffer, OTA_PACKET_LENGTH_2G4);


    // // check for variaton in the intervals between sending packets
    // static LPF LPF_sentInterval(4);
    // static unsigned long tSentLast = 0;
    // unsigned long tSent = micros();
    // unsigned long delta = tSent - tSentLast;
    // int32_t avgDelta = LPF_sentInterval.update(delta);
    // int32_t variance = delta - avgDelta;
    // if (variance < 0) variance = -variance;
    // if (variance > 10) {
    //    printf("delta %lu avg %ld\n", delta, avgDelta);
    // }
    // tSentLast = tSent;
}

/**
 * @return currently always returns 1
 */
uint8_t ICACHE_RAM_ATTR ProcessRFPacket(uint8_t *rxBuffer, uint32_t tPacketReceived, RadioSelection whichRadio)
{
    uint8_t type;

    // uint32_t beginProcessing;
    // uint32_t doneProcessing;

    // beginProcessing = micros();

    #ifdef USE_PWM6

    // XXX this code needs review and updating for dual modem

    type = ((Pwm6Payload_t*) radio1->RXdataBuffer)->header;
    inCRC = ((Pwm6Payload_t*) radio1->RXdataBuffer)->crc;
    ((Pwm6Payload_t*) radio1->RXdataBuffer)->crc = 0; // zero out the crc bits to match the sender

    uint16_t calculatedCRC = ota_crc.calc(radio1->RXdataBuffer, OTA_PACKET_LENGTH-1);

    #else

    type = rxBuffer[0] & 0b11;

    #endif // USE_PWM6

    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", rxBuffer[i]);
    // printf(", inCRC %04X calcCRC %04X\n", inCRC, calculatedCRC);

    // TODO since the crc checking was moved out of this function does it make sense to move the pfd extEvent() call
    // out as well?


    switch (whichRadio)
    {
        uint32_t pfdOffset;
        case RadioSelection::first:
            // get the per mode offset
            pfdOffset = airRateRFPerf915.pfdOffset;
            pfdLoop.extEvent915(tPacketReceived + pfdOffset);
            break;
        case RadioSelection::second:
            // get the per mode offset
            pfdOffset = ExpressLRS_currAirRate_RFperfParams->pfdOffset;
            pfdLoop.extEvent2G4(tPacketReceived + pfdOffset);
            break;
        default:
            std::cout << "radioselection not handled\n";
    }

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
        (void)SwitchEncModeExpected; // suppress compiler warnings
        (void)SwitchEncMode;

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

            #ifdef USE_ELRS_CRSF_EXTENSIONS
            crsf.LinkStatistics.txPower = rxBuffer[4];
            #endif

            uint16_t newTimerNonce = (rxBuffer[1] << 8) + rxBuffer[2] + 0;


            if (airRateConfig915.TLMinterval != (expresslrs_tlm_ratio_e)TLMrateIn)
            { // change link parameters if required
                #ifndef DEBUG_SUPPRESS
                printf("New TLMrate: %u\n", TLMrateIn);
                #endif
                airRateConfig915.TLMinterval = (expresslrs_tlm_ratio_e)TLMrateIn;
                //  telemBurstValid = false;
            }

            #ifdef ELRS_OG_COMPATIBILITY

            if (connectionState == disconnected
                || nonceRX915 != rxBuffer[2] // XXX add mask for 8 bit compare
                || FHSSgetCurrIndex() != rxBuffer[1]-1) // XXX don't forget this one when fixing the offsets
            {
                //Serial.print(nonceRX915, DEC); Serial.write('x'); Serial.println(rxBuffer[2], DEC);
                FHSSsetCurrIndex(rxBuffer[1]-1); // XXX take the +1 off the transmitter and stop messing around
                nonceRX915 = rxBuffer[2];
                TentativeConnection();
                doStartTimer = true;
            }

            #else // not ELRS_OG_COMPATIBILITY

            const uint32_t TimerNonceOffset195 = 0;

            if (connectionState == disconnected)
            {
                // timerNonce = 2 * (newTimerNonce + rfModeDivisor); // rfModeDivisor approximates how many increments happened on the tx while the packet was on the air
                timerNonce = newTimerNonce + TimerNonceOffset195;

                nonceRX915 = timerNonce / (2 * rfModeDivisor915);

                // printf("tN %u, cI %u fI %u\n", timerNonce, currentIndex, newFhssIndex);

                TentativeConnection();
                doStartTimer = true;

            } else { // not disconnected
                // const uint8_t nonceOffsets[] = { 0, 1, 3, 6}; // n(n+1)/2 - is this going to be stable?

                // uint16_t expectedNonce = (newTimerNonce + nonceOffsets[ExpressLRS_currAirRate_Modparams->index]) & 0x7FFF; // XXX should the tx run the nonce at 2kHz as well?
                uint16_t expectedNonce = (newTimerNonce + TimerNonceOffset195 - 6);
                uint16_t currentNonce = timerNonce;
                if (currentNonce != expectedNonce)
                {
                    #ifndef DEBUG_SUPPRESS
                    printf("nonce slip: current %u, expected %u\n", currentNonce, expectedNonce);
                    #endif
                    // timerNonce = 2 * (newTimerNonce + rfModeDivisor);
                    timerNonce = newTimerNonce + TimerNonceOffset195;
                    nonceRX915 = timerNonce / (2 * rfModeDivisor915);
                }

            }
            #endif // not ELRS_OG_COMPATIBILITY

            // if (ExpressLRS_currAirRate_Modparams->index != (expresslrs_tlm_ratio_e)indexIN)
            // if (ExpressLRS_currAirRate_Modparams->index != indexIN)
            // {
                // // change link parameters
                // ExpressLRS_nextAirRateIndex = indexIN;
                // liveUpdateRFLinkRate(ExpressLRS_nextAirRateIndex);

                // // printf("changing rate index from %u to %u\n", ExpressLRS_currAirRate_Modparams->index, indexIN);
                // uint32_t packetHz = 1000 / (1 << indexIN);
                // printf("\nrate change: %u Hz\n", packetHz);

                // dynamicRateTargetIndex = indexIN;

                // Initiate DR change by moving one of the radios to the new freq
                // setRadioParams(indexIN, RadioSelection::second); // XXX choose the radio with the weakest signal
                // radio2->setRxTimeout(10000);
                // radio2->RXnb();
                // drState = DynamicRateStates::ShiftDown; // XXX rename as it doesn't matter which direction we're trying to go in
                // tDrStart = millis();

                // #ifndef DEBUG_SUPPRESS
                // std::cout << "DR start\n";
                // #endif
                // std::cout << "rate change disabled\n";

            // }


        } else {
            #ifndef DEBUG_SUPPRESS
            printf("rejected sync pkt:");
            if (SwitchEncModeExpected != SwitchEncMode) printf(" switch mode");
            #ifdef ELRS_OG_COMPATIBILITY
            if (rxBuffer[4] != UID[3]) printf(", uid3");
            #endif
            if (rxBuffer[6] != UID[5]) printf(", uid5");
            printf("\n");
            #endif
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

/** For dual band format 2G4 packets.
 * 
 * These only contain the primary channels plus arm state
 * 
 * @return currently always returns 1
 */
uint8_t ICACHE_RAM_ATTR ProcessRFPacket2G4DB(uint8_t *rxBuffer, uint32_t tPacketReceived, RadioSelection whichRadio)
{
    uint8_t type;

    // TODO since the crc checking was moved out of this function does it make sense to move the pfd extEvent() call
    // out as well?

    uint32_t pfdOffset = ExpressLRS_currAirRate_RFperfParams->pfdOffset;
    pfdLoop.extEvent2G4(tPacketReceived + pfdOffset);

    lastValidPacket = millis(); // XXX add a specific 2G4 timestamp so that we can do data merging

    DB2G4Packet_t * packet = (DB2G4Packet_t *)rxBuffer;

    // copy data into crsf storage (or find a better place to keep it
    crsf.PackedHiResRCdataOut.chan0 = packet->ch0;
    crsf.PackedHiResRCdataOut.chan1 = packet->ch1;
    crsf.PackedHiResRCdataOut.chan2 = packet->ch2;
    crsf.PackedHiResRCdataOut.chan3 = packet->ch3;

    // XXX figure out the arm channel

    // send to fc
    if (connectionState != disconnected)
    {
        #ifdef CRSF_TX_PIN
        crsf.sendRCFrameToFC();
        #endif
    }

    return 1;
}

/** Process packets from the tx to the rx on the 915 channel
 * 
 * Combined sync and rc data in every packet
 * 
 * 
 * @return currently always returns 1
 */
uint8_t ICACHE_RAM_ATTR ProcessRFPacket915DB(uint8_t *rxBuffer, uint32_t tPacketReceived)
{
    bool doStartTimer = false;
    DB915Packet_t * packet = (DB915Packet_t *)rxBuffer;

    // TODO since the crc checking was moved out of this function does it make sense to move the pfd extEvent() call
    // out as well?
    pfdLoop.extEvent915(tPacketReceived + airRateRFPerf915.pfdOffset);

    lastValidPacket = millis();
    lastSyncPacket = lastValidPacket; // XXX get rid of this

    uint16_t newTimerNonce = packet->nonce;
    uint8_t indexIn = packet->rateIndex;

    // roughly OTA/<hwtimer interval (500us)>
    const uint32_t TimerNonceOffset195 = (indexIn == 0) ? 14 : 13; // XXX why does it make any difference what the 2G4 packet rate is?

    // If we're not currently connected then get the sync info and try and establish the link
    if (connectionState == disconnected)
    {
        // if we need to set the 2G4 packet rate do it first so that rfModeDivisor2G4 is set correctly
        if (indexIn != ExpressLRS_currAirRate_Modparams->index)
        {
            SetRFLinkRate(indexIn);
        }

        timerNonce = newTimerNonce + TimerNonceOffset195;
        nonceRX915 = timerNonce / (2 * rfModeDivisor915);
        nonceRX2G4 = timerNonce / (2 * rfModeDivisor2G4);

        // printf("tN %u, cI %u fI %u\n", timerNonce, currentIndex, newFhssIndex);

        TentativeConnection();
        doStartTimer = true;

    } else {

        // If we are connected then store the RC command data and check for rate changes and nonce slips
        crsf.LinkStatistics.txPower = packet->txPower;

        // XXX Only merge the primary channels if the data is newer than the last 2G4 packet
        // crsf.ChannelDataOut[0] = packet->ch0;
        // crsf.ChannelDataOut[1] = packet->ch1;
        // crsf.ChannelDataOut[2] = packet->ch2;
        // crsf.ChannelDataOut[3] = packet->ch3;

        // XXX and unpack other channels

        #ifdef CRSF_TX_PIN
        crsf.sendRCFrameToFC();
        #endif

        // check for packet rate change
        if (ExpressLRS_currAirRate_Modparams->index != indexIn)
        {
            // change link parameters
            ExpressLRS_nextAirRateIndex = indexIn;
            liveUpdateRFLinkRate(ExpressLRS_nextAirRateIndex);

            uint32_t packetHz = 1000 / (1 << indexIn);
            printf("\nrate change: %u Hz\n", packetHz);
        }

        // check for nonce slip
        uint16_t expectedNonce = (newTimerNonce + TimerNonceOffset195);
        uint16_t currentNonce = timerNonce;
        int16_t deltaNonce = expectedNonce - currentNonce;
        if (deltaNonce < 0) deltaNonce = -deltaNonce;
        if (deltaNonce > 1) // single step slips seem to happen at rate change (why?) but fix themselves. XXX figure out what's going on
        {
            #ifndef DEBUG_SUPPRESS
            // printf("nonce slip: current %u, expected %u\n", currentNonce, expectedNonce);
            #endif

            // std::cout << "slip correction disabled\n";
            timerNonce = newTimerNonce + TimerNonceOffset195;
            nonceRX915 = timerNonce / (2 * rfModeDivisor915);
            nonceRX2G4 = timerNonce / (2 * rfModeDivisor2G4);
        }

    } // if (disconnected vs connected/tentative)

    if (doStartTimer)
        HwTimer::resume(); // will throw an interrupt immediately (JBK: not sure if this is true across all impls)

    return 1;
}


/** Frequency hopping code for the 915 channel
 * 
 */
bool ICACHE_RAM_ATTR HandleFHSS915()
{
    static uint32_t lastNonceHopChecked = 0; // STATIC for persistence across calls

    // Optimise for the case where we've already done the hop or will never hop - we want to get out as quickly as possible
    // These conditions will have short circuit evaluation, left to right.
    if (alreadyFHSS915 || (airRateConfig915.FHSShopInterval == 0))
    {
        // if (alreadyFHSS) std::cout << "H-already\n";
        // if (connectionState == disconnected) std::cout << "H-disc\n";
        return false;
    }

    // On the rx side we don't hop when disconnected, but on the tx we always hop
    if (!TRANSMITTER && (connectionState == disconnected)) {
        return false;
    }

    // ok, so we might hop if we're on the right nonce, time to do the calculation
    // const uint32_t nonce = TRANSMITTER ? nonceTX915 : nonceRX915 + 1;
    const uint32_t nonce = TRANSMITTER ? nonceTX915 : nonceRX915;

    // if we've already checked for the current nonce then there's no need to check again
    if (nonce == lastNonceHopChecked) {
        return false;
    }

    #ifdef DEBUG_PIN
    gpio_set_level(DEBUG_PIN, 0);
    #endif

    lastNonceHopChecked = nonce;

    uint8_t modresultFHSS = nonce % airRateConfig915.FHSShopInterval;
    if (modresultFHSS != 0)
    {
        // std::cout << "H-nonce\n";
        return false;
    }

    // printf("fhss...\n");

    alreadyFHSS915 = true;

    // privatize so that the debug is consistent with what was used in the calculation
    // uint16_t localTimerNonce = timerNonce;

    // We need to make sure that we're not calculating the index right on the boundary of the change,
    // so this offset moves the calculation into the middle of the range.
    // uint16_t roundingOffset = rfModeDivisor * ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    // uint8_t newIndex = (localTimerNonce - roundingOffset) / (2 * rfModeDivisor * ExpressLRS_currAirRate_Modparams->FHSShopInterval);

    uint8_t newIndex = nonce / airRateConfig915.FHSShopInterval;

    // if (newIndex != newIndex2) printf("newIndex %u newIndex2 %u nonceRX915 %u TN %u\n", newIndex, newIndex2, nonceRX915, localTimerNonce);

    uint8_t currentIndex = FHSSgetCurrIndex915();

    #ifndef DEBUG_SUPPRESS
    static uint16_t lastTimerNonce = 0; // static for persistence between calls
    uint8_t expectedIndex = currentIndex + 1;
    if (newIndex != expectedIndex) {
        // printf("!!! fhss current %u, new %u, lastTN %u, current TN %u\n", currentIndex, newIndex, lastTimerNonce, localTimerNonce);
        printf("!!! fhss915 current %u, new %u, lastTN %u, current TN %u\n", currentIndex, newIndex, lastTimerNonce, timerNonce);
    }
    lastTimerNonce = timerNonce;
    #endif

    // If the newIndex is the same as the current index then we don't need to do anything
    if (newIndex == currentIndex) return false;

    FHSSsetCurrIndex915(newIndex);
    uint32_t freq = FHSSgetCurrFreq915();

    SpiTaskInfo taskInfo = {SpiTaskID::SetFrequency, RadioSelection::first, freq};
    xQueueSend(rx_evt_queue, &taskInfo, 1000);

    // std::cout << 'H';
    // printf("done\n");
    // printf("f %u\n", freq);

    // XXX do we need this rxnb call?
    // looks like we might need it on sx1262 for the receiver

    if (!TRANSMITTER) {
        taskInfo.id = SpiTaskID::StartRx;
        xQueueSend(rx_evt_queue, &taskInfo, 1000);
    }

    // uint8_t modresultTLM = (nonceRX915 + 1) % (TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval));
    // if (modresultTLM != 0 || ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) // if we are about to send a tlm response don't bother going back to rx
    // {
    //     radio1->RXnb();
    // }

    return true;
}


bool ICACHE_RAM_ATTR HandleFHSS2G4()
{
    // privatize the hop interval so that it can't change between testing it for 0 and using it as a divisor
    const uint8_t hopInterval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    // Optimise for the case where we've already done the hop or will never hop - we want to get out as quickly as possible
    // These conditions will have short circuit evaluation, left to right.
    if (alreadyFHSS2G4 || (hopInterval == 0))
    {
        return false;
    }

    // On the rx side we don't hop when disconnected, but on the tx we always hop
    if (!TRANSMITTER && (connectionState == disconnected)) {
        return false;
    }

    // ok, so we might hop if we're on the right nonce, time to do the calculation

    // const uint8_t HopOffsets[4] = {8, 4, 2, 1};
    const uint8_t HopOffsets[4] = {0, 0, 0, 0};
    const uint32_t nonce = TRANSMITTER ? nonceTX2G4 : nonceRX2G4 + HopOffsets[ExpressLRS_currAirRate_Modparams->index];

    // const uint32_t nonce = TRANSMITTER ? nonceTX2G4 : nonceRX2G4 + 2; // for 250Hz
    // const uint32_t nonce = TRANSMITTER ? nonceTX2G4 : nonceRX2G4 + 4; // for 500Hz
    // const uint32_t nonce = TRANSMITTER ? nonceTX2G4 : nonceRX2G4 + 8; // for 1kHz

    uint8_t modresultFHSS = nonce % hopInterval;
    if (modresultFHSS != 0)
    {
        return false;
    }

    alreadyFHSS2G4 = true;

    const uint8_t newIndex = nonce / hopInterval;
    const uint8_t currentIndex = FHSSgetCurrIndex2G4();

    #ifndef DEBUG_SUPPRESS
    static uint16_t lastTimerNonce = 0; // static for persistence between calls
    uint8_t expectedIndex = currentIndex + 1;
    if (newIndex != expectedIndex) {
        printf("!!! fhss2G4 current %u, new %u, lastTN %u, current TN %u\n", currentIndex, newIndex, lastTimerNonce, timerNonce);
    }
    lastTimerNonce = timerNonce;
    #endif

    // If the newIndex is the same as the current index then we don't need to do anything
    if (newIndex == currentIndex) return false;

    FHSSsetCurrIndex2G4(newIndex);
    uint32_t freq = FHSSgetCurrFreq2G4();

    SpiTaskInfo taskInfo = {SpiTaskID::SetFrequency, RadioSelection::second, freq};
    xQueueSend(rx_evt_queue, &taskInfo, 1000);

    // std::cout << 'H';
    // printf("done\n");
    // printf("f %u\n", freq);

    // XXX do we need this rxnb call for sx1280?

    if (!TRANSMITTER) {
        taskInfo.id = SpiTaskID::StartRx;
        xQueueSend(rx_evt_queue, &taskInfo, 1000);
    }

    return true;
}


bool ICACHE_RAM_ATTR isTelemetryFrame()
{
    if (airRateConfig915.TLMinterval == TLM_RATIO_NO_TLM)
    {
        return false;
    }

    uint32_t nonce = (TRANSMITTER) ? nonceTX915 : nonceRX915;

    const uint8_t modresult = nonce % TLMratioEnumToValue(airRateConfig915.TLMinterval);

    return (modresult == 0);
}


char * telemWhere = (char *)"not set";

bool ICACHE_RAM_ATTR sendTelemetryResponse()
{
    // printf("telem...\n");
    alreadyTLMresp = true;

    #ifdef USE_DB_PACKETS

    #ifndef DEBUG_SUPPRESS
    if (sizeof(DB915Telem_t) != OTA_PACKET_LENGTH_TELEM)
    {
        std::cout << "telem packet length wrong\n";
        while(true) {};
    }
    #endif

    DB915Telem_t * telemP = (DB915Telem_t *) radio1->TXdataBuffer;

    telemP->crc = 0;
    telemP->packetType = TLM_PACKET;
    telemP->rssi915 = -crsf.LinkStatistics.rssi0;
    telemP->lq915 = lqRadio1.getLQ();
    telemP->rssi2G4 = -crsf.LinkStatistics.rssi1;
    telemP->lq2G4 = lqRadio2.getLQ();

    uint16_t crc = ota_crc.calc(radio1->TXdataBuffer, OTA_PACKET_LENGTH_TELEM);

    telemP->crc = crc;

    #elif defined(USE_ELRS_CRSF_EXTENSIONS)

    // OpenTX RSSI as -dBm is fine and supports +dBm values as well
    // but the value in linkstatistics is "positivized" (inverted polarity)
    sendingRadio->TXdataBuffer[2] = -crsf.LinkStatistics.rssi0;
    sendingRadio->TXdataBuffer[3] = -crsf.LinkStatistics.rssi1;
    // sendingRadio->TXdataBuffer[4] = crsf.LinkStatistics.snr;
    sendingRadio->TXdataBuffer[5] = crsf.LinkStatistics.link_quality & 0x7F; // mask off the antenna bit for now

    #else

    // OpenTX RSSI as -dBm is fine and supports +dBm values as well
    // but the value in linkstatistics is "positivized" (inverted polarity)
    sendingRadio->TXdataBuffer[2] = -crsf.LinkStatistics.uplink_RSSI_1;
    sendingRadio->TXdataBuffer[3] = -crsf.LinkStatistics.uplink_RSSI_2;
    sendingRadio->TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
    sendingRadio->TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;

    #endif

    #ifndef USE_DB_PACKETS
    telemWhere = (char *)"crc";

    // XXX hard coded packet length - does it matter?
    uint16_t crc = ota_crc.calc(sendingRadio->TXdataBuffer, 7);
    sendingRadio->TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    sendingRadio->TXdataBuffer[7] = crc & 0xFF;
    #endif // USE_DB_PACKETS

    telemWhere = (char *)"tx";

    // packet length is set in the rx task when it detects the receiver is doing a send

    SpiTaskInfo taskInfo;
    taskInfo.id = SpiTaskID::StartTx;
    taskInfo.radioID = RadioSelection::first;

    xQueueSend(rx_evt_queue, &taskInfo, 1000);

    // printf("telem sent ");
    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", radio1->TXdataBuffer[i]);
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
 * XXX figure out how to best handle mixed 1262/1280 support
 */
static void ICACHE_RAM_ATTR tx_taskXXX(void* arg)
{
    SpiTaskInfo taskInfo;

    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(tx_evt_queue, &dummyData, portMAX_DELAY))
        {
            r1LastIRQms = millis();

            uint16_t irqs = radio1->GetIrqStatus(); // XXX how can we do this safely?

            // Sometimes we get preamble events even when not configured, so this needs to be guarded
            // if (irqs & SX1280_IRQ_PREAMBLE_DETECTED)
            // {
            //     // store the packet time here for use in PFD to reduce jitter on CRC failover
            //     tPacketR1 = esp_timer_get_time();
            //     radio1->ClearIrqStatus(SX1280_IRQ_PREAMBLE_DETECTED);
            //     // printf("p1");
            //     std::cout << "p1";

            // } else if (irqs & SX1280_IRQ_TX_DONE) {
            if (irqs & SX1262_IRQ_TX_DONE) {
                // give fhss a chance to run before tock()
                HandleFHSS915();

                HandleFHSS2G4(); // XXX move to 2G4 specific tx task

                // start the next receive
                // XXX should this test if the next frame is for telem?
                // taskInfo.id = SpiTaskID::StartRx;
                // taskInfo.radioID = 0; // 0 for both radios
                // xQueueSend(rx_evt_queue, &taskInfo, 1000);

                // radio1->RXnb();
                // radio2->RXnb();

            } else if (irqs & SX1262_IRQ_RX_TX_TIMEOUT) {
                // printf("r1 timeout\n");
                timeout1Counter++;
                // If we're connected it's safer to restart the receive from tock(), but
                // if we're not connected then the hwtimer isn't running and tock won't be called
                if (HwTimer::isRunning()) {
                    radio1Timedout = true;

                    taskInfo.id = SpiTaskID::ClearIRQs;
                    taskInfo.radioID = 1;
                    taskInfo.irqMask = SX1262_IRQ_RX_TX_TIMEOUT;
                    xQueueSend(rx_evt_queue, &taskInfo, 1000);

                    // radio1->ClearIrqStatus(SX1280_IRQ_RX_TX_TIMEOUT); // XXX replace with a new rxTask event
                    // std::cout << "DT1";
                } else {
                    // radio1->RXnb(); // clears all IRQs
                    taskInfo.id = SpiTaskID::StartRx;
                    taskInfo.radioID = 1;
                    xQueueSend(rx_evt_queue, &taskInfo, 1000);

                    // std::cout << "T1";
                }
            } else {
                #ifndef DEBUG_SUPPRESS
                printf("!!! tx_task irqs %04X\n", irqs);
                #endif
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

            if (irqs & SX1280_IRQ_TX_DONE) {
                // printf("tx2!\n");

                taskInfo.id = SpiTaskID::ClearIRQs;
                taskInfo.radioID = 2;
                taskInfo.irqMask = SX1280_IRQ_RX_TX_TIMEOUT;
                xQueueSend(rx_evt_queue, &taskInfo, 1000);

                // give fhss a chance to run before tock()
                HandleFHSS2G4();

                // start the next receive
                // taskInfo.id = SpiTaskID::StartRx;
                // taskInfo.radioID = 0; // 0 for both radios
                // xQueueSend(rx_evt_queue, &taskInfo, 1000);

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
                // somehow we get here repeatedly with irqs set to 0. Why?
                #ifndef DEBUG_SUPPRESS
                printf("!!! tx2_task irqs %04X\n", irqs);
                #endif

                taskInfo.id = SpiTaskID::ClearIRQs;
                taskInfo.radioID = 2;
                taskInfo.irqMask = SX1280_IRQ_RADIO_ALL;
                xQueueSend(rx_evt_queue, &taskInfo, 1000);                
            }
        }
    }
}

/** Handle all the possible DIO interrupts for the 915 modem
 * 
 *  Will be called on both the transmitter and receiver.
 * 
 * The 915 channel uses a ping-pong command/telemetry exchange in each frame, so direction reverses on every *done interrupt
 */
static void handleEvents915()
{
    r1LastIRQms = millis();

    uint16_t irqS = radio1->GetIrqStatus();

    if (irqS & SX1262_IRQ_RX_TX_TIMEOUT)    // TODO move this to the tx task so that our mainline rx path is shorter
    {
        timeout1Counter++;
        if (TRANSMITTER) {
            // on the transmitter we will drive the next transmit from tock as the recovery for timeouts
            radio1->ClearIrqStatus(SX1262_IRQ_RX_TX_TIMEOUT);
        } else {
            // on the receiver we'd better startup another RC packet receive
            // radio1->setPacketLength(OTA_PACKET_LENGTH_915);            
            radio1->RXnb();
        }
        std::cout << "915 timeout\n";

    } else if (irqS & SX1262_IRQ_TX_DONE) {

        // std::cout << "txdone\n";
        HandleFHSS915();

        if (TRANSMITTER) {

            if (isTelemetryFrame())
            {
                // start a receive to get the telemetry
                // implicitly clears all IRQs
                // radio1->setPacketLength(OTA_PACKET_LENGTH_TELEM);
                radio1->RXnb();
            } else {
                radio1->ClearIrqStatus(SX1262_IRQ_TX_DONE);
            }

        } else { // receiver

            // set the packet length for receiving an RC packet
            // radio1->setPacketLength(OTA_PACKET_LENGTH_915);

            // and start a receive for RC packet
            static const SpiTaskInfo taskInfo = {.id = SpiTaskID::StartRx, .radioID = RadioSelection::first, .unused = 0};
            xQueueSend(rx_evt_queue, &taskInfo, 1000);
        }

    } else if (irqS & SX1262_IRQ_RX_DONE) {

        HandleFHSS915();

        if (TRANSMITTER) {
            // telemetry packet
            radio1->readRXData(); // if we passed in the buffer we wouldn't have to do an extra memcpy

            if (hasValidCRC915DB((DB915Telem_t *)radio1->RXdataBuffer)) {
                memcpy(&telemData, (const void *)(radio1->RXdataBuffer), OTA_PACKET_LENGTH_TELEM);
            }

            // need to clear irqs, next transmit will be started from tock
            radio1->ClearIrqStatus(SX1262_IRQ_RX_DONE);

        } else {
            // Receiver gets normal RC packets on the 915 channel

            radio1->readRXData(); // get the data from the radio chip

            if (hasValidCRC915DB((DB915Packet_t *)radio1->RXdataBuffer))
            {
                ProcessRFPacket915DB((uint8_t*)radio1->RXdataBuffer, tPacketR1);
                totalRX1Events++;
                packetReceived = true; // needed?
                lqEffective.add();
                lqRadio1.add();
                antenna = 0;
                getRFlinkInfo();

                #if defined(PRINT_RX_SCOREBOARD)
                // printf("1");
                std::cout << '1';
                #endif

            } else {
                // failed CRC check
                crc1Counter++;
                #ifdef PRINT_RX_SCOREBOARD
                lastPacketCrcError = true;
                #endif

                // for(int i=0; i<OTA_PACKET_LENGTH_915; i++) printf("%02X ", radio1->RXdataBuffer[i]);
                // std::cout << '\n';
            }
            
            radio1->ClearIrqStatus(SX1262_IRQ_RX_DONE);

            if (!isTelemetryFrame()) {
                // and start a receive for next RC packet
                static const SpiTaskInfo taskInfo = {.id = SpiTaskID::StartRx, .radioID = RadioSelection::first, .unused = 0};
                xQueueSend(rx_evt_queue, &taskInfo, 1000);
            }

            // XXX How are we going to make sure that sending linkstats is thread safe?

            // static unsigned long SendLinkStatstoFCintervalLastSent = 0; // static for persistence between calls
            // if (connectionState != disconnected && 
            //     (alreadyTLMresp || ((millis() - SendLinkStatstoFCintervalLastSent) > SEND_LINK_STATS_TO_FC_INTERVAL))
            //    )
            // {
            //     crsf.sendLinkStatisticsToFC();
            //     SendLinkStatstoFCintervalLastSent = millis();
            // }

            // check for variation in the intervals between sending packets
            // static unsigned long tRcvLast = 0;
            // unsigned long delta = tRcv - tRcvLast;
            // int32_t avgDelta = LPF_rcvInterval.update(delta);
            // int32_t variance = delta - avgDelta;
            // if (variance < 0) variance = -variance;
            // if (variance > 10) {
            //     printf("fhhsMod %u delta %lu avg %ld, variance %ld\n", nonceRX915 % ExpressLRS_currAirRate_Modparams->FHSShopInterval, delta, avgDelta, variance);
            // }
            // tRcvLast = tRcv;

        } // if (transmitter or receiver)

    } else {
        printf("handleEvents915 irqS %u\n", irqS);
    }

}


/** receive a packet from the 2G4 channel
 * Only expected to be used on the receiver since 2G4 is currently unidirectional
 * 
 */
static void receive2G4()
{
    r2LastIRQms = millis();

    radio2->readRXData(); // get the data from the radio chip
    if (hasValidCRC2G4DB((DB2G4Packet_t *)radio2->RXdataBuffer))
    {
        ProcessRFPacket2G4DB((uint8_t*)radio2->RXdataBuffer, tPacketR2, RadioSelection::second);

        #if defined(PRINT_RX_SCOREBOARD)
        // printf("2");
        std::cout << '2';
        #endif

        totalRX2Events++;
        lqEffective.add();
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

    HandleFHSS2G4();
    
    radio2->RXnb();  // includes clearing the irqs

    // send link stats here so that it can never collide with the uart use in ProcessRFPacket.
    static unsigned long SendLinkStatstoFCintervalLastSent = 0; // static for persistence between calls
    if (connectionState != disconnected && 
        ((millis() - SendLinkStatstoFCintervalLastSent) > SEND_LINK_STATS_TO_FC_INTERVAL)
       )
    {
        crsf.sendLinkStatisticsToFC();
        SendLinkStatstoFCintervalLastSent = millis();
    }


    // check for variaton in the intervals between sending packets
    // static unsigned long tRcvLast = 0;
    // unsigned long delta = tRcv - tRcvLast;
    // int32_t avgDelta = LPF_rcvInterval.update(delta);
    // int32_t variance = delta - avgDelta;
    // if (variance < 0) variance = -variance;
    // if (variance > 10) {
    //     printf("fhhsMod %u delta %lu avg %ld, variance %ld\n", nonceRX915 % ExpressLRS_currAirRate_Modparams->FHSShopInterval, delta, avgDelta, variance);
    // }
    // tRcvLast = tRcv;
}


/** Handle received packets
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
static void ICACHE_RAM_ATTR rx_task915(void* arg)
{
    // uint32_t radioID;
    SpiTaskInfo taskInfo;
    for(;;) {
        if(xQueueReceive(rx_evt_queue, &taskInfo, portMAX_DELAY))
        {
            switch (taskInfo.id)
            {
                case SpiTaskID::RxDone:
                    // has to handle all ISR cases for sx1262 (because we don't have enough pins on the prototype board for dio2)
                    handleEvents915();
                    break;

                // case SpiTaskID::SetStandby: // not used, remove?
                //     switch(taskInfo.radioID)
                //     {
                //         case 1:
                //             radio1->SetMode(SX1280_MODE_FS);
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
                        case RadioSelection::first:
                            if (TRANSMITTER) {
                                // tx doesn't come through here yet?
                                std::cout << "unexpected tx code path\n";
                            } else {
                                // std::cout << "sending telem\n";
                                // set the length for telem packet
                                // radio1->setPacketLength(OTA_PACKET_LENGTH_TELEM);
                                // start the transmission
                                radio1->TXnb(radio1->TXdataBuffer, OTA_PACKET_LENGTH_TELEM);
                            }
                            break;
                        case RadioSelection::second:
                            std::cout << "no send on radio2\n";
                            // not sending on radio2 yet
                            // turn off the 'other' radio
                            // radio1->SetMode(SX1280_MODE_FS);
                            // start the transmission
                            // radio2->TXnb(radio2->TXdataBuffer, 8); // Currently only used to send telem packets which are fixed at 8 bytes
                            break;
                        default:
                            printf("unexpected radioID for StartTx %d\n", taskInfo.radioID);
                    }
                    break;

                case SpiTaskID::SetFrequency:
                    switch(taskInfo.radioID)
                    {
                        case RadioSelection::both: // change on both radios
                            std::cout << "can't set both freqs on dual band\n";
                            // radio1->SetFrequency(taskInfo.frequency);
                            // radio2->SetFrequency(taskInfo.frequency);
                            break;
                        case RadioSelection::first:
                            radio1->SetFrequency(taskInfo.frequency);
                            break;
                        case RadioSelection::second:
                            radio2->SetFrequency(taskInfo.frequency);
                            break;
                        default:
                            printf("unexpected radioID for StartRx %d\n", taskInfo.radioID);
                    }
                    break;

                case SpiTaskID::StartRx:
                    switch(taskInfo.radioID)
                    {
                        case RadioSelection::both: // start rx on both radios
                            radio1->RXnb();
                            radio2->RXnb();
                            break;
                        case RadioSelection::first:
                            radio1->RXnb();
                            break;
                        case RadioSelection::second:
                            radio2->RXnb();
                            break;
                        default:
                            printf("unexpected radioID for StartRx %d\n", taskInfo.radioID);
                    }
                    break;

                case SpiTaskID::ClearIRQs:
                    switch(taskInfo.radioID)
                    {
                        case RadioSelection::first:
                            radio1->ClearIrqStatus(taskInfo.irqMask);
                            break;
                        case RadioSelection::second:
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


/** Handle received packets on 2G4
 * 
 * This should be simpler than 915, it only needs to deal with RC data on the 4 primary channels
 * 
 */
static void ICACHE_RAM_ATTR rx_task2G4(void* arg)
{
    // uint32_t radioID;
    SpiTaskInfo taskInfo;
    for(;;) {
        if(xQueueReceive(rx2G4_evt_queue, &taskInfo, portMAX_DELAY))
        {
            switch (taskInfo.id)
            {
                case SpiTaskID::RxDone:
                    receive2G4();
                    break;

                default:
                    printf("2G4 unhandled SpiTaskID %u\n", taskInfo.id);
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
    static const SpiTaskInfo taskInfo = {.id = SpiTaskID::RxDone, .radioID = 2, .unused = 0};

    BaseType_t taskWoken = 0;

    // store the packet time here for use in PFD to reduce jitter
    tPacketR2 = esp_timer_get_time();

    // send to the same queue as radio1
    xQueueSendFromISR(rx2G4_evt_queue, &taskInfo, &taskWoken);
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

    rfModeDivisor2G4 = ModParams->interval / 1000;

    // Since we have a new divisor, we need to refresh the NonceRX
    nonceRX2G4 = timerNonce / (2 * rfModeDivisor2G4);

    uint8_t newIndex = (nonceRX2G4 / ExpressLRS_currAirRate_Modparams->FHSShopInterval) - 1;

    // uint8_t currentIndex = FHSSgetCurrIndex();
    // printf("fhss current %u, new %u\n", currentIndex, newIndex);
    FHSSsetCurrIndex2G4(newIndex);
    uint32_t freq = FHSSgetCurrFreq2G4();

    #ifdef USE_FLRC
    if (index == 0)
    { // special case FLRC for testing
        radio1->ConfigFLRC(GetInitialFreq());
    }
    else
    #endif
    {
        #ifdef USE_SECOND_RADIO
        radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, freq, ModParams->PreambleLen, invertIQ);
        #endif
    }

    #ifdef USE_SECOND_RADIO
    // Set timeout based on the interval
    radio2->setRxTimeout(ModParams->interval * 5 / 2);
    radio2->RXnb();
    #endif

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    pfdDebugCounter = 10; // trigger pfd debug
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
        radio1->ConfigFLRC(GetInitialFreq());
    }
    else
#endif
    {
        // radio1->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
        #ifdef USE_SECOND_RADIO
        radio2->Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq2G4(), ModParams->PreambleLen, invertIQ);
        #endif
    }

    rfModeDivisor2G4 = ModParams->interval / 1000;

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    // if (TRANSMITTER) {
    //     isRXconnected = false;
    // }

    // if (UpdateRFparamReq)
    //    UpdateRFparamReq = false;
}


// Why isn't this part of PFD?
void ICACHE_RAM_ATTR updatePhaseLock915()
{
    if (connectionState != disconnected)
    {
        RawOffset915 = pfdLoop.calcResult915();
        pfdLoop.reset915();

        // XXX shouldn't the min/max be related to the actual timer interval, not the (much slower) 915 interval?
        const int32_t max = (int32_t)airRateConfig915.interval * 2;
        const int32_t min = -max;

        // ignore values that are out of reasonable range
        // for reasons unknown, combining these two tests causes the < min clause to always evaluate true
        if (RawOffset915 > max)
        {
            printf("ign %d, max = %d\n", RawOffset915, max);
            return;
        }

        if (RawOffset915 < min)
        {
            printf("ign %d, min = %d\n", RawOffset915, min);
            return;
        }


        Offset = LPF_Offset.update(RawOffset915);
        OffsetDx = LPF_OffsetDx.update(RawOffset915 - prevRawOffset915);

        if (Offset > 1000000 || Offset < -1000000) {
            printf("raw %d off %d\n", RawOffset915, Offset);
        }


        if (RXtimerState == tim_locked && (lqRadio1.currentIsSet() || lqRadio2.currentIsSet()))
        {
            if (nonceRX915 % 8 == 0) //limit rate of freq offset adjustment slightly
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
            HwTimer::setPhaseShift(RawOffset915 >> 1);
        } else {
            HwTimer::setPhaseShift(Offset >> 2);
        }

        prevOffset = Offset;
        prevRawOffset915 = RawOffset915;

        // if (pfdDebugCounter > 0) {
        //     printf("Offset: %d\n", Offset);
        //     pfdDebugCounter--;
        // }
    }

#ifndef DEBUG_SUPPRESS
    // printf("%ld %ld %ld %ld %u\n", Offset, RawOffset915, OffsetDx, 0, uplinkLQ);
    // Serial.print(Offset);
    // Serial.print(":");
    // Serial.print(RawOffset915);
    // Serial.print(":");
    // Serial.print(OffsetDx);
    // Serial.print(":");
    // Serial.print(HwTimer::getFreqOffset());
    // Serial.print(":");
    // Serial.println(uplinkLQ);
#endif
}

void ICACHE_RAM_ATTR updatePhaseLock2G4()
{
    if (connectionState != disconnected)
    {
        RawOffset2G4 = pfdLoop.calcResult2G4();
        pfdLoop.reset2G4();

        // pfdLoop returns 0 for no valid value
        if (RawOffset2G4 == 0) return;

        const int32_t max = (int32_t) ExpressLRS_currAirRate_Modparams->interval * 2;

        const int32_t min = -max;

        // ignore values that are out of reasonable range
        // for reasons unknown, combining these two tests causes the < min clause to always evaluate true
        if (RawOffset2G4 > max)
        {
            printf("2G4 ign %d, max = %d\n", RawOffset2G4, max);
            return;
        }

        if (RawOffset2G4 < min)
        {
            printf("2G4 ign %d, min = %d\n", RawOffset2G4, min);
            return;
        }


        int32_t Offset2G4 = LPF_Offset2G4.update(RawOffset2G4);
        // int32_t OffsetDx2G4 = LPF_OffsetDx2G4.update(RawOffset2G4 - prevRawOffset2G4);

        if (Offset2G4 > 1000000 || Offset2G4 < -1000000) {
            printf("2G4 raw %d off %d\n", RawOffset2G4, Offset2G4);
        }

        #ifdef PFD_CALIBRATION

        // debug
        printf("2G4 offset %d\n", Offset2G4);

        #else // NOT PFD_CALIBRATIO

        // Can't use the 2G4 for phase shift until we're already locked as it prevents the 915 from
        // getting the sync onto the right sub-frame. But once we're locked 2G4 can be used to fine tune
        // the sync

        if (RXtimerState == tim_locked && (lqRadio2.currentIsSet()) && Offset < 100 && Offset > -100)
        {
            // loop frequency adjustment
            if (nonceRX2G4 % 8 == 0) //limit rate of freq offset adjustment slightly
            {
                if (Offset2G4 > 0)
                {
                    HwTimer::incFreqOffset();
                }
                else if (Offset2G4 < 0)
                {
                    HwTimer::decFreqOffset();
                }
            }

            // apply phase adjustment
            HwTimer::setPhaseShift(Offset2G4 >> 2);
        }

        #endif // PFD_CALIBRATION

        // static int32_t prevOffset2G4 = Offset2G4;
        // prevRawOffset2G4 = RawOffset2G4;

        // if (pfdDebugCounter > 0) {
        //     printf("Offset: %d\n", Offset);
        //     pfdDebugCounter--;
        // }
    }

#ifndef DEBUG_SUPPRESS
    // printf("%ld %ld %ld %ld %u\n", Offset, RawOffset915, OffsetDx, 0, uplinkLQ);
    // Serial.print(Offset);
    // Serial.print(":");
    // Serial.print(RawOffset915);
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
void ICACHE_RAM_ATTR tick915()
{
    tickStartMs = millis();

    // printf("tickIn\n");
    // static unsigned long last = 0;
    // unsigned long now = micros();

    // if (last != 0) {
    //     printf("tick %lu\n", (now-last));
    // }
    // last = now;

    // #ifdef DEBUG_PIN
    // gpio_set_level(DEBUG_PIN, 1);
    // #endif

    if (!TRANSMITTER)
    {
        updatePhaseLock915();
    }

    // XXX remind me why we need rx and tx nonces again?
    nonceRX915 = timerNonce / (2 * rfModeDivisor915);
    nonceTX915 = timerNonce / (2 * rfModeDivisor915);

    // reset the flag for dual radio diversity reception
    packetReceived = false;

    // Save the LQ value before the inc() reduces it by 1
    // uplinkLQ = lqEffective.getLQ();
    // lpfLq.update(uplinkLQ);

    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!alreadyTLMresp) {
        lqRadio1.inc();
        // lqEffective.inc();
    }

    alreadyTLMresp = false;
    alreadyFHSS915 = false;


    // printf("tickOut\n");
    tickStartMs = 0;

}


/** Aligned with (approximately) the midpoint of the radio interval (not necessarily the midpoint of the radio signal since duty cycle < 1)
 * 
 *  Assume that the radio modem is busy doing either rx or tx during this call, so we can only do non-radio related housekeeping in here.
 * 
 *
 */
void ICACHE_RAM_ATTR tick2G4()
{
    // tickStartMs = millis();

    // printf("tickIn\n");
    // static unsigned long last = 0;
    // unsigned long now = micros();

    // if (last != 0) {
    //     printf("tick %lu\n", (now-last));
    // }
    // last = now;

    // #ifdef DEBUG_PIN
    // gpio_set_level(DEBUG_PIN, 1);
    // #endif

    if (!TRANSMITTER)
    {
        updatePhaseLock2G4();
    }

    // XXX remind me why we need rx and tx nonces again?
    alreadyFHSS2G4 = false;
    nonceRX2G4 = timerNonce / (2 * rfModeDivisor2G4);
    nonceTX2G4 = timerNonce / (2 * rfModeDivisor2G4);

    // Save the LQ value before the inc() reduces it by 1
    uplinkLQ = lqEffective.getLQ();
    lpfLq.update(uplinkLQ);

    lqEffective.inc();
    lqRadio2.inc();

    // printf("tickOut\n");
    // tickStartMs = 0;
}


char *tockWhere = (char *)"not set";

/** Aligned with (approximately) the start of the radio signal
 * 
 */
void ICACHE_RAM_ATTR tockRX915()
{
    tockStartMs = millis();

    tockWhere = (char *)"start";

    pfdLoop.intEvent915(esp_timer_get_time());

    #ifdef DEBUG_PIN
    gpio_set_level(DEBUG_PIN, 1);
    #endif

    tockWhere = (char *)"FHSS";
    HandleFHSS915();
    tockWhere = (char *)"telem";


    if (isTelemetryFrame())
    {
        sendTelemetryResponse();
    }


    tockWhere = (char *)"HM";

    // This is a hail-Mary check for if the radios haven't been talking and kick off new RXnb calls.
    // But if it's here then it will only be active when the timer is running. Maybe we need two levels of hail-Mary

    // privatize the *Last times so they can't change after we assign 'now' and cause false positives
    uint32_t p1 = r1LastIRQms; //, p2 = r2LastIRQms;
    uint32_t now = millis();
    if ((now - p1) > 2000)
    {
        radio1->RXnb(); // XXX convert to event messages
        radio1Timedout = false;
        std::cout << "HM1";
        r1LastIRQms = now; // stop it spamming
    }

    // if ((now - p2) > 2000)
    // {
    //     radio2->RXnb();
    //     radio2Timedout = false;
    //     std::cout << "HM2";
    //     r2LastIRQms = now; // stop it spamming
    // }

    tockWhere = (char *)"dTs";

    SpiTaskInfo taskInfo;
    taskInfo.id = SpiTaskID::StartRx;

    // deferred receives from rx timeouts
    if (!alreadyTLMresp) // if we're doing telem then the radio will be restarted after anyway.
    {
        if (radio1Timedout) {
            taskInfo.radioID = 1;
            xQueueSend(rx_evt_queue, &taskInfo, 1000);
            radio1Timedout = false;
        }

    }

    // XXX should radio2 stuff move to tockRX2G4?
    if (radio2Timedout) {
        taskInfo.radioID = 2;
        xQueueSend(rx_evt_queue, &taskInfo, 1000);
        radio2Timedout = false;
    }

    #if defined(PRINT_RX_SCOREBOARD)
    static bool lastPacketWasTelemetry = false; // NB static for perstence across calls
    if (!lqEffective.currentIsSet() && !lastPacketWasTelemetry)
        // printf(lastPacketCrcError ? "X" : "_");
        std::cout << (lastPacketCrcError ? 'X' : '_');

    lastPacketCrcError = false;
    lastPacketWasTelemetry = alreadyTLMresp;
    #endif
    // printf("tockOut\n");

    tockWhere = (char *)"end";

    tockStartMs = 0;
}


/** Aligned with (approximately) the start of the radio signal
 * 
 */
void ICACHE_RAM_ATTR tockRX2G4()
{

    pfdLoop.intEvent2G4(esp_timer_get_time());

    // #ifdef DEBUG_PIN
    // gpio_set_level(DEBUG_PIN, 1);
    // #endif

    HandleFHSS2G4();

    #if defined(PRINT_RX_SCOREBOARD)
    if (!lqRadio2.currentIsSet() )
    {
        // XXX need a 2G4 crc error flag
        // std::cout << (lastPacketCrcError ? 'X' : '_');
        std::cout << '?';
    }
    // lastPacketCrcError = false;
    #endif

}


/** Aligned with the start of the radio signal
 * 
 */
void ICACHE_RAM_ATTR tockTX915()
{
    // std::cout << "tockTX\n";

    #ifdef DEBUG_PIN
    gpio_set_level(DEBUG_PIN, 1);
    #endif

    // hopefully this will already have been done from txDone interrupt
    HandleFHSS915();

    if (!isTelemetryFrame()) {
        sendRCdataToRF915_DB();
    }
}

/** Aligned with the start of the radio signal
 * 
 */
void ICACHE_RAM_ATTR tockTX2G4()
{
    // std::cout << "tockTX\n";

    // #ifdef DEBUG_PIN
    // gpio_set_level(DEBUG_PIN, 0);
    // #endif

    // hopefully this will already have been done from txDone interrupt
    // XXX check the return value and debug if it wasn't done
    HandleFHSS2G4();

    #ifndef DISABLE_2G4
    sendRCdataToRF2G4DB();
    #endif
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

    if (timerNonce % (2 * rfModeDivisor2G4) == 0)
    {
        if (TRANSMITTER) {
            tockTX2G4();
        } else {
            tockRX2G4();
        }
    } else if ((timerNonce + rfModeDivisor2G4) % (2 * rfModeDivisor2G4) == 0) {
        tick2G4();
    }

    if (timerNonce % (2 * rfModeDivisor915) == 0)
    {
        if (TRANSMITTER) {
            tockTX915();
        } else {
            tockRX915();
        }
    } else if ((timerNonce + rfModeDivisor915) % (2 * rfModeDivisor915) == 0) {
        tick915();
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
    RawOffset915 = 0;
    prevOffset = 0;
    GotConnectionMillis = 0;
    uplinkLQ = 0;
    lqRadio1.reset();
    lqRadio2.reset();
    LPF_Offset.init(0);
    LPF_OffsetDx.init(0);
    alreadyTLMresp = false;
    alreadyFHSS915 = false;
    // LED = false; // Make first LED cycle turn it on

    HwTimer::stop();

    drState = DynamicRateStates::Normal; // Cancel any pending dynamic rate change

    // XXX These two are potential SPI collisions if there might still be activity on the rx/tx tasks
    // SetRFLinkRate(ExpressLRS_nextAirRateIndex); // also sets to initialFreq 
    // std::cout << "setrflinkrate call disabled\n";

    radio1->SetFrequency(GetInitialFreq915());

    // set large timeout
    radio1->setRxTimeout(2000000);
    radio1->RXnb();

    #ifdef USE_SECOND_RADIO
    radio2->setRxTimeout(ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval * 1000);
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
    uint8_t interval = airRateConfig915.FHSShopInterval;
    
    return interval * ((interval * NR_FHSS_ENTRIES_915 + 99) / (interval * NR_FHSS_ENTRIES_915));
}

// No need to cycle in dual band mode
// XXX do need to move the led blinking code somewhere

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
// static void cycleRfMode()
// {
//     // statics for persistence accross calls
//     static uint32_t LEDLastUpdate = 0; 
//     static bool LED = false; 

//     const uint32_t LED_INTERVAL_DISCONNECTED = 1000;

//     if (connectionState != disconnected) // || InBindingMode || webUpdateMode)
//         return;

//     // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
//     if (LockRFmode == false && (millis() - RFmodeLastCycled) > (ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval * RFmodeCycleMultiplier))
//     {
//         printf("cycling\n");
//         unsigned long now = millis();
//         RFmodeLastCycled = now;
//         lastSyncPacket = now;           // reset this variable
//         // SetRFLinkRate(scanIndex % RATE_MAX); // switch between rates
//         std::cout << "setrflinkrate call disabled\n";
//         linkStatstoFCLastSent = now;
//         lqRadio1.reset();
//         lqRadio2.reset();
//         printf("%u\n", ExpressLRS_currAirRate_Modparams->interval);
//         scanIndex++;
//         getRFlinkInfo();
//         crsf.sendLinkStatisticsToFC();
//         // delay(100);
//         // crsf.sendLinkStatisticsToFC(); // need to send twice, not sure why, seems like a BF bug?


//         radio1->RXnb();
//         #ifdef USE_SECOND_RADIO
//         radio2->RXnb();
//         #endif

//         // Switch to FAST_SYNC if not already in it (won't be if was just connected)
//         RFmodeCycleMultiplier = 1;
//     } // if time to switch RF mode

//     // Always blink the LED at a steady rate when not connected, independent of the cycle status
//     if (millis() - LEDLastUpdate > LED_INTERVAL_DISCONNECTED)
//     {
//         #ifdef GPIO_PIN_LED
//             digitalWrite(GPIO_PIN_LED, LED ^ GPIO_LED_RED_INVERTED);
//         #elif GPIO_PIN_LED_GREEN
//             digitalWrite(GPIO_PIN_LED_GREEN, LED ^ GPIO_LED_GREEN_INVERTED);
//         #endif

//         #ifdef LED_STATUS_INDEX
//         setLedColour(LED_STATUS_INDEX, LED ? LED_BRIGHTNESS : 0, 0, 0);
//         #endif

//         LED = !LED;
//         LEDLastUpdate = millis();
//     } // if cycle LED


// }

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
    #ifdef LED2812_PIN

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

    #endif // LED2812_PIN
}

void setLedColour(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    #ifdef LED2812_PIN
    strip->set_pixel(strip, index, red, green, blue);
    strip->refresh(strip, 100);
    #endif // LED2812_PIN
}

extern "C"
{

extern char * wcWhere;


void app_main()
{
    // uart_set_baudrate(UART_NUM_0, 921600);
    uart_set_baudrate(UART_NUM_0, 2000000);
    // uart_set_baudrate(UART_NUM_0, 420000);
    std::cout << "ESP32-C3 FreeRTOS Dual Band RX\n";

    // The default uart setup doesn't enable input, so have to do it manually
    const int uart_buffer_size = (1024 * 2);
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, GPIO_NUM_20, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 0, 0, 0));

    #ifdef DEBUG_PIN

    // Set the pin as an output
    gpio_reset_pin(DEBUG_PIN);
    gpio_set_direction(DEBUG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DEBUG_PIN, 0);

    #endif // DEBUG_PIN

    printf("sizeof new 915 packet is %d\n", sizeof(DB915Packet_t));
    printf("sizeof new 2G4 packet is %d\n", sizeof(DB2G4Packet_t));


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

    radio1 = new SX1262Driver();
    // radio1 = new SX1280Driver();
    radio2 = new SX1280Driver(RADIO2_NSS_PIN);

    radio1->currFreq = GetInitialFreq915();
    // radio2->currFreq = GetInitialFreq2G4();

    const bool usePreambleDetect = false; // this didn't work out, but maybe worth another look someday?

    // XXX retry if either radio doesn't initialise properly

    radio1->Begin(); // XXX need to add result value to sx1262 driver

    // if (radio1->Begin(usePreambleDetect) == 0) {
        // set led green
        // setLedColour(LED_RADIO1_INDEX, 0, LED_BRIGHTNESS, 0);
    // } else {
        // set led red
        // setLedColour(LED_RADIO1_INDEX, LED_BRIGHTNESS, 0, 0);
    // }

    if (radio2->Begin(usePreambleDetect) == 0) {
        // set led green
        setLedColour(LED_STATUS_INDEX, 0, LED_BRIGHTNESS, 0);
    } else {
        // set led red
        setLedColour(LED_STATUS_INDEX, LED_BRIGHTNESS, 0, 0);
    }

    radio1->SetOutputPower(-9);    // XXX figure out what to do for tx and rx powers
    radio2->SetOutputPower(0);

    // radio1 settings are fixed, so we can do them once here
    radio1->Config(airRateConfig915.bw, airRateConfig915.sf, airRateConfig915.cr, GetInitialFreq915(), airRateConfig915.PreambleLen, false);
    rfModeDivisor915 = airRateConfig915.interval / 1000;

    //create a queue to handle gpio event from isr
    rx_evt_queue = xQueueCreate(10, sizeof(SpiTaskInfo));
    // Since we don't have enough pins for dio2 on the sx1262 everything is currently routed to the rx915 task
    // tx_evt_queue = xQueueCreate(10, 0);

    rx2G4_evt_queue = xQueueCreate(10, sizeof(SpiTaskInfo));
    tx2_evt_queue = xQueueCreate(10, 0);

    //start tasks
    xTaskCreate(rx_task915, "rx_task915", 2048, NULL, 16, NULL); // Task priority 1=min, max is ??? Default main task is pri 1
    xTaskCreate(rx_task2G4, "rx_task2G4", 2048, NULL, 17, NULL);

    // xTaskCreate(tx_task, "tx_task", 2048, NULL,  5, NULL);

    // std::cout << "app_main tx2_task disabled\n";
    xTaskCreate(tx2_task, "tx2_task", 2048, NULL,  5, NULL);


    // need to attach ISR handlers to DIO pins

    gpio_set_intr_type(RADIO_DIO1_PIN, GPIO_INTR_POSEDGE);
    #ifdef RADIO_DIO2_PIN
    gpio_set_intr_type(RADIO_DIO2_PIN, GPIO_INTR_POSEDGE);
    #endif

    gpio_set_intr_type(RADIO2_DIO1_PIN, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(RADIO2_DIO2_PIN, GPIO_INTR_POSEDGE);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    //hook isr handler for specific gpio pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO1_PIN, dio1_isr_handler, NULL));
    // ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO2_PIN, dio2_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO2_DIO1_PIN, r2_dio1_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO2_DIO2_PIN, r2_dio2_isr_handler, NULL));

    uint8_t linkRateIndex = INITIAL_LINK_RATE_INDEX;
    SetRFLinkRate(linkRateIndex); // sets radio2

    // delay(50);
    // printf("setting FS\n\r");
    // radio.SetMode(SX1262_MODE_FS);
    // delay(50);
    // radio.GetStatus();

    // XXX What's a reasonable timeout?
    // timeout in us
    radio1->setRxTimeout(2000000);
    radio2->setRxTimeout(2000000);

    FHSSrandomiseFHSSsequences();

    if (TRANSMITTER)
    {
        HwTimer::resume();
    } else {
        radio1->RXnb();
        radio2->RXnb();
        crsf.Begin();
    }

    delay(500);

    #ifdef LED2812_PIN
    strip->clear(strip, 10);
    #endif

    #ifndef DEBUG_SUPPRESS
    unsigned long lastDebug = 0;
    #endif

    // unsigned long lastLEDUpdate = 0;
    unsigned long prevTime = 0;

    // uint32_t lastRX1Events = 0, lastRX2Events = 0;

    // loop
    while(true) {
        // things that don't have to happen very quickly
        //delay(100);   // idle watchdog has to be disabled if we comment this out

        // taskYIELD(); // does this do anything useful? Need to check what the freertos idle task actually does

        unsigned long now = millis();
        if (now - prevTime > 200) {
            printf("big gap %u to %u\n", prevTime, now);
        }
        prevTime = now;

        // if (lastLEDUpdate + 100 < now)
        // {
        //     lastLEDUpdate = now;

            // uint32_t delta1 = totalRX1Events - lastRX1Events;
            // uint32_t delta2 = totalRX2Events - lastRX2Events;

            // lastRX1Events = totalRX1Events;
            // lastRX2Events = totalRX2Events;

            // uint32_t totalPackets = delta1 + delta2 + timeout1Counter + timeout2Counter + crc1Counter + crc2Counter + 1;

            // uint32_t brightness1 = delta1 * LED_BRIGHTNESS / totalPackets;
            // uint32_t brightness2 = delta2 * LED_BRIGHTNESS / totalPackets;

            // uint32_t green1 = lqRadio1.getLQ() * brightness1 / 100;
            // uint32_t green2 = lqRadio2.getLQ() * brightness2 / 100;

            // uint32_t red1 = (crc1Counter * brightness1 / totalPackets) + (brightness1 - green1);
            // uint32_t red2 = (crc2Counter * brightness2 / totalPackets) + (brightness2 - green2);
            // uint32_t red1 = (brightness1 - green1);
            // uint32_t red2 = (brightness2 - green2);

            // uint32_t blue1 = timeout1Counter * LED_BRIGHTNESS / totalPackets;
            // uint32_t blue2 = timeout2Counter * LED_BRIGHTNESS / totalPackets;

            // strip->set_pixel(strip, LED_RADIO1_INDEX, red1, green1, blue1);
            // strip->set_pixel(strip, LED_RADIO2_INDEX, red2, green2, blue2);
            // strip->refresh(strip, 10);

            // printf("delta %u bright %u green %u\n", delta1, brightness1, green1);
        // }

        #ifndef DEBUG_SUPPRESS
        if (lastDebug + 1000 < now)
        {
            uint32_t elapsedT = now - lastDebug;
            lastDebug = now;

            if (TRANSMITTER)
            {
                // Can't print the packed values directly, have to localize them first (printf bug?)
                int8_t rssi915 = telemData.rssi915;
                uint8_t lq915 = telemData.lq915;
                int8_t rssi2G4 = telemData.rssi2G4;
                uint8_t lq2G4 = telemData.lq2G4;
                printf("Telem 915: %d %u, 2G4: %d, %u\n", rssi915, lq915, rssi2G4, lq2G4);

            } else {

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
            } // if (transmitter or receiver)
        }
        #endif // DEBUG_SUPPRESS

        // Fail a tentative connection if we don't get a sync packet in RFmodeCycleAddtionalTime
        if (connectionState == tentative)
        {
            const uint32_t localLastSyncPacket = lastSyncPacket;
            const uint32_t tBS = millis();
            if ((tBS - localLastSyncPacket) > airRateRFPerf915.RFmodeCycleAddtionalTime)
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
        if ((connectionState == connected) && (((int32_t)airRateRFPerf915.RFmodeCycleInterval * 2) < (int32_t)(tLost - localLastValidPacket)))
        {
            printf("loop: connection lost, localLVP %lu, tLost %lu, LQ %u\n", localLastValidPacket, tLost, uplinkLQ);
            lostConnection();
        }

        // detects when we are connected
        if ((connectionState == tentative) && (abs(OffsetDx) <= 10) && (Offset < 100) && 
            // (uplinkLQ > minLqForChaos())
            (lqRadio1.getLQPercent() > minLqForChaos())
            )
        {
            #ifndef DEBUG_SUPPRESS
            printf("loop: connected\n");
            #endif
            gotConnection();
        }

        // If the link is looking stable we can move the timer to locked mode (and start doing loop freq compensation)
        if ((RXtimerState == tim_tentative) && ((millis() - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(OffsetDx) <= 5))
        {
            #ifndef DEBUG_SUPPRESS
            printf("Timer Locked\n");
            #endif
            RXtimerState = tim_locked;
        }

        // if (RATE_MAX > 1) cycleRfMode(); // No cycling in dual band mode

        if (TRANSMITTER)
        {
            static uint32_t lastModeChange = 0; // STATIC for persistence

            uint8_t buf;
            int nread = uart_read_bytes(UART_NUM_0, &buf, 1, 0);
            if (nread != 0) {
                printf("read %d bytes, '%c'\n", nread, buf);
                if (buf == '+' && (ExpressLRS_currAirRate_Modparams->index > 0)) {
                    uint8_t newIndex = ExpressLRS_currAirRate_Modparams->index - 1;
                    SetRFLinkRate(newIndex);
                } else if (buf == '-' && (ExpressLRS_currAirRate_Modparams->index < (RATE_MAX-1))) {
                    uint8_t newIndex = ExpressLRS_currAirRate_Modparams->index + 1;
                    SetRFLinkRate(newIndex);
                }
            }
        }


    } // while true

}

} // extern "C"