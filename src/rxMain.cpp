/**
 * Initial test code for C3 based receivers
 * 
 */

#include <stdio.h>
#include <string.h>
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

#define LED_BRIGHTNESS 64


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


// XXX can this be reduced at all? Should it have a component based on (interval - tOTA)?
#define PACKET_TO_TOCK_SLACK 200 // Desired buffer time between Packet ISR and Tock ISR
// #define PACKET_TO_TOCK_SLACK 100 // Desired buffer time between Packet ISR and Tock ISR

#ifdef RADIO_E22

SX1262Driver radio1;

#elif defined (RADIO_E28_12) || defined(RADIO_E28_20) || defined(RADIO_E28_27)

// XXX Having one of these as a pointer and one not is going to be really annoying. Fix
SX1280Driver radio1;
SX1280Driver * radio2 = NULL;

#else
error("must define a radio module")

#endif // RADIO_*

CRSF crsf;
GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);
PFD pfdLoop;

uint8_t ExpressLRS_nextAirRateIndex = 0;


uint32_t timeoutCounter = 0;
uint32_t mismatchCounter = 0;
uint32_t totalPackets = 0;
uint32_t totalRX1Events = 0;
uint32_t totalRX2Events = 0;
uint32_t totalMatches = 0;
uint32_t totalRX2 = 0;
int32_t cumulativeRSSI = 0;
uint32_t crcCounter = 0;
uint32_t errCounter = 0;

static xQueueHandle rx_evt_queue = NULL;
static xQueueHandle rx2_evt_queue = NULL;
static xQueueHandle tx_evt_queue = NULL;
static xQueueHandle tx2_evt_queue = NULL;

volatile uint8_t NonceRX = 0; // nonce that we THINK we are up to.

bool alreadyFHSS = false;
bool alreadyTLMresp = false;
volatile bool packetReceived = false;

// uint32_t beginProcessing;
// uint32_t doneProcessing;

volatile uint32_t tPacketR1 = 0;
volatile uint32_t tPacketR2 = 0;
volatile uint32_t lastValidPacket = 0;
uint32_t lastSyncPacket = 0;

uint32_t linkStatstoFCLastSent = 0;

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


// LPF LPF_UplinkRSSI(5);
LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
LPF LPF_UplinkRSSI1(5);

LPF LPF_rcvInterval(4);

/// LQ Calculation //////////
LQCALC<100> LQCalc;
uint8_t uplinkLQ;

uint8_t scanIndex = RATE_DEFAULT;


uint8_t antenna = 0;    // which antenna is currently in use

bool isRXconnected = false;

#if defined(PRINT_RX_SCOREBOARD)
static bool lastPacketCrcError;
#endif

led_strip_t *strip = nullptr;

// forward refs
void setLedColour(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);



// XXX TODO move this somewhere sensible
void delay(uint32_t millis)
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

    // #ifdef USE_SECOND_RADIO
    //     rssiDBM0 = LPF_UplinkRSSI0.update(radio1.GetLastPacketRSSI());   // running this code causes spi conflicts - why?
    //     rssiDBM1 = LPF_UplinkRSSI1.update(radio2->GetLastPacketRSSI());

    // #else // not a full diversity RX

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

    // #endif // USE_SECOND_RADIO

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
    printf("tentative conn\n");
    FreqCorrection = 0;
    Offset = 0;
    prevOffset = 0;
    LPF_Offset.init(0);
    RFmodeLastCycled = millis(); // give another 3 sec for lock to occur

    LPF_rcvInterval.init(ExpressLRS_currAirRate_Modparams->interval);

    ExpressLRS_nextAirRateIndex = ExpressLRS_currAirRate_Modparams->index;

#if WS2812_LED_IS_USED
    uint8_t LEDcolor[3] = {0};
    LEDcolor[(2 - ExpressLRS_currAirRate_Modparams->index) % 3] = 50;
    WS281BsetLED(LEDcolor);
    LEDWS2812LastUpdate = millis();
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

/**
 * @return 1 if packet passed crc check, 0 otherwise
 */
uint8_t ICACHE_RAM_ATTR ProcessRFPacket(uint8_t *rxBuffer, uint32_t tPacketReceived)
{
    uint8_t type;
    uint16_t inCRC;

    // uint32_t beginProcessing;
    // uint32_t doneProcessing;

    // beginProcessing = micros();

    #ifdef USE_PWM6

    type = ((Pwm6Payload_t*) radio1.RXdataBuffer)->header;
    inCRC = ((Pwm6Payload_t*) radio1.RXdataBuffer)->crc;
    ((Pwm6Payload_t*) radio1.RXdataBuffer)->crc = 0; // zero out the crc bits to match the sender

    uint16_t calculatedCRC = ota_crc.calc(radio1.RXdataBuffer, OTA_PACKET_LENGTH-1);

    #else

    type = rxBuffer[0] & 0b11;

    inCRC = ( ( (uint16_t)(rxBuffer[0] & 0b11111100) ) << 6 ) | rxBuffer[OTA_PACKET_LENGTH-1];

    rxBuffer[0] = type;
    uint16_t calculatedCRC = ota_crc.calc(rxBuffer, OTA_PACKET_LENGTH-1);

    #endif // USE_PWM6

    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", rxBuffer[i]);
    // printf(", inCRC %04X calcCRC %04X\n", inCRC, calculatedCRC);

    if (inCRC != calculatedCRC)
    {
        #ifndef DEBUG_SUPPRESS
        // printf("CRC error on RF packet: ");
        // for (int i = 0; i < OTA_PACKET_LENGTH; i++)
        // {
        //     printf("%02X ", rxBuffer[i]);
        // }
        // printf("\n");
        #endif
        #if defined(PRINT_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        crcCounter++;
        return 0;     // EARLY RETURN
    }

    // XXX if we're currently processing the second packet of a frame, this could introduce a lot
    // of jitter. Perhaps we should only adjust pfd on first packets?
    // XXX How about recording packet arrival time in the ISRs and then passing the appropriate one as a param?
    // pfdLoop.extEvent(beginProcessing + PACKET_TO_TOCK_SLACK);
    pfdLoop.extEvent(tPacketReceived + PACKET_TO_TOCK_SLACK);

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
            Serial.write('s');
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

            if (connectionState == disconnected)
            {
                //Serial.print(NonceRX, DEC); Serial.write('x'); Serial.println(rxBuffer[2], DEC);
                FHSSsetCurrIndex(rxBuffer[1]);
                NonceRX = rxBuffer[2] + 1;
                TentativeConnection();
                doStartTimer = true;
            } else if (NonceRX != (rxBuffer[2] + 1))
            {
                printf("nonce slip: current %d, expected %d\n", NonceRX, rxBuffer[2] + 1);
                NonceRX = rxBuffer[2] + 1;
            } else if (FHSSgetCurrIndex() != rxBuffer[1]) 
            {
                printf("fhss index mismatch: current %d, expected %d\n", FHSSgetCurrIndex(), rxBuffer[1]);
                FHSSsetCurrIndex(rxBuffer[1]);
            }

            #endif // ELRS_OG_COMPATIBILITY

            // if (ExpressLRS_currAirRate_Modparams->index != (expresslrs_tlm_ratio_e)indexIN)
            if (ExpressLRS_currAirRate_Modparams->index != indexIN)
            { // change link parameters if required
                ExpressLRS_nextAirRateIndex = indexIN;
                printf("changing rate index from %u to %u\n", ExpressLRS_currAirRate_Modparams->index, indexIN);
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

    LQCalc.add(); // Received a packet, that's the definition of LQ
    // Extend sync duration since we've received a packet at this rate
    // but do not extend it indefinitely
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

    // doneProcessing = micros();
    #if defined(PRINT_RX_SCOREBOARD)
    if (type != SYNC_PACKET) Serial.write('R');
    #endif

    if (doStartTimer)
        HwTimer::resume(); // will throw an interrupt immediately

    return 1;
}

bool ICACHE_RAM_ATTR HandleFHSS()
{
    // Optimise for the case where we've already done the hop or will never hop - we want to get out as quickly as possible
    // These conditions will have short circuit evaluation, left to right.
    if (alreadyFHSS || (ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || (connectionState == disconnected))
    {
        return false;
    }

    // ok, so we might hop if we're on the right nonce, time to do the calculation
    // uint8_t modresultFHSS = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    uint8_t modresultFHSS = (NonceRX + 0) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    if (modresultFHSS != 0)
    {
        return false;
    }

    // printf("fhss...\n");

    alreadyFHSS = true;
    uint32_t freq = FHSSgetNextFreq();
    radio1.SetFrequency(freq);

    #ifdef USE_SECOND_RADIO
    radio2->SetFrequency(freq);
    #endif

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
    #ifdef ELRS_OG_COMPATIBILITY
    const uint8_t modresult = (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    #else
    const uint8_t modresult = (NonceRX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    #endif

    return modresult == 0;
}

bool ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    #ifdef ENABLE_TELEMETRY
    uint8_t *data;
    uint8_t maxLength;
    uint8_t packageIndex;
    #endif

    // don't bother sending tlm if disconnected or TLM is off
    if ((connectionState == disconnected) || (ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) || (alreadyTLMresp == true))
    {
        return false;
    }

    if (!isTelemetryFrame())
    {
        return false;
    }

    // printf("telem...\n");
    SX1280Driver *sendingRadio = &radio1;
    SX1280Driver *otherRadio = radio2;

    // turn off the other radio so it doesn't report our own telem packet
    otherRadio->SetMode(SX1280_MODE_FS);

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

    // XXX hard coded packet length - does it matter?
    uint16_t crc = ota_crc.calc(sendingRadio->TXdataBuffer, 7);
    sendingRadio->TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    sendingRadio->TXdataBuffer[7] = crc & 0xFF;

    sendingRadio->TXnb(sendingRadio->TXdataBuffer, 8);
    // printf("telem sent ");
    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", radio1.TXdataBuffer[i]);
    // printf("crc %04X\n", crc);

    // printf("...done\n");

    return true;
}


static void tx_task(void* arg)
{
    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(tx_evt_queue, &dummyData, portMAX_DELAY))
        {
            uint16_t irqs = radio1.GetIrqStatus();

            if (irqs & SX1280_IRQ_TX_DONE)
            {
                // give fhss a chance to run before tock()
                HandleFHSS();
                // start the next receive
                radio1.RXnb();
                radio2->RXnb();
            } else if (irqs & SX1280_IRQ_RX_TX_TIMEOUT) {
                // printf("r1 timeout\n");
                radio1.RXnb();
                timeoutCounter++;
            } else {
                printf("!!! tx_task irqs %04X\n", irqs);
            }
        }
    }
}

static void tx2_task(void* arg)
{
    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(tx2_evt_queue, &dummyData, portMAX_DELAY))
        {
            uint16_t irqs = radio2->GetIrqStatus();

            if (irqs & SX1280_IRQ_TX_DONE)
            {
                // printf("tx2!\n");            
                // give fhss a chance to run before tock()
                HandleFHSS();
                // start the next receive for both radios
                radio1.RXnb();
                radio2->RXnb();
            } else if (irqs & SX1280_IRQ_RX_TX_TIMEOUT) {
                // printf("r2 timeout\n");
                radio2->RXnb();
                timeoutCounter++;
            } else {
                printf("!!! tx2_task irqs %04X\n", irqs);
            }
        }
    }
}

/** Handle received packets
 * 
 * The following might not be needed now that we're not getting telem related spi collisions...
 * 
 * Instead of passing the radioID via the queue, use an eventDescriptor instead?
 * 
 * Events that will need to be handled:
 *   packet received (from either modem)
 *   set frequency
 *   start receive (for either or both modems)
 * 
 */
static void rx_task(void* arg)
{
    uint32_t radioID;
    for(;;) {
        if(xQueueReceive(rx_evt_queue, &radioID, portMAX_DELAY))
        {
            // unsigned long tRcv = micros();


            // timeouts for sx1280 are currently routed into the tx_tasks
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
            uint8_t processResult = 0;

            switch (radioID) {
                case 1:
                    if (!packetReceived)
                    {
                        radio1.readRXData(); // get the data from the radio chip
                        processResult = ProcessRFPacket((uint8_t*)radio1.RXdataBuffer, tPacketR1);
                        totalRX1Events++;

                        if (processResult) {
                            antenna = 0;
                            getRFlinkInfo();
                        }
                    } else {
                        // grab the rssi, otherwise the unused channel gets very slow update
                        antenna = 0;
                        getRFlinkInfo();
                    }
                    
                    // start the next receive
                    if (connectionState != connected || !isTelemetryFrame()) {
                        radio1.RXnb();  // includes clearing the irq
                    } else {
                        radio1.ClearIrqStatus(SX1280_IRQ_RX_DONE);
                    }

                    break;

                case 2:
                    if (!packetReceived)
                    {
                        radio2->readRXData(); // get the data from the radio chip
                        processResult = ProcessRFPacket((uint8_t*)radio2->RXdataBuffer, tPacketR2); 
                        totalRX2Events++;

                        if (processResult) {
                            antenna = 1;
                            getRFlinkInfo();
                        }
                    } else {
                        // grab the rssi, otherwise the unused channel gets very slow update
                        antenna = 1;
                        getRFlinkInfo();
                    }

                    if (connectionState != connected || !isTelemetryFrame()) {
                        radio2->RXnb();  // includes clearing the irq
                    } else {
                        radio2->ClearIrqStatus(SX1280_IRQ_RX_DONE);
                    }

                    break;

                default:
                    printf("unexpected radioID %u\n", radioID);
            }

            packetReceived = (processResult == 1);

            // give fhss a chance to run before tock()
            if (processResult == 1)
                HandleFHSS();

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

            // totalPackets++;
        }
    }
}


/** dio1 interrupt when packet is received
 */
static void IRAM_ATTR dio1_isr_handler(void* arg)
{
    BaseType_t taskWoken = 0;
    uint32_t radioID = 1;

    // store the packet time here for use in PFD to reduce jitter on CRC failover
    tPacketR1 = esp_timer_get_time();

    xQueueSendFromISR(rx_evt_queue, &radioID, &taskWoken);

    if (taskWoken) portYIELD_FROM_ISR();
}

/**
*/
static void IRAM_ATTR r2_dio1_isr_handler(void* arg)
{
    BaseType_t taskWoken = 0;
    uint32_t radioID = 2;

    // store the packet time here for use in PFD to reduce jitter on CRC failover
    tPacketR2 = esp_timer_get_time();

    // send to the same queue as radio1
    xQueueSendFromISR(rx_evt_queue, &radioID, &taskWoken);

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

    // printf("in SetRFLinkRate, setTimerISRInterval disabled\n");
    //    setTimerISRInterval(ModParams->interval);
    HwTimer::setInterval(ModParams->interval);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    isRXconnected = false;

// #ifdef USE_DYNAMIC_POWER
//     // The dynamic power thresholds are relative to the rx sensitivity, so need to be updated
//     dynamicPowerRSSIIncreaseThreshold = RFperf->RXsensitivity + DYN_POWER_INCREASE_MARGIN;
//     dynamicPowerRSSIDecreaseThreshold = dynamicPowerRSSIIncreaseThreshold + DYN_POWER_DECREASE_MARGIN;
// #endif

    // if (UpdateRFparamReq)
    //    UpdateRFparamReq = false;
}

// Why isn't this part of PFD?
void ICACHE_RAM_ATTR updatePhaseLock()
{
    if (connectionState != disconnected)
    {
        pfdLoop.calcResult();
        pfdLoop.reset();
        RawOffset = pfdLoop.getResult();
        Offset = LPF_Offset.update(RawOffset);
        OffsetDx = LPF_OffsetDx.update(RawOffset - prevRawOffset);

        if (RXtimerState == tim_locked && LQCalc.currentIsSet())
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
        }
        else
        {
            HwTimer::setPhaseShift(Offset >> 2);
        }

        prevOffset = Offset;
        prevRawOffset = RawOffset;
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
void tick()
{
    // printf("tickIn\n");
    // static unsigned long last = 0;
    // unsigned long now = micros();

    // if (last != 0) {
    //     printf("tick %lu\n", (now-last));
    // }
    // last = now;

    updatePhaseLock();
    NonceRX++;

    // reset the flag for dual radio diversity reception
    packetReceived = false;


    // This is commented out in upstream

    // if (!alreadyTLMresp && !alreadyFHSS && !LQCalc.currentIsSet()) // packet timeout AND didn't DIDN'T just hop or send TLM
    // {
    //     radio1.RXnb(); // put the radio cleanly back into RX in case of garbage data
    // }

    // Save the LQ value before the inc() reduces it by 1
    uplinkLQ = LQCalc.getLQ();
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!alreadyTLMresp)
        LQCalc.inc();

    alreadyTLMresp = false;
    alreadyFHSS = false;
    // printf("tickOut\n");
}

/** Aligned with (approximately) the start of the radio signal
 * 
 */
void tock()
{
    // printf("tockIn\n");
    // pfdLoop.intEvent(micros()); // our internal osc just fired
    pfdLoop.intEvent(esp_timer_get_time()); // our internal osc just fired

    // updateDiversity();
    HandleFHSS();
    HandleSendTelemetryResponse();


    #if defined(PRINT_RX_SCOREBOARD)
    static bool lastPacketWasTelemetry = false;
    if (!LQCalc.currentIsSet() && !lastPacketWasTelemetry)
        Serial.write(lastPacketCrcError ? '.' : '_');
    lastPacketCrcError = false;
    lastPacketWasTelemetry = tlmSent;
    #endif
    // printf("tockOut\n");
}

#ifdef PINGPONG
void pingPongTest()
{
    unsigned long tPrev = millis();
    uint32_t lastTotalP = 0;

    radio1.setRxTimeout(1000000 / 15); // XXX move the scaling inside the setRxTimeout function

    while (true)
    {
        unsigned long now = millis();
        unsigned long elapsed = now - tPrev;
        if (elapsed > 1000)
        {
            uint32_t nPackets = totalPackets - lastTotalP;
            uint32_t rate = (nPackets * 2000) / elapsed; // factor of 2 for ping + echo
            int32_t averageRSSI = cumulativeRSSI / (int32_t)nPackets;
            printf("total: %lu, timeouts: %lu, rate %lu pkts/s, average RSSI %ld\n\r", totalPackets, timeoutCounter, rate, averageRSSI);
            tPrev = now;
            lastTotalP = totalPackets;
            cumulativeRSSI = 0;
        }
        delay(500);
        // radio1.GetStatus();
   }
}
#endif

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
    LQCalc.reset();
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

    radio1.RXnb();
    #ifdef USE_SECOND_RADIO
    radio2->RXnb();
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

#if WS2812_LED_IS_USED
    uint8_t LEDcolor[3] = {0};
    LEDcolor[(2 - ExpressLRS_currAirRate_Modparams->index) % 3] = 255;
    WS281BsetLED(LEDcolor);
    LEDWS2812LastUpdate = millis();
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
    if (connectionState == connected) // || InBindingMode || webUpdateMode)
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
        LQCalc.reset();
        printf("%u\n", ExpressLRS_currAirRate_Modparams->interval);
        scanIndex++;
        getRFlinkInfo();
        crsf.sendLinkStatisticsToFC();
        delay(100);
        crsf.sendLinkStatisticsToFC(); // need to send twice, not sure why, seems like a BF bug?

        radio1.RXnb();
        #ifdef USE_SECOND_RADIO
        radio2->RXnb();
        #endif

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode

    // Always blink the LED at a steady rate when not connected, independent of the cycle status
    // if (millis() - LEDLastUpdate > LED_INTERVAL_DISCONNECTED)
    // {
    //     #ifdef GPIO_PIN_LED
    //         digitalWrite(GPIO_PIN_LED, LED ^ GPIO_LED_RED_INVERTED);
    //     #elif GPIO_PIN_LED_GREEN
    //         digitalWrite(GPIO_PIN_LED_GREEN, LED ^ GPIO_LED_GREEN_INVERTED);
    //     #endif
    //     LED = !LED;
    //     LEDLastUpdate = millis();
    // } // if cycle LED
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
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(2, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        printf("install WS2812 driver failed\n");
        return;
    }

    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    // printf("set red\n");

    // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, brightness, 0, 0));
    // ESP_ERROR_CHECK(strip->set_pixel(strip, 1, 0, brightness, 0));
    // ESP_ERROR_CHECK(strip->refresh(strip, 100));

    // delay(1000);

    // printf("set green\n");

    // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, brightness, 0));
    // ESP_ERROR_CHECK(strip->set_pixel(strip, 1, 0, 0, brightness));

    // ESP_ERROR_CHECK(strip->refresh(strip, 100));

    // delay(1000);

    // printf("set blue\n");

    // ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, 0, brightness));
    // ESP_ERROR_CHECK(strip->set_pixel(strip, 1, brightness, 0, 0));

    // ESP_ERROR_CHECK(strip->refresh(strip, 100));

}

void setLedColour(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
    strip->set_pixel(strip, index, red, green, blue);
    strip->refresh(strip, 100);
}

extern "C"
{

void app_main()
{
    printf("ESP32-C3 FreeRTOS RX\n");

    setupPWM(); // does nothing if pwm not being used

    initLeds();

    // The HwTimer is using priority 15 - should it be higher or lower than the rx task?
    HwTimer::init();
    HwTimer::setInterval(2000000);
    HwTimer::stop();

    HwTimer::setCallbackTick(tick);
    HwTimer::setCallbackTock(tock);

    radio2 = new SX1280Driver(RADIO2_NSS_PIN);

    radio1.currFreq = GetInitialFreq();
    radio2->currFreq = GetInitialFreq();


    if (radio1.Begin() == 0) {
        // set led green
        setLedColour(0, 0, LED_BRIGHTNESS, 0);
    } else {
        // set led red
        setLedColour(0, LED_BRIGHTNESS, 0, 0);
    }

    if (radio2->Begin() == 0) {
        // set led green
        setLedColour(1, 0, LED_BRIGHTNESS, 0);
    } else {
        // set led red
        setLedColour(1, LED_BRIGHTNESS, 0, 0);
    }

    radio1.SetOutputPower(DISARM_POWER);
    radio2->SetOutputPower(DISARM_POWER);

    //create a queue to handle gpio event from isr
    rx_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    rx2_evt_queue = xQueueCreate(10, 0);
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

    delay(100);
    strip->clear(strip, 10);

    radio1.RXnb();
    radio2->RXnb();
    crsf.Begin();

    unsigned long lastDebug = 0;
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
        if (lastDebug + 1000 < now)
        {
            uint32_t elapsedT = now - lastDebug;
            lastDebug = now;
            // printf("rx1Events %u, rx2Events %u,  matches %u, LQ %u, timeouts/s %u crcErrors/s %u\n", totalRX1Events, totalRX2Events, totalMatches, uplinkLQ, timeoutCounter*1000/elapsedT, crcCounter*1000/elapsedT);
            printf("rx1Events %u, rx2Events %u, LQ %u, crcErrors/s %u\n", totalRX1Events, totalRX2Events, uplinkLQ, crcCounter*1000/elapsedT);
            int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
            int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;
            printf("rss1 %d, rssi2 %d\n", rssiDBM0, rssiDBM1);
            // printf("Offset %4d DX %4d freqOffset %d\n", Offset, OffsetDx, HwTimer::getFreqOffset());

            uint32_t delta1 = totalRX1Events - lastRX1Events;
            uint32_t delta2 = totalRX2Events - lastRX2Events;

            lastRX1Events = totalRX1Events;
            lastRX2Events = totalRX2Events;

            uint32_t green1 = delta1 * LED_BRIGHTNESS / (delta1 + delta2 + timeoutCounter + crcCounter);
            uint32_t green2 = delta2 * LED_BRIGHTNESS / (delta1 + delta2 + timeoutCounter + crcCounter);

            strip->set_pixel(strip, 0, 0, green1, 0);
            strip->set_pixel(strip, 1, 0, green2, 0);
            strip->refresh(strip, 10);

            // totalPackets = 0;
            timeoutCounter = 0;
            crcCounter = 0;

            if (connectionState == tentative)
            {
                printf("tentative:");
                if (abs(OffsetDx) > 10) printf(" offsetDx %d", OffsetDx);
                if (Offset >= 100) printf(" offset %d", Offset);
                if (uplinkLQ <= minLqForChaos()) printf(" lq %u", uplinkLQ);
                printf("\n");
            }
        }

        // This isn't great to do from here, we might get an SPI collision with the still active rx/tx tasks
        if ((connectionState != disconnected) && (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRate_Modparams->index)){ // forced change
            printf("Air rate change req via sync\n");
            lostConnection();
            lastSyncPacket = now;           // reset this variable to stop rf mode switching and add extra time
            RFmodeLastCycled = now;         // reset this variable to stop rf mode switching and add extra time
            crsf.sendLinkStatisticsToFC();
            crsf.sendLinkStatisticsToFC(); // need to send twice, not sure why, seems like a BF bug?
            continue; // no point looking at all the other cases in the loop after this
        }


        // again this is potentially dangerous if rx may still be in progress
        if (connectionState == tentative && (now - lastSyncPacket > ExpressLRS_currAirRate_RFperfParams->RFmodeCycleAddtionalTime))
        {
            printf("Bad sync, aborting\n");
            lostConnection();
            RFmodeLastCycled = now;
            lastSyncPacket = now;
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