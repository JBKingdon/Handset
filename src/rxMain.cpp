/**
 * Initial test code for C3 based receivers
 * 
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"

#include "freertos/queue.h"

// #include "user_config.h"
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


// defs for SPI

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10

#define PIN_NUM_RST  4

#define PACKET_TO_TOCK_SLACK 200 // Desired buffer time between Packet ISR and Tock ISR


SX1262Driver radio;
CRSF crsf;
GENERIC_CRC14 ota_crc(ELRS_CRC14_POLY);
PFD pfdLoop;

uint8_t ExpressLRS_nextAirRateIndex = 0;


uint32_t timeoutCounter = 0;
uint32_t mismatchCounter = 0;
uint32_t totalPackets = 0;
int32_t cumulativeRSSI = 0;
uint32_t crcCounter = 0;
uint32_t errCounter = 0;

static xQueueHandle rx_evt_queue = NULL;
static xQueueHandle tx_evt_queue = NULL;

volatile uint8_t NonceRX = 0; // nonce that we THINK we are up to.

bool alreadyFHSS = false;
bool alreadyTLMresp = false;

uint32_t beginProcessing;
uint32_t doneProcessing;

uint32_t lastValidPacket = 0;           //Time the last valid packet was recv
uint32_t lastSyncPacket = 0;            //Time the last valid packet was recv

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


/// LQ Calculation //////////
LQCALC<100> LQCalc;
uint8_t uplinkLQ;

uint8_t scanIndex = RATE_DEFAULT;


uint8_t antenna = 0;    // which antenna is currently in use

bool isRXconnected = false;

#if defined(PRINT_RX_SCOREBOARD)
static bool lastPacketCrcError;
#endif


// XXX TODO move this somewhere sensible
void delay(uint32_t millis)
{
    if (millis < 30) {
        esp_rom_delay_us(millis);
    } else {
        vTaskDelay(millis / portTICK_PERIOD_MS);
    }
}

void ICACHE_RAM_ATTR getRFlinkInfo()
{

    int32_t rssiDBM0 = LPF_UplinkRSSI0.SmoothDataINT;
    int32_t rssiDBM1 = LPF_UplinkRSSI1.SmoothDataINT;
    switch (antenna) {
        case 0:
            rssiDBM0 = LPF_UplinkRSSI0.update(radio.GetLastPacketRSSI());
            break;
        case 1:
            rssiDBM1 = LPF_UplinkRSSI1.update(radio.GetLastPacketRSSI());
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
    crsf.LinkStatistics.uplink_SNR = radio.LastPacketSNR;
    crsf.LinkStatistics.uplink_Link_quality = uplinkLQ;
    crsf.LinkStatistics.rf_Mode = (uint8_t)RATE_4HZ - (uint8_t)ExpressLRS_currAirRate_Modparams->enum_rate;
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

#if WS2812_LED_IS_USED
    uint8_t LEDcolor[3] = {0};
    LEDcolor[(2 - ExpressLRS_currAirRate_Modparams->index) % 3] = 50;
    WS281BsetLED(LEDcolor);
    LEDWS2812LastUpdate = millis();
#endif

    // The caller MUST call hwTimer.resume(). It is not done here because
    // the timer ISR will fire immediately and preempt any other code
}


void ICACHE_RAM_ATTR ProcessRFPacket()
{
    beginProcessing = micros();

    uint8_t type = radio.RXdataBuffer[0] & 0b11;

    uint16_t inCRC = ( ( (uint16_t)(radio.RXdataBuffer[0] & 0b11111100) ) << 6 ) | radio.RXdataBuffer[7];

    radio.RXdataBuffer[0] = type;
    uint16_t calculatedCRC = ota_crc.calc(radio.RXdataBuffer, 7);

    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", radio.RXdataBuffer[i]);
    // printf(", inCRC %04X calcCRC %04X\n", inCRC, calculatedCRC);

    if (inCRC != calculatedCRC)
    {
        #ifndef DEBUG_SUPPRESS
        // printf("CRC error on RF packet: ");
        // for (int i = 0; i < 8; i++)
        // {
        //     printf("%02X ", radio.RXdataBuffer[i]);
        // }
        // printf("\n");
        #endif
        #if defined(PRINT_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        crcCounter++;
        return;
    }
    pfdLoop.extEvent(beginProcessing + PACKET_TO_TOCK_SLACK);

#ifdef HYBRID_SWITCHES_8
    uint8_t SwitchEncModeExpected = 0b01;
#else
    uint8_t SwitchEncModeExpected = 0b00;
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

    getRFlinkInfo();

    switch (type)
    {
    case RC_DATA_PACKET: //Standard RC Data Packet
        #ifdef USE_HIRES_DATA
        UnpackHiResChannelData(radio.RXdataBuffer, &crsf);
        #else
        UnpackChannelDataHybridSwitches8(radio.RXdataBuffer, &crsf);
        #endif

        #ifdef ENABLE_TELEMETRY
        telemetryConfirmValue = Radio.RXdataBuffer[6] & (1 << 7);
        TelemetrySender.ConfirmCurrentPayload(telemetryConfirmValue);
        #endif
        
        if (connectionState != disconnected)
        {
            crsf.sendRCFrameToFC();
        }
        break;

    // case MSP_DATA_PACKET:
    //     currentMspConfirmValue = MspReceiver.GetCurrentConfirm();
    //     MspReceiver.ReceiveData(Radio.RXdataBuffer[1], Radio.RXdataBuffer + 2);
    //     if (currentMspConfirmValue != MspReceiver.GetCurrentConfirm())
    //     {
    //         NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
    //     }

    //     if (Radio.RXdataBuffer[1] == 1 && MspData[0] == MSP_ELRS_BIND)
    //     {
    //         OnELRSBindMSP(MspData);
    //         MspReceiver.ResetState();
    //     }
    //     else if (MspReceiver.HasFinishedData())
    //     {
    //         crsf.sendMSPFrameToFC(MspData);
    //         MspReceiver.Unlock();
    //     }
    //     break;

    case TLM_PACKET: //telemetry packet from master

        // not implimented yet
        break;

    case SYNC_PACKET: //sync packet from master
         indexIN = (radio.RXdataBuffer[3] & 0b11000000) >> 6;
         TLMrateIn = (radio.RXdataBuffer[3] & 0b00111000) >> 3;
         SwitchEncMode = (radio.RXdataBuffer[3] & 0b00000110) >> 1;

         if (SwitchEncModeExpected == SwitchEncMode && radio.RXdataBuffer[4] == UID[3] && radio.RXdataBuffer[5] == UID[4] && radio.RXdataBuffer[6] == UID[5])
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

             if (ExpressLRS_currAirRate_Modparams->index != (expresslrs_tlm_ratio_e)indexIN)
             { // change link parameters if required
                ExpressLRS_nextAirRateIndex = indexIN;
                printf("changing index from %d to %u\n", ExpressLRS_currAirRate_Modparams->index, indexIN);
             }

             if (connectionState == disconnected
                || NonceRX != radio.RXdataBuffer[2]
                || FHSSgetCurrIndex() != radio.RXdataBuffer[1]-1) // XXX don't forget this one when fixing the offsets
             {
                 //Serial.print(NonceRX, DEC); Serial.write('x'); Serial.println(Radio.RXdataBuffer[2], DEC);
                 FHSSsetCurrIndex(radio.RXdataBuffer[1]-1); // XXX take the +1 off the transmitter and stop messing around
                 NonceRX = radio.RXdataBuffer[2];
                 TentativeConnection();
                 doStartTimer = true;
             }
         } else {
             printf("rejected sync pkt\n");
         }
         break;

    default:
        break;
    }

    LQCalc.add(); // Received a packet, that's the definition of LQ
    // Extend sync duration since we've received a packet at this rate
    // but do not extend it indefinitely
    RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

    doneProcessing = micros();
#if defined(PRINT_RX_SCOREBOARD)
    if (type != SYNC_PACKET) Serial.write('R');
#endif
    if (doStartTimer)
        HwTimer::resume(); // will throw an interrupt immediately
}

bool ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t modresultFHSS = (NonceRX + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || alreadyFHSS == true || (modresultFHSS != 0) || (connectionState == disconnected))
    {
        return false;
    }

    alreadyFHSS = true;
    uint32_t freq = FHSSgetNextFreq();
    radio.SetFrequency(freq);

    // printf("f %u\n", freq);

    // uint8_t modresultTLM = (NonceRX + 1) % (TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval));

    // if (modresultTLM != 0 || ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) // if we are about to send a tlm response don't bother going back to rx
    // {
    //     radio.RXnb();
    // }
    return true;
}

bool ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    #ifdef ENABLE_TELEMETRY
    uint8_t *data;
    uint8_t maxLength;
    uint8_t packageIndex;
    #endif
    // uint8_t modresult = (NonceRX) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);
    uint8_t modresult = (NonceRX + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate_Modparams->TLMinterval);

    if ((connectionState == disconnected) || (ExpressLRS_currAirRate_Modparams->TLMinterval == TLM_RATIO_NO_TLM) || (alreadyTLMresp == true) || (modresult != 0))
    {
        return false; // don't bother sending tlm if disconnected or TLM is off
    }

    alreadyTLMresp = true;
    radio.TXdataBuffer[0] = 0b11; // tlm packet

    radio.TXdataBuffer[1] = 1; // ELRS_TELEMETRY_TYPE_LINK; XXX 

    // OpenTX RSSI as -dBm is fine and supports +dBm values as well
    // but the value in linkstatistics is "positivized" (inverted polarity)
    radio.TXdataBuffer[2] = -crsf.LinkStatistics.uplink_RSSI_1;
    radio.TXdataBuffer[3] = -crsf.LinkStatistics.uplink_RSSI_2;
    radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
    radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;
    // radio.TXdataBuffer[6] = MspReceiver.GetCurrentConfirm() ? 1 : 0; XXX

    // switch (NextTelemetryType)
    // {
    //     case ELRS_TELEMETRY_TYPE_LINK:
    //         #ifdef ENABLE_TELEMETRY
    //         NextTelemetryType = ELRS_TELEMETRY_TYPE_DATA;
    //         // Start the count at 1 because the next will be DATA and doing +1 before checking
    //         // against Max below is for some reason 10 bytes more code
    //         telemetryBurstCount = 1;
    //         #else
    //         NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
    //         #endif
    //         Radio.TXdataBuffer[1] = ELRS_TELEMETRY_TYPE_LINK;

    //         // OpenTX RSSI as -dBm is fine and supports +dBm values as well
    //         // but the value in linkstatistics is "positivized" (inverted polarity)
    //         Radio.TXdataBuffer[2] = -crsf.LinkStatistics.uplink_RSSI_1;
    //         Radio.TXdataBuffer[3] = -crsf.LinkStatistics.uplink_RSSI_2;
    //         Radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
    //         Radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;
    //         Radio.TXdataBuffer[6] = MspReceiver.GetCurrentConfirm() ? 1 : 0;

    //         break;
    //     #ifdef ENABLE_TELEMETRY
    //     case ELRS_TELEMETRY_TYPE_DATA:
    //         if (telemetryBurstCount < telemetryBurstMax)
    //         {
    //             telemetryBurstCount++;
    //         }
    //         else
    //         {
    //             NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
    //         }

    //         TelemetrySender.GetCurrentPayload(&packageIndex, &maxLength, &data);
    //         Radio.TXdataBuffer[1] = (packageIndex << ELRS_TELEMETRY_SHIFT) + ELRS_TELEMETRY_TYPE_DATA;
    //         Radio.TXdataBuffer[2] = maxLength > 0 ? *data : 0;
    //         Radio.TXdataBuffer[3] = maxLength >= 1 ? *(data + 1) : 0;
    //         Radio.TXdataBuffer[4] = maxLength >= 2 ? *(data + 2) : 0;
    //         Radio.TXdataBuffer[5] = maxLength >= 3 ? *(data + 3): 0;
    //         Radio.TXdataBuffer[6] = maxLength >= 4 ? *(data + 4): 0;
    //         break;
    //     #endif
    // }

    uint16_t crc = ota_crc.calc(radio.TXdataBuffer, 7);
    radio.TXdataBuffer[0] |= (crc >> 6) & 0b11111100;
    radio.TXdataBuffer[7] = crc & 0xFF;

    radio.TXnb(radio.TXdataBuffer, 8);
    // printf("telem sent ");
    // for(int i=0; i<OTA_PACKET_LENGTH; i++) printf("%02X ", radio.TXdataBuffer[i]);
    // printf("crc %04X\n", crc);
    return true;
}


static void tx_task(void* arg)
{
    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(tx_evt_queue, &dummyData, portMAX_DELAY))
        {
            // start the next receive
            radio.RXnb();
        }
    }
}

static void rx_task(void* arg)
{
    uint8_t dummyData;
    for(;;) {
        if(xQueueReceive(rx_evt_queue, &dummyData, portMAX_DELAY)) {

            // Need to check for timeout, increment a counter, set the readyTo flag if this is the transmitter
            uint16_t irqS = radio.GetIrqStatus();

            if (irqS & SX1262_IRQ_RX_TX_TIMEOUT)
            {
                timeoutCounter++;
                // need to start another rx
                radio.RXnb();

                continue;
            }

            // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // rx has finished, so clear the debug pin


            radio.readRXData(); // get the data from the radio chip

            ProcessRFPacket();

            // start the next receive
            radio.RXnb();

            // XXX testing
            // uint8_t counter = radio.RXdataBuffer[0];

            // int8_t rssi = radio.GetLastPacketRSSI();
            // int8_t snr = radio.GetLastPacketSNR();

            // printf("packet counter %u, rssi %d snr %d\n\r", counter, rssi, snr);
            // printf("packet counter %u, rssi %d\n\r", counter, rssi);

            // send the echo
            // printf("sending echo\n\r");
            // radio.TXnb(radio.RXdataBuffer, OTA_PACKET_LENGTH);
            totalPackets++;            
        }
    }
}


static void IRAM_ATTR dio1_isr_handler(void* arg)
{
    BaseType_t taskWoken = 0;

    xQueueGiveFromISR(rx_evt_queue, &taskWoken); 

    if (taskWoken) portYIELD_FROM_ISR();
}

static void IRAM_ATTR dio2_isr_handler(void* arg)
{
    BaseType_t taskWoken = 0;

    xQueueGiveFromISR(tx_evt_queue, &taskWoken); 

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
        radio.ConfigFLRC(GetInitialFreq());
    }
    else
#endif
    {
        radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
    }

    // printf("in SetRFLinkRate, setTimerISRInterval disabled\n");
    //    setTimerISRInterval(ModParams->interval);
    HwTimer::setInterval(ModParams->interval);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;

    isRXconnected = false;

#ifdef USE_DYNAMIC_POWER
    // The dynamic power thresholds are relative to the rx sensitivity, so need to be updated
    dynamicPowerRSSIIncreaseThreshold = RFperf->RXsensitivity + DYN_POWER_INCREASE_MARGIN;
    dynamicPowerRSSIDecreaseThreshold = dynamicPowerRSSIIncreaseThreshold + DYN_POWER_DECREASE_MARGIN;
#endif

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

void tick()
{
    // static unsigned long last = 0;
    // unsigned long now = micros();

    // if (last != 0) {
    //     printf("tick %lu\n", (now-last));
    // }
    // last = now;

    updatePhaseLock();
    NonceRX++;

    // This is commented out in upstream

    // if (!alreadyTLMresp && !alreadyFHSS && !LQCalc.currentIsSet()) // packet timeout AND didn't DIDN'T just hop or send TLM
    // {
    //     Radio.RXnb(); // put the radio cleanly back into RX in case of garbage data
    // }

    // Save the LQ value before the inc() reduces it by 1
    uplinkLQ = LQCalc.getLQ();
    // Only advance the LQI period counter if we didn't send Telemetry this period
    if (!alreadyTLMresp)
        LQCalc.inc();

    alreadyTLMresp = false;
    alreadyFHSS = false;

    // XXX implement output
    // crsf.RXhandleUARTout();

}

void tock()
{
    // printf("tock\n");
    pfdLoop.intEvent(micros()); // our internal osc just fired

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
}

void pingPongTest()
{
    unsigned long tPrev = millis();
    uint32_t lastTotalP = 0;

    radio.setRxTimeout(1000000 / 15); // XXX move the scaling inside the setRxTimeout function

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
        // radio.GetStatus();
   }
}

/** docs?
 */
void lostConnection()
{
    printf("lost conn fc=%d fo= ", FreqCorrection);
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

    while(micros() - pfdLoop.getIntEventTime() > 250); // time it just after the tock()
    HwTimer::stop();

    // XXX These two are potential SPI collisions if there might still be activity on the rx/tx tasks
    SetRFLinkRate(ExpressLRS_nextAirRateIndex); // also sets to initialFreq 
    radio.RXnb();


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
        radio.RXnb();

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

extern "C"
{

void app_main()
{
    printf("ESP32-C3 FreeRTOS RX\n");

    HwTimer::init();
    HwTimer::setInterval(2000000);
    HwTimer::stop();

    HwTimer::setCallbackTick(tick);
    HwTimer::setCallbackTock(tock);

    radio.currFreq = GetInitialFreq();
    radio.Begin();

    radio.SetOutputPower(DISARM_POWER);
    
    //create a queue to handle gpio event from isr
    rx_evt_queue = xQueueCreate(10, 0);
    tx_evt_queue = xQueueCreate(10, 0);
    //start tasks
    xTaskCreate(rx_task, "rx_task", 2048, NULL, 10, NULL); // Tpriority 1=min, max is ??? Default main task is pri 1
    xTaskCreate(tx_task, "tx_task", 2048, NULL,  5, NULL); // Tpriority 1=min, max is ??? Default main task is pri 1


    // need to attach an ISR handler to DIO1 and 2

    gpio_set_intr_type(RADIO_DIO1_PIN, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(RADIO_DIO2_PIN, GPIO_INTR_POSEDGE);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    //hook isr handler for specific gpio pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO1_PIN, dio1_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO2_PIN, dio2_isr_handler, NULL));

    uint8_t linkRateIndex = 0;
    SetRFLinkRate(linkRateIndex);

    // delay(50);
    // printf("setting FS\n\r");
    // radio.SetMode(SX1262_MODE_FS);
    // delay(50);
    // radio.GetStatus();

    // What's a reasonable timeout? What will we do with the event?
    radio.setRxTimeout(100000 / 15); // XXX move the scaling inside the setRxTimeout function

    FHSSrandomiseFHSSsequence();


    radio.RXnb();
    crsf.Begin();
    // hwTimer.init();
    // hwTimer.stop();

    unsigned long lastDebug = 0;

    // loop
    while(true) {
        // things that don't have to happen very quickly
        delay(100);
        unsigned long now = millis();
        if (lastDebug + 2000 < now)
        {
            uint32_t elapsedT = now - lastDebug;
            lastDebug = now;
            printf("Total packets %u, LQ %u, timeouts/s %u crcErrors/s %u\n", totalPackets, uplinkLQ, timeoutCounter*1000/elapsedT, crcCounter*1000/elapsedT);
            // printf("Offset %4d DX %4d freqOffset %d\n", Offset, OffsetDx, HwTimer::getFreqOffset());
            // totalPackets = 0;
            timeoutCounter = 0;
            crcCounter = 0;
        }

        // This isn't great to do from here, we might get an SPI collision with the still active rx/tx tasks
        if ((connectionState != disconnected) && (ExpressLRS_nextAirRateIndex != ExpressLRS_currAirRate_Modparams->index)){ // forced change
            lostConnection();
            lastSyncPacket = now;           // reset this variable to stop rf mode switching and add extra time
            RFmodeLastCycled = now;         // reset this variable to stop rf mode switching and add extra time
            printf("Air rate change req via sync\n");
            crsf.sendLinkStatisticsToFC();
            crsf.sendLinkStatisticsToFC(); // need to send twice, not sure why, seems like a BF bug?
            continue; // no point looking at all the other cases in the loop after this
        }


        // again this is potentially dangerous if rx may still be in progress
        if (connectionState == tentative && (millis() - lastSyncPacket > ExpressLRS_currAirRate_RFperfParams->RFmodeCycleAddtionalTime))
        {
            lostConnection();
            printf("Bad sync, aborting\n");
            RFmodeLastCycled = millis();
            lastSyncPacket = millis();
        }

        // check if we lost conn.
        uint32_t localLastValidPacket = lastValidPacket; // Required to prevent race condition due to LastValidPacket getting updated from ISR
        if ((connectionState == connected) && ((int32_t)ExpressLRS_currAirRate_RFperfParams->RFmodeCycleInterval < (int32_t)(millis() - localLastValidPacket)))
        {
            lostConnection();
        }

        // detects when we are connected
        if ((connectionState == tentative) && (abs(OffsetDx) <= 10) && (Offset < 100) && (uplinkLQ > minLqForChaos()))
        {
            gotConnection();
        }

        // If the link is looking stable we can move the timer to locked mode (and start doing loop freq compensation)
        if ((RXtimerState == tim_tentative) && ((millis() - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(OffsetDx) <= 5))
        {
            RXtimerState = tim_locked;
            // #ifndef DEBUG_SUPPRESS
            printf("Timer Locked\n");
            // #endif
        }


        cycleRfMode();


    }

}

} // extern "C"