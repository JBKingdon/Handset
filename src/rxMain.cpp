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

// #include "ElrsSPI.h"

#include "SX1262Driver.h"

#include "HwTimer.h"

// defs for SPI

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10

#define PIN_NUM_RST  4

SX1262Driver radio;

uint32_t timeoutCounter = 0;
uint32_t mismatchCounter = 0;
uint32_t totalPackets = 0;
int32_t cumulativeRSSI = 0;

static xQueueHandle rx_evt_queue = NULL;
static xQueueHandle tx_evt_queue = NULL;


// XXX TODO move this somewhere sensible
void delay(uint32_t millis)
{
    if (millis < 30) {
        esp_rom_delay_us(millis);
    } else {
        vTaskDelay(millis / portTICK_PERIOD_MS);
    }
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
                // radio.ClearIrqStatus(SX1262_IRQ_RX_TX_TIMEOUT);
                timeoutCounter++;
                // for receiver, need to start another rx
                radio.RXnb();

                // return; // EARLY RETURN
                continue;
            }


            // XXX testing, needed for continuous receive mode
            // radio.ClearIrqStatus(SX1280_IRQ_RADIO_ALL);

            // gpio_bit_reset(LED_GPIO_PORT, LED_PIN);  // rx has finished, so clear the debug pin

            // TODO move to the event loop?
            radio.readRXData(); // get the data from the radio chip

            // XXX testing
            // uint8_t counter = radio.RXdataBuffer[0];

            // int8_t rssi = radio.GetLastPacketRSSI();
            // int8_t snr = radio.GetLastPacketSNR();

            // printf("packet counter %u, rssi %d snr %d\n\r", counter, rssi, snr);
            // printf("packet counter %u, rssi %d\n\r", counter, rssi);

            // delayMicros(50);

            // testing. If this is the transmitter we should validate the data then set the flag for the next tx
            // if this is the receiver we need to echo the packet back

            // send the echo
            // printf("sending echo\n\r");
            radio.TXnb(radio.RXdataBuffer, OTA_PACKET_LENGTH);
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
   if (index == 0) { // special case FLRC for testing
      radio.ConfigFLRC(GetInitialFreq());
   } else 
   #endif
   {
      radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, GetInitialFreq(), ModParams->PreambleLen, invertIQ);
   }

    printf("in SetRFLinkRate, setTimerISRInterval disabled\n");
//    setTimerISRInterval(ModParams->interval);

   ExpressLRS_currAirRate_Modparams = ModParams;
   ExpressLRS_currAirRate_RFperfParams = RFperf;

    printf("in SetRFLinkRate, isRXconnected disabled\n");
//    isRXconnected = false;

   #ifdef USE_DYNAMIC_POWER
   // The dynamic power thresholds are relative to the rx sensitivity, so need to be updated
   dynamicPowerRSSIIncreaseThreshold = RFperf->RXsensitivity + DYN_POWER_INCREASE_MARGIN;
   dynamicPowerRSSIDecreaseThreshold = dynamicPowerRSSIIncreaseThreshold + DYN_POWER_DECREASE_MARGIN;
   #endif

   // if (UpdateRFparamReq)
   //    UpdateRFparamReq = false;

}


void tick()
{
    static unsigned long last = 0;

    unsigned long now = micros();

    if (last == 0) {
        last = now;
    } else {

        printf("tick %lu\n", (now-last));
        last = now;
    }
}

void tock()
{
    printf("tock\n");
}


extern "C"
{

void app_main()
{
    printf("Hello World!\n");

    HwTimer::init();
    HwTimer::setInterval(2000000);

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

    delay(50);
    printf("setting FS\n\r");
    radio.SetMode(SX1262_MODE_FS);
    delay(50);
    radio.GetStatus();

    // XXX For testing, just go into a continuous loop here

    // fill the packet with some non-zero values
    radio.TXdataBuffer[0] = 0;
    for (int i = 1; i < OTA_PACKET_LENGTH; i++)
        radio.TXdataBuffer[i] = i + 127;


    radio.setRxTimeout(1000000 / 15); // XXX move the scaling inside the setRxTimeout function

    // start the receiver
    printf("calling RXnb\n\r");
    radio.RXnb();
    radio.GetStatus();


    unsigned long tPrev = millis();
    uint32_t lastTotalP = 0;

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

} // extern "C"