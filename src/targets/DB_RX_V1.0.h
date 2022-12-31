// Board defines for the dual band receiver V1.0

// Not actually an E28, but equivalent for power output
#define RADIO_E28_12    // CAREFUL - this will break _20 and _27 if you use it by accident

// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_0
#define RADIO_SCK_PIN   GPIO_NUM_2


// These are for the sx1262

#define RADIO_RESET_PIN GPIO_NUM_6  // MTCK == 6
#define RADIO_NSS_PIN   GPIO_NUM_7  // MTDO == 7
#define RADIO_BUSY_PIN  GPIO_NUM_5  // MTDI == 5
#define RADIO_DIO1_PIN  GPIO_NUM_4  // MTMS == 4
// #define RADIO_DIO2_PIN  GPIO_NUM_3  XXX needs testing
#define RADIO_TXEN_PIN      GPIO_NUM_9  // XXX need to check if this is active high or active low


// For the sx1280

#define RADIO2_NSS_PIN   GPIO_NUM_8
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_10
#define RADIO2_DIO2_PIN  GPIO_NUM_19
#define RADIO2_RESET_PIN GPIO_NUM_12

#define LED2812_PIN   GPIO_NUM_13
#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1


// UART

#ifdef IS_TRANSMITTER
#define CRSF_SPORT_PIN  GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21
#endif // IS_TRANSMITTER

#ifdef IS_RECEIVER
// Comment these out to disable CRSF for development
#define CRSF_TX_PIN   GPIO_NUM_21
#define CRSF_RX_PIN   GPIO_NUM_20
// #define DEBUG_TX_PIN    GPIO_NUM_21     // shared with crsf :(
#endif // IS_RECEIVER

