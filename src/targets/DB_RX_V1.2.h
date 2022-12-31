// Board defines for the dual band receiver V1.2

// Not actually an E28, but equivalent for power output
#define RADIO_E28_12

// V1.2 has dio2 used as txen for the rf switch
#define SX1262_DIO2_IS_TXEN


// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_0
#define RADIO_SCK_PIN   GPIO_NUM_7  // MTDO == 7


// These are for the sx1262

#define RADIO_RESET_PIN GPIO_NUM_6  // MTCK == 6
#define RADIO_NSS_PIN   GPIO_NUM_2
#define RADIO_BUSY_PIN  GPIO_NUM_5  // MTDI == 5
#define RADIO_DIO1_PIN  GPIO_NUM_4  // MTMS == 4
// #define RADIO_DIO3_PIN  GPIO_NUM_3  XXX needs testing
// TXEN connected to DIO2


// For the sx1280

#define RADIO2_NSS_PIN   GPIO_NUM_8
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_10
#define RADIO2_DIO2_PIN  GPIO_NUM_19
#define RADIO2_RESET_PIN GPIO_NUM_12

#define LED2812_PIN   GPIO_NUM_9
#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1


// UART

#if defined(IS_TRANSMITTER)
    #define CRSF_SPORT_PIN  GPIO_NUM_20
    #define DEBUG_TX_PIN    GPIO_NUM_21

#elif defined(IS_RECEIVER)
    #ifdef DEV_MODE
    #define DEBUG_TX_PIN    GPIO_NUM_21     // shared with crsf :(
    #else
    // Comment these out to disable CRSF for development
    #define CRSF_TX_PIN   GPIO_NUM_21
    #define CRSF_RX_PIN   GPIO_NUM_20
    #endif // DEV_MODE

#else
    #error "neither IS_TRANSMITTER nor IS_RECEIVER are defined?"

#endif // IS_TRANSMITTER
