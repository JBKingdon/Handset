// Board defines for the dual band breadboard prototypes

#define RADIO_E28_12    // CAREFUL - this will break _20 and _27 if you use it by accident

// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_2
#define RADIO_SCK_PIN   GPIO_NUM_0

// These are for the sx1262

#define RADIO_RESET_PIN GPIO_NUM_9
#define RADIO_NSS_PIN   GPIO_NUM_8
#define RADIO_BUSY_PIN  GPIO_NUM_7
#define RADIO_DIO1_PIN  GPIO_NUM_10
// #define RADIO_DIO2_PIN  GPIO_NUM_19  pin shortage on the breadboard devkit
#define RADIO_TXEN_PIN    GPIO_NUM_19

// LQ improves with RXEN properly setup - or does it? Needs retesting
// #define RADIO_RXEN_PIN    GPIO_NUM_9

// SX1280

#define RADIO2_NSS_PIN   GPIO_NUM_6
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_5
#define RADIO2_DIO2_PIN  GPIO_NUM_3
#define RADIO2_RESET_PIN GPIO_NUM_4


// #define DEBUG_PIN     GPIO_NUM_12
// #define LED2812_PIN   GPIO_NUM_12
#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1

// #define LATENCY_INPUT_PIN GPIO_NUM_9

#ifdef IS_TRANSMITTER
// For the transmitter module, s.port pin:

// NB pin 9 is a bad choice for s.port, don't use it. It has a relatively strong pull-up for the boot button that overrides the weak pulldown on the gpio

// #define CRSF_SPORT_PIN     GPIO_NUM_12  // swapped from RADIO_RESET_PIN, use this

// #define DEBUG_RX_PIN    GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21
// #define DEBUG_TX_PIN    GPIO_NUM_19
#endif

#ifdef IS_RECEIVER
// #define CRSF_TX_PIN   GPIO_NUM_9
// #define CRSF_RX_PIN   GPIO_NUM_20
// #define CRSF_TX_PIN   GPIO_NUM_21
#define DEBUG_RX_PIN    GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21
#endif


// XXX This doesn't make much sense, need to fix and so that dev_mode and port nums are independent and unambiguous

// Known working combos:
//   DEV_MODE off, DEBUG_RX_PIN not defined, CRSF_SPORT_PIN defined
//   DEV_MODE on, DEBUG_RX_PIN defined, CRSF_SPORT_INT not defined

#ifdef DEV_MODE
// The breadboard prototype can use uart1 with pin 12 for crossfire s.port
#define CRSF_PORT_NUM 1
#else
// use port 0 for crsf and port 1 for dev with keyboard input to change modes
#define CRSF_PORT_NUM 0 
#endif
