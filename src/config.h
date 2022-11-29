#pragma once

#include "user_config.h"

// which board are we using

// NB moved to platformio.ini:
// #define T_DISPLAY
// #define LONGAN_NANO
// #define PCB_V1_0
// #define RX_C3

// Simple test mode with slow lora signals to see if the chirps are measurable with hackrf
// #define LORA_TEST

// Caution, there's another def for this in platformio.ini
// #define USE_FLRC

#ifdef ESPC3
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif

//-------------------------------------------------
// Compile for TX or RX (Don't forget to select the right Hardware type as well!)
// #define IS_RECEIVER

#ifndef IS_RECEIVER
#define IS_TRANSMITTER
#endif
//-------------------------------------------------


#ifdef IS_TRANSMITTER
// For crsf tx module to openTX handset
#define UART_INVERTED
#endif

// XXX where's the best place for these?


// Hardware revision:

#define DUAL_BAND_BREADBOARD

// The first DB PCB: bare C3, modules for radios
// #define DUAL_BAND_PROTOTYPE

// DB pcb with no modules
// #define DB_PCB_V1

// This was for dual sx1280 with a c3 module on a PCB
// #define C3_PCB_V0

// ---------------------------

// PCB_V1 uses an ordinary XO for the 1262, the others (with E22 modules) have TCXO
#if !defined(DB_PCB_V1)

#define USE_SX1262_TCXO

#endif

// PCB_V1 test inverted txen. Looks to work better non-inverted
// #ifdef DB_PCB_V1
// #define SX1262_TXEN_INVERTED
// #endif


// Features

#define USE_SECOND_RADIO

// Enable the new DB specific packet formats
#define USE_DB_PACKETS

// Not sure where best to set this, but it uses a set of modes tuned for the fullD rx code
// Which sounds like an unnecessary thing anyway, so try and get rid of this long term
#define FULL_DIVERSITY_MODES


#ifdef RADIO_E22

    #define Regulatory_Domain_FCC_915

#endif // E22

#if defined(RADIO_E28_12) || defined(RADIO_E28_20) || defined(RADIO_E28_27)

#ifdef ELRS_OG_COMPATIBILITY
    #define Regulatory_Domain_ISM_2400
#else
    #define Regulatory_Domain_ISM_2400_NA
#endif

#endif // 2G4 radios


#ifdef RADIO_E28_12
// E28-12 and both GNICERF modules can use the full output range
#define MAX_PRE_PA_POWER 13
#define DISARM_POWER (0)
#elif defined(RADIO_E28_20)
#define MAX_PRE_PA_POWER (-2)
#define DISARM_POWER (-12)
#elif defined(RADIO_E28_27)
#define MAX_PRE_PA_POWER 0
#define DISARM_POWER (-15)
#else
#error "Must define a radio module to use"
#endif


#if defined(RADIO_E22)
#define MAX_PRE_PA_POWER_915 22  // sx1262 can be configured to scale the max power down from the commanded value
// #define MAX_PRE_PA_POWER_915 10     // while testing
#define DISARM_POWER_915 (-9)
#endif

// Not used by dual band
// how many switches do we have?
// hybrid 8 currently deals with up to 8 switches, but if we only have 4 fitted we can save some time by
// setting the lower value here
#define MAX_SWITCHES    4


// =====================

#ifdef LONGAN_NANO

// #define RADIO_BUSY_PORT GPIOA
// #define RADIO_BUSY_PIN  GPIO_PIN_11

// #define RADIO_RESET_PORT GPIOA
// #define RADIO_RESET_PIN GPIO_PIN_12

#define RADIO_BUSY_PORT GPIOA
#define RADIO_BUSY_PIN  GPIO_PIN_12

#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11

#define RADIO_RXEN_PORT GPIOB
#define RADIO_RXEN_PIN GPIO_PIN_11

#define RADIO_TXEN_PORT GPIOA
#define RADIO_TXEN_PIN GPIO_PIN_8


// rotary encoder button
// #define RE_BUTTON_PORT GPIOA
// #define RE_BUTTON_PIN GPIO_PIN_4

#define RE_BUTTON_PORT GPIOC
#define RE_BUTTON_PIN GPIO_PIN_13

// for testing on the nano
// #define SWB_TMP PA14

// #define SWA_LOW  PC15
// #define SWA_HIGH PC14

// #define SWB_LOW  PA13
// #define SWB_HIGH PA14

// #define SWC_LOW PA15
// #define SWC_HIGH PB3

// #define SWD_LOW  PB5
// #define SWD_HIGH PB4

// need adc scaling constants even if we don't have anything connected
// #define ADC_PITCH_REVERSED false
// #define ADC_PITCH_MIN 726u
// #define ADC_PITCH_CTR 2080u
// #define ADC_PITCH_MAX 3497u

// #define ADC_ROLL_REVERSED true
// #define ADC_ROLL_MIN 592u
// #define ADC_ROLL_CTR 2083u
// #define ADC_ROLL_MAX 3547u

// #define ADC_THROTTLE_REVERSED true
// #define ADC_THROTTLE_MIN 561u
// #define ADC_THROTTLE_MAX 3504u

// #define ADC_YAW_REVERSED false
// #define ADC_YAW_MIN 741u
// #define ADC_YAW_CTR 2081u
// #define ADC_YAW_MAX 3436u


#elif defined(T_DISPLAY)

#define RADIO_BUSY_PORT GPIOA
#define RADIO_BUSY_PIN  GPIO_PIN_12

#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11

#define RADIO_RXEN_PORT GPIOB
#define RADIO_RXEN_PIN GPIO_PIN_11

#define RADIO_TXEN_PORT GPIOA
#define RADIO_TXEN_PIN GPIO_PIN_8

// rotary encoder button
#define RE_BUTTON_PORT GPIOC
#define RE_BUTTON_PIN GPIO_PIN_13

// RC switches
#define SWA_LOW  PC15
#define SWA_HIGH PC14

// #define SWB_TMP PA13
#define SWB_LOW  PA13
#define SWB_HIGH PA14

#define SWC_LOW PA15
#define SWC_HIGH PB3

#define SWD_LOW  PB5
#define SWD_HIGH PB4


#elif defined(PCB_V1_0)

#define RADIO_BUSY_PORT GPIOA
#define RADIO_BUSY_PIN  GPIO_PIN_12

#define RADIO_BUSY  PA12


#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11

#define RADIO_RESET PA11


#define RADIO_RXEN_PORT GPIOB
#define RADIO_RXEN_PIN GPIO_PIN_11



#define RADIO_TXEN_PORT GPIOA
#define RADIO_TXEN_PIN GPIO_PIN_8

// rotary encoder button
#define RE_BUTTON_PORT GPIOC
#define RE_BUTTON_PIN GPIO_PIN_13


// Buzzer
#define GPIO_BUZZER PA4

#elif defined(RX_C3)

#include "driver/gpio.h"

#ifdef C3_PCB_V0

#define RADIO_RESET_PIN GPIO_NUM_4
#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_0
#define RADIO_SCK_PIN   GPIO_NUM_2
#define RADIO_NSS_PIN   GPIO_NUM_8
#define RADIO_BUSY_PIN  GPIO_NUM_7
#define RADIO_DIO1_PIN  GPIO_NUM_5
#define RADIO_DIO2_PIN  GPIO_NUM_19

#elif defined(DUAL_BAND_BREADBOARD)

// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_2
#define RADIO_SCK_PIN   GPIO_NUM_0

// These are for the sx1262

#define RADIO_RESET_PIN GPIO_NUM_12
#define RADIO_NSS_PIN   GPIO_NUM_8
#define RADIO_BUSY_PIN  GPIO_NUM_7
#define RADIO_DIO1_PIN  GPIO_NUM_10
// #define RADIO_DIO2_PIN  GPIO_NUM_19  pin shortage on the breadboard devkit
#define RADIO_TXEN_PIN    GPIO_NUM_19

// LQ improves with RXEN properly setup
#define RADIO_RXEN_PIN    GPIO_NUM_9

#elif defined(DUAL_BAND_PROTOTYPE)

// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_0
#define RADIO_SCK_PIN   GPIO_NUM_2

// These are for the sx1262, the sx1280 pins are later under "USE_SECOND_RADIO"

#define RADIO_RESET_PIN GPIO_NUM_12
#define RADIO_NSS_PIN   GPIO_NUM_8
#define RADIO_BUSY_PIN  GPIO_NUM_7
#define RADIO_DIO1_PIN  GPIO_NUM_10
// #define RADIO_DIO2_PIN  GPIO_NUM_19  XXX needs testing
#define RADIO_TXEN_PIN  GPIO_NUM_13

#elif defined(DB_PCB_V1)

// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_0
#define RADIO_SCK_PIN   GPIO_NUM_2

// These are for the sx1262, the sx1280 pins are later under "USE_SECOND_RADIO"

#define RADIO_RESET_PIN GPIO_NUM_6  // MTCK == 6
#define RADIO_NSS_PIN   GPIO_NUM_7  // MTDO == 7
#define RADIO_BUSY_PIN  GPIO_NUM_5  // MTDI == 5
#define RADIO_DIO1_PIN  GPIO_NUM_4  // MTMS == 4
// #define RADIO_DIO2_PIN  GPIO_NUM_3  XXX needs testing
#define RADIO_TXEN_PIN      GPIO_NUM_9  // XXX need to check if this is active high or active low

#else // these are for the dual 1280 breadboard prototype

#define RADIO_RESET_PIN GPIO_NUM_18
#define RADIO_MOSI_PIN  GPIO_NUM_5
#define RADIO_MISO_PIN  GPIO_NUM_4
#define RADIO_SCK_PIN   GPIO_NUM_6
#define RADIO_NSS_PIN   GPIO_NUM_7
#define RADIO_BUSY_PIN  GPIO_NUM_19
#define RADIO_DIO1_PIN  GPIO_NUM_10
#define RADIO_DIO2_PIN  GPIO_NUM_2  // might do without this if needed? // problematical pin, used for strapping

#endif // C3_PCB_V0

// These 2 are for sx1262
// #define RADIO_RXEN_PIN  GPIO_NUM_8  // try tying to 3v3
// #define RADIO_TXEN_PIN  GPIO_NUM_1  // maybe experiment with auto mode using dio2?



// second radio for full diversity.
// reset, mosi, miso and sck common with first radio

#ifdef USE_SECOND_RADIO

#ifdef C3_PCB_V0

#define RADIO2_NSS_PIN   GPIO_NUM_6
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_10
#define RADIO2_DIO2_PIN  GPIO_NUM_3

#elif defined(DUAL_BAND_BREADBOARD)

#define RADIO2_NSS_PIN   GPIO_NUM_6
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_5
#define RADIO2_DIO2_PIN  GPIO_NUM_3
#define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DUAL_BAND_PROTOTYPE)

#define RADIO2_NSS_PIN   GPIO_NUM_6
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_5
#define RADIO2_DIO2_PIN  GPIO_NUM_3
#define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DB_PCB_V1)

#define RADIO2_NSS_PIN   GPIO_NUM_8
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_10
#define RADIO2_DIO2_PIN  GPIO_NUM_19
#define RADIO2_RESET_PIN GPIO_NUM_12

#else // for dual 1280 breadboard prototype

// #define RADIO2_RESET_PIN GPIO_NUM_8
#define RADIO2_NSS_PIN   GPIO_NUM_1
#define RADIO2_BUSY_PIN  GPIO_NUM_12
#define RADIO2_DIO1_PIN  GPIO_NUM_3
#define RADIO2_DIO2_PIN  GPIO_NUM_0

#endif // C3_PCB_V0

#else // not using the second radio, but we still need the reset pin during init
#ifdef C3_PCB_V0

// XXX Where is the radio2 reset pin for this layout?

#elif defined(DUAL_BAND_BREADBOARD)

#define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DUAL_BAND_PROTOTYPE)

#define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DB_PCB_V1)

#define RADIO2_RESET_PIN GPIO_NUM_12

#else // for dual 1280 breadboard prototype

// need to check - did I move to shared reset for dual 1280?
// #define RADIO2_RESET_PIN GPIO_NUM_8

#endif // C3_PCB_V0



#endif // USE_SECOND_RADIO



#if defined(USE_PWM6)

#define PWM_CH1_PIN     GPIO_NUM_3
#define PWM_CH2_PIN     GPIO_NUM_0
#define PWM_CH3_PIN     GPIO_NUM_8  // strapping pin, should be ok for output
#define PWM_CH4_PIN     GPIO_NUM_9  // problematical pin, used for strapping
#define PWM_CH5_PIN     GPIO_NUM_20 // overlaps with the debug/flash uart. Maybe use 1 (after txen is auto)
#define PWM_CH6_PIN     GPIO_NUM_21 // overlaps with the debug/flash uart. Maybe use 2 (after disabling dio2)
// #define PWM_CH5_PIN     -1 //
// #define PWM_CH6_PIN     -1 //

#else // not using pwm


// For PCB proto
#ifdef C3_PCB_V0

#define LED2812_PIN     GPIO_NUM_9
#define CRSF_TX_PIN     GPIO_NUM_21

#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 1
#define LED_RADIO2_INDEX 2

#elif defined(DUAL_BAND_BREADBOARD)

// #define DEBUG_PIN     GPIO_NUM_9

// #define LED2812_PIN   GPIO_NUM_9
#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1

// #define LATENCY_INPUT_PIN GPIO_NUM_9

#ifdef IS_TRANSMITTER
// For the transmitter module, s.port pin:
// #define CRSF_SPORT_PIN     GPIO_NUM_9    // This was most recent in use. Disable to allow keyboard input for testing

// #define CRSF_SPORT_PIN     GPIO_NUM_19
// #define CRSF_SPORT_PIN     GPIO_NUM_20  // 20 is normally the uart0 rx pin
// #define CRSF_SPORT_PIN     GPIO_NUM_21  // 21 is normally the uart0 tx pin
#define DEBUG_RX_PIN    GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21
// #define DEBUG_TX_PIN    GPIO_NUM_19
#endif

#ifdef IS_RECEIVER
// #define CRSF_TX_PIN   GPIO_NUM_9
// #define CRSF_TX_PIN   GPIO_NUM_21
// #define CRSF_RX_PIN   GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21
#endif


#elif defined(DUAL_BAND_PROTOTYPE)

// #define DEBUG_PIN     GPIO_NUM_9
#define LED2812_PIN   GPIO_NUM_9
// #define LATENCY_INPUT_PIN GPIO_NUM_9

#ifdef IS_TRANSMITTER
// For the transmitter module, s.port pin: XXX maybe switch to pin 21 which has the safety resistor
// #define CRSF_SPORT_PIN     GPIO_NUM_3
// #define CRSF_SPORT_PIN     GPIO_NUM_9
#define CRSF_SPORT_PIN  GPIO_NUM_21
#define DEBUG_TX_PIN    GPIO_NUM_20
// #define DEBUG_TX_PIN    GPIO_NUM_21
#endif // IS_TRANSMITTER

#ifdef IS_RECEIVER
#define CRSF_TX_PIN   GPIO_NUM_21
// #define CRSF_RX_PIN   GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21     // shared with crsf :(

#endif // IS_RECEIVER

#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1

#elif defined(DB_PCB_V1)

#define LED2812_PIN   GPIO_NUM_13

#ifdef IS_TRANSMITTER
// For the transmitter module, s.port pin: XXX maybe switch to pin 21 which has the safety resistor
// #define CRSF_SPORT_PIN     GPIO_NUM_3
// #define CRSF_SPORT_PIN     GPIO_NUM_9
// #define CRSF_SPORT_PIN  GPIO_NUM_21
// #define DEBUG_TX_PIN    GPIO_NUM_20
#define CRSF_SPORT_PIN  GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21
#endif // IS_TRANSMITTER

#ifdef IS_RECEIVER
// disabling CRSF for development
// #define CRSF_TX_PIN   GPIO_NUM_21
// #define CRSF_RX_PIN   GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21     // shared with crsf :(

#endif // IS_RECEIVER

#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1


#else

// #define DEBUG_PIN     GPIO_NUM_9
// #define CRSF_TX_PIN     GPIO_NUM_9

#define LED2812_PIN     GPIO_NUM_8

#define LED_RADIO1_INDEX 0
#define LED_RADIO2_INDEX 1

#endif // C3_PCB_V0


#endif // USE_PWM6

#else
#error "define the board type in config.h"
#endif // Board type


#ifndef RX_C3

// common to all handset boards

#define RADIO_NSS_PORT GPIOB
#define RADIO_NSS_PIN GPIO_PIN_12

#define RADIO_NSS PB12


#define RADIO_DIO1_PORT GPIOB
#define RADIO_DIO1_PIN GPIO_PIN_8


#define RADIO_DIO2_PORT GPIOB
#define RADIO_DIO2_PIN GPIO_PIN_9

#endif // !RX_C3
