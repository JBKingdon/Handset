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

// Instead of trying to make edgeTX match the packet rate, just run it at a fixed high rate
#define USE_MODULE_FIXED_PKT_RATE

#ifdef ESPC3
#include "esp_attr.h" // for IRAM_ATTR
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif // ESPC3

//-------------------------------------------------
// Compile for TX or RX (Don't forget to select the right Hardware type as well!)
// #define IS_RECEIVER

#ifndef IS_RECEIVER
#define IS_TRANSMITTER
#endif
//-------------------------------------------------

// Use CH8 to set packet rate
#define RATE_ON_CHANNEL

// Reduces output power (but may be configured out for tx), enables debug - what else?
// enables packet rate change by keyboard?
// does it suppress packet rate change by channel?
// #define DEV_MODE

// Send tx power for both 915 and 2G4. If not set, only send 915 power
// #define SEND_BOTH_TX_POWERS

// Hardware revision:

// #define DUAL_BAND_BREADBOARD

// The first DB PCB: bare C3, modules for radios
// #define DUAL_BAND_PROTOTYPE

// DB pcb with no modules
// #define DB_PCB_V1_0

// new version of the above with tcxo for 1280, lpf for 915 and dio2 as txen on the 1262
// #define DB_PCB_V1_2

// DB specific TX module with e28-27
#define DB_TX_V1


// This was for dual sx1280 with a c3 module on a PCB
// #define C3_PCB_V0

// ---------------------------

// figure out how to make this board specific without messing up pfdOffsets
// sx1262 is max 16MHz, and at the moment we use the same speed for both radios
// Looks like changing the SPI speed changes the timing so needs different pfd offsets
// Telemetry LQ goes down the toilet at 16MHz for some packet rates
#define SPI_CLOCK_SPEED (12*1000*1000)

#ifdef DUAL_BAND_BREADBOARD

    // for testing llcc68 - need to know because it doesn't have tcxo
    // currently being used on the transmitter board
    #if defined(IS_TRANSMITTER)
    #define E220
    #else
    #define E22
    #endif

    #include "targets/DB_BREADBOARD.h"

#elif defined(DB_PCB_V1_0)

    #include "targets/DB_RX_V1.0.h"

#elif defined(DB_PCB_V1_2)

    #include "targets/DB_RX_V1.2.h"

#endif

// The E22 module uses a tcxo. E220 and my bare 1262 pcbs don't
// TODO refactor DUAL_BAND_PROTOTYPE and define E22
#if defined(E22) || defined(DUAL_BAND_PROTOTYPE) || defined(DB_TX_V1)

    #define USE_SX1262_TCXO

#endif

// PCB_V1 test inverted txen. Looks to work better non-inverted
// #ifdef DB_PCB_V1_0
// #define SX1262_TXEN_INVERTED
// #endif

#ifdef IS_TRANSMITTER
// For crsf tx module to openTX handset
#define UART_INVERTED
#endif

// A define to indicate that we have 915 support for DB mode
#define DB_USE_915

// define the type of radio module being used 

// everything can use the same E22 definition as they're all unamplified
// E220 behaves as an E22 except it doesn't have a TCXO
#define RADIO_E22

// Set the right E28 version depending on type
#if defined(DUAL_BAND_PROTOTYPE) || defined(C3_PCB_V0)

#define RADIO_E28_12    // CAREFUL - this will break _20 and _27 if you use it by accident

#elif defined(DB_TX_V1)

// #define RADIO_E28_20
#define RADIO_E28_27    // Slightly careful - this goes 2 steps higher than E28_20. Probably won't break anything, but not ideal

#elif defined(DUAL_BAND_BREADBOARD) || defined(DB_PCB_V1_0) || defined(DB_PCB_V1_2) 

// nothing, temporary until all targets are converted to individual include files

#else
#error("Problem with mapping board type to 1280 power amp")
#endif

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

// XXX TODO min/max power defines should be part of the radio classes
#define MIN_PRE_PA_POWER_2G4 (-18)

#ifdef ELRS_OG_COMPATIBILITY
    #define Regulatory_Domain_ISM_2400
#else
    #define Regulatory_Domain_ISM_2400_NA
#endif

#endif // 2G4 radios


#ifdef RADIO_E28_12
// XXX TODO rename to be explicit about 2G4
// E28-12 and both GNICERF modules can use the full output range
#define MAX_PRE_PA_POWER 13
#define DISARM_POWER (0)
#define PA_OFFSET -1
#elif defined(RADIO_E28_20)
#define MAX_PRE_PA_POWER (-2)
#define DISARM_POWER (-12)
#define PA_OFFSET 22
#elif defined(RADIO_E28_27)
#define MAX_PRE_PA_POWER 0
#define DISARM_POWER (-15)
#define PA_OFFSET 27
#else
#error "Must define a radio module to use"
#endif


#if defined(RADIO_E22)
#define MAX_PRE_PA_POWER_915 22  // sx1262 can be configured to scale the max power down from the commanded value, so this may result in either 20 or 22dBm
// #define MAX_PRE_PA_POWER_915 10     // while testing

#define MIN_PRE_PA_POWER_915 (-9)
#define DISARM_POWER_915 MIN_PRE_PA_POWER_915
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

#elif defined(DB_TX_V1)

// common pins for spi

#define RADIO_MOSI_PIN  GPIO_NUM_1
#define RADIO_MISO_PIN  GPIO_NUM_0
#define RADIO_SCK_PIN   GPIO_NUM_8

// These are for the sx1262, the sx1280 pins are later under "USE_SECOND_RADIO"

#define RADIO_RESET_PIN GPIO_NUM_5
#define RADIO_NSS_PIN   GPIO_NUM_3
#define RADIO_BUSY_PIN  GPIO_NUM_6
#define RADIO_DIO1_PIN  GPIO_NUM_7
// #define RADIO_DIO2_PIN  Not connected
#define RADIO_TXEN_PIN  GPIO_NUM_2

#elif defined(DB_PCB_V1_0) || defined(DB_PCB_V1_2)

// temporary until refactoring complete


#elif defined(C3_PCB_V0) // these are for the dual 1280 breadboard prototype

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

#elif defined(DUAL_BAND_PROTOTYPE)

#define RADIO2_NSS_PIN   GPIO_NUM_6
#define RADIO2_BUSY_PIN  GPIO_NUM_18
#define RADIO2_DIO1_PIN  GPIO_NUM_5
#define RADIO2_DIO2_PIN  GPIO_NUM_3
#define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DB_TX_V1)

#define RADIO2_NSS_PIN   GPIO_NUM_9
#define RADIO2_BUSY_PIN  GPIO_NUM_13
#define RADIO2_DIO1_PIN  GPIO_NUM_12
#define RADIO2_DIO2_PIN  GPIO_NUM_10
#define RADIO2_RESET_PIN GPIO_NUM_18
#define RADIO2_TXEN_PIN  GPIO_NUM_19


#elif defined(DB_PCB_V1_0) || defined(DB_PCB_V1_2) 

// temporary until refactoring complete

#elif defined(C3_PCB_V0) // these are for the dual 1280 breadboard prototype

// #define RADIO2_RESET_PIN GPIO_NUM_8
#define RADIO2_NSS_PIN   GPIO_NUM_1
#define RADIO2_BUSY_PIN  GPIO_NUM_12
#define RADIO2_DIO1_PIN  GPIO_NUM_3
#define RADIO2_DIO2_PIN  GPIO_NUM_0

#endif // C3_PCB_V0

#else // not using the second radio, but we still need the reset pin during init
#ifdef C3_PCB_V0

// XXX Where is the radio2 reset pin for this layout?

// #elif defined(DUAL_BAND_BREADBOARD)

// #define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DUAL_BAND_PROTOTYPE)

#define RADIO2_RESET_PIN GPIO_NUM_4

#elif defined(DB_TX_V1)

#define RADIO2_RESET_PIN GPIO_NUM_18

#elif defined(DB_PCB_V1_0) || defined(DB_PCB_V1_2) 

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


#ifdef C3_PCB_V0
// For dual sx1280 rx using modules

#define LED2812_PIN     GPIO_NUM_9
#define CRSF_TX_PIN     GPIO_NUM_21

#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 1
#define LED_RADIO2_INDEX 2


#elif defined(DUAL_BAND_PROTOTYPE)
// bare C3, modules for radios

// #define DEBUG_PIN     GPIO_NUM_9
#define LED2812_PIN   GPIO_NUM_9
// #define LATENCY_INPUT_PIN GPIO_NUM_9

#ifdef IS_TRANSMITTER
// For the transmitter module, s.port pin: XXX maybe switch to pin 21 which has the safety resistor

// for prod
#define CRSF_SPORT_PIN  GPIO_NUM_21
#define DEBUG_TX_PIN    GPIO_NUM_20

// for dev without handset
// #define DEBUG_RX_PIN    GPIO_NUM_20
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

#elif defined(DB_TX_V1)

#define LED2812_PIN   GPIO_NUM_4

#define LED_STATUS_INDEX 0
#define LED_RADIO1_INDEX 2
#define LED_RADIO2_INDEX 1


#ifdef DEV_MODE

// for dev without handset
#define DEBUG_RX_PIN    GPIO_NUM_20
#define DEBUG_TX_PIN    GPIO_NUM_21

#else // not DEV_MODE

// for prod
#define CRSF_SPORT_PIN  GPIO_NUM_21
#define DEBUG_TX_PIN    GPIO_NUM_20

#endif // DEV_MODE

#elif defined(DB_PCB_V1_0) || defined(DB_PCB_V1_2)

// temporary until refactoring complete



#elif defined(DUAL_BAND_BREADBOARD)

// empty, temporary until refactored

#else   // XXX what board are these for?

// #define DEBUG_PIN     GPIO_NUM_9
// #define CRSF_TX_PIN     GPIO_NUM_9

#define LED2812_PIN     GPIO_NUM_8

#define LED_RADIO1_INDEX 0
#define LED_RADIO2_INDEX 1

#endif // C3_PCB_V0


#endif // USE_PWM6 or not

#else
#error "define the board type in config.h"
#endif // Board type


// Set the UART to be used by CRSF. Moved from crsf.h
// TODO refactor into targets

#if defined(DUAL_BAND_PROTOTYPE) || defined(DB_PCB_V1_0)  || defined(DB_PCB_V1_2) || defined(DB_TX_V1)

    #ifdef IS_RECEIVER

        // The receiver PCB needs to share uart0 with the normal debug output

        #define CRSF_PORT_NUM 0

    #else

        // The transmitter can use uart1 and keep debug on uart0
        // XXX but isn't for now - needs to be 0 for use in the handset, and 1 for stand alone dev. Can't remember why
        #ifdef DEV_MODE
            #define CRSF_PORT_NUM 1
        #else
            #define CRSF_PORT_NUM 0
        #endif

    #endif // IS_RECEIVER

#elif defined(DUAL_BAND_BREADBOARD)

    // moved to target file

#else

    #error("Must define boardtype")

#endif // DUAL_BAND_PROTOTYPE || ...



// old stuff unrelated to DB

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
