#pragma once

#include "user_config.h"

// which board are we using

// #define T_DISPLAY
// #define LONGAN_NANO
// #define PCB_V1_0
// #define RX_C3


#ifdef RADIO_E22

    #define Regulatory_Domain_FCC_915

#else // not E22

#ifdef ELRS_OG_COMPATIBILITY
    #define Regulatory_Domain_ISM_2400
#else
    #define Regulatory_Domain_ISM_2400_NA
#endif

#endif // E22


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
#elif defined(RADIO_E22)
// #define MAX_PRE_PA_POWER 22  // sx1262 can be configured to scale the max power down from the commanded value
#define MAX_PRE_PA_POWER 10     // while testing
#define DISARM_POWER (-9)
#else
#error "Must define a radio module to use"
#endif


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

#define RADIO_RESET_PORT GPIOA
#define RADIO_RESET_PIN GPIO_PIN_11

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


#define RADIO_RESET_PIN GPIO_NUM_18
// #define RADIO_RXEN_PIN  GPIO_NUM_8  // try tying to 3v3
#define RADIO_TXEN_PIN  GPIO_NUM_1  // maybe experiment with auto mode using dio2?

#define RADIO_MOSI_PIN  GPIO_NUM_5
#define RADIO_MISO_PIN  GPIO_NUM_4
#define RADIO_SCK_PIN   GPIO_NUM_6
#define RADIO_NSS_PIN   GPIO_NUM_7

#define RADIO_BUSY_PIN  GPIO_NUM_19
#define RADIO_DIO1_PIN  GPIO_NUM_10
#define RADIO_DIO2_PIN  GPIO_NUM_2  // might do without this if needed?

// #define CRSF_TX_PIN     GPIO_NUM_9  // need to make conditional with a #def

#define PWM_CH1_PIN     GPIO_NUM_3
#define PWM_CH2_PIN     GPIO_NUM_0
#define PWM_CH3_PIN     GPIO_NUM_8  // need to free up rxen
#define PWM_CH4_PIN     GPIO_NUM_9  // need to disable crsf output
// #define PWM_CH5_PIN     GPIO_NUM_20 // overlaps with the debug/flash uart. Maybe use 1 (after txen is auto)
// #define PWM_CH6_PIN     GPIO_NUM_21 // overlaps with the debug/flash uart. Maybe use 2 (after disabling dio2)
#define PWM_CH5_PIN     -1 // GPIO_NUM_20 // overlaps with the debug/flash uart. Maybe use 1 (after txen is auto)
#define PWM_CH6_PIN     -1 // GPIO_NUM_21 // overlaps with the debug/flash uart. Maybe use 2 (after disabling dio2)


#else
#error "define the board type in config.h"
#endif


#ifndef RX_C3

// common to all handset boards

#define RADIO_NSS_PORT GPIOB
#define RADIO_NSS_PIN GPIO_PIN_12

#define RADIO_DIO1_PORT GPIOB
#define RADIO_DIO1_PIN GPIO_PIN_8

#define RADIO_DIO2_PORT GPIOB
#define RADIO_DIO2_PIN GPIO_PIN_9

#endif // !RX_C3
