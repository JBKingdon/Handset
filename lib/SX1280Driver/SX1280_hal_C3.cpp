/* C3 specific functions
*/

#ifdef ESPC3

#include "../../src/config.h"
#include "SX1280_Regs.h"
#include "SX1280_hal_C3.h"
#include <stdio.h>
#include <iostream>

#include "../../src/utils.h"

#include "driver/gpio.h"
#include "freertos/task.h"


SX1280Hal_C3::SX1280Hal_C3()
{
    isSecondRadio = false;
}

/** NB This constructor with an explicit cssPin marks the radio as the 'second' radio
 * and assumes that another radio has already setup the spi bus.
 * This is horrible. Replace it with something better.
 * 
 */
SX1280Hal_C3::SX1280Hal_C3(uint32_t cssPin)
{
    spi = new ElrsSPI(cssPin);
    isSecondRadio = true;
}


// void SX1280Hal_C3::end()
// {
//     // XXX todo
//     // SPI.end(); 
//     // detachInterrupt(GPIO_PIN_DIO1);
// }

void SX1280Hal_C3::init()
{
    if (isSecondRadio)
    {
        #ifdef USE_SECOND_RADIO
        // The secondary radio only has to initialise it's unique pins...
        gpio_reset_pin(RADIO2_RESET_PIN);
        gpio_set_direction(RADIO2_RESET_PIN, GPIO_MODE_OUTPUT);

        gpio_reset_pin(RADIO2_BUSY_PIN);
        gpio_set_direction(RADIO2_BUSY_PIN, GPIO_MODE_INPUT);

        gpio_reset_pin(RADIO2_DIO1_PIN);
        gpio_set_direction(RADIO2_DIO1_PIN, GPIO_MODE_INPUT);

        gpio_reset_pin(RADIO2_DIO2_PIN);
        gpio_set_direction(RADIO2_DIO2_PIN, GPIO_MODE_INPUT);

        // ...and call init to setup the device on the spi bus
        spi->init();

        printf("Secondary radio initialised\n");
        #else
        std::cout << "second radio not enabled\n";
        #endif // USE_SECOND_RADIO

    } else {
        // The primary radio is responsible for setting it's pins and the spi instance

        // Need to setup pins for reset, busy, dios, tx and rx enable
        gpio_reset_pin(RADIO_RESET_PIN);
        gpio_set_direction(RADIO_RESET_PIN, GPIO_MODE_OUTPUT);

        gpio_reset_pin(RADIO_BUSY_PIN);
        gpio_set_direction(RADIO_BUSY_PIN, GPIO_MODE_INPUT);

        gpio_reset_pin(RADIO_DIO1_PIN);
        gpio_set_direction(RADIO_DIO1_PIN, GPIO_MODE_INPUT);

        #ifdef RADIO_DIO2_PIN
        gpio_reset_pin(RADIO_DIO2_PIN);
        gpio_set_direction(RADIO_DIO2_PIN, GPIO_MODE_INPUT);
        #endif

        #ifdef RADIO_TXEN_PIN
        gpio_reset_pin(RADIO_TXEN_PIN);
        gpio_set_direction(RADIO_TXEN_PIN, GPIO_MODE_OUTPUT);
        #endif

        #ifdef RADIO_RXEN_PIN
        gpio_reset_pin(RADIO_RXEN_PIN);
        gpio_set_direction(RADIO_RXEN_PIN, GPIO_MODE_OUTPUT);
        #endif

        // Create and init the SPI instance

        // figure out how we're going to deal with the pin info
        spi = new ElrsSPI(RADIO_MOSI_PIN, RADIO_MISO_PIN, RADIO_SCK_PIN, RADIO_NSS_PIN);

        spi->init();
        spi->debug();

        printf("Primary radio initialised\n");
    }
}

// NB reset for double 1280 is hard-coded into the SPI class
void SX1280Hal_C3::reset(void)
{
    #ifdef USE_SECOND_RADIO
    const gpio_num_t resetPin = isSecondRadio ? RADIO2_RESET_PIN : RADIO_RESET_PIN;
    #else
    const gpio_num_t resetPin = RADIO_RESET_PIN;
    #endif

    printf("reset on pin %u\n", resetPin);

    gpio_set_level(resetPin, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(resetPin, 0);    
    vTaskDelay(200 / portTICK_PERIOD_MS);
    gpio_set_level(resetPin, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (WaitOnBusy()) {
        printf("WARNING SX1280 busy didn't go low after reset\n\r");
    } else {
        printf("SX1280 Ready!\n\r");
    }

}

/** Wait for the SX1280 busy flag to be low
 * Returns true if we reach the timeout before busy goes low
 * TODO pass in the timeout
 */
bool  SX1280Hal_C3::WaitOnBusy()
{
    // printf("%s \r\n", "waitOnBusy...");
    const unsigned int MAX_WAIT = 5000; // in us
    const unsigned long t0 = micros();

    // static unsigned long tMax = 0;
    // static unsigned int debugCounter = 0;

    #ifdef USE_SECOND_RADIO
    gpio_num_t busyPin = isSecondRadio ? RADIO2_BUSY_PIN : RADIO_BUSY_PIN;
    #else
    gpio_num_t busyPin = RADIO_BUSY_PIN;
    #endif

    while (gpio_get_level(busyPin) == 1)
    {
        if (micros() > (t0 + MAX_WAIT)) {
            printf("busy timeout\n\r");
            return true;
        }
    }
    // printf("waitOnBusy done in %lu us\n", micros()-t0);
    // unsigned long delay = micros()-t0;

    // std::cout << 'W' << delay << ' ';

    // if (delay > tMax) {
    //     tMax = delay;
    // }

    // if ((debugCounter++ & 0b11111) == 0) {
    //     std::cout << '\n' << tMax << ' ';
    // }

    return false;
}

void  SX1280Hal_C3::TXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_set_level(RADIO_RXEN_PIN, 0);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_set_level(RADIO_TXEN_PIN, 1);
    #endif
}

void  SX1280Hal_C3::RXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_set_level(RADIO_RXEN_PIN, 1);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_set_level(RADIO_TXEN_PIN, 0);
    #endif
}

void  SX1280Hal_C3::TXRXdisable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_set_level(RADIO_RXEN_PIN, 0);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_set_level(RADIO_TXEN_PIN, 0);
    #endif
}

#endif // ESPC3