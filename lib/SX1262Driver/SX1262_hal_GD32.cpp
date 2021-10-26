/**
 * Provides code that is specific to the GD32 platform.
 * 
*/

// don't compile unless we're running on the right platform
#ifdef GD32

#include "../../src/config.h" // for? Probably pin defs. Pass them in to the constructor instead
#include "SX1262_Regs.h"
#include "SX1262_hal_GD32.h"
#include <stdio.h>

#include "../../src/utils.h"
#include <string.h> // for memcpy

extern "C" {
#include "../../include/systick.h"
}

SX1262Hal_GD32::SX1262Hal_GD32()
{
}


void SX1262Hal_GD32::init()
{
    // all pin setup done in main.cpp setup routine for now

    // Create and init the SPI instance

    // convert the nss port/pin to the combi format
    // TODO need a utility method for this
    uint32_t nssPin;
    switch (RADIO_NSS_PORT) {
    case GPIOB:
        nssPin = (PORTB << 16) | RADIO_NSS_PIN;
        break;
    default:
        printf("Need to fix port map in SX1262Hal::init\n\r");
    }

    // other pins are hard coded on GD32, so just pass zeroes as placeholders
    spi = new ElrsSPI(0, 0, 0, nssPin);

    spi->init();
    // spi->debug();
}

void  SX1262Hal_GD32::reset(void)
{
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);
    gpio_bit_reset(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(100);
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);

    if (WaitOnBusy()) {
        printf("WARNING SX1262 busy didn't go low\n\r");
    } else {
        printf("SX1262 Ready!\n\r");
    }
}


/** Wait for the SX1262 busy flag to be low
 * Returns true if we reach the timeout before busy goes low
 * TODO pass in the timeout
 */
bool SX1262Hal_GD32::WaitOnBusy()
{
    // printf("%s \r\n", "waitOnBusy...");
    const unsigned int MAX_WAIT = 5000; // in us
    const unsigned long t0 = micros();
    while (gpio_input_bit_get(RADIO_BUSY_PORT, RADIO_BUSY_PIN) == SET)
    {
        if (micros() > (t0 + MAX_WAIT)) {
            printf("busy timeout \n\r");
            return true;
        }
    }
    // printf("waitOnBusy done in %lu us\n", micros()-t0);
    return false;
}


void  SX1262Hal_GD32::TXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_set(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1262Hal_GD32::RXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_set(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1262Hal_GD32::TXRXdisable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

#endif // GD32