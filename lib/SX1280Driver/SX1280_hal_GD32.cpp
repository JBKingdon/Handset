/* GD32 specific code for the sx1280
*/

#ifdef GD32

#include "../../src/config.h"
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include <stdio.h>

#include "../../src/utils.h"


extern "C" {
#include "../../include/systick.h"
}

#include "SX1280_hal_GD32.h"

// SX1280Hal *SX1280Hal::instance = nullptr;

// void  SX1280Hal::nullCallback(void){};

// void (*SX1280Hal::TXdoneCallback)() = &nullCallback;
// void (*SX1280Hal::RXdoneCallback)() = &nullCallback;

SX1280Hal_GD32::SX1280Hal_GD32()
{
    instance = this;
}

// void SX1280Hal_GD32::end()
// {
//     // XXX todo
//     // SPI.end(); 
//     // detachInterrupt(GPIO_PIN_DIO1);
// }

void SX1280Hal_GD32::init()
{
    // The mosi, miso and sck pins are hard coded and setup in main(), but we
    // need to pass in nss for use by the library
    spi = new ElrsSPI(0, 0, 0, RADIO_NSS);

    // spi->init(); // the code in init is currently duplicated in main for the tx
    // spi->debug();
}

void  SX1280Hal_GD32::reset(void)
{
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);
    gpio_bit_reset(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(100);
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);

    if (WaitOnBusy()) {
        printf("WARNING SX1280 busy didn't go low\n\r");
    } else {
        printf("SX1280 Ready!\n\r");
    }
}


/** Wait for the SX1280 busy flag to be low
 * Returns true if we reach the timeout before busy goes low
 * TODO pass in the timeout
 */
bool  SX1280Hal_GD32::WaitOnBusy()
{
    // printf("%s \r\n", "waitOnBusy...");
    const uint MAX_WAIT = 1000; // in us
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


void  SX1280Hal_GD32::TXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_set(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1280Hal_GD32::RXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_set(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1280Hal_GD32::TXRXdisable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}


#endif // GD32