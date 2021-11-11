/* C3 specific functions
*/

#ifdef ESPC3

#include "../../src/config.h"
#include "SX1280_Regs.h"
#include "SX1280_hal_C3.h"
#include <stdio.h>

#include "../../src/utils.h"

#include "driver/gpio.h"
#include "freertos/task.h"


SX1280Hal_C3::SX1280Hal_C3()
{
}

// void SX1280Hal_C3::end()
// {
//     // XXX todo
//     // SPI.end(); 
//     // detachInterrupt(GPIO_PIN_DIO1);
// }

void SX1280Hal_C3::init()
{
    // Need to setup pins for reset, busy, dios, tx and rx enable
    gpio_reset_pin(RADIO_RESET_PIN);
    gpio_set_direction(RADIO_RESET_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(RADIO_BUSY_PIN);
    gpio_set_direction(RADIO_BUSY_PIN, GPIO_MODE_INPUT);

    gpio_reset_pin(RADIO_DIO1_PIN);
    gpio_set_direction(RADIO_DIO1_PIN, GPIO_MODE_INPUT);

    gpio_reset_pin(RADIO_DIO2_PIN);
    gpio_set_direction(RADIO_DIO2_PIN, GPIO_MODE_INPUT);

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
}

void SX1280Hal_C3::reset(void)
{
    gpio_set_level(RADIO_RESET_PIN, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RADIO_RESET_PIN, 0);    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(RADIO_RESET_PIN, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    if (WaitOnBusy()) {
        printf("WARNING SX1262 busy didn't go low\n\r");
    } else {
        printf("SX1262 Ready!\n\r");
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
    
    while (gpio_get_level(RADIO_BUSY_PIN) == 1)
    {
        if (micros() > (t0 + MAX_WAIT)) {
            printf("busy timeout \n\r");
            return true;
        }
    }
    // printf("waitOnBusy done in %lu us\n", micros()-t0);
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