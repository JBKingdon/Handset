/**
 * Provides an implementation of the ElrsSPI class based on the ESPIDF for ESP32-C3
*/

#ifdef ESPC3

#include "ElrsSPI.h"

#include <stdio.h>
#include <string.h>

#include "freertos/task.h"

#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"

#include "../../src/config.h"
#include "../../src/utils.h"


ElrsSPI::ElrsSPI(uint32_t _pinMosi, uint32_t _pinMiso, uint32_t _pinSCK, uint32_t _pinCSS) : 
    pinMosi{_pinMosi}, pinMiso{_pinMiso}, pinSCK{_pinSCK}, pinCSS{_pinCSS}
{
    isPrimary = true;
}

ElrsSPI::ElrsSPI(uint32_t pinCSS2)
{
    isPrimary = false;
    pinCSS = pinCSS2;
}


void ElrsSPI::debug()
{
    printf("pinMosi %u, pinMiso %u, pinSCK %u, pinCSS %u\n", pinMosi, pinMiso, pinSCK, pinCSS);
}


/** Perform a special reset sequence to try and get around the
 * limitations of having two sx1280 with a single reset line.
 * 
 * This will be called after setting up the bus and primary device.
 * 
 * send the low pulse on the shared reset line,
 * Assert the secondary CSS line
 * send a single SPI transaction using the primary device.
 * 
 * Note that this is a terrible hack and will cause the MISO lines
 * to fight if the 1280s reply with different values. A safety resistor
 * would be a good idea.
 * 
 */
void ElrsSPI::doubleReset()
{
    printf("doubleReset start\n");

    // enable the secondary CS pin for output
    gpio_reset_pin(RADIO2_NSS_PIN);
    gpio_set_direction(RADIO2_NSS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RADIO2_NSS_PIN, 1);


    // perform the reset
    gpio_set_level(RADIO_RESET_PIN, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RADIO_RESET_PIN, 0);    
    vTaskDelay(200 / portTICK_PERIOD_MS);
    gpio_set_level(RADIO_RESET_PIN, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // wait for busy x 2

    const unsigned int MAX_WAIT = 5000; // in us
    unsigned long t0 = micros();

    gpio_num_t busyPin = RADIO_BUSY_PIN;
    
    while (gpio_get_level(busyPin) == 1)
    {
        if (micros() > (t0 + MAX_WAIT)) {
            printf("busy timeout on R1\n");
        }
    }

    t0 = micros();
    busyPin = RADIO2_BUSY_PIN;

    while (gpio_get_level(busyPin) == 1)
    {
        if (micros() > (t0 + MAX_WAIT)) {
            printf("busy timeout on R2\n");
        }
    }


    // send the SPI command
    int32_t tmp = 0;
    uint8_t * buffer = (uint8_t*)&tmp;
    buffer[0] = 0x03; // get packet type, length 3

    // assert the secondary radio's CS pin
    gpio_set_level(RADIO2_NSS_PIN, 0);

    transfer(buffer, 3);

    gpio_set_level(RADIO2_NSS_PIN, 1);

    // reset the pin for later use
    gpio_reset_pin(RADIO2_NSS_PIN);


    printf("doubleReset end\n");
}




/**
 * Perform any setup required to use SPI
 */
int ElrsSPI::init()
{
    esp_err_t ret;

    // only do the bus config if this is the primary instance
    if (isPrimary)
    {
        spi_bus_config_t buscfg={
            .mosi_io_num = (int)pinMosi,
            .miso_io_num = (int)pinMiso,
            .sclk_io_num = (int)pinSCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 16, // XXX double check what this really is
            .flags = SPICOMMON_BUSFLAG_MASTER,
            .intr_flags = 0,
        };
        //Initialize the SPI bus
        ret=spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        ESP_ERROR_CHECK(ret);
        printf("SPI2 bus initialised\n");
    }

    spi_device_interface_config_t devcfg;
    
    memset(&devcfg, 0, sizeof(devcfg));
    
    // 16MHz seems to be too fast
    // 12 looks good - maybe a small number of errors. Check again once pcb is done, might be messy wiring.
    // 10 seems solid
    // printf("SPI clk set slow for testing\n");
    // printf("SPI clock at 16MHz\n");
    devcfg.clock_speed_hz = 10*1000*1000,   // Clock speed in Hz (approximate, driver selects nearest possible)
    // devcfg.clock_speed_hz = 4*1000*1000,   // Clock speed in Hz (approximate, driver selects nearest possible)
    devcfg.spics_io_num = pinCSS,           // CS pin
    devcfg.queue_size = 2,                  // Are we going to use queing at all?

    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spiHandle);
    ESP_ERROR_CHECK(ret);

    if (isPrimary) {
        doubleReset();
    }

    char const *spiType;
    if (isPrimary) {
        spiType = "primary";
    } else {
        spiType = "secondary";
    }

    printf("SPI %s device initialised\n", spiType);

    return 0; // XXX decide on ret values
}


/**
 * Send nBytes over SPI. Return data will overwrite the original contents
 * of buffer.
 * 
 * @return 0 on success, -1 on failure
 */
int8_t ElrsSPI::transfer(uint8_t *buffer, const uint8_t nBytes)
{
    esp_err_t ret;

    spi_transaction_t trans_desc;

    memset(&trans_desc, 0, sizeof(trans_desc));

    trans_desc.length = nBytes * 8;
    trans_desc.tx_buffer = buffer;
    trans_desc.rx_buffer = buffer;

    // XXX try using the underlying start and end commands so we can check for spi collisions and retry
    // XXX there's also the whole queued api to test out

    ret = spi_device_polling_transmit(spiHandle, &trans_desc);

    return (ret != ESP_OK) ? -1 : 0;
}

#endif // ESPC3