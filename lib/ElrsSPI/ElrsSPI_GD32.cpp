/**
 * Provides an implementation of the ElrsSPI class based on the GD32V SDK
 */

// only compile if using the right platform
#ifdef GD32

#include "ElrsSPI.h"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "gd32vf103.h"
}

#include "../../src/config_constants.h"
#include "../../src/config.h"

// need a header for this, and probably a better place for the impl
uint32_t PORT(uint32_t compositeGPIOid, const char* caller);


/** Constructor
 * Pass the pin definitions using the combined port/pin values from config_constants.h
 */
ElrsSPI::ElrsSPI(uint32_t _pinMosi, uint32_t _pinMiso, uint32_t _pinSCK, uint32_t _pinCSS) : 
    pinMosi{_pinMosi}, pinMiso{_pinMiso}, pinSCK{_pinSCK}, pinCSS{_pinCSS}
{
    printf("pinCSS is %08lX\n\r", pinCSS);
} 

/** alternative constructor for dual modems - not yet implemented
 */
ElrsSPI::ElrsSPI(uint32_t pinCSS2)
{
    printf("dual modem not supported for GD32");
    while(true) {}
}



uint32_t safePORT(uint32_t compositePin)
{
    return (compositePin < 3) ? PORT(compositePin,"safePORT") : -1;
}

void ElrsSPI::debug()
{
    printf("pinMosi %lX:%u, pinMiso %lX:%u, pinSCK %lX:%u, pinCSS %lX:%u\n", 
        safePORT(pinMosi), PIN(pinMosi), 
        safePORT(pinMiso), PIN(pinMiso),
        safePORT(pinSCK), PIN(pinSCK),
        safePORT(pinCSS), PIN(pinCSS)
    );
}

/**
 * Perform any setup required to use SPI
 */
int ElrsSPI::init()
{
    spi_parameter_struct spi_init_struct;

    printf("init: pinCSS is %08lX\n\r", pinCSS);

    gpio_bit_set(PORT(pinCSS, "spi::init"), PIN(pinCSS)); // set NSS high
    spi_struct_para_init(&spi_init_struct);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE; // seems to work with sx1280 and sx1262
    spi_init_struct.nss                  = SPI_NSS_SOFT;

    // TODO - confirm these clock rates are what they claim to be
    // spi_init_struct.prescale             = SPI_PSC_32; // 1.6 MHz - must be running off a ~50MHz clock
    // spi_init_struct.prescale             = SPI_PSC_16; // 3.2 MHz
    // spi_init_struct.prescale             = SPI_PSC_8; // 6.4 MHz
    spi_init_struct.prescale             = SPI_PSC_4; // 12.8 MHz -- works for gd32 with both sx1280 and sx1262

    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);

    spi_enable(SPI1);

    printf("SPI initialised\n\r");

    return 0; // XXX decide on ret values
}

/**
 * Send nBytes over SPI. Return data will overwrite the original contents
 * of buffer.
 * @return 0 on success, -1 on failure
 */
int8_t ElrsSPI::transfer(uint8_t *buffer, const uint8_t nBytes)
{
    // printf("transfer: pinCSS is %08lX\n\r", pinCSS);

    // tx is failing with a bad value for pinCSS being passed from here to PORT().
    // suggests that something is calling transfer before this instance has been properly setup, but how?
    // Try working around it for now with the constant for pinCSS

    // gpio_bit_reset(PORT(pinCSS, "spi::transfer"), PIN(pinCSS)); // set NSS low
    gpio_bit_reset(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS low

    for(int i=0; i<nBytes; i++) {
        while (0 == spi_i2s_flag_get(SPI1, SPI_FLAG_TBE))
            ; // wait for any previous tx to complete

        spi_i2s_data_transmit(SPI1, buffer[i]);

        while (0 == spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE))
            ; // wait for rx data to be ready

        buffer[i] = spi_i2s_data_receive(SPI1);
    }

    // gpio_bit_set(PORT(pinCSS, "spi::transfer"), PIN(pinCSS)); // set NSS high
    gpio_bit_set(RADIO_NSS_PORT, RADIO_NSS_PIN); // set NSS high

    // XXX add error checking and return code
    return 0;

}

#endif // GD32