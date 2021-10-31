/** Simple abstraction layer for the SPI interface
 * 
 *  Constructor: save the pin definitions
 *  init: do any required setup to make the SPI usable
 *  transfer: do a bi-directional SPI data transfer with CSS assertion
*/

#pragma once

#include <stdint.h>

#ifdef GD32
#define ICACHE_RAM_ATTR
#else
#include "driver/spi_master.h"
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif // GD32


class ElrsSPI
{
    private:

    uint32_t pinMosi, pinMiso, pinSCK, pinCSS;

    #ifndef GD32
    spi_device_handle_t spiHandle;
    #endif

    public:

    ElrsSPI(uint32_t pinMosi, uint32_t pinMiso, uint32_t pinSCK, uint32_t pinCSS);
    int init();
    void ICACHE_RAM_ATTR transfer(uint8_t *buffer, const uint8_t nBytes);

    // print to stdout 
    void debug();

};


