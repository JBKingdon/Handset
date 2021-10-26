#pragma once

/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian

Heavily modified/simplified by Alessandro Carcione 2020 for ELRS project

Hack around some more by James Kingdon 2021.
*/

#define ICACHE_RAM_ATTR

#include <stdint.h>

#include "SX1262_hal.h"

#include "SX1262_Regs.h"

#include "ElrsSPI.h"

#ifdef GD32
extern "C" {
#include "gd32vf103.h"
}
#endif

class SX1262Hal_GD32 : public SX1262Hal
{

private:


public:

    SX1262Hal_GD32();

    virtual void init();
    // void end();
    // // void SetSpiSpeed(uint32_t spiSpeed);
    virtual void reset();

    
    virtual bool ICACHE_RAM_ATTR WaitOnBusy();
    
    virtual void ICACHE_RAM_ATTR TXenable();
    virtual void ICACHE_RAM_ATTR RXenable();
    virtual void ICACHE_RAM_ATTR TXRXdisable();

};
