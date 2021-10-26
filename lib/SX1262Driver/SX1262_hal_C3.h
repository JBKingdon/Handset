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

#ifdef GD32
#define ICACHE_RAM_ATTR
#endif

#include <stdint.h>
#include "SX1262_Regs.h"

#include "ElrsSPI.h"


class SX1262Hal_C3 : public SX1262Hal
{

private:


public:

    SX1262Hal_C3();

    void init() override;
    // void end();
    // void SetSpiSpeed(uint32_t spiSpeed);
    void reset() override;

    bool ICACHE_RAM_ATTR WaitOnBusy() override;
    
    void ICACHE_RAM_ATTR TXenable() override;
    void ICACHE_RAM_ATTR RXenable() override;
    void ICACHE_RAM_ATTR TXRXdisable() override;

};
