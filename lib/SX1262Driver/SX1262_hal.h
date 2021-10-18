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
#include "SX1262_Regs.h"

extern "C" {
#include "gd32vf103.h"
}


class SX1262Hal
{

private:

public:
    static SX1262Hal *instance;

    SX1262Hal();

    void init();
    void end();
    // void SetSpiSpeed(uint32_t spiSpeed);
    void reset();

    void ICACHE_RAM_ATTR WriteCommand(SX1262_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR fastWriteCommand(uint8_t *buffer, uint8_t size);

    void ICACHE_RAM_ATTR WriteCommand(SX1262_RadioCommands_t command, uint8_t val);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t value);
    void ICACHE_RAM_ATTR fastWriteSingleRegister(uint8_t *buffer);


    void ICACHE_RAM_ATTR ReadCommand(SX1262_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    uint8_t ICACHE_RAM_ATTR ReadRegister(uint16_t address);
    uint8_t ICACHE_RAM_ATTR fastReadSingleRegister(uint8_t *buffer);

    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size); // Writes and Reads to FIFO
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size);

    // static void ICACHE_RAM_ATTR nullCallback(void);
    
    bool ICACHE_RAM_ATTR WaitOnBusy();
    // static ICACHE_RAM_ATTR void dioISR();
    
    void ICACHE_RAM_ATTR TXenable();
    void ICACHE_RAM_ATTR RXenable();
    void ICACHE_RAM_ATTR TXRXdisable();

};
