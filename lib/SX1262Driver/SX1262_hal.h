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

#include <stdint.h>
#include "SX1262_Regs.h"

#include "ElrsSPI.h"

#ifdef GD32
#define ICACHE_RAM_ATTR

extern "C" {
#include "gd32vf103.h"
}
#endif

class SX1262Hal
{

private:

    void ICACHE_RAM_ATTR ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size); // TODO get rid of this method that's only used internally
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size); // TODO get rid of this method that's only used internally

    void readModifyWriteRegister(const uint16_t address, const uint8_t mask, const bool set);

protected:

    ElrsSPI *spi;   // Wrapper for using SPI, specialised for different MCUs


public:

    // SX1262Hal();

    virtual void init() = 0;
    void end();
    // void SetSpiSpeed(uint32_t spiSpeed);
    virtual void reset() = 0;

    void ICACHE_RAM_ATTR WriteCommand(SX1262_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR fastWriteCommand(uint8_t *buffer, uint8_t size);

    void ICACHE_RAM_ATTR WriteCommand(SX1262_RadioCommands_t command, uint8_t val);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t value);
    void ICACHE_RAM_ATTR fastWriteSingleRegister(uint8_t *buffer);


    void ICACHE_RAM_ATTR ReadCommand(SX1262_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    uint8_t ICACHE_RAM_ATTR ReadRegister(uint16_t address);
    uint8_t ICACHE_RAM_ATTR fastReadSingleRegister(uint8_t *buffer);

    void readSetWriteRegister(uint16_t address, uint8_t mask);
    void readClearWriteRegister(uint16_t address, uint8_t mask);


    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size); // Writes and Reads to FIFO
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size);

    // static void ICACHE_RAM_ATTR nullCallback(void);
    
    virtual bool ICACHE_RAM_ATTR WaitOnBusy() = 0;
    // static ICACHE_RAM_ATTR void dioISR();
    
    virtual void ICACHE_RAM_ATTR TXenable() = 0;
    virtual void ICACHE_RAM_ATTR RXenable() = 0;
    virtual void ICACHE_RAM_ATTR TXRXdisable() = 0;

};
