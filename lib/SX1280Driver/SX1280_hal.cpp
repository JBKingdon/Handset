/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project 

*/

#include "../../src/config.h"
#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include <stdio.h>
#include <string.h> // for memcpy

// void  SX1280Hal::nullCallback(void){};

// void (*SX1280Hal::TXdoneCallback)() = &nullCallback;
// void (*SX1280Hal::RXdoneCallback)() = &nullCallback;

SX1280Hal::SX1280Hal()
{
}

void SX1280Hal::end()
{
    // XXX todo
    // SPI.end(); 
    // detachInterrupt(GPIO_PIN_DIO1);
}


void SX1280Hal::WriteCommand(const SX1280_RadioCommands_t command, const uint8_t val)
{
    uint8_t buffer[2] = {command, val};

    WaitOnBusy();

    // spi1_transferBytes(buffer, 2);
    spi->transfer(buffer, 2);
}

// TODO get rid of all uses and remove this function
void SX1280Hal::WriteCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 1];

    OutBuffer[0] = (uint8_t)command;
    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy();

    // spi1_transferBytes(OutBuffer, size+1);
    spi->transfer(OutBuffer, size+1);
}

/** faster version of Writecommand.
 * The command is passed in the first byte of the buffer.
 * size includes the command.
 * Contents of buffer will be overwritten
*/
void SX1280Hal::fastWriteCommand(uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();

    spi->transfer(buffer, size);
}


// TODO add fast read without the memory copying
void SX1280Hal::ReadCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 2];

    WaitOnBusy();

    if (command == SX1280_RADIO_GET_STATUS)
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        OutBuffer[2] = 0x00;
        // spi1_transferBytes(OutBuffer, 3);
        spi->transfer(OutBuffer, 3);
        buffer[0] = OutBuffer[0];
    }
    else
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        memcpy(OutBuffer + 2, buffer, size);
        // spi1_transferBytes(OutBuffer, sizeof(OutBuffer));
        spi->transfer(OutBuffer, sizeof(OutBuffer));
        memcpy(buffer, OutBuffer + 2, size);
    }
}

void  SX1280Hal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];

    OutBuffer[0] = SX1280_RADIO_WRITE_REGISTER;
    OutBuffer[1] = address >> 8;
    OutBuffer[2] = address & 0x00FF;

    memcpy(OutBuffer + 3, buffer, size);

    WaitOnBusy();

    // spi1_transferBytes(OutBuffer, (uint8_t)sizeof(OutBuffer));
    spi->transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
}

void  SX1280Hal::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegister(address, &value, 1);
}

void  SX1280Hal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 4];

    OutBuffer[0] = SX1280_RADIO_READ_REGISTER;
    OutBuffer[1] = address >> 8;
    OutBuffer[2] = address & 0x00FF;
    OutBuffer[3] = 0x00;

    memcpy(OutBuffer + 4, buffer, size);

    WaitOnBusy();

    // spi1_transferBytes(OutBuffer, uint8_t(sizeof(OutBuffer)));
    spi->transfer(OutBuffer, uint8_t(sizeof(OutBuffer)));
    memcpy(buffer, OutBuffer + 4, size);
}

uint8_t  SX1280Hal::ReadRegister(uint16_t address)
{
    uint8_t data=0;
    ReadRegister(address, &data, 1);
    return data;
}

void  SX1280Hal::WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 2];

    OutBuffer[0] = SX1280_RADIO_WRITE_BUFFER;
    OutBuffer[1] = offset;

    memcpy(OutBuffer + 2, (void *)buffer, size);

    WaitOnBusy();

    // spi1_transferBytes(OutBuffer, (uint8_t)sizeof(OutBuffer));
    spi->transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
}

void  SX1280Hal::ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];

    OutBuffer[0] = SX1280_RADIO_READ_BUFFER;
    OutBuffer[1] = offset;
    OutBuffer[2] = 0x00;

    memset(OutBuffer + 3, 0, size); // XXX is this needed?

    WaitOnBusy();

    // spi1_transferBytes(OutBuffer, uint8_t(sizeof(OutBuffer)));
    spi->transfer(OutBuffer, uint8_t(sizeof(OutBuffer)));

    memcpy((void *)buffer, OutBuffer + 3, size);
}


// void  SX1280Hal::dioISR()
// {
//     if (instance->InterruptAssignment == SX1280_INTERRUPT_RX_DONE)
//     {
//         //Serial.println("HalRXdone");
//         RXdoneCallback();
//     }
//     else if (instance->InterruptAssignment == SX1280_INTERRUPT_TX_DONE)
//     {
//         //Serial.println("HalTXdone");
//         TXdoneCallback();
//     }
// }
