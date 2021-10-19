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

This has been hacked around a lot since it's original form. Errors and ommissions are probably mine (JBK)
*/

#include "../../src/config.h"
#include "SX1262_Regs.h"
#include "SX1262_hal.h"
#include <stdio.h>

#include "../../src/utils.h"
#include <string.h> // for memcpy

extern "C" {
#include "../../include/systick.h"
}

SX1262Hal *SX1262Hal::instance = nullptr;

// void  SX1262Hal::nullCallback(void){};

// void (*SX1262Hal::TXdoneCallback)() = &nullCallback;
// void (*SX1262Hal::RXdoneCallback)() = &nullCallback;

SX1262Hal::SX1262Hal()
{
    instance = this;
}

void SX1262Hal::end()
{
    printf("end() not implemented\n\r");
    // XXX todo
    // SPI.end(); 
    // detachInterrupt(GPIO_PIN_DIO1);
}

void SX1262Hal::init()
{
    // all pin/spi setup done in main.cpp
}

void  SX1262Hal::reset(void)
{
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);
    gpio_bit_reset(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(100);
    gpio_bit_set(RADIO_RESET_PORT, RADIO_RESET_PIN);
    delay(50);

    if (WaitOnBusy()) {
        printf("WARNING SX1262 busy didn't go low\n\r");
    } else {
        printf("SX1262 Ready!\n\r");
    }
}

void SX1262Hal::WriteCommand(const SX1262_RadioCommands_t command, const uint8_t val)
{
    uint8_t buffer[2] = {command, val};

    WaitOnBusy();

    spi1_transferBytes(buffer, 2);
}


void SX1262Hal::WriteCommand(SX1262_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 1];

    OutBuffer[0] = (uint8_t)command;
    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, size+1);
}

/** faster version of Writecommand.
 * The command is passed in the first byte of buffer
 * size includes the command
 * contents of buffer will be overwritten
*/
void SX1262Hal::fastWriteCommand(uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();

    spi1_transferBytes(buffer, size);
}


// TODO add fast read without the memory copying
void SX1262Hal::ReadCommand(SX1262_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 2];

    WaitOnBusy();

    if (command == SX1262_RADIO_GET_STATUS)
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        OutBuffer[2] = 0x00;
        spi1_transferBytes(OutBuffer, 3);
        buffer[0] = OutBuffer[0];
    }
    else
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        memcpy(OutBuffer + 2, buffer, size);
        spi1_transferBytes(OutBuffer, sizeof(OutBuffer));
        memcpy(buffer, OutBuffer + 2, size);
    }
}

/** faster write register for performance critical paths
 * 
 *  buffer must have the register address in bytes 1,2 and the value in
 *  byte 3
 * 
 *  This function will insert the command into buffer[0]
 */
void  SX1262Hal::fastWriteSingleRegister(uint8_t *buffer)
{
    buffer[0] = SX1262_RADIO_WRITE_REGISTER;

    WaitOnBusy();

    spi1_transferBytes(buffer, 4);
}

void  SX1262Hal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];

    OutBuffer[0] = SX1262_RADIO_WRITE_REGISTER;
    OutBuffer[1] = address >> 8;
    OutBuffer[2] = address & 0xFF;

    memcpy(OutBuffer + 3, buffer, size);

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, (uint8_t)sizeof(OutBuffer));
}

void  SX1262Hal::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegister(address, &value, 1);
}

/** faster read for single register
 * 
 * buffer must be at least 5 bytes long and contain the address in
 * bytes 1,2
 * 
 * This function will insert the command in buffer[0] and zeros in 3,4
 * 
 * The result will be returned in buffer[4] as well as the returned value
 * 
 * Note that the contents of the buffer including the address values will be overwritten
 */
uint8_t SX1262Hal::fastReadSingleRegister(uint8_t *buffer)
{
    buffer[0] = SX1262_RADIO_READ_REGISTER;
    buffer[3] = 0x00;
    buffer[4] = 0x00;

    WaitOnBusy();

    spi1_transferBytes(buffer, 5);

    return buffer[4];
}

/** Set bits of a register
 * Performs a read/modify/write operation on a register, where
 * the modification is to bitwise-or the existing value with the mask parameter.
 */
void SX1262Hal::readModifyWriteRegister(const uint16_t address, const uint8_t mask, const bool set)
{
    uint8_t buffer[5];

    buffer[1] = address >> 8;
    buffer[2] = address & 0xFF;

    uint8_t v = fastReadSingleRegister(buffer);
    
    if (set) {
        v |= mask;
    } else {
        v &= ~mask; // clear bits by and-ing with the inverted mask
    }

    // address gets overwritten so we have to set it again
    buffer[1] = address >> 8;
    buffer[2] = address & 0xFF;
    buffer[3] = v;

    fastWriteSingleRegister(buffer);
}


/** Set bits of a register
 * Performs a read/modify/write operation on a register, where
 * the modification is to bitwise-or the existing value with the mask parameter.
 */
void SX1262Hal::readSetWriteRegister(uint16_t address, uint8_t mask)
{
    readModifyWriteRegister(address, mask, true);
}

/** Clear bits of a register
 * Performs a read/modify/write operation on a register, where
 * the modification is to bitwise-and the existing value with the mask parameter.
 */
void SX1262Hal::readClearWriteRegister(uint16_t address, uint8_t mask)
{
    readModifyWriteRegister(address, mask, false);
}



void  SX1262Hal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 4];

    OutBuffer[0] = SX1262_RADIO_READ_REGISTER;
    OutBuffer[1] = address >> 8;
    OutBuffer[2] = address & 0x00FF;
    OutBuffer[3] = 0x00;

    memcpy(OutBuffer + 4, buffer, size);

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, size + 4);

    memcpy(buffer, OutBuffer + 4, size);
}


uint8_t  SX1262Hal::ReadRegister(uint16_t address)
{
    uint8_t data=0;
    ReadRegister(address, &data, 1);
    return data;
}

// TODO this needs a fast version
/** Write size bytes from buffer to the 1262's memory
 * 
 * @param offset - the address in the 1262 memory to start writing to
 * @param buffer - data to be written (starting from the beginning of buffer)
 * @param size - number of bytes to send
 * 
 */
void  SX1262Hal::WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 2];

    OutBuffer[0] = SX1262_RADIO_WRITE_BUFFER;
    OutBuffer[1] = offset;

    memcpy(OutBuffer + 2, (void *)buffer, size);

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, sizeof(OutBuffer));
}

void  SX1262Hal::ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    uint8_t OutBuffer[size + 3];

    OutBuffer[0] = SX1262_RADIO_READ_BUFFER;
    OutBuffer[1] = offset;
    OutBuffer[2] = 0x00;

    memset(OutBuffer + 3, 0, size); // XXX is this needed?

    WaitOnBusy();

    spi1_transferBytes(OutBuffer, sizeof(OutBuffer));

    memcpy((void *)buffer, OutBuffer + 3, size);
}

/** Wait for the SX1262 busy flag to be low
 * Returns true if we reach the timeout before busy goes low
 * TODO pass in the timeout
 */
bool SX1262Hal::WaitOnBusy()
{
    // printf("%s \r\n", "waitOnBusy...");
    const unsigned int MAX_WAIT = 5000; // in us
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

// void  SX1262Hal::dioISR()
// {
//     if (instance->InterruptAssignment == SX1262_INTERRUPT_RX_DONE)
//     {
//         //Serial.println("HalRXdone");
//         RXdoneCallback();
//     }
//     else if (instance->InterruptAssignment == SX1262_INTERRUPT_TX_DONE)
//     {
//         //Serial.println("HalTXdone");
//         TXdoneCallback();
//     }
// }

void  SX1262Hal::TXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_set(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1262Hal::RXenable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_set(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

void  SX1262Hal::TXRXdisable()
{
    #if defined(RADIO_RXEN_PIN)
    gpio_bit_reset(RADIO_RXEN_PORT, RADIO_RXEN_PIN);
    #endif

    #if defined(RADIO_TXEN_PIN)
    gpio_bit_reset(RADIO_TXEN_PORT, RADIO_TXEN_PIN);
    #endif
}

// void  SX1262Hal::setIRQassignment(SX1262_InterruptAssignment_ newInterruptAssignment)
// {

//     // if (InterruptAssignment == newInterruptAssignment)
//     // {
//     //     return;
//     // }
//     // else
//     // {
//     if (newInterruptAssignment == SX1262_INTERRUPT_TX_DONE)
//     {
//         this->InterruptAssignment = SX1262_INTERRUPT_TX_DONE;
//     }
//     else if (newInterruptAssignment == SX1262_INTERRUPT_RX_DONE)
//     {
//         this->InterruptAssignment = SX1262_INTERRUPT_RX_DONE;
//     }
//     //}
// }