/**
 * This file is part of ExpressLRS
 * See https://github.com/AlessandroAU/ExpressLRS
 *
 * This file provides utilities for packing and unpacking the data to
 * be sent over the radio link.
 */

#include <stdio.h>

#include "OTA.h"
#include "crsf_protocol.h"

// /**
//  * Hybrid switches packet encoding for sending over the air
//  *
//  * Analog channels are reduced to 10 bits to allow for switch encoding
//  * Switch[0] is sent on every packet.
//  * A 3 bit switch index and 2 bit value is used to send the remaining switches
//  * in a round-robin fashion.
//  * If any of the round-robin switches have changed
//  * we take the lowest indexed one and send that, hence lower indexed switches have
//  * higher priority in the event that several are changed at once.
//  * 
//  * Inputs: crsf.ChannelDataIn, crsf.currentSwitches
//  * Outputs: Radio.TXdataBuffer, side-effects the sentSwitch value
//  */

// void ICACHE_RAM_ATTR GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, uint16_t *adcData, uint8_t addr)
// {
//    uint8_t PacketHeaderAddr;
//    PacketHeaderAddr = (addr << 2) + RC_DATA_PACKET; // ELRS V1 RC2 doesn't use addr, but it will be overwritten by crc anyway
//    Buffer[0] = PacketHeaderAddr;
//    Buffer[1] = ((adcData[0]) >> 3);
//    Buffer[2] = ((adcData[1]) >> 3);
//    Buffer[3] = ((adcData[2]) >> 3);
//    Buffer[4] = ((adcData[3]) >> 3);
//    Buffer[5] = ((adcData[0] & 0b110) << 5) +
//                ((adcData[1] & 0b110) << 3) +
//                ((adcData[2] & 0b110) << 1) +
//                ((adcData[3] & 0b110) >> 1);


//    #if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)

//    // ELRS compatibility V1 RC2 switch format
//    // find the next switch to send
//    uint8_t nextSwitchIndex = getNextSwitchIndex();

//    uint8_t sentValue = currentSwitches[nextSwitchIndex];

//    // Actually send switchIndex - 1 in the packet, to shift down 1-7 (0b111) to 0-6 (0b110)
//    // If the two high bits are 0b11, the receiver knows it is the last switch and can use
//    // that bit to store data
//    uint8_t bitclearedSwitchIndex = nextSwitchIndex - 1;
//    // currentSwitches[] is 0-15 for index 1, 0-2 for index 2-7 // XXX this seems wrong. most switches have 3 bits available, and last switch is the multiway?
//    // Rely on currentSwitches to *only* have values in that range
//    // TODO ok, so the new switch position values are _slightly_ counterintuitive, and will need mapping from the existing 0,1,2 encoding.
//    //   case 0: return CRSF_CHANNEL_VALUE_1000;
//    //   case 5: return CRSF_CHANNEL_VALUE_2000;
//    //   case 6: // fallthrough
//    //   case 7: return CRSF_CHANNEL_VALUE_MID;
   
//    uint8_t value = 0; // default to low position
//    switch(currentSwitches[nextSwitchIndex]) {
//       case 1:  // middle position
//         value = 7;
//         break;
//       case 2: // high position
//         value = 5;
//         break;
//    }

//    Buffer[6] =
//          // switch 0 is one bit sent on every packet - intended for low latency arm/disarm
//          (currentSwitches[0] / 2) << 6 |   // down-convert the arm switch to on/off, using only the highest setting for on
//          // tell the receiver which switch index this is
//          bitclearedSwitchIndex << 3 |
//          // include the switch value
//          value;

//    #else

//    // switch 0 is sent on every packet - intended for low latency arm/disarm
//    Buffer[6] = (currentSwitches[0] & 0b11) << 5; // note this leaves the top bit of byte 6 unused

//    // find the next switch to send
//    uint8_t nextSwitchIndex = getNextSwitchIndex() & 0b111;     // mask for paranoia
//    uint8_t sentValue = currentSwitches[nextSwitchIndex];       // avoid the possibility that the mask changes the value
//    uint8_t value = sentValue & 0b11; // mask for paranoia

//    // put the bits into buf[6]. nextSwitchIndex is in the range 1 through 7 so takes 3 bits
//    // currentSwitches[nextSwitchIndex] is in the range 0 through 2, takes 2 bits.
//    Buffer[6] += (nextSwitchIndex << 2) + value;

//    #endif // ELRS compatibility for V1 RC2

//    // update the sent value
//    setSentSwitch(nextSwitchIndex, sentValue);
// }

// /** Send 12 bit gimbal data
//  * 
//  */
// void ICACHE_RAM_ATTR GenerateHiResChannelData(volatile uint8_t* Buffer, uint16_t *adcData, uint8_t addr)
// {
//    uint8_t PacketHeaderAddr;
//    PacketHeaderAddr = (addr << 2) | RC_HIRES_DATA;
//    Buffer[0] = PacketHeaderAddr;
//    Buffer[1] = ((adcData[0]) >> 4);
//    Buffer[2] = ((adcData[1]) >> 4);
//    Buffer[3] = ((adcData[2]) >> 4);
//    Buffer[4] = ((adcData[3]) >> 4);
//    Buffer[5] = ((adcData[0] & 0b1111) << 4) | (adcData[1] & 0b1111);
//    Buffer[6] = ((adcData[2] & 0b1111) << 4) | (adcData[3] & 0b1111);

//    // switch 0 is sent on every packet - intended for low latency arm/disarm
//    Buffer[7] = (currentSwitches[0] & 0b11) << 5; // note this leaves the top bit of byte 7 unused

//    // find the next switch to send
//    uint8_t nextSwitchIndex = getNextSwitchIndex() & 0b111;     // mask for paranoia
//    uint8_t sentValue = currentSwitches[nextSwitchIndex];       // avoid the possibility that the mask changes the value
//    uint8_t value = sentValue & 0b11; // mask for paranoia

//    // put the bits into buf[7]. nextSwitchIndex is in the range 1 through 7 so takes 3 bits
//    // currentSwitches[nextSwitchIndex] is in the range 0 through 2, takes 2 bits.
//    Buffer[7] += (nextSwitchIndex << 2) + value;

//    // update the sent value
//    setSentSwitch(nextSwitchIndex, sentValue);
// }


// #if defined HYBRID_SWITCHES_8 or defined UNIT_TEST

/**
 * Hybrid switches packet encoding for sending over the air
 *
 * Analog channels are reduced to 10 bits to allow for switch encoding
 * Switch[0] is sent on every packet.
 * A 3 bit switch index and 2 bit value is used to send the remaining switches
 * in a round-robin fashion.
 * If any of the round-robin switches have changed
 * we take the lowest indexed one and send that, hence lower indexed switches have
 * higher priority in the event that several are changed at once.
 * 
 * Inputs: crsf.ChannelDataIn, crsf.currentSwitches
 * Outputs: Radio.TXdataBuffer, side-effects the sentSwitch value
 */
// void ICACHE_RAM_ATTR GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, CRSF *crsf, uint8_t addr)
void ICACHE_RAM_ATTR GenerateChannelDataHybridSwitch8(volatile uint8_t* Buffer, const uint16_t scaledADC[], 
                                                      const uint8_t currentSwitches[], const uint8_t nextSwitchIndex, uint8_t addr)
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (addr << 2) + RC_DATA_PACKET;
  Buffer[0] = PacketHeaderAddr;
  Buffer[1] = ((scaledADC[0]) >> 3);
  Buffer[2] = ((scaledADC[1]) >> 3);
  Buffer[3] = ((scaledADC[2]) >> 3);
  Buffer[4] = ((scaledADC[3]) >> 3);
  Buffer[5] = ((scaledADC[0] & 0b110) << 5) + 
                           ((scaledADC[1] & 0b110) << 3) +
                           ((scaledADC[2] & 0b110) << 1) + 
                           ((scaledADC[3] & 0b110) >> 1);


  // switch 0 is sent on every packet - intended for low latency arm/disarm
  Buffer[6] = (currentSwitches[0] & 0b11) << 5; // note this leaves the top bit of byte 6 unused

  // find the next switch to send
  
  uint8_t value = currentSwitches[nextSwitchIndex] & 0b11; // mask for paranoia

  // put the bits into buf[6]. nextSwitchIndex is in the range 1 through 7 so takes 3 bits
  // currentSwitches[nextSwitchIndex] is in the range 0 through 2, takes 2 bits.
  Buffer[6] += (nextSwitchIndex << 2) + value;
}

void ICACHE_RAM_ATTR GenerateChannelDataPWM6(Pwm6Payload_t* outputBuffer, const uint16_t scaledADC[], 
                                             const uint8_t currentSwitches[])
{
    outputBuffer->header = RC_DATA_PACKET;

    outputBuffer->ch0 = scaledADC[0]; // assuming scaledADC holds 11 bit CRSF values already
    outputBuffer->ch1 = scaledADC[1];
    outputBuffer->ch2 = scaledADC[2];
    outputBuffer->ch3 = scaledADC[3];


    // Need to fake up 4 and 5 for testing as we don't have physical analog inputs for them
    // Use switch D to map adc0 onto 4 or 5
    switch (currentSwitches[3])
    {
        case 0:
            outputBuffer->ch4 = CRSF_CHANNEL_VALUE_MID;
            outputBuffer->ch5 = CRSF_CHANNEL_VALUE_MID;
            break;

        case 1:
            outputBuffer->ch4 = scaledADC[0];
            outputBuffer->ch5 = CRSF_CHANNEL_VALUE_MID;
            outputBuffer->ch0 = CRSF_CHANNEL_VALUE_MID;
            break;

        case 2:
            outputBuffer->ch5 = scaledADC[0];
            outputBuffer->ch4 = CRSF_CHANNEL_VALUE_MID;
            outputBuffer->ch0 = CRSF_CHANNEL_VALUE_MID;
            break;

        default:
            printf("unexpected switch value %d\n\r", currentSwitches[3]);
    }

    // TODO add the switches
}


// TODO replace with IS_RX
#ifdef ESPC3

/**
 * Hybrid switches decoding of over the air data
 *
 * Hybrid switches uses 10 bits for each analog channel,
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 *
 * Input: Buffer
 * Output: crsf->PackedRCdataOut
 */
void ICACHE_RAM_ATTR UnpackChannelDataHybridSwitches8(volatile uint8_t* Buffer, CRSF *crsf)
{
    #ifdef USE_ELRS_CRSF_EXTENSIONS
    // When using the elrs extensions we send in our native format and the FC will do the
    // conversions
    // The analog channels
    crsf->PackedRCdataOut.chan0 = (Buffer[1] << 2) + ((Buffer[5] & 0b11000000) >> 6);
    crsf->PackedRCdataOut.chan1 = (Buffer[2] << 2) + ((Buffer[5] & 0b00110000) >> 4);
    crsf->PackedRCdataOut.chan2 = (Buffer[3] << 2) + ((Buffer[5] & 0b00001100) >> 2);
    crsf->PackedRCdataOut.chan3 = (Buffer[4] << 2) + ((Buffer[5] & 0b00000011) >> 0);

    // The low latency switch
    crsf->PackedRCdataOut.aux1 = (Buffer[6] & 0b01100000) >> 5;

    // The round-robin switch
    uint8_t switchIndex = (Buffer[6] & 0b11100) >> 2;
    uint16_t switchValue = Buffer[6] & 0b11;

    switch (switchIndex) {
        case 0:   // we should never get index 0 here since that is the low latency switch
            Serial.println("BAD switchIndex 0");
            break;
        case 1:
            crsf->PackedRCdataOut.aux2 = switchValue;
            break;
        case 2:
            crsf->PackedRCdataOut.aux3 = switchValue;
            break;
        case 3:
            crsf->PackedRCdataOut.aux4 = switchValue;
            break;
        case 4:
            crsf->PackedRCdataOut.aux5 = switchValue;
            break;
        case 5:
            crsf->PackedRCdataOut.aux6 = switchValue;
            break;
        case 6:
            crsf->PackedRCdataOut.aux7 = switchValue;
            break;
        case 7:
            crsf->PackedRCdataOut.aux8 = switchValue;
            break;
    }

    #else
    // Standard crossfire protocol, we do the conversion up front
    // The analog channels
    crsf->PackedRCdataOut.ch0 = (Buffer[1] << 3) + ((Buffer[5] & 0b11000000) >> 5);
    crsf->PackedRCdataOut.ch1 = (Buffer[2] << 3) + ((Buffer[5] & 0b00110000) >> 3);
    crsf->PackedRCdataOut.ch2 = (Buffer[3] << 3) + ((Buffer[5] & 0b00001100) >> 1);
    crsf->PackedRCdataOut.ch3 = (Buffer[4] << 3) + ((Buffer[5] & 0b00000011) << 1);

    // The low latency switch
    crsf->PackedRCdataOut.ch4 = SWITCH2b_to_CRSF((Buffer[6] & 0b01100000) >> 5);

    // The round-robin switch
    uint8_t switchIndex = (Buffer[6] & 0b11100) >> 2;
    uint16_t switchValue = SWITCH2b_to_CRSF(Buffer[6] & 0b11);

    switch (switchIndex) {
        case 0:   // we should never get index 0 here since that is the low latency switch
            printf("BAD switchIndex 0\n");
            break;
        case 1:
            crsf->PackedRCdataOut.ch5 = switchValue;
            break;
        case 2:
            crsf->PackedRCdataOut.ch6 = switchValue;
            break;
        case 3:
            crsf->PackedRCdataOut.ch7 = switchValue;
            break;
        case 4:
            crsf->PackedRCdataOut.ch8 = switchValue;
            break;
        case 5:
            crsf->PackedRCdataOut.ch9 = switchValue;
            break;
        case 6:
            crsf->PackedRCdataOut.ch10 = switchValue;
            break;
        case 7:
            crsf->PackedRCdataOut.ch11 = switchValue;
            break;
    }
    #endif // normal crossfire protocol
}

void UnpackChannelDataPWM6(Pwm6Payload_t *buffer, CRSF *crsf)
{
    // static uint32_t debugCounter = 0;

    // The analog channels
    crsf->PackedRCdataOut.ch0 = buffer->ch0;
    crsf->PackedRCdataOut.ch1 = buffer->ch1;
    crsf->PackedRCdataOut.ch2 = buffer->ch2;
    crsf->PackedRCdataOut.ch3 = buffer->ch3;
    crsf->PackedRCdataOut.ch4 = buffer->ch4;
    crsf->PackedRCdataOut.ch5 = buffer->ch5;

    // if (debugCounter++ % 200 == 0) {
    //     printf("ch0 %u\n", buffer->ch0);
    //     printf("ch1 %u\n", buffer->ch1);
    //     printf("ch2 %u\n", buffer->ch2);
    //     printf("ch3 %u\n", buffer->ch3);
    //     printf("ch4 %u\n", buffer->ch4);
    //     printf("ch5 %u\n", buffer->ch5);
    // }


    // Switches
    crsf->PackedRCdataOut.ch6  = N_to_CRSF(1, buffer->sw0);
    crsf->PackedRCdataOut.ch7  = N_to_CRSF(1, buffer->sw1);
    crsf->PackedRCdataOut.ch8  = N_to_CRSF(1, buffer->sw2);
    crsf->PackedRCdataOut.ch9  = N_to_CRSF(1, buffer->sw3);
    crsf->PackedRCdataOut.ch10 = N_to_CRSF(1, buffer->sw4);
    crsf->PackedRCdataOut.ch11 = N_to_CRSF(1, buffer->sw5);
    // crsf->PackedRCdataOut.ch12 = N_to_CRSF(1, buffer->sw6);
    // crsf->PackedRCdataOut.ch13 = N_to_CRSF(1, buffer->sw7);
}


#endif // ESPC3

// #endif // HYBRID_SWITCHES_8

#ifdef USE_HIRES_DATA

/**
 * HiRes channel
 *
 * uses 12 bits for each analog channel, range 0 - 4095
 * 2 bits for the low latency switch[0]
 * 3 bits for the round-robin switch index and 2 bits for the value
 * 
 * XXX This requires an extra byte in the OTA packet and hence new modes and timings
 * 
 * Input: Buffer
 * Output: crsf->PackedRCdataOut
 */
void ICACHE_RAM_ATTR UnpackHiResChannelData(volatile uint8_t* Buffer, CRSF *crsf)
{
    // When using the elrs extensions we send in our native format and the FC will do the
    // conversions

    // const static uint32_t MAX_OUT = 1811;
    // const static uint32_t MID_OUT =  992;
    // const static uint32_t MIN_OUT =  172;

    // The analog channels
    crsf->PackedHiResRCdataOut.chan0 = (Buffer[1] << 4) + ((Buffer[5] & 0b11110000) >> 4); 
    crsf->PackedHiResRCdataOut.chan1 = (Buffer[2] << 4) + ((Buffer[5] & 0b00001111));
    crsf->PackedHiResRCdataOut.chan2 = (Buffer[3] << 4) + ((Buffer[6] & 0b11110000) >> 4);
    crsf->PackedHiResRCdataOut.chan3 = (Buffer[4] << 4) + ((Buffer[6] & 0b00001111));

    // The low latency switch
    crsf->PackedHiResRCdataOut.aux1 = (Buffer[7] & 0b01100000) >> 5;

    // The round-robin switch
    uint8_t switchIndex = (Buffer[7] & 0b11100) >> 2;
    uint16_t switchValue = Buffer[7] & 0b11;

    switch (switchIndex) {
        case 0:   // we should never get index 0 here since that is the low latency switch
            printf("BAD switchIndex 0\n");
            break;
        case 1:
            crsf->PackedHiResRCdataOut.aux2 = switchValue;
            break;
        case 2:
            crsf->PackedHiResRCdataOut.aux3 = switchValue;
            break;
        case 3:
            crsf->PackedHiResRCdataOut.aux4 = switchValue;
            break;
        default: // higher numbered channels are ignored
            break;
    }

}

#endif // USE_HIRES_DATA


