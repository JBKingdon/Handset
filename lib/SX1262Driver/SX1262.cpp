// Support for SX1262 module


// enable write/read test of the sx1262 fifo
#define RUN_RADIO_BUFFER_TEST

// get access to gnu specific pow10 function
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "SX1262_Regs.h"
#include "SX1262_hal.h"
#include "SX1262.h"

#include "../../src/utils.h"
#include "../../src/config.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#ifdef GD32
extern "C" {
#include "../../include/systick.h"
}

#include "SX1262_hal_GD32.h"
SX1262Hal_GD32 hal;
#else
#include "SX1262_hal_C3.h"
extern void delay(uint32_t x); // TODO find a home for this
SX1262Hal_C3 hal;
#endif
/////////////////////////////////////////////////////////////////
SX1262Driver *SX1262Driver::instance = NULL;

//InterruptAssignment_ InterruptAssignment = NONE;

uint32_t beginTX;
uint32_t endTX;

void ICACHE_RAM_ATTR SX1262Driver::nullCallback(void){return;};

/** Get the "device errors" value and print it
 * 
Table 13-84
    Byte 0    1     2-3
        RFU Status OpError(15:0)

    bit 0 RC64K_CALIB_ERR RC64k calibration failed
    bit 1 RC13M_CALIB_ERR RC13M calibration failed
    bit 2 PLL_CALIB_ERR PLL calibration failed
    bit 3 ADC_CALIB_ERR ADC calibration failed
    bit 4 IMG_CALIB_ERR IMG calibration failed
    bit 5 XOSC_START_ERR XOSC failed to start
    bit 6 PLL_LOCK_ERR PLL failed to lock
    bit 7 RFU RFU
    bit 8 PA_RAMP_ERR PA ramping failed
    bit 15:9 RFU RFU
*/
void debugErrorStatus()
{
    uint8_t status[4];

    hal.ReadCommand(SX1262_RADIO_GET_DEVICE_ERRORS, status, 3);

    uint16_t errors = (status[2] << 8) | status[3];

    printf("device errors: %04X\n\r", errors);

    if (errors & 1) printf("RC64k calibration failed\n\r");
    if (errors & 2) printf("RC13M calibration failed\n\r");
    if (errors & 4) printf("PLL calibration failed\n\r");
    if (errors & 8) printf("ADC calibration failed\n\r");
    if (errors & 16) printf("IMG calibration failed\n\r");
    if (errors & 32) printf("XOSC calibration failed\n\r");
    if (errors & 64) printf("PLL failed to lock\n\r");
    // bit 7 RFU
    if (errors & 256) printf("PA ramping failed\n\r");

}

/**
  13.3.6 SetDIO3AsTCXOCtrl
    This command is used to configure the chip for an external TCXO reference voltage controlled by DIO3.

    Table 13-34: SetDIO3asTCXOCtrl SPI Transaction
    Byte 0          1              2-4
    Opcode = 0x97 tcxoVoltage delay(23:0)
 */
void enableTCXO()
{
    uint8_t command[5];

    const uint32_t delay = 2000/15.625; // delay when turning on the tcxo in units of 15.6 us

    command[0] = SX1262_RADIO_SET_DIO3_AS_TXCO_CTRL;
    command[1] = 0x02;      // 2 = 1v8, which is the 'typical' value according to the docs
    command[2] = (delay >> 16) & 0xFF;
    command[3] = (delay >> 8) & 0xFF;
    command[4] = delay & 0xFF;

    hal.fastWriteCommand(command, sizeof(command));
}

/** 13.1.12 Calibrate Function
    Table 13-17: Calibrate SPI Transaction
    Byte 0              1
        Opcode = 0x89 calibParam

    Table 13-18
        Bit 0   1: R C64k calibration enabled
        Bit 1   1: R C13M calibration enabled
        Bit 2   1: PLL calibration enabled
        Bit 3   1: ADC pulse calibration enabled
        Bit 4   1: ADC bulk N calibration enabled
        Bit 5   1: ADC bulk P calibration enabled
        Bit 6   1: Image calibration enabled
        Bit 7   RFU
*/
void calibrate()
{
    uint8_t command[2];

    // set standby RC mode
    SX1262Driver::instance->SetMode(SX1262_MODE_STDBY_RC);
    // SX1262Driver::instance->SetMode(SX1262_MODE_STDBY_XOSC);

    SX1262Driver::instance->GetStatus();

    // start calibration
    command[0] = SX1262_RADIO_CALIBRATE;
    command[1] = 0b01111111;     // calibrate everything

    hal.fastWriteCommand(command, sizeof(command));

    // needs 3.5ms to complete, no need to be fancy, we're not calibrating often
    delay(5);

    SX1262Driver::instance->GetStatus();
}

/**
    Opcode = 0x98 freq1 freq2
    902 - 928 0xE1 (default) 0xE9 (default)
*/
void calibrateImage()
{
    uint8_t buf[3];
    buf[0] = SX1262_RADIO_CALIBRATE_IMAGE;
    buf[1] = 0xE1;
    buf[2] = 0xE9;

    hal.fastWriteCommand(buf, sizeof(buf));

    delay(5);
}

/**
    Table 13-86
        Opcode= 0x07 0x00 0x00

    Calling clear when there are no errors set seems to cause the adc calibration error to be asserted (?!)
 */
void clearDeviceErrors()
{
    uint8_t command[3];

    command[0] = SX1262_RADIO_CLEAR_DEVICE_ERRORS;
    command[1] = 0;
    command[2] = 0;

    hal.fastWriteCommand(command, sizeof(command));
}

/**
 * 13.1.14 SetPaConfig
SetPaConfig is the command which is used to differentiate the SX1261 from the SX1262. When using this command, the
user selects the PA to be used by the device as well as its configuration.
Table 13-20: SetPaConfig SPI Transaction
Byte 0 1 2 3 4
Opcode = 0x95 paDutyCycle hpMax deviceSel paLut

// Settings for max power
paDutyCycle: 0x04
hpMax:       0x07
deviceSel:   0
paLut:       1

Setting paConfig for lower max power is more efficient than simply reducing the commanded output.

For lower max output (at configured 22dBm out):

power   paDutyCycle hpMax
+22 dBm 0x04        0x07
+20 dBm 0x03        0x05
+17 dBm 0x02        0x03
+14 dBm 0x02        0x02
*/
void setPaConfig()
{
    uint8_t buf[5];

    printf("setPaConfig using max 20dBm settings\n\r");

    buf[0] = SX1262_RADIO_SET_PA_CONFIG;
    buf[1] = 0x03;
    buf[2] = 0x05;
    buf[3] = 0;
    buf[4] = 1;

    hal.fastWriteCommand(buf, sizeof(buf));
}

void setRegulatorMode()
{
    uint8_t buf[2];

    buf[0] = SX1262_RADIO_SET_REGULATORMODE;
    buf[1] = SX1262_USE_DCDC;

    hal.fastWriteCommand(buf, sizeof(buf));
}

// 15.2 Better Resistance of the SX1262 Tx to Antenna Mismatch
// On the SX1262, during the chip initialization, the register TxClampConfig should be modified to optimize the PA clamping
// threshold. Bits 4-1 must be set to “1111” (default value “0100”).
// This register modification must be done after a Power On Reset, or a wake-up from cold Start.
void setTxClampConfig()
{
    const uint16_t txClampReg = 0x08D8;
    // printf("clamp before %02X\n\r", hal.ReadRegister(txClampReg));
    hal.readSetWriteRegister(txClampReg, 0b00011110);
    // printf("clamp after %02X\n\r", hal.ReadRegister(txClampReg));
}

SX1262Driver::SX1262Driver()
{
    instance = this; // TODO get rid of singleton variables
}

void SX1262Driver::End()
{
    hal.end();
    instance->TXdoneCallback = &nullCallback; // remove callbacks (or needed for esp?)
    instance->RXdoneCallback = &nullCallback;
}

/**
 *   @param newTimeout in us
 */
void SX1262Driver::setRxTimeout(uint32_t newTimeout)
{
    rxTimeout = ((float)newTimeout) / 15.625f;
}

/** Clear the timeout registers after rx_done.
 * Not sure I really like having this errata function in the public api as it will make the high
 * level code messy. Would it be enough to hide a call in readRXData() on the assumption that the
 * ISR will always want to get the buffer?
 * 
15.3 Implicit Header Mode Timeout Behavior

When receiving LoRa® packets in Rx mode with Timeout active, and no header (Implicit Mode), the timer responsible for
generating the Timeout (based on the RTC timer) is not stopped on RxDone event. Therefore, it may trigger an unexpected
timeout in any subsequent mode where the RTC isn’t re-invoked, and therefore reset and re-programmed.

It is advised to add the following commands after ANY Rx with Timeout active sequence, which stop the RTC and clear the
timeout event, if any. The register at address 0x0902 will be used to stop the counter, while the register at address 0x0944
will clear the potential event.
 */
void SX1262Driver::clearRxTimeout()
{
    hal.WriteRegister(0x0902, 0);

    hal.readSetWriteRegister(0x0944, 2);
}


/** 15.4 Optimizing the Inverted IQ Operation
    When exchanging LoRa® packets with inverted IQ polarity, some packet losses may be observed for longer packets.
    Bit 2 at address 0x0736 must be set to:
    • “0” when using inverted IQ polarity (see the SetPacketParam(...) command)
    • “1” when using standard IQ polarity
 */
void SX1262Driver::optimizeIQ(bool isInverted)
{
    const uint16_t addr = 0x0736;

    // printf("iq opt before %02X\n\r", hal.ReadRegister(addr));

    if (isInverted) {
        // clear bit 2
        hal.readClearWriteRegister(addr, 4);
    } else {
        // set bit 2
        hal.readSetWriteRegister(addr, 4);
    }

    // printf("iq opt after %02X\n\r", hal.ReadRegister(addr));
}


// lora specific setup
/**
From the datasheet:

Indeed, it is mandatory to set the radio protocol using the command SetPacketType(...) as a first
step before issuing any other radio configuration commands. In a second step, the user should define the modulation
parameter according to the chosen protocol with the command SetModulationParams(...). Finally, the user should then
select the packet format with the command SetPacketParams(...).

1. If not in STDBY_RC mode, then set the circuit in this mode with the command SetStandby()
2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
3. Define the RF frequency with the command SetRfFrequency(...)
4. Define where the data will be stored inside the data buffer in Rx with the command SetBufferBaseAddress(...)
5. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...) 1
6. Define the frame format to be used with the command SetPacketParams(...)
7. Configure DIO and irq: use the command SetDioIrqParams(...) to select the IRQ RxDone and map this IRQ to a DIO (DIO1
or DIO2 or DIO3), set IRQ Timeout as well.
8. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access.
9. Set the circuit in reception mode: use the command SetRx(). Set the parameter to enable timeout or continuous mode
10. Wait for IRQ RxDone 2 or Timeout: the chip will stay in Rx and look for a new packet if the continuous mode is selected
otherwise it will goes to STDBY_RC mode.
11. In case of the IRQ RxDone, check the status to ensure CRC is correct: use the command GetIrqStatus()

*/
void SX1262Driver::setupLora()
{
    // stdby xosc works and saves the tcxo startup time on mode changes
    this->SetMode(SX1262_MODE_STDBY_XOSC);                                    //step 1 put in STDBY mode

    // GetStatus();

    // printf("set packettype\n\r");
    hal.WriteCommand(SX1262_RADIO_SET_PACKETTYPE, SX1262_PACKET_TYPE_LORA); //Step 2: set packet type to LoRa
    // GetStatus();

    // printf("setting frequency\n\r");
    SetFrequency(currFreq);         //Step 3: Set Freq
    // GetStatus();

    // printf("setting fifo addr\n\r");
    // TODO any benefit if we were to have rx and tx buffers at different addresses?
    SetFIFOaddr(0x00, 0x00);        //Step 4: Config FIFO addr
    // GetStatus();

    // printf("set mod params\n\r");
    this->ConfigModParams(currBW, currSF, currCR);                          //Step 5: Configure Modulation Params XXX step 4?
    // GetStatus();

    #ifdef USE_HARDWARE_CRC
    this->SetPacketParams(12, SX1262_LORA_PACKET_IMPLICIT, OTA_PACKET_LENGTH, SX1262_LORA_CRC_ON, SX1262_LORA_IQ_NORMAL);
    #else
    // TODO this ignores the UID based setup of IQ. Doesn't seem to matter, but seems like a problem waiting to happen
    // TODO also note use of fixed preamble length instead of the value from the configured radio params
    // printf("set packet params\n\r");
    this->SetPacketParams(12, SX1262_LORA_PACKET_FIXED_LENGTH, OTA_PACKET_LENGTH_915, SX1262_LORA_CRC_OFF, SX1262_LORA_IQ_NORMAL);
    #endif
    // GetStatus();


    hal.WriteCommand(SX1262_RADIO_SET_RXTX_FALLBACK_MODE, 0x40);    // Table 13-23: enable FS after rx and tx finish
    // GetStatus();

}

void SX1262Driver::setPacketLength(uint8_t packetLen)
{
    SetPacketParams(12, SX1262_LORA_PACKET_FIXED_LENGTH, packetLen, SX1262_LORA_CRC_OFF, SX1262_LORA_IQ_NORMAL);
}

void SX1262Driver::Begin()
{
    // The E22 uses a tcxo so the power-on calibration will fail and has to be done manually after
    // enabling the tcxo via dio3

    hal.init();
    // hal.TXdoneCallback = &SX1262Driver::TXnbISR;
    // hal.RXdoneCallback = &SX1262Driver::RXnbISR;

    // printf("reset\n\r");
    hal.reset();

    // Validate that the sx1262 is working.
    // Read the lora syncword register, should be 0x1424 at power on
    const uint8_t syncWordMSB = hal.ReadRegister(0x0740);
    const uint8_t syncWordLSB = hal.ReadRegister(0x0741);
    const uint16_t loraSyncword = ((uint16_t)syncWordMSB << 8) | syncWordLSB;

    if (loraSyncword == 0x1424) {
        printf("SX126x detected\n\r");
    } else {
        printf("ERROR Failed to find SX1262!\n\r");
        // XXX need to return an error status so we can display a message on the LCD (for the tx)
    }

#ifdef RUN_RADIO_BUFFER_TEST
    printf("testing buffer...\n\r");

    const uint8_t bytesToTest = 100;

    // test that we can write to and read from the radio's buffer
    memset((void*)TXdataBuffer, 0, bytesToTest);
    hal.WriteBuffer(0, TXdataBuffer, bytesToTest); // can't reach the last byte, good enough

    // GetStatus();

    // read it back
    hal.ReadBuffer(0, RXdataBuffer, bytesToTest);
    for(int i=0; i<bytesToTest; i++) {
        if (RXdataBuffer[i] != 0) {
            printf("!!! not 0 at %d:%d !!!\n\r", i, RXdataBuffer[i]);
            break;
        }
    }

    memset((void*)TXdataBuffer, 0xFF, bytesToTest);
    hal.WriteBuffer(0, TXdataBuffer, bytesToTest);

    // read it back
    hal.ReadBuffer(0, RXdataBuffer, bytesToTest);
    for(int i=0; i<bytesToTest; i++) {
        if (RXdataBuffer[i] != 0xFF) {
            printf("!!! not FF at %d:%d !!!\n\r", i, RXdataBuffer[i]);
            break;
        }
    }

    for(int i=0; i<bytesToTest; i++) TXdataBuffer[i] = i;
    hal.WriteBuffer(0, TXdataBuffer, bytesToTest);

    // read it back
    hal.ReadBuffer(0, RXdataBuffer, bytesToTest);
    for(int i=0; i<bytesToTest; i++) {
        if (RXdataBuffer[i] != i) {
            printf("!!! not i at %d:%d !!!\n\r", i, RXdataBuffer[i]);
            break;
        }
    }

    printf("buffer test complete\n\r");
#endif //RUN_RADIO_BUFFER_TEST

    setTxClampConfig();

    setRegulatorMode();

    // GetStatus();

    debugErrorStatus();

    printf("enabling tcxo\n\r");
    enableTCXO();

    // debugErrorStatus();

    // clear errors
    // printf("Clearing errors\n\r");
    clearDeviceErrors();

    // debugErrorStatus();

    // printf("setting xosc\n\r");
    SetMode(SX1262_MODE_STDBY_XOSC);

    delay(2); // settling time
    hal.WaitOnBusy(); // to be sure
    // GetStatus();

    // run calibration
    printf("running calibration\n\r");
    calibrate();

    debugErrorStatus();

    // clear errors
    // printf("Clearing errors\n\r");
    // clearDeviceErrors();

    // debugErrorStatus();

    printf("calibrate image\n\r");
    calibrateImage();
    debugErrorStatus();

    printf("Setting paConfig\n\r");
    setPaConfig();
    // GetStatus();

    setupLora();

    // printf("setting dios\n\r");

    // Using dual dios for rx and tx done
    // this->SetDioIrqParams(SX1262_IRQ_RADIO_ALL, SX1262_IRQ_RX_DONE | SX1262_IRQ_RX_TX_TIMEOUT, SX1262_IRQ_TX_DONE, SX1262_IRQ_RADIO_NONE);
    // this->SetDioIrqParams(SX1262_IRQ_RADIO_ALL, SX1262_IRQ_RX_DONE, SX1262_IRQ_RADIO_ALL, SX1262_IRQ_RADIO_NONE);

    // single dio
    this->SetDioIrqParams(SX1262_IRQ_RADIO_ALL, SX1262_IRQ_RX_DONE | SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_TX_TIMEOUT, SX1262_IRQ_RADIO_NONE, SX1262_IRQ_RADIO_NONE);


    // GetStatus();
}




void ICACHE_RAM_ATTR SX1262Driver::Config(const SX1262_RadioLoRaBandwidths_t bw, const SX1262_RadioLoRaSpreadingFactors_t sf, 
                                          const SX1262_RadioLoRaCodingRates_t cr, const uint32_t freq, 
                                          const uint8_t preambleLength, const bool invertIQ)
{
    SX1262_RadioLoRaIQModes_t iqMode;

    if (invertIQ) {
        iqMode = SX1262_LORA_IQ_INVERTED;
    } else {
        iqMode = SX1262_LORA_IQ_NORMAL;
    }

    // XXX testing, keep IQ normal
    // printf("forcing normal IQ\n\r");
    // iqMode = SX1262_LORA_IQ_NORMAL;
    // printf("forcing inverted IQ\n\r");   // XXX inverted IQ not working? No packets. Try with more preamble
    // iqMode = SX1262_LORA_IQ_INVERTED;

    this->SetMode(SX1262_MODE_STDBY_XOSC);
    // might need to wait for tcxo
    hal.WaitOnBusy();

    ConfigModParams(bw, sf, cr);

    #ifdef USE_HARDWARE_CRC
    SetPacketParams(preambleLength, SX1262_LORA_PACKET_IMPLICIT, OTA_PACKET_LENGTH, SX1262_LORA_CRC_ON, iqMode);
    #else
    SetPacketParams(preambleLength, SX1262_LORA_PACKET_FIXED_LENGTH, OTA_PACKET_LENGTH_915, SX1262_LORA_CRC_OFF, iqMode);
    #endif

    optimizeIQ(iqMode == SX1262_LORA_IQ_INVERTED);

    SetFrequency(freq);
}

/** convert prePA power to mW
* @param power the prePA power as used by currPWR
*/
uint16_t SX1262Driver::convertPowerToMw(int power)
{
    uint16_t mw = 0; // default value if the radio module isn't recognised

    // convert from dBm to mW
    #if defined(RADIO_E22)
    mw = pow10(float(power)/10.0f);
    #else
    printf("unknown radio module\n\r");
    #endif

    return mw;
}

// TODO = should this return a float to better represet output of low power modules?
uint16_t ICACHE_RAM_ATTR SX1262Driver::getPowerMw()
{
    return convertPowerToMw(currPWR);
}

/** Set the SX1262 power output
 *  @param power The output level of the SX1262 chip in dBm,
 * range -9 to +22.

    13.4.4 SetTxParams
        Opcode = 0x8E   power   RampTime

 */
void ICACHE_RAM_ATTR SX1262Driver::SetOutputPower(int8_t power)
{
    #ifndef MAX_PRE_PA_POWER
    #error "Must set MAX_PRE_PA_POWER for SX1262 modules"
    #endif

    // printf("setPower %d\n\r", power);

    if (power > MAX_PRE_PA_POWER) {
        printf("power capped for E22\n");
        power = MAX_PRE_PA_POWER;
    } else if (power < -9) {
        printf("power min limit\n");
        power = -9;
    }

    uint8_t buf[3];
    buf[0] = SX1262_RADIO_SET_TXPARAMS;
    buf[1] = power;
    // printf("power buf[1] %d\n\r", buf[1]);
    buf[2] = (uint8_t)SX1262_RADIO_RAMP_10_US;
    hal.fastWriteCommand(buf, sizeof(buf));

    currPWR = power;

    return;
}

/** 13.4.6 SetPacketParams - 7 byte transfer
 * Table 13-66: LoRa® PacketParam1 & PacketParam2 - PreambleLength
 * Table 13-67: LoRa® PacketParam3 - HeaderType
 * Table 13-68: LoRa® PacketParam4 - PayloadLength
 * Table 13-69: LoRa® PacketParam5 - CRCType
 * Table 13-70: LoRa® PacketParam6 - InvertIQ
 */
void SX1262Driver::SetPacketParams(const uint8_t preambleLength, const SX1262_RadioLoRaPacketLengthsModes_t headerType, 
                                   const uint8_t payloadLength, const SX1262_RadioLoRaCrcModes_t crc, const SX1262_RadioLoRaIQModes_t invertIQ)
{
    uint8_t buf[7];

    buf[0] = SX1262_RADIO_SET_PACKETPARAMS;
    buf[1] = 0;     // high bits of preamble length, but we've only passed 8 bits into this function (and that should be enough for anybody :) )
    buf[2] = preambleLength;
    buf[3] = headerType;
    buf[4] = payloadLength;
    buf[5] = crc;
    buf[6] = invertIQ;

    hal.fastWriteCommand(buf, sizeof(buf));
}


/** Not convinced that this function adds value
 * 
 */
void SX1262Driver::SetMode(SX1262_RadioOperatingModes_t OPmode)
{

    //if (OPmode == currOpmode)
    //{
     //   return;
    //}


    switch (OPmode)
    {
        // 13.1.1 SetSleep 
        // Opcode = 0x84    sleepConfig
        // Table 13-2: Sleep Mode Definition
        // SleepConfig[7:3] SleepConfig [2] SleepConfig [1] SleepConfig [0]
        // RESERVED         0: cold start       0: RFU      0: RTC timeout disable
        // RESERVED         1: warm start       0: RFU      1: wake-up on RTC timeout
        //                  (device configuration               (RC64k)
        //                   in retention) 

    case SX1262_MODE_SLEEP: // never tested
        hal.WriteCommand(SX1262_RADIO_SET_SLEEP, 0);
        break;

    case SX1262_MODE_CALIBRATION:
        printf("calibration NI for setMode\n\r");
        break;

    // 13.1.2 SetStandby
    case SX1262_MODE_STDBY_RC:
        hal.WriteCommand(SX1262_RADIO_SET_STANDBY, SX1262_STDBY_RC);
        break;
    case SX1262_MODE_STDBY_XOSC:
        hal.WriteCommand(SX1262_RADIO_SET_STANDBY, SX1262_STDBY_XOSC);
        break;

    // 13.1.3 SetFs
    case SX1262_MODE_FS:
        {
            uint8_t buf[1] = {SX1262_RADIO_SET_FS};
            // no val
            hal.fastWriteCommand(buf, 1);
            break;
        }

    // 13.1.5 SetRx
    case SX1262_MODE_RX:

        // val is 3 bytes of timeout. 0 = single shot receive with no timeout, 0xFFFFFF is continuous receive mode
        // other vals are actual timeouts in units of 15.625 µs
        {
            uint8_t buf[4];
            buf[0] = SX1262_RADIO_SET_RX;
            buf[1] = (rxTimeout >> 16) & 0xFF;
            buf[2] = (rxTimeout >> 8) & 0xFF;
            buf[3] = (rxTimeout >> 0) & 0xFF;

            hal.fastWriteCommand(buf, sizeof(buf));
            break;
        }
    // 13.1.4 SetTx
    case SX1262_MODE_TX:
        // val is 3 bytes of timeout in units of 15.625 µs.
        // 0 disables the timeout
        {
            uint32_t tmp = 0; // single cycle 0 init
            uint8_t *buf = (uint8_t*)&tmp;
            buf[0] = SX1262_RADIO_SET_TX;
            hal.fastWriteCommand(buf, 4);
            break;
        }
    case SX1262_MODE_CAD:
        printf("CAD not implemented\n\r");
        break;

    default:
        break;
    }

    // currOpmode = OPmode;
}


/** default is low power mode, switch to high sensitivity instead
 
    Rx Gain register: 0x08AC
    Set the gain used in Rx mode: Rx Power Saving gain: 0x94, Rx Boosted gain: 0x96

 */
void setHighSensitivity()
{
    hal.WriteRegister(0x08AC, 0x96); // XXX define the constants
}


/** 13.4.5 SetModulationParams
 *  5 byte transaction
 * 
    SF is defined by the parameter Param[1] . 
    BW corresponds to the bandwidth onto which the LoRa® signal is spread. BW in LoRa® is defined by the parameter Param[2] .
    The Coding Rate (CR) is defined by the parameter Param[3]
    The parameter LdOpt corresponds to the Low Data Rate Optimization (LDRO) Param[4]
 */
void SX1262Driver::ConfigModParams(SX1262_RadioLoRaBandwidths_t bw, SX1262_RadioLoRaSpreadingFactors_t sf, SX1262_RadioLoRaCodingRates_t cr)
{
    uint8_t rfparams[5];

    rfparams[0] = SX1262_RADIO_SET_MODULATIONPARAMS;

    rfparams[1] = (uint8_t)sf;
    rfparams[2] = (uint8_t)bw;
    rfparams[3] = (uint8_t)cr;
    rfparams[4] = 0; // low data rate optimisation, we want it off (0)

    hal.fastWriteCommand(rfparams, sizeof(rfparams));

    setHighSensitivity();
}


/** 13.4.1 SetRfFrequency
 * Opcode = 0x86 RfFreq(31:0)
 */
void SX1262Driver::SetFrequency(uint32_t Reqfreq)
{
    uint8_t buf[5]; //TODO make word alignmed

    buf[0] = SX1262_RADIO_SET_RFFREQUENCY;

    // TODO do the scaling at compile time
    uint32_t freq = (double)Reqfreq * SX1262_FREQ_STEP;
    // printf("reqFreq %lu, scaledFreq %lu\n\r", Reqfreq, freq);

    // val is 4 bytes of frequency
    buf[1] = (uint8_t)((freq >> 24) & 0xFF);
    buf[2] = (uint8_t)((freq >> 16) & 0xFF);
    buf[3] = (uint8_t)((freq >> 8) & 0xFF);
    buf[4] = (uint8_t)(freq & 0xFF);

    hal.fastWriteCommand(buf, sizeof(buf));
    currFreq = Reqfreq;
}


/** 13.4.8 SetBufferBaseAddress
 * Opcode = 0x8F TX base address RX base address
 */
void SX1262Driver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[3];

    buf[0] = SX1262_RADIO_SET_BUFFERBASEADDRESS;
    buf[1] = txBaseAddr;
    buf[2] = rxBaseAddr;

    hal.fastWriteCommand(buf, sizeof(buf));
}

/** 13.3.1 SetDioIrqParams
 *  9 byte transfer
 * 0                        1-2             3-4             5-6            7-8
 * SetDioIrqParams(0x08) Irq Mask(15:0) DIO1Mask(15:0) DIO2Mask(15:0) DIO3Mask(15:0)
 */
void SX1262Driver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[9];

    buf[0] = SX1262_RADIO_SET_DIOIRQPARAMS;

    buf[1] = (uint8_t)((irqMask >> 8) & 0xFF);
    buf[2] = (uint8_t)(irqMask & 0xFF);
    buf[3] = (uint8_t)((dio1Mask >> 8) & 0xFF);
    buf[4] = (uint8_t)(dio1Mask & 0xFF);
    buf[5] = (uint8_t)((dio2Mask >> 8) & 0xFF);
    buf[6] = (uint8_t)(dio2Mask & 0xFF);
    buf[7] = (uint8_t)((dio3Mask >> 8) & 0xFF);
    buf[8] = (uint8_t)(dio3Mask & 0xFF);

    hal.fastWriteCommand(buf, sizeof(buf));
}

/** 13.3.4 ClearIrqStatus
 * 3 byte transfer
 * Opcode = 0x02    ClearIrqParam(15:0)
 */
void SX1262Driver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[3];

    buf[0] = SX1262_RADIO_CLR_IRQSTATUS;

    buf[1] = (uint8_t)(((uint16_t)irqMask >> 8) & 0xFF);
    buf[2] = (uint8_t)((uint16_t)irqMask & 0xFF);

    hal.fastWriteCommand(buf, sizeof(buf));
}

// void SX1262Driver::TXnbISR()
// {
//     //endTX = micros();
//     instance->ClearIrqStatus(SX1262_IRQ_RADIO_ALL);
//     instance->currOpmode = SX1262_MODE_FS; // radio goes to FS
//     //Serial.print("TOA: ");
//     //Serial.println(endTX - beginTX);
//     //instance->GetStatus();

//     // Serial.println("TXnbISR!");
//     //instance->GetStatus();
    
//     //instance->GetStatus();
//     instance->TXdoneCallback();
// }


/**
 *  NB the length param is currently only used for copying the data into the radio's memory,
 * and not for setting the packet length used by the lora modem over the air.
 * This doesn't make much sense, so update the drivers to consistently use a length param on
 * rx and tx calls.
 */
void SX1262Driver::TXnb(volatile uint8_t *data, uint8_t length)
{
    hal.TXenable(); // do first to allow PA to stabilise

    // Errata 15.1:
    // Before any packet transmission, bit #2 at address 0x0889 shall be set to:
    // • 0 if the LoRa BW = 500 kHz
    // • 1 for any other LoRa BW
    // • 1 for any (G)FSK configuration
    if (instance->currBW == SX1262_LORA_BW_500)
    {
        const uint16_t REG_ADDR = 0x889;
        uint8_t buffer[5];
        buffer[1] = (REG_ADDR >> 8) & 0xFF;
        buffer[2] = (REG_ADDR >> 0) & 0xFF;
        uint8_t reg889 = hal.fastReadSingleRegister(buffer);

        reg889 &= ~0x04; // clear bit 2
        // unfortunately the addr got overwritten during the read call, so we have to put it back again
        buffer[1] = (REG_ADDR >> 8) & 0xFF;
        buffer[2] = (REG_ADDR >> 0) & 0xFF;
        buffer[3] = reg889;
        hal.fastWriteSingleRegister(buffer);
    }
    // XXX do we need an else to set the bit for other bandwidths? Surely we could just do that once?


    instance->ClearIrqStatus(SX1262_IRQ_RADIO_ALL);
    hal.WriteBuffer(0x00, data, length); //todo fix offset to equal fifo addr
    instance->SetMode(SX1262_MODE_TX);

    // beginTX = micros();

    // instance->GetStatus();
}

// void SX1262Driver::RXnbISR()
// {
//     instance->currOpmode = SX1262_MODE_FS; // XXX is this true? Unless we're doing single rx with timeout, we'll still be in rx mode

//     // Need to check for hardware crc error
//     #ifdef USE_HARDWARE_CRC
//     // grab the status before we clear it
//     uint16_t irqStat = GetIrqStatus();
//     instance->ClearIrqStatus(SX1262_IRQ_RADIO_ALL);
//     if (irqStat & 0b1000000) {
//         // Serial.println("bad hw crc");
//         return;
//     }
//     #else
//     instance->ClearIrqStatus(SX1262_IRQ_RADIO_ALL);
//     #endif

//     uint8_t FIFOaddr = instance->GetRxBufferAddr();
//     hal.ReadBuffer(FIFOaddr, instance->RXdataBuffer, 8);
//     instance->RXdoneCallback();
// }

// TODO pass in the buffer to use and get rid of the databuffer declarations in this class
void SX1262Driver::readRXData()
{
    // first do the errata fixup for the rx timeout
    clearRxTimeout();
    uint8_t fifoAddr = GetRxBufferAddr();
    hal.ReadBuffer(fifoAddr, instance->RXdataBuffer, OTA_PACKET_LENGTH_915);
}

void SX1262Driver::RXnb()
{
    hal.RXenable();     // sets the rx enable and tx enable pins if required
    instance->ClearIrqStatus(SX1262_IRQ_RADIO_ALL);
    instance->SetMode(SX1262_MODE_RX);  // issues the rx command with continuous mode

    // instance->GetStatus();
}

/** 13.5.2 GetRxBufferStatus
 * Opcode = 0x13    0 0 0
 * returns:
 * rfu Status PayloadLengthRx RxStartBufferPointer
 */
uint8_t ICACHE_RAM_ATTR SX1262Driver::GetRxBufferAddr()
{
    int32_t tmp = 0;    // single instruction zero init
    uint8_t *status = (uint8_t*)&tmp;

    status[0] = SX1262_RADIO_GET_RXBUFFERSTATUS;
    hal.fastWriteCommand(status, 4);
    return status[3];
}

/** This is currently more of a debugStatus than a getStatus.
 * TODO split into a pure getter and a debug routine that calls it 
 * 
 * 
 * 13.5.1 GetStatus
 * Opcode = 0xC0 NOP
 * returns:
 * RFU Status
 * 
 * Table 13-76
 bit 7 reserved
 bits 6:4 mode
 bits 3:1 command status, 2=data available, 3=timeout, 4=command error, 5=execute failure, 6=tx done
 */
void ICACHE_RAM_ATTR SX1262Driver::GetStatus()
{
    uint8_t buf = 0;

    uint8_t mode, status;


    hal.ReadCommand(SX1262_RADIO_GET_STATUS, (uint8_t *)&buf, 1);
    mode = (0b01110000 & buf) >> 4;
    status = (0b00001110 & buf) >> 1;

    printf("mode %u, status: %u\n\r", mode, status);
    switch (mode)
    {
        case 2:
            printf("STBY_RC");
            break;
        case 3:
            printf("STBY_XOSC");
            break;
        case 4:
            printf("FS");
            break;
        case 5:
            printf("RX");
            break;
        case 6:
            printf("TX");
            break;
        default:
            printf("?");
            break;
    }
    printf(", ");

    switch(status)
    {
        case 1:
            printf("rfu");
            break;
        case 2:
            printf("data available");
            break;
        case 3:
            printf("command timeout");
            break;
        case 4:
            printf("command processing error");
            break;
        case 5:
            printf("command execute failure");
            break;
        case 6:
            printf("command tx done");
            break;
    }
    printf("\n\r");

}


/** 13.5.3 GetPacketStatus
 * 
 * 0            1     2       3      4
 * Opcode=0x14 NOP    NOP     NOP    NOP
 * RFU         Status RssiPkt SnrPkt SignalRssiPkt
 * 
 * Average over last packet received of RSSI. Actual signal power is –RssiPkt/2 (dBm)
 * Estimation of SNR on last packet received in two’s compliment format multiplied by 4. Actual SNR in dB =SnrPkt/4
 */
int8_t ICACHE_RAM_ATTR SX1262Driver::GetLastPacketRSSI()
{
    uint8_t buf[5];

    buf[0] = SX1262_RADIO_GET_PACKETSTATUS;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;

    hal.fastWriteCommand(buf, sizeof(buf));

    LastPacketRSSI = -(int8_t)(buf[2]/2);
    // Serial.print("rssi read "); Serial.println(LastPacketRSSI);

    // grab snr while we have the buffer
    LastPacketSNR = (int8_t)(buf[3]/4);

    return LastPacketRSSI;
}

int8_t ICACHE_RAM_ATTR SX1262Driver::GetLastPacketSNR()
{
    GetLastPacketRSSI();
    return LastPacketSNR;
}

/** 13.3.3 GetIrqStatus
 * 0            1     2-3
 * Opcode=0x12 NOP    NOP
 * RFU         Status IrqStatus(15:0)
 */
uint16_t ICACHE_RAM_ATTR SX1262Driver::GetIrqStatus()
{
    int32_t tmp = 0;
    uint8_t *buf = (uint8_t*)&tmp;

    buf[0] = SX1262_RADIO_GET_IRQSTATUS;

    hal.fastWriteCommand(buf, 4);

    return (((uint16_t)buf[2]) << 8) + buf[3];
}
