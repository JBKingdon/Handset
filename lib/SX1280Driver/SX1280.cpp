#define RUN_RADIO_BUFFER_TEST

// get access to gnu specific pow10 function
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <math.h>

#include "SX1280_Regs.h"
#include "SX1280_hal.h"
#include "SX1280.h"

extern "C" {
#include "../../include/systick.h"
}
#include "../../src/utils.h"
#include "../../src/config.h"

#include "OTA.h"

#include <stdio.h>
#include <string.h> // for memset

#ifdef GD32

extern "C" {
#include "../../include/systick.h"
}

#include "SX1280_hal_GD32.h"

#else // not GD32

extern void delay(uint32_t x); // TODO find a home for this
#include "SX1280_hal_C3.h"


#endif // GD32


/////////////////////////////////////////////////////////////////
// SX1280Driver *SX1280Driver::instance = NULL;
//InterruptAssignment_ InterruptAssignment = NONE;

//uint8_t SX127xDriver::_syncWord = SX127X_SYNC_WORD;

//uint8_t SX127xDriver::currPWR = 0b0000;
//uint8_t SX127xDriver::maxPWR = 0b1111;

/* Steps for startup 

1. If not in STDBY_RC mode, then go to this mode by sending the command:
SetStandby(STDBY_RC)

2. Define the LoRa® packet type by sending the command:
SetPacketType(PACKET_TYPE_LORA)

3. Define the RF frequency by sending the command:
SetRfFrequency(rfFrequency)
The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.

4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
byte of the data payload by sending the command:
SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
Note:
txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.

5. Define the modulation parameter signal BW SF CR
*/



// uint32_t beginTX;
// uint32_t endTX;

// void ICACHE_RAM_ATTR SX1280Driver::nullCallback(void){return;};

SX1280Driver::SX1280Driver()
{
    isPrimary = true;
    // instance = this;
    #ifdef GD32
    hal = new SX1280Hal_GD32();
    #elif defined(ESPC3)
    hal = new SX1280Hal_C3();
    #else
    #error("define GD32 or ESPC3")
    #endif
}

SX1280Driver::SX1280Driver(const uint32_t cssPin)
{
    isPrimary = false;
    #ifdef ESPC3
    hal = new SX1280Hal_C3(cssPin);
    #else
    printf("secondary radio only impl for C3\n");
    while(true) {}  // HANG if we get here
    #endif
}


void SX1280Driver::End()
{
    hal->end();
    // instance->TXdoneCallback = &nullCallback; // remove callbacks
    // instance->RXdoneCallback = &nullCallback;
}

// flrc specific setup
void SX1280Driver::setupFLRC(flrc_modem_settings_t modemSettings)
{
    this->SetMode(SX1280_MODE_STDBY_RC);
    hal->WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_FLRC);

    this->ConfigModParamsFLRC(modemSettings.bw, modemSettings.cr, BT_DIS);

    //enable auto FS
    hal->WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);

    // setpacketparams for flrc mode
    SetPacketParamsFLRC();
}

// lora specific setup
void SX1280Driver::setupLora()
{
    this->SetMode(SX1280_MODE_STDBY_RC);                                    //step 1 put in STDBY_RC mode
    hal->WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA); //Step 2: set packet type to LoRa
    this->ConfigModParams(currBW, currSF, currCR);                          //Step 5: Configure Modulation Params
    hal->WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);                        //enable auto FS
    #ifdef USE_HARDWARE_CRC
    this->SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, OTA_PACKET_LENGTH, SX1280_LORA_CRC_ON, SX1280_LORA_IQ_NORMAL);
    #else
    // TODO this ignores the UID based setup of IQ. Doesn't seem to matter, but seems like a problem waiting to happen
    this->SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, OTA_PACKET_LENGTH_2G4, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL);
    #endif
}

void SX1280Driver::hardwareInit()
{
    hal->init();
}

void SX1280Driver::reset()
{
    hal->reset();
}

void SX1280Driver::checkVersion()
{
    uint16_t firmwareRev = (((hal->ReadRegister(REG_LR_FIRMWARE_VERSION_MSB)) << 8) | (hal->ReadRegister(REG_LR_FIRMWARE_VERSION_MSB + 1)));
    printf("Firmware Revision: %u (%X)\n\r", firmwareRev, firmwareRev);
    if (firmwareRev != 0xA9B7) {
        printf("WARNING: firmware not the expected value of 0xA9B7\n\r");
    }
}

int32_t SX1280Driver::Begin()
{
    return Begin(false);
}

/** Initialise the radio module
 * 
 * The config for the DIOs needs to be different on the rx and tx, should we split the DIO setup into
 * it's own function?
 * 
 * @param usePreamble configure DIOs to use preamble detection or not
 * 
 * @return 0 for success, -1 for failure
 */
int32_t SX1280Driver::Begin(const bool usePreamble)
{
    int32_t result = 0;

    // hal->TXdoneCallback = &SX1280Driver::TXnbISR;
    // hal->RXdoneCallback = &SX1280Driver::RXnbISR;
    hardwareInit();

    hal->reset();

    // This block specific to shared reset configs (needs a #define)
    // don't call reset() from the secondary or we wipe out the settings on the primary radio
    // if (isPrimary)
    // {
    //     printf("reset moved to spi impl\n\r");
    //     // hal->reset();
    // } else {
    //     #ifdef ESPC3
    //     // delay(50);
    //     vTaskDelay(50/portTICK_PERIOD_MS);
    //     #endif
    // }

    // expected value is 43447 (A9B7) (TODO add list of other good values as we see them)
    uint16_t firmwareRev = (((hal->ReadRegister(REG_LR_FIRMWARE_VERSION_MSB)) << 8) | (hal->ReadRegister(REG_LR_FIRMWARE_VERSION_MSB + 1)));
    printf("Firmware Revision: %u (%X)\n\r", firmwareRev, firmwareRev);
    if (firmwareRev != 0xA9B7) {
        printf("WARNING: firmware not the expected value of 0xA9B7\n\r");
        result = -1;
    }

    #ifdef RUN_RADIO_BUFFER_TEST
    printf("testing buffer...\n\r");

    const uint8_t bytesToTest = 100;

    // test that we can write to and read from the radio's buffer
    memset((void*)TXdataBuffer, 0, bytesToTest);

    // delay(50);

    hal->WriteBuffer(0, TXdataBuffer, bytesToTest);

    // GetStatus();
    // delay(50);

    // read it back
    hal->ReadBuffer(0, RXdataBuffer, bytesToTest);
    for(int i=0; i<bytesToTest; i++) {
        if (RXdataBuffer[i] != 0) {
            printf("!!! not 0 at %d:%d !!!\n\r", i, RXdataBuffer[i]);
            break;
        }
    }

    // delay(50);

    memset((void*)TXdataBuffer, 0xFF, bytesToTest);
    hal->WriteBuffer(0, TXdataBuffer, bytesToTest);

    // delay(50);

    // read it back
    hal->ReadBuffer(0, RXdataBuffer, bytesToTest);
    for(int i=0; i<bytesToTest; i++) {
        if (RXdataBuffer[i] != 0xFF) {
            printf("!!! not FF at %d:%d !!!\n\r", i, RXdataBuffer[i]);
            break;
        }
    }

    // delay(50);

    for(int i=0; i<bytesToTest; i++) TXdataBuffer[i] = i;
    hal->WriteBuffer(0, TXdataBuffer, bytesToTest);

    // delay(50);

    // read it back
    hal->ReadBuffer(0, RXdataBuffer, bytesToTest);
    for(int i=0; i<bytesToTest; i++) {
        if (RXdataBuffer[i] != i) {
            printf("!!! not i at %d:%d !!!\n\r", i, RXdataBuffer[i]);
            break;
        }
    }

    printf("buffer test complete\n\r");
    #endif //RUN_RADIO_BUFFER_TEST


    #ifdef USE_FLRC
    setupFLRC();
    #else
    setupLora();
    #endif // USE_FLRC

    this->SetFrequency(this->currFreq); //Step 3: Set Freq
    this->SetFIFOaddr(0x00, 0x00);      //Step 4: Config FIFO addr

    // Using dual dios for rx and tx done
    // TODO need to be able to configure if the timeout irq is on dio1 or dio2
    if (usePreamble) {
        this->SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_RX_DONE, SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_PREAMBLE_DETECTED, SX1280_IRQ_RADIO_NONE);
    } else {
        printf("DIO enabling done and timeout\n");
        // unexpected irqs seem to leak through to the isrs, so only enable the ones we actually want
        this->SetDioIrqParams(SX1280_IRQ_RX_DONE | SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT, // enabled irqs
                                SX1280_IRQ_RX_DONE,                             // dio1 mapping
                                SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT,  // dio2 mapping
                                SX1280_IRQ_RADIO_NONE);                         // dio3 mapping
    }
    return result;
}


// void ICACHE_RAM_ATTR SX1280Driver::ConfigFLRC(flrc_modem_settings_t modemSettings, uint32_t freq)
void SX1280Driver::ConfigFLRC(flrc_modem_settings_t modemSettings, uint32_t freq)
{
    this->setupFLRC(modemSettings);
    SetFrequency(freq);
}

/**
 * Used to reconfigure the modem for different packet rates
 * 
 *
*/
void ICACHE_RAM_ATTR SX1280Driver::Config(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, 
                                          SX1280_RadioLoRaCodingRates_t cr, uint32_t freq, const uint8_t preambleLength, 
                                          const bool invertIQ, const bool _explicitHeaders)
{
    SX1280_RadioLoRaIQModes_t iqMode;

    if (invertIQ) {
        iqMode = SX1280_LORA_IQ_INVERTED;
    } else {
        iqMode = SX1280_LORA_IQ_NORMAL;
    }

    explicitHeaders = _explicitHeaders;

    // this->SetMode(SX1280_MODE_STDBY_XOSC); // Try using _FS for quicker changeover ?
    this->SetMode(SX1280_MODE_FS); // using _FS for quicker changeover than STDBY_XOSC

    hal->WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA);

    ConfigModParams(bw, sf, cr);
    #ifdef USE_HARDWARE_CRC
    SetPacketParams(PreambleLength, SX1280_LORA_PACKET_IMPLICIT, OTA_PACKET_LENGTH, SX1280_LORA_CRC_ON, iqMode);
    #else
    if (explicitHeaders) {
        SetPacketParams(preambleLength, SX1280_LORA_PACKET_EXPLICIT, OTA_PACKET_LENGTH_2G4, SX1280_LORA_CRC_OFF, iqMode);
    } else {
        SetPacketParams(preambleLength, SX1280_LORA_PACKET_IMPLICIT, OTA_PACKET_LENGTH_2G4, SX1280_LORA_CRC_OFF, iqMode);
    }
    #endif

    SetFrequency(freq);
}

/** convert prePA power to mW
* @param power the prePA power as used by currPWR
*/
uint16_t SX1280Driver::convertPowerToMw(int power)
{
    // convert from dBm to mW
    #if defined(RADIO_E28_27)
    // for e28-27, PA output is +27dBm of the pre-PA setting, up to a max of 0 input.
    uint16_t mw = pow10(float(power+27)/10.0f);
    #elif defined(RADIO_E28_20)
    // for e28-20, PA output is +22dBm of the pre-PA setting, up to a max of -2 input.
    uint16_t mw = pow10(float(power+22)/10.0f);
    #elif defined(RADIO_E28_12)
    // for e28-12 output is just the current setting
    uint16_t mw = pow10(float(power)/10.0f) + 0.5; // round to nearest
    #else
    // #error("must define a radio module")
    const uint16_t mw = 0;
    #endif

    return mw;
}

// TODO - make subclasses of sx1280 that provide implementations of this method for each radio module 
// TODO = should this return a float to better represet output of low power modules?
uint16_t ICACHE_RAM_ATTR SX1280Driver::getPowerMw()
{
    return convertPowerToMw(currPWR);
}

/** Set the SX1280 power output
 *  @param power The output level of the sx1280 chip itself (pre any PA) in dBm,
 * range -18 to +13 for non-PA modules. The max power will be capped by MAX_PRE_PA_POWER
 * which must be correctly set for the module being used.
 */
void ICACHE_RAM_ATTR SX1280Driver::SetOutputPower(int8_t power)
{
    #ifndef MAX_PRE_PA_POWER
    #error "Must set MAX_PRE_PA_POWER for sx1280 modules"
    #endif

    if (power > MAX_PRE_PA_POWER) {
        printf("power capped for E28\n");
        power = MAX_PRE_PA_POWER;
    } else if (power < -18) {
        printf("power min limit\n");
        power = -18;
    }

    uint8_t buf[2];
    buf[0] = power + 18;
    buf[1] = (uint8_t)SX1280_RADIO_RAMP_04_US;
    hal->WriteCommand(SX1280_RADIO_SET_TXPARAMS, buf, 2);

    currPWR = power;

    // Serial.print("SetPower raw: ");
    // Serial.println(buf[0]);
    return;
}

/**
 * return the current power level in dBm, adjusted for any PA
*/
int8_t SX1280Driver::getPowerDBM()
{
    int8_t result = currPWR;

    #ifdef RADIO_E28_20
    result += 22; 
    #elif defined(RADIO_E28_27)
    result += 27;
    #endif

    return result;
}


/** Set the timeout in us
 * 
 */
void SX1280Driver::setRxTimeout(uint32_t t) 
{
    const uint32_t PeriodBase1000 = 15625; // units of period base in nanoseconds
    const uint32_t tScaled = t * 1000 / PeriodBase1000;
    timeoutHigh = tScaled >> 8;
    timeoutLow = tScaled & 0xFF;
}

/** Set the rx timeout to the special value for continuous receives 
 * 
*/
void SX1280Driver::setRxContinuous() 
{
    timeoutHigh = 0xFF;
    timeoutLow = 0xFF;
}



void SX1280Driver::SetPacketParams(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType, uint8_t PayloadLength, 
                                    SX1280_RadioLoRaCrcModes_t crc, SX1280_RadioLoRaIQModes_t InvertIQ)
{
    uint8_t buf[8];

    buf[0] = SX1280_RADIO_SET_PACKETPARAMS;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = crc;
    buf[5] = InvertIQ;
    buf[6] = 0x00;
    buf[7] = 0x00;

    hal->fastWriteCommand(buf, sizeof(buf));
}

/**
* packetParam1 = AGCPreambleLength
• packetParam2 = SyncWordLength
• packetParam3 = SyncWordMatch
• packetParam4 = PacketType
• packetParam5 = PayloadLength
• packetParam6 = CrcLength
• packetParam7 = Whitening
 */
void SX1280Driver::SetPacketParamsFLRC()
{
    uint8_t buf[8];

    buf[0] = SX1280_RADIO_SET_PACKETPARAMS;
    buf[1] = 0x30;  // PREAMBLE_LENGTH_16_BITS                  0x30
    // buf[1] = 0x70;  // PREAMBLE_LENGTH_32_BITS                  0x70
    buf[2] = 0x04;  // SyncWordLength  FLRC_SYNC_WORD_LEN_P32S  0x04
    buf[3] = 0x10;  // SyncWordMatch  RX_MATCH_SYNC_WORD_1      0x10
    buf[4] = 0x00;  // PacketType PACKET_FIXED_LENGTH           0x00
    buf[5] = OTA_PACKET_LENGTH_2G4_FLRC;     // PayloadLength
    buf[6] = SX1280_RADIO_CRC_2_BYTES;  // CrcLength CRC_2_BYTE                     
    buf[7] = 0x08;  // Whitening must be disabled for FLRC      0x08

    hal->fastWriteCommand(buf, sizeof(buf));

    // Need to write the sync value to registers for syncword 1

    // must avoid 
    //   0x 8C 38 XX XX
    //   0x 63 0E XX XX

    hal->WriteRegister(0x09CF, 0x47);
    hal->WriteRegister(0x09D0, 0x17);
    hal->WriteRegister(0x09D1, 0x23);
    hal->WriteRegister(0x09D2, 0x13);
}


void SX1280Driver::SetMode(SX1280_RadioOperatingModes_t OPmode)
{
    uint8_t buf4[4];

    switch (OPmode)
    {

    case SX1280_MODE_SLEEP:
        hal->WriteCommand(SX1280_RADIO_SET_SLEEP, 0x01);
        break;

    case SX1280_MODE_CALIBRATION:
        break;

    case SX1280_MODE_STDBY_RC:
        hal->WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;
    case SX1280_MODE_STDBY_XOSC:
        hal->WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
        break;

    case SX1280_MODE_FS:
        hal->WriteCommand(SX1280_RADIO_SET_FS, 0x00);
        break;

    case SX1280_MODE_RX:

        buf4[0] = SX1280_RADIO_SET_RX;
        buf4[1] = 0x00; // periodBase = 15.625 μs
        buf4[2] = timeoutHigh;
        buf4[3] = timeoutLow;
        hal->fastWriteCommand(buf4, 4);
        break;

    case SX1280_MODE_TX:
        buf4[0] = SX1280_RADIO_SET_TX;
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf4[1] = 0x00; // periodBase = 15.625 μs
        buf4[2] = 0xFF; // no timeout set for now
        buf4[3] = 0xFF; // TODO dynamic timeout based on expected onairtime
        hal->fastWriteCommand(buf4, 4);
        break;

    case SX1280_MODE_CAD:
        break;

    default:
        break;
    }

    currOpmode = OPmode;
}

/** default is low power mode, switch to high sensitivity instead
 * */
void SX1280Driver::setHighSensitivity()
{
    hal->WriteRegister(0x0891, (hal->ReadRegister(0x0891) | 0xC0));
}



void SX1280Driver::ConfigModParams(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3]; //TODO make word alignmed

    rfparams[0] = (uint8_t)sf;
    rfparams[1] = (uint8_t)bw;
    rfparams[2] = (uint8_t)cr;

    hal->WriteCommand(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));

    /**
     * If the Spreading Factor selected is SF5 or SF6, it is required to use WriteRegister( 0x925, 0x1E )
     • If the Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
     • If the Spreading Factor is SF9, SF10, SF11 or SF12, then the command WriteRegister( 0x925, 0x32 ) must be used
     * In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C
    */
    switch (sf) {
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF5:
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF6:
            hal->WriteRegister(0x925, 0x1E); // for SF5 or SF6
            break;
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF7:
        case SX1280_RadioLoRaSpreadingFactors_t::SX1280_LORA_SF8:
            hal->WriteRegister(0x925, 0x37); // for SF7 or SF8
            break;
        default:
            hal->WriteRegister(0x925, 0x32); // for SF9 and above
    }

    hal->WriteRegister(0x93C, 1); // Freq error compensation mode

    setHighSensitivity();
}

void SX1280Driver::ConfigModParamsFLRC(SX1280_RadioFLRCBandwidths_t bw, SX1280_RadioFLRCCodingRates_t cr, SX1280_RadioFLRCBTFilter_t bt)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3]; //TODO make word aligned

    rfparams[0] = (uint8_t)bw;
    rfparams[1] = (uint8_t)cr;
    rfparams[2] = (uint8_t)bt;

    hal->WriteCommand(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));

    setHighSensitivity();
}

double SX1280Driver::getFreqCompensation()
{
    return freqCompensation;
}

/**
 * Adjust the freqCompensation factor by a ratio expressed as a value
 * close to 1
 * 
 * Multiplying the freqCompensation value by a ratio allows us to close
 * in on a zero error without tracking the absolute correction in the caller.
 * 
 * Limits are applied to freqCompensation to ensure it isn't driven to
 * an unreasonable extreme.
*/
void SX1280Driver::adjustFreqCompensation(double adjFactor)
{
    // Limits, 50ppm is a bit conservative for really bad hardware
    const double MAX_FREQ_COMP = 1.000050;
    const double MIN_FREQ_COMP = 0.999950;

    freqCompensation *= adjFactor;
    if (freqCompensation > MAX_FREQ_COMP) {
        freqCompensation = MAX_FREQ_COMP;
    } else if (freqCompensation < MIN_FREQ_COMP) {
        freqCompensation = MIN_FREQ_COMP;
    }

    freqScalar = freqCompensation / (double)SX1280_FREQ_STEP;
}

void SX1280Driver::SetFrequency(uint32_t Reqfreq, bool startReceive)
{
    //Serial.println(Reqfreq);
    uint8_t buf[3]; //TODO make word alignmed

    // SetMode(SX1280_MODE_STDBY_XOSC);
    // SetMode(SX1280_MODE_FS);

    // TODO C3 doesn't have hardware floats - rewrite for fixed point
    uint32_t freq = (uint32_t)((double)Reqfreq * freqScalar);
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    hal->WriteCommand(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
    currFreq = Reqfreq;

    // if (startReceive) {
    //     SetMode(SX1280_MODE_RX);
        // RXnb(); // also does rxen pin and clears irqs - what's the min we need?
    // } else {
    //     SetMode(SX1280_MODE_FS);
    // }
}

// NB FEI requires explicit header (reasons unknown)
int32_t SX1280Driver::GetFrequencyError()
{
    uint32_t efe;
    uint8_t *efeRaw = ((uint8_t *) &efe) + 1;
    double efeHz = 0.0;

    // read the three registers in one SPI transaction
    hal->ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, efeRaw, 3);

    // only 4 bits of MSB are valid, so mask off the unused bits
    int32_t efeSigned = ((efeRaw[0] & 0xF) << 16) | (efeRaw[1] << 8) | efeRaw[2];

    // sign extend
    if (efeRaw[0] & 0x8) efeSigned |= 0xFFF00000;

    // XXX Need to get the current BW instead of the hardcoded 800.0
    efeHz = 1.55 * (double)efeSigned / (1600.0 / 800.0);

    // printf("FE Hz %f\n", efeHz);

    return efeHz;
}

void SX1280Driver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal->WriteCommand(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

// TODO use bigger buffer and a fastWrite
void SX1280Driver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    hal->WriteCommand(SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void SX1280Driver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    hal->WriteCommand(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
}

// void SX1280Driver::TXnbISR()
// {
//     //endTX = micros();
//     instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
//     instance->currOpmode = SX1280_MODE_FS; // radio goes to FS
//     //Serial.print("TOA: ");
//     //Serial.println(endTX - beginTX);
//     //instance->GetStatus();

//     // Serial.println("TXnbISR!");
//     //instance->GetStatus();
    
//     //instance->GetStatus();
//     instance->TXdoneCallback();
// }

/**
 * Send data over the air
 * 
 * Copy length bytes from *data into the sx1280 buffer and then
 * start the transmission
 * 
 * Since FLRC double sends send the same data twice, if *data is null
 * we can skip the copy and send the existing data. (NB this implies you 
 * shouldn't interleave a receive between the two sends. Who would do that?)
 * 
 * 
 * @param data pointer to the data to be sent, or null if the previous data should be re-used
 * @param length how many bytes to send
*/
void SX1280Driver::TXnb(volatile uint8_t *data, uint8_t length)
{
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    hal->TXenable(); // do first to allow PA stablise
    SetFIFOaddr(0x00, 0x00);   // not 100% sure if needed again
    if (data != NULL) {
        hal->WriteBuffer(0x00, data, length); //todo fix offset to equal fifo addr
    }
    SetMode(SX1280_MODE_TX);
    // beginTX = micros();
}

// void SX1280Driver::RXnbISR()
// {
//     instance->currOpmode = SX1280_MODE_FS; // XXX is this true? Unless we're doing single rx with timeout, we'll still be in rx mode

//     // Need to check for hardware crc error
//     #ifdef USE_HARDWARE_CRC
//     // grab the status before we clear it
//     uint16_t irqStat = GetIrqStatus();
//     instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
//     if (irqStat & 0b1000000) {
//         // Serial.println("bad hw crc");
//         return;
//     }
//     #else
//     instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
//     #endif

//     uint8_t FIFOaddr = instance->GetRxBufferAddr();
//     hal->ReadBuffer(FIFOaddr, instance->RXdataBuffer, 8);
//     instance->RXdoneCallback();
// }

void SX1280Driver::readRXData()
{
    uint8_t nBytes;
    #ifdef USE_FLRC
    nBytes = OTA_PACKET_LENGTH_2G4_FLRC;
    #else
    nBytes = OTA_PACKET_LENGTH_2G4;
    #endif
    uint8_t FIFOaddr = GetRxBufferAddr();
    hal->ReadBuffer(FIFOaddr, RXdataBuffer, nBytes);

    // if (FIFOaddr != 0) {
        // printf("offset %u ", FIFOaddr);
    // }

    // assume we always read from offset 0 (won't work with free running receives)
    // hal->ReadBuffer(FIFOaddr, RXdataBuffer, OTA_PACKET_LENGTH_2G4);
}

void SX1280Driver::RXnb()
{
    //Serial.println("Start RX nb");
    hal->RXenable();
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    // SetFIFOaddr(0x00, 0x00);
    SetMode(SX1280_MODE_RX);
}

// uint8_t ICACHE_RAM_ATTR SX1280Driver::GetRxBufferAddr()
uint8_t SX1280Driver::GetRxBufferAddr()
{
    uint8_t status[2];
    hal->ReadCommand(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

// uint8_t ICACHE_RAM_ATTR SX1280Driver::getStatus()
uint8_t SX1280Driver::getStatus()
{
    uint8_t status = 0;

    hal->ReadCommand(SX1280_RADIO_GET_STATUS, (uint8_t *)&status, 1);

    return status;
}

// void ICACHE_RAM_ATTR SX1280Driver::printStatus(uint8_t status)
void SX1280Driver::printStatus(uint8_t status)
{
    uint8_t stat1;
    uint8_t stat2;
    // bool busy;

    stat1 = (0b11100000 & status) >> 5;
    stat2 = (0b00011100 & status) >> 2;
    // busy = 0b00000001 & status;  // Not according to the datasheet
    // printf("Status: %u, %u, %u (%X)\n", stat1, stat2, busy, status);
    printf("Status: %u, %u (%X)\n", stat1, stat2, status);
}

// bool ICACHE_RAM_ATTR SX1280Driver::GetFrequencyErrorbool()
bool SX1280Driver::GetFrequencyErrorbool()
{
    printf("GetFrequencyErrorbool IMPL NEEDED\n");
    //uint8_t val = hal->ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    //uint8_t val1 = hal->ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
    //uint8_t val2 = hal->ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
    // uint8_t regEFI[3];

    // hal->ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, regEFI, 3);

    //Serial.println(val);
    //Serial.println(val1);
    //Serial.println(val2);

    // Serial.println(regEFI[0]);
    // Serial.println(regEFI[1]);
    // Serial.println(regEFI[2]);

    //bool result = (val & 0b00001000) >> 3;
    //return result; // returns true if pos freq error, neg if false
    return 0;
}

// void ICACHE_RAM_ATTR SX1280Driver::SetPPMoffsetReg(int32_t offset) { return; };
void SX1280Driver::SetPPMoffsetReg(int32_t offset) { return; };


/** Get the packet status data for FLRC mode

 *  Reads the rssi and errors for the last packet.
    Stores the results in the instance packetStatus structure as well as setting LastPacketRSSI
    returns a pointer to the packetStatus struct.

    packetStatus[7:0]   RFU
    packetStatus[15:8]  rssiSync
    packetStatus[16:23] errors
    packetStatus[24:31] status      // doesn't look useful for now
    packetStatus[32:39] sync        // reports which sync word matched, not needed

errors:
    bit 7   reserved
    bit 6   SyncError       sync address detection status for the current packet. Only applicable in Rx when sync address detection is enabled.
    bit 5   LengthError     Asserted when the length of the received packet is greater than the Max length defined in the PAYLOAD_LENGTH parameter.
                            Only applicable in Rx for dynamic length packets.
    bit 4   CrcError        CRC check status of the current packet. The packet is available anyway in the FIFO. Only applicable in Rx when the CRC check is enabled
    bit 3   AbortError      Abort status indicates if the current packet in Rx/Tx was aborted. Applicable both in Rx & Tx.
    bit 2   headerReceived  Indicates if the header for the current packet was received. Only applicable in Rx for dynamic length packets
    bit 1   packetReceived  Indicates that the packet reception is complete. Does not signify packet validity. Only applicable in Rx.
    bit 0   packetCtrlBusy  Indicates that the packet controller is busy. Applicable both in Rx/Tx

 */
// FlrcPacketStatus_s * ICACHE_RAM_ATTR SX1280Driver::GetLastPacketStatusFLRC()
FlrcPacketStatus_s * SX1280Driver::GetLastPacketStatusFLRC()
{
    uint8_t buffer[8] = {0,};

    buffer[0] = SX1280_RADIO_GET_PACKETSTATUS;
    hal->fastWriteCommand(buffer, 5);       // Only need to transfer 5 bytes since status and sync are not currently used
    LastPacketRSSI = -(int8_t)(buffer[3]/2);
    packetStatus.rssi = LastPacketRSSI;

    // Serial.print("rssi read "); Serial.println(LastPacketRSSI);

    packetStatus.errors = buffer[4];

    return &packetStatus;
}


// TODO get rssi/snr can be made more efficient by using a single call to get
// both values, which is hacked in here for now but needs a better interface.
// Not for use with FLRC, see GetLastPacketStatusFLRC() instead.
// int8_t ICACHE_RAM_ATTR SX1280Driver::GetLastPacketRSSI()
int8_t SX1280Driver::GetLastPacketRSSI()
{
    // uint8_t status[2];
    uint32_t tmpB = 0; // cheap clear to 0
    uint8_t *buffer = (uint8_t*)&tmpB;

    // hal->ReadCommand(SX1280_RADIO_GET_PACKETSTATUS, status, 2);
    // LastPacketRSSI = -(int8_t)(status[0]/2);

    buffer[0] = SX1280_RADIO_GET_PACKETSTATUS;
    hal->fastWriteCommand(buffer, 4);
    LastPacketRSSI = -(int8_t)(buffer[2]/2);

    // Serial.print("rssi read "); Serial.println(LastPacketRSSI);

    // grab snr while we have the buffer
    // LastPacketSNR = (int8_t)(status[1]/4);
    LastPacketSNR = (int8_t)buffer[3] / 4;

    return LastPacketRSSI;
}

// int8_t ICACHE_RAM_ATTR SX1280Driver::GetLastPacketSNR()
int8_t SX1280Driver::GetLastPacketSNR()
{
    uint8_t status[2];

    hal->ReadCommand(SX1280_RADIO_GET_PACKETSTATUS, status, 2);
    LastPacketSNR = ((int8_t)status[1])/4;

    // grab rssi while we have the buffer
    LastPacketRSSI = -(int8_t)(status[0]/2);

    return LastPacketSNR;
}

// uint16_t ICACHE_RAM_ATTR SX1280Driver::GetIrqStatus()
uint16_t SX1280Driver::GetIrqStatus()
{
    int32_t buf = 0;
    uint8_t *status = (uint8_t*) &buf;

    hal->ReadCommand(SX1280_RADIO_GET_IRQSTATUS, status, 2);

    return (((uint16_t)status[0]) << 8) + status[1];
}


void SX1280Driver::startCWTest(int8_t power, uint32_t freq)
{
    SetOutputPower(power);
    SetFrequency(freq);

    uint8_t buffer[4];

    buffer[0] = SX1280_RADIO_SET_TXCONTINUOUSWAVE;
    hal->fastWriteCommand(buffer, 1);
}
