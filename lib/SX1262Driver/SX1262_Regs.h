#pragma once

#define SX1262_XTAL_FREQ 32000000

// step is xo / 2^25
#define SX1262_FREQ_STEP (33554432.0D / (double)SX1262_XTAL_FREQ )


/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    SX1262_MODE_SLEEP = 0x00, //! The radio is in sleep mode
    SX1262_MODE_CALIBRATION,  //! The radio is in calibration mode
    SX1262_MODE_STDBY_RC,     //! The radio is in standby mode with RC oscillator
    SX1262_MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
    SX1262_MODE_FS,           //! The radio is in frequency synthesis mode
    SX1262_MODE_RX,           //! The radio is in receive mode
    SX1262_MODE_TX,           //! The radio is in transmit mode
    SX1262_MODE_CAD           //! The radio is in channel activity detection mode
} SX1262_RadioOperatingModes_t;


/**
 * Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 * 
 Table 13-4
    STDBY_RC 0 Device running on RC13M, set STDBY_RC mode
    STDBY_XOSC 1 Device running on XTAL 32MHz, set STDBY_XOSC mode
 */
typedef enum
{
    SX1262_STDBY_RC   = 0x00,
    SX1262_STDBY_XOSC = 0x01
} SX1262_RadioStandbyModes_t;

/**
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
    Table 13-16
        0: Only LDO used for all modes
        1: DC_DC+LDO used for STBY_XOSC,FS, RX and TX modes
 */
typedef enum
{
    SX1262_USE_LDO  = 0x00,
    SX1262_USE_DCDC = 0x01
} SX1262_RadioRegulatorModes_t;

/**
 * Represents the possible packet type (i.e. modem) used
    PACKET_TYPE_GFSK 0x00 GFSK packet type
    PACKET_TYPE_LORA 0x01 LORA mode
 */
typedef enum
{
    SX1262_PACKET_TYPE_GFSK = 0x00,
    SX1262_PACKET_TYPE_LORA = 0x01
} SX1262_RadioPacketTypes_t;


/**
 * 
  Table 13-70
    0x00 Standard IQ setup
    0x01 Inverted IQ setup
 */
typedef enum
{
    SX1262_LORA_IQ_NORMAL   = 0x00,
    SX1262_LORA_IQ_INVERTED = 0x01 // XXX if using inverted check the datasheet for work-around
} SX1262_RadioLoRaIQModes_t;


/**
 * Represents the ramping time for power amplifier
 
Table 13-41
    SET_RAMP_10U    0x00 10
    SET_RAMP_20U    0x01 20
    SET_RAMP_ 40U   0x02 40
    SET_RAMP_80U    0x03 80
    SET_RAMP_200U   0x04 200
    SET_RAMP_800U   0x05 800
    SET_RAMP_1700U  0x06 1700
    SET_RAMP_3400U  0x07 3400
 */
typedef enum
{
    SX1262_RADIO_RAMP_10_US = 0x00,
    SX1262_RADIO_RAMP_20_US = 0x01
} SX1262_RadioRampTimes_t;

/**
 * Represents the number of symbols to be used for channel activity detection operation
   XXX check
 */
// typedef enum
// {
//     SX1262_LORA_CAD_01_SYMBOL  = 0x00,
//     SX1262_LORA_CAD_02_SYMBOLS = 0x20,
//     SX1262_LORA_CAD_04_SYMBOLS = 0x40,
//     SX1262_LORA_CAD_08_SYMBOLS = 0x60,
//     SX1262_LORA_CAD_16_SYMBOLS = 0x80,
// } SX1262_RadioLoRaCadSymbols_t;

/**
 * Represents the possible spreading factor values in LORA packet types

 Table 13-47
    0x05 SF5
    0x06 SF6
    0x07 SF7
    0x08 SF8
    0x09 SF9
    0x0A SF10
    0x0B SF11
    0x0C SF12

 */
typedef enum
{
    SX1262_LORA_SF5  = 0x5,
    SX1262_LORA_SF6  = 0x6,
    SX1262_LORA_SF7  = 0x7,
    SX1262_LORA_SF8  = 0x8,
    SX1262_LORA_SF9  = 0x9,
    SX1262_LORA_SF10 = 0xA,
    SX1262_LORA_SF11 = 0xB,
    SX1262_LORA_SF12 = 0xC
} SX1262_RadioLoRaSpreadingFactors_t;

/**
 * Represents the bandwidth values for the LORA packet type

Table 13-48

    0x00 LORA_BW_7 (7.81 kHz real)
    0x08 LORA_BW_10 (10.42 kHz real)
    0x01 LORA_BW_15 (15.63 kHz real)
    0x09 LORA_BW_20 (20.83 kHz real)
    0x02 LORA_BW_31 (31.25 kHz real)
    0x0A LORA_BW_41 (41.67 kHz real)
    0x03 LORA_BW_62 (62.50 kHz real)
    0x04 LORA_BW_125 (125 kHz real)
    0x05 LORA_BW_250 (250 kHz real)
    0x06 LORA_BW_500 (500 kHz real)
 
 */
typedef enum SX1262_Bandwidth
{
    SX1262_LORA_BW_250 = 0x05,
    SX1262_LORA_BW_500 = 0x06,
 
} SX1262_RadioLoRaBandwidths_t;


/**
 * Represents the coding rate values for the LORA packet type

  Table 13-49
    0x01 LORA_CR_4_5
    0x02 LORA_CR_4_6
    0x03 LORA_CR_4_7
    0x04 LORA_CR_4_8
 */
typedef enum
{
    SX1262_LORA_CR_4_5 = 0x01,
    SX1262_LORA_CR_4_6 = 0x02,
    SX1262_LORA_CR_4_7 = 0x03,
    SX1262_LORA_CR_4_8 = 0x04
} SX1262_RadioLoRaCodingRates_t;

/**
  Table 13-67
    0x00 Variable length packet (explicit header)
    0x01 Fixed length packet (implicit header)
*/
typedef enum
{
    SX1262_LORA_PACKET_VARIABLE_LENGTH = 0x00,
    SX1262_LORA_PACKET_FIXED_LENGTH    = 0x01
} SX1262_RadioLoRaPacketLengthsModes_t;

/**
  Table 13-69
    0x00 CRC OFF
    0x01 CRC ON
 */
typedef enum
{
    SX1262_LORA_CRC_ON  = 0x01,
    SX1262_LORA_CRC_OFF = 0x00
} SX1262_RadioLoRaCrcModes_t;

/**
 * From Chapter 11
 */
typedef enum SX1262_RadioCommands
{
    SX1262_RADIO_GET_STATUS                 = 0xC0,
    SX1262_RADIO_WRITE_REGISTER             = 0x0D,
    SX1262_RADIO_READ_REGISTER              = 0x1D,
    SX1262_RADIO_WRITE_BUFFER               = 0x0E,
    SX1262_RADIO_READ_BUFFER                = 0x1E,
    SX1262_RADIO_SET_SLEEP                  = 0x84,
    SX1262_RADIO_SET_STANDBY                = 0x80,
    SX1262_RADIO_SET_FS                     = 0xC1,
    SX1262_RADIO_SET_TX                     = 0x83,
    SX1262_RADIO_SET_RX                     = 0x82,
    SX1262_RADIO_SET_RXTX_FALLBACK_MODE     = 0x93,
    SX1262_RADIO_SET_RXDUTYCYCLE            = 0x94,
    SX1262_RADIO_SET_CAD                    = 0xC5,
    SX1262_RADIO_SET_TXCONTINUOUSWAVE       = 0xD1,
    SX1262_RADIO_SET_TXCONTINUOUSPREAMBLE   = 0xD2,
    SX1262_RADIO_SET_PACKETTYPE             = 0x8A,
    SX1262_RADIO_GET_PACKETTYPE             = 0x11,
    SX1262_RADIO_SET_RFFREQUENCY            = 0x86,
    SX1262_RADIO_SET_TXPARAMS               = 0x8E,
    SX1262_RADIO_SET_CADPARAMS              = 0x88,
    SX1262_RADIO_SET_BUFFERBASEADDRESS      = 0x8F,
    SX1262_RADIO_SET_MODULATIONPARAMS       = 0x8B,
    SX1262_RADIO_SET_PACKETPARAMS           = 0x8C,
    SX1262_RADIO_GET_RXBUFFERSTATUS         = 0x13,
    SX1262_RADIO_GET_PACKETSTATUS           = 0x14,
    SX1262_RADIO_GET_RSSIINST               = 0x15,
    SX1262_RADIO_GET_DEVICE_ERRORS          = 0x17,
    SX1262_RADIO_CLEAR_DEVICE_ERRORS        = 0x07,
    SX1262_RADIO_SET_DIOIRQPARAMS           = 0x08, 
    SX1262_RADIO_SET_DIO3_AS_TXCO_CTRL      = 0x97,
    SX1262_RADIO_GET_IRQSTATUS              = 0x12,
    SX1262_RADIO_CLR_IRQSTATUS              = 0x02, 
    SX1262_RADIO_CALIBRATE                  = 0x89,
    SX1262_RADIO_CALIBRATE_IMAGE            = 0x98,
    SX1262_RADIO_SET_PA_CONFIG              = 0x95,
    SX1262_RADIO_SET_REGULATORMODE          = 0x96,

} SX1262_RadioCommands_t;


/** Interrupt bits
 *
  Table 13-29 
    0 TxDone Packet transmission completed All
    1 RxDone Packet received All
    2 PreambleDetected Preamble detected All
    3 SyncWordValid Valid sync word detected FSK
    4 HeaderValid Valid LoRa header received LoRa®
    5 HeaderErr LoRa header CRC error LoRa®
    6 CrcErr Wrong CRC received All
    7 CadDone Channel activity detection finished LoRa®
    8 CadDetected Channel activity detected LoRa®
    9 Timeout Rx or Tx timeout All
*/
typedef enum
{
    SX1262_IRQ_RADIO_NONE        = 0x0000,
    SX1262_IRQ_TX_DONE           = 0x0001,
    SX1262_IRQ_RX_DONE           = 0x0002,
    SX1262_IRQ_PREAMBLE_DETECTED = 0x0004,
    SX1262_IRQ_SYNCWORD_VALID    = 0x0008,
    SX1262_IRQ_HEADER_VALID      = 0x0010,
    SX1262_IRQ_HEADER_ERROR      = 0x0020,
    SX1262_IRQ_CRC_ERROR         = 0x0040,
    SX1262_IRQ_CAD_DONE          = 0x0080,
    SX1262_IRQ_CAD_DETECTED      = 0x0100,
    SX1262_IRQ_RX_TX_TIMEOUT     = 0x0200,
    SX1262_IRQ_RADIO_ALL         = 0x03FF
} SX1262_RadioIrqMasks_t;
