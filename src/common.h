#pragma once

#include "FHSS.h"
#include "config.h"

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#include "SX1262Driver.h"
#endif

#if defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)
#include "SX1280Driver.h"
#endif

// XXX what does this do?
// #define One_Bit_Switches

extern uint8_t UID[6];
extern uint8_t CRCCaesarCipher;
extern uint8_t DeviceAddr;

typedef enum
{
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7,
    TLM_RATIO_NUM_VALUES = 8

} expresslrs_tlm_ratio_e;

typedef enum
{
    bad_sync_retry = 4,
    bad_sync = 3,
    connected = 2,
    tentative = 1,
    disconnected = 0
} connectionState_e;

typedef enum
{
    tim_disconnected = 0,
    tim_tentative = 1,
    tim_locked = 2
} RXtimerState_e;

extern connectionState_e connectionState;
extern connectionState_e connectionStatePrev;

typedef enum
{
    RF_DOWNLINK_INFO = 0,
    RF_UPLINK_INFO = 1,
    RF_AIRMODE_PARAMETERS = 2
} expresslrs_tlm_header_e;

#ifdef ELRS_OG_COMPATIBILITY
typedef enum
{
    RATE_500HZ = 0,
    RATE_250HZ = 1,
    RATE_200HZ = 2,
    RATE_150HZ = 3,
    RATE_100HZ = 4,
    RATE_50HZ = 5,
    RATE_25HZ = 6,
    RATE_4HZ = 7,
    // RATE_ENUM_MAX = 8
    RATE_LAST = RATE_4HZ
} expresslrs_RFrates_e; // Max value of 16 since only 4 bits have been assigned in the sync package.
#elif defined(USE_DB_PACKETS)
typedef enum
{
    RATE_1KHZ  = 0,
    RATE_500HZ = 1,
    RATE_250HZ = 2,
    RATE_125HZ = 3,
    RATE_LAST = RATE_125HZ
} expresslrs_RFrates_e; // Max value of 16 since only 4 bits have been assigned in the sync package.
#else
typedef enum
{
    RATE_1KHZ  = 0,
    RATE_800HZ = 1,
    RATE_500HZ = 2,
    RATE_250HZ = 3,
    RATE_200HZ = 4,
    RATE_150HZ = 5,
    RATE_125HZ = 6,
    RATE_100HZ = 7,
    RATE_50HZ  = 8,
    RATE_25HZ  = 9,
    RATE_LAST = RATE_25HZ
} expresslrs_RFrates_e; // Max value of 16 since only 4 bits have been assigned in the sync package.
#endif // ELRS_OG_COMPATIBILITY

typedef struct expresslrs_rf_pref_params_s
{
    int8_t index;
    expresslrs_RFrates_e enum_rate; // Max value of 16 since only 4 bits have been assigned in the sync package.
    int32_t RXsensitivity;          //expected RF sensitivity based on
    uint32_t TOA;                   //time on air in microseconds
    uint32_t RFmodeCycleInterval;   // ms before switching to the next mode when trying to establish the connection
    uint32_t RFmodeCycleAddtionalTime;  // the time to keep trying in tentative mode before giving up and starting over
    uint32_t SyncPktIntervalDisconnected;
    uint32_t SyncPktIntervalConnected;
    uint32_t pfdOffset;

} expresslrs_rf_pref_params_s;

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
#ifdef USE_PWM6
// XXX change this to N_RATES
#define RATE_MAX 2
#elif defined(DUAL_BAND_BREADBOARD) || defined(DUAL_BAND_PROTOTYPE)
// dual band uses a single rate on the 915 side, RATE_MAX is only used for 2G4
#else
#define RATE_MAX 4
#define RATE_DEFAULT 0
#endif // USE_PWM6


typedef struct expresslrs_mod_settings_915_s
{
    int8_t index;
    expresslrs_RFrates_e enum_rate;     // Max value of 16 since only 4 bits have been assigned in the sync package.
    SX1262_Bandwidth bw;
    SX1262_RadioLoRaSpreadingFactors_t sf;
    SX1262_RadioLoRaCodingRates_t cr;
    uint32_t interval;                  //interval in us seconds that corresponds to that frequnecy
    expresslrs_tlm_ratio_e TLMinterval; // every X packets is a response TLM packet, should be a power of 2
    uint8_t FHSShopInterval;            // every X packets we hope to a new frequnecy. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t PreambleLen;

} expresslrs_mod_settings_915_t;

#endif // sub ghz bands

#if defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)

#ifdef ELRS_OG_COMPATIBILITY
#define RATE_MAX 4  // actually the number of rates, so the max value is RATE_MAX-1
#define RATE_DEFAULT 0

#else // not compatibility mode

#if defined(USE_HIRES_DATA) || 1
#define RATE_MAX 4  // actually the number of rates, so the max value is RATE_MAX-1
#define RATE_DEFAULT 1
#else
#define RATE_MAX 6  // actually the number of rates, so the max value is RATE_MAX-1
#define RATE_DEFAULT 2
#endif // USE_HIRES_DATA

#endif // ELRS_OG_COMPATIBILITY

typedef enum
{
    LORA,
    FLRC
} ModemType;

typedef struct lora_modem_settings_s
{
    SX1280_RadioLoRaBandwidths_t bw;
    SX1280_RadioLoRaSpreadingFactors_t sf;
    SX1280_RadioLoRaCodingRates_t cr;
    uint8_t PreambleLen;
} lora_modem_settings_t;

typedef struct flrc_modem_settings_s
{
    // flrc stuff
} flrc_modem_settings_t;

typedef struct expresslrs_mod_settings_s
{
    uint8_t index;                      // XXX get rid of?
    expresslrs_RFrates_e enum_rate;     // Max value of 16 since only 4 bits have been assigned in the sync package. XXX get rid of?
    expresslrs_tlm_ratio_e TLMinterval; // every X packets is a response TLM packet, should be a power of 2
    uint8_t FHSShopInterval;            // every X packets we hope to a new frequnecy. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint32_t interval;                  // interval in us seconds between packets

    ModemType modemType;                // whether this mode uses lora or flrc

    union {
        lora_modem_settings_t lora_settings;
        flrc_modem_settings_t flrc_settings;
    };

} expresslrs_mod_settings_t;

#endif // defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)


expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index);
expresslrs_rf_pref_params_s *get_elrs_RFperfParams(int8_t index);

uint8_t ICACHE_RAM_ATTR TLMratioEnumToValue(expresslrs_tlm_ratio_e enumval);

extern expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
extern expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;
//extern expresslrs_mod_settings_s *ExpressLRS_nextAirRate;
//extern expresslrs_mod_settings_s *ExpressLRS_prevAirRate;

extern bool ExpressLRS_AirRateNeedsUpdate;

#ifdef LORA_TEST
extern expresslrs_mod_settings_915_s airRateConfig_LoraTest;
#endif

// XXX needs a better define
#if defined(DUAL_BAND_BREADBOARD) || defined(DUAL_BAND_PROTOTYPE)

extern expresslrs_mod_settings_915_s airRateConfig915;
extern expresslrs_rf_pref_params_s airRateRFPerf915;

#endif // DUAL_BAND


//ELRS SPECIFIC OTA CRC 
//Koopman formatting https://users.ece.cmu.edu/~koopman/crc/
#if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2) || defined(USE_CRC14)
#define ELRS_CRC14_POLY 0x2E57 // 0x372B
#else
#define ELRS_CRC_POLY 0x83 // XXX this is wrong, should be 0x07, but need to keep until we no longer need the older compat level
#endif

