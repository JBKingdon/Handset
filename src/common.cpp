#include "config.h"
#include "common.h"

// TODO: Validate values for RFmodeCycleAddtionalTime and RFmodeCycleInterval for rates lower than 50HZ

#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_FCC_915) || defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)

#include "SX1262Driver.h"
extern SX1262Driver Radio;

#ifdef LORA_TEST
expresslrs_mod_settings_915_s airRateConfig_LoraTest = 

    // enum_rate,       bw,                 sf,                 cr,         interval, TLMinterval, FHSShopInterval, PreambleLen

    // OTA is about 240ms
    {0, RATE_125HZ, SX1262_LORA_BW_125, SX1262_LORA_SF12, SX1262_LORA_CR_4_5,  500000, TLM_RATIO_1_16,       4,             12};

#endif // LORA_TEST

#if defined(DUAL_BAND_BREADBOARD) || defined(DUAL_BAND_PROTOTYPE) || defined(DB_PCB_V1)

expresslrs_mod_settings_915_s airRateConfig915 = 
    // enum_rate,       bw,                 sf,                 cr,         interval, TLMinterval, FHSShopInterval, PreambleLen

    // 125Hz
    {0, RATE_125HZ, SX1262_LORA_BW_500, SX1262_LORA_SF6, SX1262_LORA_CR_4_6,  8000, TLM_RATIO_1_8,       4,             12}; // 4896 us @ 8 bytes, 6432 @ 15b, 7200 @ 18b
                                                                                                          // pdfOffset at 125Hz 2750               1198
    // measured: 4920 @ 7 bytes, 6520 @ 15 bytes with CR_4_6, preamble 12
    // measured: 4680 @ 7 bytes, 6000 @ 15 bytes with CR_4_5, preamble 10 (turn-around 440us)

expresslrs_rf_pref_params_s airRateRFPerf915 =
    //      rate    sens  TOA RFmodeCycleInterval RFmodeCycleAddtionalTime SyncPktIntervalDisconnected SyncPktIntervalConnected pfdOffset   minSNR, maxSNR
    //       83                     NA                  NA                          NA                      NA
    {0, RATE_125HZ, -112,  6432,    3500,               4000,                       200,                   5000,                1185,       0,0};

#else

expresslrs_mod_settings_915_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    // enum_rate,       bw,                 sf,                 cr,            interval, TLMinterval, FHSShopInterval, PreambleLen

    #ifdef USE_PWM6
    // Fat packets for PWM. 125Hz
    {0, RATE_100HZ, SX1262_LORA_BW_500, SX1262_LORA_SF6, SX1262_LORA_CR_4_8,  8000, TLM_RATIO_1_64,   4,  12},
    {1, RATE_50HZ,  SX1262_LORA_BW_500, SX1262_LORA_SF8, SX1262_LORA_CR_4_6, 20000, TLM_RATIO_1_32,   4,  12}

    #else

    // 333, but dodgy due to the out of spec pre-amble. Maybe with tigher pfd slack or a faster SPI impl?
    {0, RATE_200HZ, SX1262_LORA_BW_500, SX1262_LORA_SF5, SX1262_LORA_CR_4_5, 3000, TLM_RATIO_1_64,   4,  8}, // preamble 8 works, but some crc errors. 10 and 12 don't fit with current pfd setting

    // so what rate can we get with pre-amble 12?
    // 307Hz:
    // {1, RATE_200HZ, SX1262_LORA_BW_500, SX1262_LORA_SF5, SX1262_LORA_CR_4_5, 3250, TLM_RATIO_1_64,   4,  12},

    // 250Hz
    // {0, RATE_250HZ, SX1262_LORA_BW_500, SX1262_LORA_SF5, SX1262_LORA_CR_4_6,  4000, TLM_RATIO_1_64,   4, 12},

    // And then a 125Hz
    {1, RATE_100HZ, SX1262_LORA_BW_500, SX1262_LORA_SF6, SX1262_LORA_CR_4_6,  8000, TLM_RATIO_1_64,   4, 10},

    // These are all based on old 1276 modes and need checking/replacing

    // {0, RATE_200HZ, SX1262_LORA_BW_500, SX1262_LORA_SF6, SX1262_LORA_CR_4_5,  5000, TLM_RATIO_1_64,   4, 8}, // XXX needs 12 preamble
    // {1, RATE_100HZ, SX1262_LORA_BW_500, SX1262_LORA_SF7, SX1262_LORA_CR_4_7, 10000, TLM_RATIO_1_64,   4, 8},  // XXX min preamble 10?
    {2, RATE_50HZ,  SX1262_LORA_BW_500, SX1262_LORA_SF8, SX1262_LORA_CR_4_7, 20000, TLM_RATIO_NO_TLM, 4, 8},
    {3, RATE_25HZ,  SX1262_LORA_BW_500, SX1262_LORA_SF9, SX1262_LORA_CR_4_5, 40000, TLM_RATIO_NO_TLM, 4, 8} // just too tight with cr_4_7 or 6.
    #endif // USE_PWM6
};

// XXX redo all these values
expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    //      rate    sens  TOA RFmodeCycleInterval RFmodeCycleAddtionalTime SyncPktIntervalDisconnected SyncPktIntervalConnected pfdOffset
    #ifdef USE_PWM6
    {0, RATE_100HZ, -112,  6432,    3500,               4000,                       2000,                   5000}, // XXX update this for 12byte payload
    {1, RATE_50HZ,  -120, 18560,    3500,               6000,                       2000,                   5000}

    #else

    {0, RATE_200HZ, -112,  4380,    3500,               2000,                       2000,                   5000},
    {1, RATE_100HZ, -117,  8770,    3500,               4000,                       2000,                   5000},
    {2, RATE_50HZ,  -120, 17540,    3500,               6000,                       2000,                   5000},
    {3, RATE_25HZ,  -123, 17540,    3500,               12000,                      2000,                   5000}
    #endif // USE_PWM6
};

#endif // DUAL_BAND_BREADBOARD

#endif // sub gHz bands


#if defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)

#include "SX1280.h"
// extern SX1280Driver Radio;

#if (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_1_0_0_RC2)
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, TLM_RATIO_1_128, 4, 12},
    {1, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 4, 14},
    {2, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 4, 12},
    {3, RATE_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 4, 12}};

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0, RATE_500HZ, -105, 1507, 2500, 2500, 2000, 4000},
    {1, RATE_250HZ, -108, 3300, 3000, 2500, 2000, 4000},
    {2, RATE_150HZ, -112, 5871, 3500, 2500, 2000, 4000},
    {3, RATE_50HZ, -117, 18443, 4000, 2500, 2000, 4000}};

#elif (ELRS_OG_COMPATIBILITY == COMPAT_LEVEL_DEV_16fbd1d011d060f56dcc9b3a33d9eead819cf440)
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    {0, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_4_5, 2000, TLM_RATIO_1_128, 2, 12},
    {1, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 2, 14},
    {2, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 2, 12},
    {3, RATE_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_4_5, 20000, TLM_RATIO_NO_TLM, 2, 12}};

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    {0, RATE_500HZ, -105, 4380, 3500, 1000, 2000, 5000}, // ~ 3 sync packets
    {1, RATE_250HZ, -108, 4380, 3500, 2500, 2000, 5000}, // ~ 3 sync packets
    {2, RATE_150HZ, -112, 8770, 3500, 2500, 2000, 5000},
    {3, RATE_50HZ, -117, 17540, 3500, 2500, 2000, 5000}}; // this means always send sync on ch[0] as soon as we can

#else // not ELRS_OG_COMPATIBILITY

#if defined(FULL_DIVERSITY_MODES) 
//                                          bw,                 sf,                 cr,             PreambleLen
lora_modem_settings_t loraModem_1khz  = { SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 12};
lora_modem_settings_t loraModem_500hz = { SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 12};
lora_modem_settings_t loraModem_250hz = { SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 12};
lora_modem_settings_t loraModem_125hz = { SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 12};

// flrc combos copied from initial testing in sx1280.cpp
    // this->ConfigModParamsFLRC(FLRC_BR_1_300_BW_1_2, FLRC_CR_1_2, BT_DIS);

    // this->ConfigModParamsFLRC(FLRC_BR_1_040_BW_1_2, FLRC_CR_3_4, BT_DIS);   // being used for D1000, ota 145us, S=-100dBm, cr_1_2 would be 189us, -101dBm

    // this->ConfigModParamsFLRC(FLRC_BR_0_650_BW_0_6, FLRC_CR_3_4, BT_DIS);
    // this->ConfigModParamsFLRC(FLRC_BR_0_650_BW_0_6, FLRC_CR_1_2, BT_DIS);
    // this->ConfigModParamsFLRC(FLRC_BR_0_325_BW_0_3, FLRC_CR_1_2, BT_DIS); // ota 605us, S=-106dBm, possible 1kHz mode

flrc_modem_settings_t flrcModem_1khz_DS = {FLRC_BR_1_040_BW_1_2, FLRC_CR_3_4};
flrc_modem_settings_t flrcModem_1khz = {FLRC_BR_0_325_BW_0_3, FLRC_CR_1_2};

// NB Any changes to these modes will require recalibrating the PFD offsets!
expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    // enum_rate,   Telem,     FHSShopInterval, interval,  modulation, double send, modem settings
    //                                                                                      len  8        9
    {0, RATE_1KHZ,  TLM_RATIO_1_128, 4,         1000,   ModemType::FLRC, true, .flrc_settings = flrcModem_1khz_DS},  //  145us, double send
    {1, RATE_1KHZ,  TLM_RATIO_1_128, 4,         1000,   ModemType::FLRC, false, .flrc_settings = flrcModem_1khz},  //  630us (6 bytes payload)
    {2, RATE_1KHZ,  TLM_RATIO_1_128, 4,         1000,   ModemType::LORA, false, loraModem_1khz},  //   675      714us
    {3, RATE_500HZ, TLM_RATIO_1_128, 4,         2000,   ModemType::FLRC, true, .flrc_settings = flrcModem_1khz},  //  630us (6 bytes payload), double send
    {4, RATE_500HZ, TLM_RATIO_1_128, 4,         2000,   ModemType::LORA, false, loraModem_500hz}, //  1507     1586us, 79%
    {5, RATE_250HZ, TLM_RATIO_1_64,  4,         4000,   ModemType::LORA, false, loraModem_250hz}, //  3172     3330us, 
    {6, RATE_125HZ, TLM_RATIO_1_32,  4,         8000,   ModemType::LORA, false, loraModem_125hz}, //  5872     6187us, 7133 with header
};

// TOA for 9 byte hires packets
expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    //      rate    sens  TOA RFmodeCycleInterval RFmodeCycleAddtionalTime SyncPktIntervalDisconnected SyncPktIntervalConnected pfdOffset   minSNR  maxSNR
    {0, RATE_1KHZ,  -100,  145, 1000,               1000,                       100,                       1000,                260,        40,        0}, // what are min/max for flrc modes?
    {1, RATE_1KHZ,  -106,  605, 1000,               1000,                       100,                       1000,                260,        40,        0},
    {2, RATE_1KHZ,   -99,  714, 1000,               1000,                       100,                       1000,                160,        40,        0},
    {3, RATE_500HZ, -106,  640, 1000,               1000,                       100,                       1000,                317,        40,       90},  // XXX flrc, needs different thresholds
    {4, RATE_500HZ, -105, 1586, 1000,               1000,                       100,                       1000,                317,        40,       90},
    {5, RATE_250HZ, -108, 3330, 1000,               2000,                       100,                       1000,                630,        40,      100},
    // {3, RATE_125HZ, -112, 6187, 2000,               4000,                       100,                       1000,               1880}, // no header
    {6, RATE_125HZ, -112, 7133, 2000,               4000,                       100,                       1000,                934,         0,      100}, // with header
};

#elif defined(USE_HIRES_DATA)

expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    // enum_rate,       bw,                 sf,                 cr,            interval, TLMinterval, FHSShopInterval, PreambleLen
    {0, RATE_1KHZ,  SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 1000,  TLM_RATIO_1_128,     8,          12}, // 714us
    {1, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000,  TLM_RATIO_1_128,     8,          12}, // 1586us, 79%
    {2, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000,  TLM_RATIO_1_64,      8,          12}, // 3330us, 
    {3, RATE_125HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 8000,  TLM_RATIO_1_32,      4,          12}, // 

};

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {
    //      rate    sens  TOA RFmodeCycleInterval RFmodeCycleAddtionalTime SyncPktIntervalDisconnected SyncPktIntervalConnected
    {0, RATE_1KHZ,   -99,  714, 1000,               1000,                       100,                       5000},   // no hw crc
    {1, RATE_500HZ, -105, 1586, 1000,               1000,                       100,                       5000},
    {2, RATE_250HZ, -108, 3330, 1000,               1000,                       100,                       5000},
    {3, RATE_125HZ, -112, 6187, 1000,               4000,                       100,                       5000},   // todo, see if the large RFmodeCycleAddtionalTime can be reduced

};

#else // not USE_HIRES_DATA

expresslrs_mod_settings_s ExpressLRS_AirRateConfig[RATE_MAX] = {
    // enum_rate,       bw,                 sf,                 cr,            interval us, TLMinterval, FHSShopInterval, PreambleLen
    {0, RATE_1KHZ,  SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 1000,  TLM_RATIO_1_128,     8,          12},   // 1000Hz
    // {0, RATE_1KHZ,  SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_7, 1200,  TLM_RATIO_1_128,     8,          12},   // 833Hz, 871us
    {1, RATE_800HZ,  SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_7, 1200,  TLM_RATIO_1_128,     8,          12},   // 871us


    // {1, RATE_800HZ, SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 1250,  TLM_RATIO_1_128,     8,          12},   //  800Hz

    // {1, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_4_5,    2000,  TLM_RATIO_1_128,     8,          12},
    {2, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_7,    2100,  TLM_RATIO_1_128,     8,          12}, // 476Hz, 1744 us

    // Getting rxfail on indoor test quad, even though this is quicker than CR_4_5
    // {1, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000,  TLM_RATIO_1_128,     8,          12},
    {3, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7,   4000,  TLM_RATIO_1_64,      8,          12},
    {4, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7,   7692,  TLM_RATIO_1_32,      4,          12},  // 130Hz
    {5, RATE_50HZ,  SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7,  13333,  TLM_RATIO_1_32,      2,          12}}; //  75Hz

expresslrs_rf_pref_params_s ExpressLRS_AirRateRFperf[RATE_MAX] = {

    // NB RFmodeCycleInterval is used both for the time between switch packet rates when searching for a connection, and also as a
    //    timeout for deciding the link has failed!
    //    RFmodeCycleAddtionalTime is used to timeout a connection that can't get out of tentative state

    //      rate    sens  TOA RFmodeCycleInterval RFmodeCycleAddtionalTime SyncPktIntervalDisconnected SyncPktIntervalConnected
    // {0, RATE_1KHZ,   -99,  753,  500,               1000,                       100,                       5000}, // with hw crc

    // long rx cycle times
    // TODO check the TOA values
    {0, RATE_1KHZ,   -99,  675,  500,               1000,                       100,                       1000},
    {1, RATE_800HZ,  -99,  871,  500,               1000,                       100,                       1000},
    {2, RATE_500HZ, -105, 1626,  500,               1000,                       100,                       1000},
    {3, RATE_250HZ, -108, 3567, 1000,               1000,                       100,                       2000},
    {4, RATE_150HZ, -112, 6660, 1000,               4000,                       100,                       2000},   // todo, see if the large RFmodeCycleAddtionalTime can be reduced
    {5, RATE_50HZ,  -120,12059, 1000,               6000,                       133,                       4000}};

#endif // USE_HIRES_DATA

#endif // ELRS_OG_COMPATIBILITY

#endif // defined(Regulatory_Domain_ISM_2400) || defined(Regulatory_Domain_ISM_2400_NA)

expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index);
//const expresslrs_mod_settings_s * ExpressLRS_nextAirRate;
expresslrs_mod_settings_s *ExpressLRS_currAirRate;
expresslrs_mod_settings_s *ExpressLRS_prevAirRate;

ICACHE_RAM_ATTR expresslrs_mod_settings_s *get_elrs_airRateConfig(int8_t index)
{
    // Protect against out of bounds rate
    if (index < 0)
    {
        // Set to first entry in the array (fastest)
        return &ExpressLRS_AirRateConfig[0];
    }
    else if (index > (RATE_MAX - 1))
    {
        // Set to last usable entry in the array (slowest)
        return &ExpressLRS_AirRateConfig[RATE_MAX - 1];
    }
    return &ExpressLRS_AirRateConfig[index];
}

ICACHE_RAM_ATTR expresslrs_rf_pref_params_s *get_elrs_RFperfParams(int8_t index)
{
    // Protect against out of bounds rate
    if (index < 0)
    {
        // Set to first entry in the array (200HZ)
        return &ExpressLRS_AirRateRFperf[0];
    }
    else if (index > (RATE_MAX - 1))
    {
        // Set to last usable entry in the array (currently 50HZ)
        return &ExpressLRS_AirRateRFperf[RATE_MAX - 1];
    }
    return &ExpressLRS_AirRateRFperf[index];
}

expresslrs_mod_settings_s *ExpressLRS_currAirRate_Modparams;
expresslrs_rf_pref_params_s *ExpressLRS_currAirRate_RFperfParams;

//expresslrs_mod_settings_s *ExpressLRS_nextAirRate;
//expresslrs_mod_settings_s *ExpressLRS_prevAirRate;
bool ExpressLRS_AirRateNeedsUpdate = false;

connectionState_e connectionState = disconnected;
connectionState_e connectionStatePrev = disconnected;

#ifndef MY_UID
//uint8_t UID[6] = {48, 174, 164, 200, 100, 50};
//uint8_t UID[6] = {180, 230, 45, 152, 126, 65}; //sandro unique ID
// uint8_t UID[6] = {180, 230, 45, 152, 125, 173}; // Wez's unique ID
#error "Must define MY_UID"
#else
uint8_t UID[6] = {MY_UID};
#endif

uint8_t CRCCaesarCipher = UID[4];
uint8_t DeviceAddr = UID[5] & 0b111111; // XXX check what this is used for

#define RSSI_FLOOR_NUM_READS 5 // number of times to sweep the noise foor to get avg. RSSI reading
#define MEDIAN_SIZE 20

uint8_t ICACHE_RAM_ATTR TLMratioEnumToValue(expresslrs_tlm_ratio_e enumval)
{
    switch (enumval)
    {
    case TLM_RATIO_NO_TLM:
        return 0;
        break;
    case TLM_RATIO_1_2:
        return 2;
        break;
    case TLM_RATIO_1_4:
        return 4;
        break;
    case TLM_RATIO_1_8:
        return 8;
        break;
    case TLM_RATIO_1_16:
        return 16;
        break;
    case TLM_RATIO_1_32:
        return 32;
        break;
    case TLM_RATIO_1_64:
        return 64;
        break;
    case TLM_RATIO_1_128:
        return 128;
        break;
    default:
        return 0;
    }
}
