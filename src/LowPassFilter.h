#pragma once
#include "stdint.h"

/////////// Super Simple Fixed Point Lowpass ////////////////
/**
 * Need to confirm the largest value this can hold.
 * It's using signed values, so we have 2^31 bits available.
 * The intermediate calculation deals with the values shifted by Beta + FP_Shift bits, 
 * default is 8, so max value looks to be 2 ^ (31-8) = 2^23 or 8388608.
 * But note that Beta and FP_Shift can be specified in the constructors per instance.
 * 
 * XXX Is it worth calculating the max and doing something if the input is out of range?
 * 
*/
class LPF
{
public:
    int32_t SmoothDataINT;
    int32_t SmoothDataFP;
    int32_t Beta = 3;     // Length = 16
    int32_t FP_Shift = 5; //Number of fractional bits
    bool NeedReset = true; // wait for the first data to upcoming.

    LPF(int Beta_, int FP_Shift_)
    {
        Beta = Beta_;
        FP_Shift = FP_Shift_;
    }

    LPF(int Beta_)
    {
        Beta = Beta_;
    }

    LPF()
    {
        Beta = 3;
        FP_Shift = 5;
    }

    int32_t ICACHE_RAM_ATTR update(int32_t Indata)
    {
        if (NeedReset)
        {
            init(Indata);
            return SmoothDataINT;
        }

        int RawData;
        RawData = Indata;
        RawData <<= FP_Shift; // Shift to fixed point
        SmoothDataFP = (SmoothDataFP << Beta) - SmoothDataFP;
        SmoothDataFP += RawData;
        SmoothDataFP >>= Beta;
        // Don't do the following shift if you want to do further
        // calculations in fixed-point using SmoothData
        SmoothDataINT = SmoothDataFP >> FP_Shift;
        return SmoothDataINT;
    }

    void ICACHE_RAM_ATTR reset()
    {
        NeedReset = true;
    }

    /** Set the current value of the filter to the specified value
     */
    void ICACHE_RAM_ATTR init(int32_t Indata)
    {
        NeedReset = false;
        SmoothDataINT = Indata;
        SmoothDataFP = SmoothDataINT << FP_Shift;
    }
};