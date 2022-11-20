#include <stdlib.h>
#include "FHSS.h"
#include "Serial.h" // for?
#include "config.h"

uint8_t volatile FHSSptr915 = 0; // XXX must be 8 bit unsigned to wrap when incremented,
uint8_t volatile FHSSptr2G4 = 0; // XXX must be 8 bit unsigned to wrap when incremented,

#if (NR_SEQUENCE_ENTRIES != 256)
#error "hop sequence table must be 256 entries"
#endif

uint8_t FHSSsequence915[NR_SEQUENCE_ENTRIES] = {0};
uint8_t FHSSsequence2G4[NR_SEQUENCE_ENTRIES] = {0};

int32_t FreqCorrection = 0;

// ---------- 915 functions ----------

void ICACHE_RAM_ATTR FHSSsetCurrIndex915(uint8_t value)
{ // set the current index of the FHSS pointer
    FHSSptr915 = value;
}

uint8_t ICACHE_RAM_ATTR FHSSgetCurrIndex915()
{ // get the current index of the FHSS pointer
    return FHSSptr915;
}

uint32_t ICACHE_RAM_ATTR GetInitialFreq915()
{
    return FHSSfreqs915[0] - FreqCorrection;
}

uint32_t ICACHE_RAM_ATTR FHSSgetCurrFreq915()
{
    return FHSSfreqs915[FHSSsequence915[FHSSptr915]] - FreqCorrection;
}

uint32_t ICACHE_RAM_ATTR FHSSgetNextFreq915()
{
    FHSSptr915++;  // as long as FHSSptr is uint8 it will wrap without extra code
    return FHSSgetCurrFreq915();
}

// ---------- 2G4 functions ----------

void ICACHE_RAM_ATTR FHSSsetCurrIndex2G4(uint8_t value)
{ // set the current index of the FHSS pointer
    FHSSptr2G4 = value;
}

uint8_t ICACHE_RAM_ATTR FHSSgetCurrIndex2G4()
{ // get the current index of the FHSS pointer
    return FHSSptr2G4;
}

uint32_t ICACHE_RAM_ATTR GetInitialFreq2G4()
{
    return FHSSfreqs2G4[0];
}

uint32_t ICACHE_RAM_ATTR FHSSgetCurrFreq2G4()
{
    return FHSSfreqs2G4[FHSSsequence2G4[FHSSptr2G4]];
}

/** return the frequency to be used for an FLRC second packet in a double send
 * 
 *  Takes the existing index into the channel table and adds half the table size,
 *  wrapping around to the beginning if necessary
 * 
 */
uint32_t ICACHE_RAM_ATTR FHSSgetCurrDupSendFreq2G4()
{
    // NR_FHSS_ENTRIES_2G4 is the number of channels in the frequency table (needs renaming)
    uint32_t dupIndex = (FHSSsequence2G4[FHSSptr2G4] + (NR_FHSS_ENTRIES_2G4 / 2)) % NR_FHSS_ENTRIES_2G4;
    return FHSSfreqs2G4[dupIndex];
}


uint32_t ICACHE_RAM_ATTR FHSSgetNextFreq2G4()
{
    FHSSptr2G4++;  // as long as FHSSptr is uint8 it will wrap without extra code
    return FHSSgetCurrFreq2G4();
}


// --------------------------------------------------------------------------

// Set all of the flags in the array to true, except for the first one
// which corresponds to the sync channel and is never available for normal
// allocation.
void ICACHE_RAM_ATTR resetIsAvailable(uint8_t *array, const uint8_t nEntries)
{
    // channel 0 is the sync channel and is never considered available
    array[0] = 0;

    // all other entires to 1
    for (unsigned int i = 1; i < nEntries; i++)
        array[i] = 1;
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pesudorandom

Approach:
  Initialise an array of flags indicating which channels have not yet been assigned and a counter of how many channels are available
  Iterate over the FHSSsequence array using index
    if index is a multiple of SYNC_INTERVAL assign the sync channel index (0)
    otherwise, generate a random number between 0 and the number of channels left to be assigned
    find the index of the nth remaining channel
    if the index is a repeat, generate a new random number
    if the index is not a repeat, assing it to the FHSSsequence array, clear the availability flag and decrement the available count
    if there are no available channels left, reset the flags array and the count
*/
void ICACHE_RAM_ATTR FHSSrandomiseFHSSsequence(uint8_t FHSSsequence[], const uint8_t nEntries, const uint8_t syncInterval)
{
    Serial.print("Number of FHSS frequencies =");
    Serial.println(nEntries);

    long macSeed = ((long)UID[2] << 24) + ((long)UID[3] << 16) + ((long)UID[4] << 8) + UID[5];
    rngSeed(macSeed);
    // srandom(macSeed);

    uint8_t isAvailable[nEntries];

    resetIsAvailable(isAvailable, nEntries);

    // Fill the FHSSsequence with channel indices
    // The 0 index is special - the 'sync' channel. The sync channel appears every
    // syncInterval hops. The other channels are randomly distributed between the
    // sync channels
    const int SYNC_INTERVAL = syncInterval;


    int nLeft = nEntries - 1; // how many channels are left to be allocated. Does not include the sync channel
    unsigned int prev = 0;    // needed to prevent repeats of the same index

    // for each slot in the sequence table
    for (int i = 0; i < NR_SEQUENCE_ENTRIES; i++)
    {
        if (i % SYNC_INTERVAL == 0)
        {
            // assign sync channel 0
            FHSSsequence[i] = 0;
            prev = 0;
        }
        else
        {
            // pick one of the available channels. May need to loop to avoid repeats
            unsigned int index;
            do
            {
                int c = rngN(nLeft); // returns 0 <= c < nLeft
                // int c = random() % nLeft;
                // find the c'th entry in the isAvailable array
                // can skip 0 as that's the sync channel and is never available for normal allocation
                index = 1;
                int found = 0;
                while (index < nEntries)
                {
                    if (isAvailable[index])
                    {
                        if (found == c)
                            break;
                        found++;
                    }
                    index++;
                }
                if (index == nEntries)
                {
                    // This should never happen
                    Serial.print("FAILED to find the available entry!\n");
                    // What to do? We don't want to hang as that will stop us getting to the wifi hotspot
                    // Use the sync channel
                    index = 0;
                    break;
                }
            } while (index == prev); // can't use index if it repeats the previous value

            FHSSsequence[i] = index; // assign the value to the sequence array
            isAvailable[index] = 0;  // clear the flag
            prev = index;            // remember for next iteration
            nLeft--;                 // reduce the count of available channels
            if (nLeft == 0)
            {
                // we've assigned all of the channels, so reset for next cycle
                resetIsAvailable(isAvailable, nEntries);
                nLeft = nEntries - 1;
            }
        }

        Serial.print(FHSSsequence[i]);
        if ((i + 1) % 10 == 0)
        {
            Serial.println();
        }
        else
        {
            Serial.print(" ");
        }
    } // for each element in FHSSsequence

    Serial.println();
}

void ICACHE_RAM_ATTR FHSSrandomiseFHSSsequences()
{
    // uint8_t FHSSsequence[], const uint8_t nEntries, const uint8_t syncInterval)

    printf("915 FHSS\n");
    FHSSrandomiseFHSSsequence(FHSSsequence915, NR_FHSS_ENTRIES_915, 20);
    printf("2G4 FHSS\n");
    FHSSrandomiseFHSSsequence(FHSSsequence2G4, NR_FHSS_ENTRIES_2G4, NR_FHSS_ENTRIES_2G4);
}


