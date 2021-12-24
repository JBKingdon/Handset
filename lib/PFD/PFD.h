#pragma once

#include <stdint.h>
#include <iostream>

class PFD
{
private:
    volatile uint32_t intEventTime915 = 0;
    volatile uint32_t extEventTime915 = 0;
    // int32_t result;
    volatile bool gotExtEvent915;
    volatile bool gotIntEvent915;

    volatile uint32_t intEventTime2G4 = 0;
    volatile uint32_t extEventTime2G4 = 0;
    // int32_t result;
    volatile bool gotExtEvent2G4;
    volatile bool gotIntEvent2G4;

public:
    inline void extEvent915(uint32_t time) // reference (external osc)
    {
        if (time == extEventTime915) {
            std::cout << "extEvent915 setting old time\n";
        }
        extEventTime915 = time;
        gotExtEvent915 = true;
    }

    inline void intEvent915(uint32_t time) // internal osc event
    {
        intEventTime915 = time;
        gotIntEvent915 = true;
    }

    inline void reset915()
    {
        gotExtEvent915 = false;
        gotIntEvent915 = false;
    }

    inline int32_t calcResult915()
    {
        int32_t result = (gotExtEvent915 && gotIntEvent915) ? (int32_t)(extEventTime915 - intEventTime915) : 0;
        if (result > 1000000 || result < -1000000) {
            printf("915 ext %u int %u\n", extEventTime915, intEventTime915);
        }
        return result;
    }

    // uint32_t getIntEventTime915() const { return intEventTime915; }
    // uint32_t getExtEventTime915() const { return extEventTime915; }

    inline void extEvent2G4(uint32_t time) // reference (external osc)
    {
        if (time == extEventTime2G4) {
            std::cout << "extEvent2G4 setting old time\n";
        }
        extEventTime2G4 = time;
        gotExtEvent2G4 = true;
    }

    inline void intEvent2G4(uint32_t time) // internal osc event
    {
        intEventTime2G4 = time;
        gotIntEvent2G4 = true;
    }

    inline void reset2G4()
    {
        gotExtEvent2G4 = false;
        gotIntEvent2G4 = false;
    }

    inline int32_t calcResult2G4()
    {
        int32_t result = (gotExtEvent2G4 && gotIntEvent2G4) ? (int32_t)(extEventTime2G4 - intEventTime2G4) : 0;
        if (result > 1000000 || result < -1000000) {
            printf("2G4 ext %u int %u\n", extEventTime2G4, intEventTime2G4);
        }
        // if (!gotExtEvent2G4) std::cout << "no ext evt ";
        // if (!gotIntEvent2G4) std::cout << "no int evt ";
        return result;
    }


    // inline int32_t getResult()
    // {
    //     return result;
    // }

};
