#pragma once

#include <stdint.h>

class PFD
{
private:
    volatile uint32_t intEventTime = 0;
    volatile uint32_t extEventTime = 0;
    int32_t result;
    volatile bool gotExtEvent;
    volatile bool gotIntEvent;

public:
    inline void extEvent(uint32_t time) // reference (external osc)
    {
        if (time == extEventTime) {
            printf("extEvent setting old time\n");
        }
        extEventTime = time;
        gotExtEvent = true;
    }

    inline void intEvent(uint32_t time) // internal osc event
    {
        intEventTime = time;
        gotIntEvent = true;
    }

    inline void reset()
    {
        gotExtEvent = false;
        gotIntEvent = false;
    }

    inline int32_t calcResult()
    {
        result = (gotExtEvent && gotIntEvent) ? (int32_t)(extEventTime - intEventTime) : 0;
        if (result > 1000000 || result < -1000000) {
            printf("ext %u int %u\n", extEventTime, intEventTime);
        }
        return result;
    }

    inline int32_t getResult()
    {
        return result;
    }

    uint32_t getIntEventTime() const { return intEventTime; }
    uint32_t getExtEventTime() const { return extEventTime; }
};
