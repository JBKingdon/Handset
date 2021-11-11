#pragma once

/* C3 specific functions for sx1280
*/

// XXX where's the best place for this to go?
#define ICACHE_RAM_ATTR IRAM_ATTR

#include <stdint.h>
#include "SX1280_hal.h"


class SX1280Hal_C3 : public SX1280Hal
{

private:

protected:

public:
    SX1280Hal_C3();

    virtual void init() override;
    // void end();
    virtual void reset() override;
    

    virtual bool ICACHE_RAM_ATTR WaitOnBusy() override;
    
    virtual void ICACHE_RAM_ATTR TXenable() override;
    virtual void ICACHE_RAM_ATTR RXenable() override;
    virtual void ICACHE_RAM_ATTR TXRXdisable() override;

};

