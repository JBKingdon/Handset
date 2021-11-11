#pragma once

/* GD32 specific functions for sx1280
 *
 * TODO If we add an ELRS_Pin class then most of this can move to common code
 */

#define ICACHE_RAM_ATTR

#include <stdint.h>
#include "SX1280_hal.h"

extern "C" {
#include "gd32vf103.h"
}

class SX1280Hal_GD32 : public SX1280Hal
{

private:

protected:

public:
    SX1280Hal_GD32();

    virtual void init() override;
    // void end();
    virtual void reset() override;
    

    virtual bool ICACHE_RAM_ATTR WaitOnBusy() override;
    
    virtual void ICACHE_RAM_ATTR TXenable() override;
    virtual void ICACHE_RAM_ATTR RXenable() override;
    virtual void ICACHE_RAM_ATTR TXRXdisable() override;

};

