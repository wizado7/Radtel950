// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
struct BatteryTxLimits {
    uint8_t low=129; // OEM fallback table at 0x0803093E + 8.
    static constexpr uint8_t High=187; // OEM 0x08009C22.
    bool load(const uint8_t (&factory)[7]) {
        if(!factory[0] || factory[0]==255 || factory[5]>=High)return false;
        for(unsigned i=1;i<6;++i)if(factory[i]<=factory[i-1])return false;
        low=factory[0];
        return true;
    }
    bool permits(uint8_t raw) const {return raw>=low && raw<=High;}
};
bool read_board_battery_limits(BatteryTxLimits &limits);
}
