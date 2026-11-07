// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
struct SqlThresholds {uint8_t open,close;};

struct SqlCalibration {
    uint8_t open[10]{},close[10]{},uhf[13]{},vhf[10]{};

    bool valid() const {
        for(unsigned i=1;i<10;++i) {
            if(!open[i] || open[i]==255 || close[i]>open[i] ||
               open[i]<open[i-1] || close[i]<close[i-1])return false;
        }
        for(uint8_t value:uhf)if(value>=open[1] || value>close[1])return false;
        for(uint8_t value:vhf)if(value>=open[1] || value>close[1])return false;
        return true;
    }

    SqlThresholds thresholds(uint32_t hz,uint8_t level) const {
        const unsigned mhz=hz/1000000u;
        const unsigned correction=mhz>=400?uhf[(mhz-400)/10]:
            vhf[(mhz-130)/5];
        return {uint8_t(open[level]-correction),uint8_t(close[level]-correction)};
    }
};

bool read_board_sql_calibration(SqlCalibration &out);

inline SqlThresholds fallback_sql(uint8_t level) {
    // Preserve the previously tested level 3 when factory SQL is unavailable.
    const unsigned open=92u+6u*level;
    return {uint8_t(open),uint8_t(open-20u)};
}
}
