// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
extern "C" {
#include "custom_clock.h"
enum ClockReason : unsigned {ClockInput=1,ClockRadio=2,ClockLink=4,ClockDraw=8};
class ClockGovernor {
public:
    bool low(uint32_t now,unsigned reasons) {
        if(reasons){hold_until_=now+1000u;return false;}
        return clock_due(now,hold_until_);
    }
private:
    uint32_t hold_until_=1000;
};

// Align background work to the same 16 ms grid without changing active deadlines.
constexpr uint32_t maintenance_at(uint32_t at,bool quiet) {
    return quiet?(at+15u)&~15u:at;
}
class WakeDeadline {
public:
    WakeDeadline(uint32_t now):now_(now){}
    void at(uint32_t deadline) {
        const auto delay=static_cast<int32_t>(deadline-now_);
        if(delay<wait_)wait_=delay<=0?1:uint8_t(delay);
    }
    uint8_t interval() const {return wait_;}
private:
    uint32_t now_;
    uint8_t wait_=25;
};
}
namespace rt950 {
constexpr uint8_t encoder_interval(bool screen_awake) {return screen_awake?2u:25u;}
constexpr uint8_t keypad_interval(bool screen_awake) {return screen_awake?10u:25u;}
}
