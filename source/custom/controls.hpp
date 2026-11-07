// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace rt950 {
class Controls final {
public:
    // Returns a debounced press/repeat code, or 255. Time is milliseconds.
    uint8_t poll(uint8_t raw,uint32_t now);
private:
    uint8_t candidate_=255,accepted_=255;
    uint32_t changed_=0,repeated_=0;
    bool repeating_=false,long_sent_=false;
};
class WakeGuard final {
public:
    void begin(){active_=true;released_=false;}
    bool poll(uint8_t raw,uint32_t now) {
        if(!active_)return true;
        if(raw!=255){released_=false;return false;}
        if(!released_){released_=true;since_=now;}
        if(uint32_t(now-since_)<40u)return false;
        active_=false;return true;
    }
private:
    bool active_=false,released_=false;uint32_t since_=0;
};
class PowerSwitch final {
public:
    bool poll(bool off,uint32_t now) {
        if(!off) {pending_=false;return false;}
        if(!pending_) {pending_=true;since_=now;}
        return uint32_t(now-since_)>=50u;
    }
private:
    bool pending_=false;
    uint32_t since_=0;
};
int key_digit(uint8_t code);
void dispatch_key(class Application &app,uint8_t code);
}
