// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
inline constexpr uint16_t VoxDelays[]={500,1000,1500,2000,3000,5000};
inline constexpr unsigned VoxDelayCount=sizeof VoxDelays/sizeof *VoxDelays;
inline constexpr uint8_t VoxSource=8;

class Vox final {
public:
    static constexpr unsigned SettlingMs=1500;
    static constexpr unsigned SampleExpiryMs=65;
    static constexpr unsigned threshold(unsigned level){return 11+10*level;}
    explicit Vox(uint32_t boot):boot_(boot),blocked_at_(boot){}
    bool ready(uint32_t now,bool blocked) {
        if(blocked) {
            blocked_at_=now;
            on_=false;sampled_=false;
        }
        boot_ready_|=uint32_t(now-boot_)>=5000u;
        return !blocked && boot_ready_ && uint32_t(now-blocked_at_)>=SettlingMs;
    }
    void sample(uint16_t raw,unsigned level,uint16_t delay,uint32_t now) {
        if(raw>=4095 || level<1 || level>9) {ready(now,true);return;}
        sampled_at_=now;sampled_=true;
        const unsigned envelope=raw>>4;
        if(envelope+(on_?8u:0u)>=threshold(level)) {
            on_=true;voice_at_=now;
        } else if(uint32_t(now-voice_at_)>=delay)on_=false;
    }
    bool requested(uint32_t now) {
        if(sampled_ && uint32_t(now-sampled_at_)>=SampleExpiryMs)ready(now,true);
        return on_;
    }
private:
    uint32_t boot_,blocked_at_,sampled_at_=0,voice_at_=0;
    bool boot_ready_=false,on_=false,sampled_=false;
};
}
