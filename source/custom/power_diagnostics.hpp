// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950 {
// A volatile history of actual battery ADC samples around the last TX.
// It neither measures RF power nor controls the transmitter.
class TxSupplyTrace final {
public:
    enum class Owner : uint8_t { Voice, Aprs };
    void observe(uint32_t now,bool transmitting,uint32_t hz,Owner owner,uint8_t code,
                 unsigned mv,uint32_t age,bool fresh) {
        const bool usable=fresh && mv && age<2000u;
        if(!transmitting) {
            active_=false;
            idle_valid_=usable;
            if(usable){idle_mv_=uint16_t(mv);idle_stamp_=now-age;}
            return;
        }
        if(!active_ || frequency_!=hz || owner_!=owner) {
            active_=true;has_session_=true;frequency_=hz;owner_=owner;started_=now;
            before_=idle_valid_ && uint32_t(now-idle_stamp_)<2000u?idle_mv_:0;
            idle_valid_=false;
            minimum_=maximum_=0;samples_=0;sample_seen_=false;
        }
        code_=code;
        // A sample from before keying is not evidence of voltage under TX.
        if(!usable || age>=uint32_t(now-started_))return;
        const uint32_t stamp=now-age;
        if(sample_seen_ && stamp==sample_stamp_)return;
        sample_seen_=true;sample_stamp_=stamp;
        if(!samples_)minimum_=maximum_=uint16_t(mv);
        else {
            if(mv<minimum_)minimum_=uint16_t(mv);
            if(mv>maximum_)maximum_=uint16_t(mv);
        }
        if(samples_<65535u)++samples_;
    }
    bool has_session() const {return has_session_;}
    uint32_t frequency() const {return frequency_;}
    Owner owner() const {return owner_;}
    unsigned before() const {return before_;}
    unsigned minimum() const {return minimum_;}
    unsigned maximum() const {return maximum_;}
    unsigned samples() const {return samples_;}
    unsigned code() const {return code_;}
    bool sag_valid() const {return before_ && samples_;}
    int sag() const {return int(before_)-int(minimum_);}
private:
    uint32_t frequency_=0,started_=0,idle_stamp_=0,sample_stamp_=0;
    uint16_t idle_mv_=0,before_=0,minimum_=0,maximum_=0,samples_=0;
    uint8_t code_=0;
    Owner owner_=Owner::Voice;
    bool active_=false,has_session_=false,idle_valid_=false,sample_seen_=false;
};
struct PowerDiagnostics {
    TxSupplyTrace trace{};
    uint32_t adc_age=0xffffffffu,adc_errors=0;
    uint16_t battery_mv=0,raw12=0;
    uint8_t raw8=0,low=0,high=0;
    bool fresh=false,factory_limits=false;
    const char *profile_check="NO DATA";
};
}
