// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
extern "C" {
#include "../diagnostic/nmea_status.h"
}

namespace rt950 {
struct GpsCommand {
    bool power=false;
    bool sleeping=false;
    bool wake=false;
    uint16_t seconds=0;
};

class GpsPower {
public:
    GpsCommand poll(uint32_t now,bool on,bool continuous,bool busy,const nav_status_t &nav) {
        if(!on) {
            *this=GpsPower{};
            return {};
        }
        const auto fresh=[&](uint32_t since) {
            return nav.last_byte_ms && int32_t(nav.last_byte_ms-since)>=0;
        };
        if(phase_==Phase::Off) {
            if(busy)return {};
            running(now);
        }
        if(phase_==Phase::Cut) {
            if(!busy && (due(now,until_) || (continuous && now-at_>=500u)))running(now);
            else return {false,true,false,0};
        }
        if(phase_==Phase::Standby) {
            if(due(now,until_))running(now);
            else {
                if(!busy && continuous) {
                    phase_=Phase::Poke;at_=now;
                    return {true,false,true,0};
                }
                if(!busy && fresh(at_+2000u)) {
                    // A module that did not stop NMEA has ignored standby.
                    casic_=false;phase_=Phase::Cut;at_=now;
                    return {false,true,false,0};
                }
                return {true,true,false,0};
            }
        }
        if(phase_==Phase::Poke) {
            if(fresh(at_))running(at_);
            else if(!busy && now-at_>=3000u) {
                silent_retried_=true;
                return cut(now,500);
            } else return {true,false,false,0};
        }
        const bool heard=fresh(at_);
        if(heard)silent_retried_=false;
        if(nav.ever_fix && nav.position_valid && now-nav.last_fix_ms<3000u &&
           int32_t(nav.last_fix_ms-at_)>=0) {
            if(!fixed_){fixed_=true;fix_at_=now;}
            misses_=0;
        }
        if(fixed_ && now-fix_at_>=60000u){eph_at_=now;eph_valid_=true;}
        if(busy)return {true,false,false,0};
        if(!heard && !silent_retried_ && now-at_>=5000u) {
            silent_retried_=true;
            return cut(now,500);
        }
        if(continuous)return {true,false,false,0};

        unsigned seconds=60;
        if(!fixed_) {
            if(now-at_<120000u)return {true,false,false,0};
            constexpr uint16_t backoff[]={60,120,240,480,900};
            seconds=backoff[misses_];
            if(misses_<4)++misses_;
        } else if(!eph_valid_ || now-eph_at_>=1800000u)return {true,false,false,0};
        if(!casic_ || !heard)return cut(now,seconds*1000u);
        phase_=Phase::Standby;at_=now;until_=now+seconds*1000u;
        return {true,true,false,uint16_t(seconds)};
    }

    static unsigned standby_frame(uint16_t seconds,uint8_t (&out)[20]) {
        constexpr char prefix[]="$PCAS12,";
        unsigned length=0;
        for(char c:prefix)if(c)out[length++]=uint8_t(c);
        char digits[5];unsigned count=0;
        do {digits[count++]=char('0'+seconds%10u);seconds/=10u;} while(seconds);
        while(count)out[length++]=uint8_t(digits[--count]);
        uint8_t checksum=0;
        for(unsigned i=1;i<length;++i)checksum^=out[i];
        constexpr char hex[]="0123456789ABCDEF";
        out[length++]='*';out[length++]=hex[checksum>>4];out[length++]=hex[checksum&15u];
        out[length++]='\r';out[length++]='\n';
        return length;
    }

private:
    enum class Phase : uint8_t {Off,Running,Standby,Poke,Cut};
    static bool due(uint32_t now,uint32_t at) {return int32_t(now-at)>=0;}
    void running(uint32_t now){phase_=Phase::Running;at_=now;fixed_=false;}
    GpsCommand cut(uint32_t now,uint32_t duration) {
        phase_=Phase::Cut;at_=now;until_=now+duration;
        return {false,duration>500u,false,0};
    }
    uint32_t at_=0,until_=0,fix_at_=0,eph_at_=0;
    Phase phase_=Phase::Off;
    uint8_t misses_=0;
    bool casic_=true,fixed_=false,eph_valid_=false,silent_retried_=false;
};
}
