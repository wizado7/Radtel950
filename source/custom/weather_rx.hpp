// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950 {
// OEM 0.29 0x08030CA4, ten consecutive words in units of 10 Hz.
inline constexpr uint32_t weather_channels[]={
    162550000,162400000,162475000,162425000,162450000,
    162500000,162525000,161650000,161775000,163275000
};

class WeatherRx final {
public:
    static constexpr unsigned ChannelCount=sizeof(weather_channels)/sizeof(weather_channels[0]);

    unsigned channel() const {return channel_;}
    uint32_t frequency() const {return weather_channels[channel_];}
    bool scanning() const {return mode_==Mode::Scan;}
    bool alerted() const {return mode_==Mode::Alert;}
    const char *same_event() const {return same_event_;}
    void same_alert(const char (&event)[4]) {
        for(unsigned i=0;i<4;++i)same_event_[i]=event[i];
        mode_=Mode::Alert;
    }

    void begin(bool alert,uint32_t now) {
        mode_=alert?Mode::Scan:Mode::Listen;
        retuned(now);
    }
    void step(bool up,uint32_t now) {
        channel_=uint8_t((channel_+(up?1u:ChannelCount-1u))%ChannelCount);
        if(alerted())mode_=Mode::Listen;
        retuned(now);
    }
    void confirm(uint32_t now) {begin(mode_==Mode::Listen,now);}

    void poll(uint32_t now,bool sampled,bool carrier,bool fault) {
        if(!scanning() || fault)return;
        if(sampled && carrier) {
            if(!tracking_) {tracking_=true;carrier_since_=now;}
            if(uint32_t(now-carrier_since_)>=60u)mode_=Mode::Alert;
        } else tracking_=false;
        // Give the receiver a bounded settling / observation window per channel.
        if(scanning() && uint32_t(now-tuned_at_)>=400u)step(true,now);
    }

private:
    enum class Mode : uint8_t {Listen, Scan, Alert};
    void retuned(uint32_t now) {tuned_at_=now;tracking_=false;same_event_[0]=0;}
    uint32_t tuned_at_=0,carrier_since_=0;
    uint8_t channel_=0;
    char same_event_[4]{};
    Mode mode_=Mode::Listen;
    bool tracking_=false;
};
}
