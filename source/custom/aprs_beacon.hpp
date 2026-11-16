// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_routing.hpp"
#include "gps_view.hpp"
#include "gps_navigation.hpp"
#include "voice_ptt.hpp"

namespace rt950::aprs {
inline int32_t beacon_altitude(const nav_status_t &nav,uint32_t now) {
    return gps_altitude_fresh(nav,now) && nav.altitude_decimetres>=0?
        gps_altitude_display(nav.altitude_decimetres,true):-1;
}

inline bool encode_beacon(FrameTransmitter &out,const char (&call)[7],uint8_t ssid,
                          const char *lat,char ns,const char *lon,char ew,char symbol='>',
                          int32_t altitude_feet=-1,const Routing &routing={}) {
    if(!lat || !lon || symbol<'!' || symbol>'~' ||
       routing.preamble>=sizeof tx_flags/sizeof *tx_flags)return false;
    Address source{};
    for(unsigned i=0;i<7;++i)source.callsign[i]=call[i];
    source.ssid=ssid;
    if(!valid_address(source))return false;
    uint8_t frame[45+14]{};
    write_address(frame,Address{"APRS"},true);
    write_address(frame+7,source);
    size_t size=14;
    constexpr Address wide[]={{"WIDE1",1},{"WIDE2",1}};
    if(routing.beacon!=BeaconPath::Direct) {
        const auto &hops=routing.beacon==BeaconPath::Custom?routing.hops:wide;
        const unsigned count=routing.beacon==BeaconPath::FillIn?1:2;
        for(unsigned i=0;i<count;++i) {
            const auto &hop=hops[i];
            if(!valid_address(hop,true))return false;
            if(!hop.callsign[0])continue;
            write_address(frame+size,hop);
            size+=7;
        }
    }
    frame[size-1]|=1;
    frame[size++]=3;frame[size++]=0xf0;frame[size++]='!';
    const size_t position=size;
    for(unsigned i=0;i<7;++i){if(!lat[i])return false;frame[size++]=uint8_t(lat[i]);}
    frame[size++]=uint8_t(ns);frame[size++]='/';
    for(unsigned i=0;i<8;++i){if(!lon[i])return false;frame[size++]=uint8_t(lon[i]);}
    frame[size++]=uint8_t(ew);
    int32_t ignored=0;
    if(!coordinate(frame+position,2,true,ignored) ||
       !coordinate(frame+position+9,3,false,ignored))return false;
    frame[size++]=uint8_t(symbol);
    if(altitude_feet>=0 && altitude_feet<=999999) {
        frame[size++]='/';frame[size++]='A';frame[size++]='=';
        const unsigned feet=unsigned(altitude_feet);
        for(unsigned divisor=100000;divisor;divisor/=10)
            frame[size++]=uint8_t('0'+feet/divisor%10);
    }
    return out.encode(frame,size,tx_flags[routing.preamble]);
}

inline void beacon_coordinate(int32_t microdegrees,unsigned digits,char *text) {
    const unsigned magnitude=unsigned(microdegrees<0?-microdegrees:microdegrees);
    const unsigned hundredths=(magnitude*6u+500u)/1000u;
    unsigned degrees=hundredths/6000u;
    const unsigned minutes=hundredths%6000u;
    text[digits]=char('0'+minutes/1000u);
    text[digits+1]=char('0'+minutes/100u%10u);
    text[digits+2]='.';
    text[digits+3]=char('0'+minutes/10u%10u);
    text[digits+4]=char('0'+minutes%10u);
    text[digits+5]=0;
    while(digits){text[--digits]=char('0'+degrees%10u);degrees/=10u;}
}

inline bool encode_beacon(FrameTransmitter &out,const char (&call)[7],uint8_t ssid,
                          const GeoPoint &point,char symbol,const Routing &routing={},int32_t altitude_metres=-1) {
    if(!point.valid || !geo_valid(point))return false;
    char lat[8],lon[9];
    beacon_coordinate(point.latitude,2,lat);beacon_coordinate(point.longitude,3,lon);
    return encode_beacon(out,call,ssid,lat,point.latitude<0?'S':'N',
        lon,point.longitude<0?'W':'E',symbol,
        altitude_metres>=0 && altitude_metres<=304799?gps_altitude_display(altitude_metres*10,true):-1,routing);
}

// RF and DAC are separate so this state machine can be tested without MMIO.
template<class Radio,class Audio>
class BeaconSession final {
public:
    BeaconSession(Radio &radio,Audio &audio):radio_(radio),audio_(audio){}
    bool active() const {return frequency_!=0;}
    bool transmitting() const {return ptt_.state()==voice::Ptt::State::Transmitting;}
    void start(uint32_t frequency,uint32_t now) {
        ptt_.poll(radio_,{},now,false);
        frequency_=frequency;
    }
    void stop(uint32_t now) {
        audio_.cancel();
        ptt_.poll(radio_,{},now,false);
        frequency_=0;
    }
    void poll(const FrameTransmitter &frame,uint32_t now,bool authorized) {
        if(!active())return;
        if(!authorized){stop(now);return;}
        const bool was_transmitting=transmitting();
        ptt_.poll(radio_,{frequency_,5,0,true,true},now,true);
        if(ptt_.state()==voice::Ptt::State::WaitRelease) {stop(now);return;}
        if(!transmitting())return;
        if(!was_transmitting && !audio_.begin(frame,now)){stop(now);return;}
        if(audio_.poll(now)!=Audio::State::Running)stop(now);
    }
private:
    Radio &radio_;
    Audio &audio_;
    voice::Ptt ptt_{};
    uint32_t frequency_=0;
};
}
