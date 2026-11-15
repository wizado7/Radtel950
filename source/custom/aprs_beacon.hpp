// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_tx.hpp"
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
                          int32_t altitude_feet=-1) {
    if(!call[0] || ssid>15 || !lat || !lon || symbol<'!' || symbol>'~')return false;
    uint8_t frame[45]{};
    const char destination[]="APRS  ";
    bool padded=false;
    for(unsigned i=0;i<6;++i) {
        frame[i]=uint8_t(destination[i]<<1);
        if(!call[i])padded=true;
        const char ch=padded?' ':call[i];
        if(!padded && !((ch>='A' && ch<='Z') || (ch>='0' && ch<='9')))return false;
        frame[7+i]=uint8_t(ch<<1);
    }
    frame[6]=0xe0;frame[13]=uint8_t(0x61u|(ssid<<1));
    frame[14]=3;frame[15]=0xf0;frame[16]='!';
    for(unsigned i=0;i<7;++i){if(!lat[i])return false;frame[17+i]=uint8_t(lat[i]);}
    frame[24]=uint8_t(ns);frame[25]='/';
    for(unsigned i=0;i<8;++i){if(!lon[i])return false;frame[26+i]=uint8_t(lon[i]);}
    frame[34]=uint8_t(ew);
    int32_t ignored=0;
    if(!coordinate(frame+17,2,true,ignored) || !coordinate(frame+26,3,false,ignored))return false;
    frame[35]=uint8_t(symbol);
    size_t size=36;
    if(altitude_feet>=0 && altitude_feet<=999999) {
        frame[size++]='/';frame[size++]='A';frame[size++]='=';
        const unsigned feet=unsigned(altitude_feet);
        for(unsigned divisor=100000;divisor;divisor/=10)
            frame[size++]=uint8_t('0'+feet/divisor%10);
    }
    return out.encode(frame,size,32);
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
                          const GeoPoint &point,char symbol) {
    if(!point.valid || !geo_valid(point))return false;
    char lat[8],lon[9];
    beacon_coordinate(point.latitude,2,lat);beacon_coordinate(point.longitude,3,lon);
    return encode_beacon(out,call,ssid,lat,point.latitude<0?'S':'N',
        lon,point.longitude<0?'W':'E',symbol);
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
