// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_tx.hpp"
#include "voice_ptt.hpp"

namespace rt950::aprs {
inline bool encode_beacon(FrameTransmitter &out,const char (&call)[7],uint8_t ssid,
                          const char *lat,char ns,const char *lon,char ew,char symbol='>') {
    if(!call[0] || ssid>15 || !lat || !lon || symbol<'!' || symbol>'~')return false;
    uint8_t frame[36]{};
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
    return out.encode(frame,sizeof frame,32);
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
