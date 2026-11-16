// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "controls.hpp"

namespace rt950::voice {
struct TxSignal {
    uint16_t first=0,second=0;
    explicit operator bool() const {return first!=0;}
};

constexpr TxSignal dtmf_pair(char symbol) {
    constexpr char keys[]="123A456B789C*0#D";
    constexpr uint16_t rows[]={697,770,852,941},columns[]={1209,1336,1477,1633};
    for(unsigned i=0;i<sizeof keys-1;++i)
        if(symbol==keys[i])return {rows[i/4],columns[i%4]};
    return {};
}

inline TxSignal ptt_signal(uint8_t key) {
    const int digit=key_digit(key);
    if(digit>=0)return dtmf_pair(char('0'+digit));
    switch(key) {
    case 6:return dtmf_pair('A');
    case 7:return dtmf_pair('B');
    case 8:return dtmf_pair('C');
    case 9:return dtmf_pair('D');
    case 13:return dtmf_pair('*');
    case 21:return dtmf_pair('#');
    case 3:return {1000,0};
    case 4:return {1450,0};
    case 2:return {1750,0};
    case 5:return {2100,0};
    default:return {};
    }
}

class KeyedSignal final {
public:
    bool active() const {return active_;}

    template<class Apply>
    bool poll(uint32_t now,bool keyed,bool allowed,uint8_t raw,uint8_t accepted,Apply apply) {
        const bool released=raw==255 && accepted==255;
        if(!keyed) {
            active_=false;armed_=released;
            return true;
        }
        if(!allowed) {
            const bool restore=active_;
            active_=false;armed_=released;
            return !restore || apply({});
        }
        if(released)armed_=true;
        if(active_) {
            if(raw==accepted && raw!=255)armed_=false;
            if(uint32_t(now-start_)<duration_)return true;
            active_=false;
            return apply({});
        }
        if(!armed_ || raw!=accepted || raw==255)return true;
        armed_=false;
        const auto signal=ptt_signal(raw);
        if(!signal)return true;
        duration_=signal.second?250:350;
        start_=now;
        active_=apply(signal);
        return active_;
    }
private:
    uint32_t start_=0;
    uint16_t duration_=0;
    bool armed_=false,active_=false;
};
}
