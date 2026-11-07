// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_rx.hpp"
extern "C" {
#include "../diagnostic/nmea_status.h"
}

namespace rt950::aprs {
// Private Custom UI KISS extension, port 0 / command 0xE.
struct GpsTelemetry {
    static constexpr uint8_t Type=0x0e;
    enum class State : uint8_t { Off,NoStream,NoFix,Fix,Stale };
    uint8_t bytes[14]{'G','P','S',1};

    GpsTelemetry(const nav_status_t &nav,bool enabled,bool seen_bytes,uint32_t now) {
        const bool stream=enabled && seen_bytes && uint32_t(now-nav.last_byte_ms)<3000u;
        const bool fresh=nav.position_valid && nav.ever_fix && uint32_t(now-nav.last_fix_ms)<3000u;
        State state=State::Off;
        int32_t latitude=0,longitude=0;
        if(enabled) {
            if(!stream)state=nav.ever_fix?State::Stale:State::NoStream;
            else if(fresh && read_coordinate(nav.lat,nav.ns[0],true,latitude) &&
                    read_coordinate(nav.lon,nav.ew[0],false,longitude))state=State::Fix;
            else state=nav.position_valid && nav.ever_fix && !fresh?State::Stale:State::NoFix;
        }
        bytes[4]=static_cast<uint8_t>(state);
        bytes[5]=stream?1:0;
        if(state==State::Fix) {
            put(6,uint32_t(latitude));
            put(10,uint32_t(longitude));
        }
    }

private:
    static bool read_coordinate(const char *text,char hemisphere,bool latitude,int32_t &out) {
        const char positive=latitude?'N':'E',negative=latitude?'S':'W';
        if(hemisphere!=positive && hemisphere!=negative)return false;
        const unsigned count=latitude?2:3;
        unsigned degrees=0,minutes=0;
        for(unsigned i=0;i<count;++i) {
            if(*text<'0' || *text>'9')return false;
            degrees=degrees*10u+unsigned(*text++-'0');
        }
        for(unsigned i=0;i<2;++i) {
            if(*text<'0' || *text>'9')return false;
            minutes=minutes*10u+unsigned(*text++-'0');
        }
        if(*text++!='.' || !*text)return false;
        unsigned micro_minutes=minutes*1000000u,scale=100000u;
        for(;*text;++text,scale/=10u) {
            if(*text<'0' || *text>'9')return false;
            micro_minutes+=unsigned(*text-'0')*scale;
        }
        const unsigned limit=latitude?90u:180u;
        if(degrees>limit || minutes>=60 || (degrees==limit && micro_minutes))return false;
        const int32_t magnitude=int32_t(degrees*100000u+(micro_minutes+300u)/600u);
        out=hemisphere==negative?-magnitude:magnitude;
        return true;
    }

    void put(unsigned offset,uint32_t value) {
        for(unsigned i=0;i<4;++i)bytes[offset+i]=uint8_t(value>>(8*i));
    }
};
}
