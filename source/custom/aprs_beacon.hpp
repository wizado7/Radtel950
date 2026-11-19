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

inline size_t beacon_header(uint8_t *frame,const char (&call)[7],uint8_t ssid,const Routing &routing,
                            const Address &destination=Address{"APRS"}) {
    if(routing.preamble>=sizeof tx_flags/sizeof *tx_flags)return 0;
    Address source{};
    for(unsigned i=0;i<7;++i)source.callsign[i]=call[i];
    source.ssid=ssid;
    if(!valid_address(source))return 0;
    write_address(frame,destination,true);
    write_address(frame+7,source);
    size_t size=14;
    constexpr Address wide[]={{"WIDE1",1},{"WIDE2",1}};
    if(routing.beacon!=BeaconPath::Direct) {
        const auto &hops=routing.beacon==BeaconPath::Custom?routing.hops:wide;
        const unsigned count=routing.beacon==BeaconPath::FillIn?1:2;
        for(unsigned i=0;i<count;++i) {
            const auto &hop=hops[i];
            if(!valid_address(hop,true))return 0;
            if(!hop.callsign[0])continue;
            write_address(frame+size,hop);
            size+=7;
        }
    }
    frame[size-1]|=1;
    frame[size++]=3;frame[size++]=0xf0;
    return size;
}
inline void append_feet(uint8_t *frame,size_t &size,int32_t feet) {
    if(feet<0 || feet>999999)return;
    frame[size++]='/';frame[size++]='A';frame[size++]='=';
    for(unsigned divisor=100000;divisor;divisor/=10)
        frame[size++]=uint8_t('0'+unsigned(feet)/divisor%10);
}
inline bool encode_beacon(FrameTransmitter &out,const char (&call)[7],uint8_t ssid,
                          const char *lat,char ns,const char *lon,char ew,char symbol='>',
                          int32_t altitude_feet=-1,const Routing &routing={},char symbol_table='/') {
    if(!lat || !lon || symbol<'!' || symbol>'~' || !beacon_table(symbol_table))return false;
    uint8_t frame[59]{};
    size_t size=beacon_header(frame,call,ssid,routing);
    if(!size)return false;
    frame[size++]='!';
    const size_t position=size;
    for(unsigned i=0;i<7;++i){if(!lat[i])return false;frame[size++]=uint8_t(lat[i]);}
    frame[size++]=uint8_t(ns);frame[size++]=uint8_t(symbol_table);
    for(unsigned i=0;i<8;++i){if(!lon[i])return false;frame[size++]=uint8_t(lon[i]);}
    frame[size++]=uint8_t(ew);
    int32_t ignored=0;
    if(!coordinate(frame+position,2,true,ignored) ||
       !coordinate(frame+position+9,3,false,ignored))return false;
    frame[size++]=uint8_t(symbol);
    append_feet(frame,size,altitude_feet);
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
                          const GeoPoint &point,char symbol,const Routing &routing={},int32_t altitude_metres=-1,char symbol_table='/') {
    if(!point.valid || !geo_valid(point))return false;
    char lat[8],lon[9];
    beacon_coordinate(point.latitude,2,lat);beacon_coordinate(point.longitude,3,lon);
    return encode_beacon(out,call,ssid,lat,point.latitude<0?'S':'N',
        lon,point.longitude<0?'W':'E',symbol,
        altitude_metres>=0 && altitude_metres<=304799?gps_altitude_display(altitude_metres*10,true):-1,routing,symbol_table);
}

struct BeaconReport {
    GeoPoint position{};
    int32_t altitude_feet=-1;
    char symbol='>',table='/';
    bool live=false,motion=false;
    uint16_t course=0,speed_knots=0;
};

// SmartBeaconing curve for a handheld radio. Times are elapsed milliseconds.
inline bool smart_beacon_due(uint32_t elapsed,unsigned kmh,int heading,int last_heading) {
    const unsigned seconds=kmh<3?1800u:kmh>=10?180u:1800u/kmh;
    if(elapsed>=seconds*1000u)return true;
    if(kmh<3 || elapsed<30000u || heading<0 || last_heading<0)return false;
    const unsigned delta=unsigned(heading>last_heading?heading-last_heading:last_heading-heading);
    const unsigned turn=delta<=180?delta:360-delta;
    return turn>=28u+255u/kmh;
}
inline void write_base91(uint8_t *out,uint32_t value,unsigned length) {
    while(length){out[--length]=uint8_t('!'+value%91);value/=91;}
}
inline uint32_t compressed_units(uint32_t microdegrees,uint32_t scale) {
    // Split the fractional degree in thousandths to keep division 32-bit.
    const uint32_t whole=microdegrees/1000000u;
    const uint32_t fraction=microdegrees%1000000u;
    const uint32_t hi=(fraction/1000u)*scale,lo=(fraction%1000u)*scale;
    return whole*scale+hi/1000u+((hi%1000u)*1000u+lo+500000u)/1000000u;
}

struct DegMinutes {unsigned degrees,minutes100;};
inline DegMinutes mic_e_axis(int32_t axis,unsigned max) {
    const unsigned magnitude=unsigned(axis<0?-axis:axis);
    const unsigned hundredths=(magnitude*6u+500u)/1000u;
    const unsigned degrees=hundredths/6000u;
    return degrees>max?DegMinutes{max,5999}:DegMinutes{degrees,hundredths%6000u};
}

inline bool encode_beacon_report(FrameTransmitter &out,const char (&call)[7],uint8_t ssid,
                                 const BeaconReport &report,BeaconFormat format,const Routing &routing) {
    const auto &point=report.position;
    if(!point.valid || !geo_valid(point) || !beacon_table(report.table) ||
       report.symbol<'!' || report.symbol>'~' || format>=BeaconFormat::Count)return false;
    if(format==BeaconFormat::Normal) {
        char lat[8],lon[9];
        beacon_coordinate(point.latitude,2,lat);beacon_coordinate(point.longitude,3,lon);
        return encode_beacon(out,call,ssid,lat,point.latitude<0?'S':'N',lon,point.longitude<0?'W':'E',
                             report.symbol,report.altitude_feet,routing,report.table);
    }
    uint8_t frame[59]{};
    if(format==BeaconFormat::Compressed) {
        size_t size=beacon_header(frame,call,ssid,routing);
        if(!size)return false;
        frame[size++]='!';
        frame[size++]=uint8_t(report.table>='0' && report.table<='9'?'a'+report.table-'0':report.table);
        write_base91(frame+size,compressed_units(uint32_t(90000000-point.latitude),380926),4);size+=4;
        write_base91(frame+size,compressed_units(uint32_t(180000000+point.longitude),190463),4);size+=4;
        frame[size++]=uint8_t(report.symbol);
        if(report.motion) {
            const unsigned course=((report.course%360u+2u)/4u)%90u;
            const uint32_t target=(uint32_t(report.speed_knots)+1u)<<16;
            uint32_t low=1u<<16;
            unsigned speed=0;
            while(speed<89) {
                const uint32_t high=low+(low*2u+12u)/25u;
                if(target<=high){
                    if(target>low && target-low>high-target)++speed;
                    break;
                }
                low=high;++speed;
            }
            frame[size++]=uint8_t('!'+course);frame[size++]=uint8_t('!'+speed);
        } else {
            frame[size++]=' ';frame[size++]=' ';
        }
        frame[size++]=uint8_t('!'+(report.live?0x22:0x02)); // Software origin, current GPS when live.
        append_feet(frame,size,report.altitude_feet);
        return out.encode(frame,size,tx_flags[routing.preamble]);
    }

    // APRS 1.0.1 ch. 10. The six AX.25 destination characters carry
    // latitude and three status bits, followed by the hemisphere/offset flags.
    const auto lat=mic_e_axis(point.latitude,89),lon=mic_e_axis(point.longitude,179);
    const unsigned digits[]={lat.degrees/10,lat.degrees%10,lat.minutes100/1000,
        lat.minutes100/100%10,lat.minutes100/10%10,lat.minutes100%10};
    const unsigned status=9u-unsigned(format); // Off duty 111 through Priority 001.
    const unsigned flags=(status<<3)|(point.latitude>=0?4u:0u)|
        (lon.degrees<10 || lon.degrees>=100?2u:0u)|(point.longitude<0?1u:0u);
    Address destination{};
    for(unsigned i=0;i<6;++i)destination.callsign[i]=char((flags&(32u>>i)?'P':'0')+digits[i]);
    size_t size=beacon_header(frame,call,ssid,routing,destination);
    if(!size)return false;
    frame[size++]=0x60;
    const unsigned longitude=lon.degrees,minutes=lon.minutes100/100;
    const unsigned degrees_byte=longitude<10?longitude+118:longitude<100?longitude+28:
        longitude<110?longitude+8:longitude-72;
    frame[size++]=uint8_t(degrees_byte);
    frame[size++]=uint8_t(minutes+(minutes<10?88:28));
    frame[size++]=uint8_t(lon.minutes100%100+28);
    const unsigned speed=report.motion?(report.speed_knots>799?799:report.speed_knots):0;
    const unsigned course=report.motion?(unsigned(report.course)+359u)%360u+1u:0;
    frame[size++]=uint8_t(speed/10+28);
    frame[size++]=uint8_t(speed%10*10+course/100+28);
    frame[size++]=uint8_t(course%100+28);
    frame[size++]=uint8_t(report.symbol);frame[size++]=uint8_t(report.table);
    if(report.altitude_feet>=0 && report.altitude_feet<=999999) {
        const unsigned metres=(unsigned(report.altitude_feet)*3048u+5000u)/10000u;
        write_base91(frame+size,metres+10000u,3);size+=3;frame[size++]='}';
    }
    return out.encode(frame,size,tx_flags[routing.preamble]);
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
