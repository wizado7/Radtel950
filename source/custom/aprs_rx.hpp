// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rt950::aprs {
inline uint16_t fcs_byte(uint16_t crc,uint8_t byte) {
    crc^=byte;
    for(unsigned i=0;i<8;++i)
        crc=uint16_t((crc>>1)^((crc&1u)?0x8408u:0u));
    return crc;
}

// Feed decoded HDLC bits, least significant bit first. Demodulation and
// NRZI decoding are the responsibility of the radio transport.
class FrameReceiver final {
public:
    using Handler=void (*)(void *,const uint8_t *,size_t);
    void bit(bool value,Handler handler,void *context) {
        window_=uint8_t((window_>>1)|(value?0x80:0));
        if(++pending_<8)return;
        if(window_==0x7e) {
            if(started_ && !bad_ && !bits_ && size_>=18 && crc_==0xf0b8 && handler)
                handler(context,frame_,size_-2);
            started_=true;bad_=false;pending_=bits_=ones_=0;
            size_=0;crc_=0xffff;return;
        }
        pending_=7;
        if(started_ && !bad_)append(window_&1u);
    }
private:
    void append(bool value) {
        if(value) {
            if(++ones_>5){bad_=true;return;}
        } else {
            if(ones_==5){ones_=0;return;}
            ones_=0;
        }
        byte_=uint8_t((byte_>>1)|(value?0x80:0));
        if(++bits_!=8)return;
        bits_=0;
        if(size_==sizeof frame_){bad_=true;return;}
        frame_[size_++]=byte_;
        crc_=fcs_byte(crc_,byte_);
    }
    uint8_t frame_[332]{};
    size_t size_=0;
    uint16_t crc_=0xffff;
    uint8_t window_=0,pending_=0,bits_=0,ones_=0,byte_=0;
    bool started_=false,bad_=false;
};

// Inspect the FCS-free frame before passing it to the KISS exporter.
inline bool ui_payload(const uint8_t *frame,size_t size,size_t &offset) {
    if(!frame || size<17)return false;
    for(unsigned address=0;address<10;++address) {
        const size_t start=address*7u;
        if(size-start<7)return false;
        bool has_call=false,padded=false;
        for(unsigned i=0;i<6;++i) {
            const uint8_t raw=frame[start+i],ch=raw>>1;
            if(raw&1u)return false;
            if(ch==' '){padded=true;continue;}
            if(padded || !((ch>='A' && ch<='Z') || (ch>='0' && ch<='9')))return false;
            has_call=true;
        }
        const uint8_t ssid=frame[start+6];
        if(!has_call || (ssid&0x60)!=0x60)return false;
        if(ssid&1) {
            offset=start+9;
            return address!=0 && offset<size && frame[start+7]==3 && frame[start+8]==0xf0;
        }
    }
    return false;
}

enum class PositionKind : uint8_t { Station, Object, Item };
struct Position {
    int32_t latitude_e5=0,longitude_e5=0;
    char callsign[7]{};
    char name[10]{};
    uint8_t ssid=0,ambiguity=0;
    PositionKind kind=PositionKind::Station;
    bool removed=false;
};
inline int32_t degrees_e5(unsigned degrees,unsigned minute_hundredths) {
    return int32_t(degrees*100000u+(minute_hundredths*1000u+30u)/60u);
}

inline unsigned centered_minutes(unsigned minutes,unsigned ambiguity) {
    if(!ambiguity)return minutes;
    constexpr unsigned span[]={1,10,100,1000,6000};
    const unsigned width=span[ambiguity];
    return minutes/width*width+width/2u;
}

inline bool coordinate(const uint8_t *p,unsigned degrees,bool latitude,int32_t &out,
                       uint8_t *ambiguity=nullptr) {
    unsigned digits=0,spaces=0;
    for(unsigned i=0;i<degrees+5;++i) {
        if(i==degrees+2){if(p[i]!='.')return false;continue;}
        if(ambiguity && i>=degrees && p[i]==' ') {
            ++spaces;
            digits*=10u;
            continue;
        }
        if(spaces || p[i]<'0' || p[i]>'9')return false;
        digits=digits*10u+unsigned(p[i]-'0');
    }
    const uint8_t hemisphere=p[degrees+5];
    const uint8_t positive=latitude?'N':'E',negative=latitude?'S':'W';
    if(hemisphere!=positive && hemisphere!=negative)return false;
    const unsigned whole=digits/10000u,minutes=digits%10000u,limit=latitude?90u:180u;
    if(whole>limit || minutes>=6000 || (whole==limit && minutes))return false;
    unsigned midpoint=minutes;
    if(ambiguity) {
        if(latitude)*ambiguity=uint8_t(spaces);
        else if(spaces && spaces!=*ambiguity)return false;
        if(*ambiguity && whole==limit)return false;
        midpoint=centered_minutes(minutes,*ambiguity);
    }
    const int32_t magnitude=degrees_e5(whole,midpoint);
    out=hemisphere==negative?-magnitude:magnitude;
    return true;
}
inline bool position_timestamp(const uint8_t *p) {
    for(unsigned i=0;i<6;++i)
        if(p[i]<'0' || p[i]>'9')return false;
    const unsigned first=(p[0]-'0')*10u+(p[1]-'0');
    const unsigned second=(p[2]-'0')*10u+(p[3]-'0');
    const unsigned third=(p[4]-'0')*10u+(p[5]-'0');
    if(p[6]=='h')return first<24 && second<60 && third<60;
    return (p[6]=='z' || p[6]=='/') && first>=1 && first<=31 && second<24 && third<60;
}

inline bool compressed_axis(const uint8_t *p,uint32_t divisor,uint32_t span,int32_t &delta) {
    uint32_t value=0;
    for(unsigned i=0;i<4;++i) {
        if(p[i]<'!' || p[i]>'{')return false;
        value=value*91u+(p[i]-'!');
    }
    if(value>divisor*span)return false;
    // Scale to five decimal places in two stages to keep division 32-bit.
    const uint32_t scaled=(value%divisor)*1000u;
    delta=int32_t((value/divisor)*100000u+(scaled/divisor)*100u+
        ((scaled%divisor)*100u+divisor/2u)/divisor);
    return true;
}

inline bool compressed_position(const uint8_t *p,size_t size,Position &out) {
    if(size<13)return false;
    const uint8_t table=p[0];
    if(!(table=='/' || table=='\\' || (table>='A' && table<='Z') ||
        (table>='a' && table<='j')))return false;
    if(p[9]<'!' || p[9]>'~')return false;
    int32_t south=0,east=0;
    if(!compressed_axis(p+1,380926,180,south) ||
       !compressed_axis(p+5,190463,360,east))return false;
    out.latitude_e5=9000000-south;
    out.longitude_e5=east-18000000;
    return true;
}

// Mic-E puts latitude and the hemisphere flags in the AX.25 destination.
inline bool mic_e_position(const uint8_t *destination,const uint8_t *p,size_t size,Position &out) {
    if(size<8 || p[0]<38 || p[0]>127 || p[1]<38 || p[1]>97 || p[2]<28 || p[2]>127)
        return false;
    for(unsigned i=3;i<6;++i)
        if(p[i]<28 || p[i]>127)return false;
    if(p[6]<'!' || p[6]>'~' || p[7]<'!' || p[7]>'~')return false;

    unsigned latitude=0,ambiguity=0;
    for(unsigned i=0;i<6;++i) {
        const unsigned ch=destination[i]>>1;
        unsigned digit;
        if(ch>='0' && ch<='9')digit=ch-'0';
        else if(ch>='P' && ch<='Y')digit=ch-'P';
        else if(i<3 && ch>='A' && ch<='J')digit=ch-'A';
        else if(i>=2 && (ch=='L' || ch=='Z' || (i==2 && ch=='K'))) {
            ++ambiguity;
            latitude*=10u;
            continue;
        } else return false;
        if(ambiguity)return false;
        latitude=latitude*10u+digit;
    }
    const unsigned lat_deg=latitude/10000u,lat_min=latitude%10000u;
    if(lat_deg>90 || lat_min>=6000 || (lat_deg==90 && (lat_min || ambiguity)))return false;

    const bool north=(destination[3]>>1)>='P';
    const bool offset=(destination[4]>>1)>='P';
    const bool west=(destination[5]>>1)>='P';
    unsigned lon_deg=p[0]-28u+(offset?100u:0u);
    if(lon_deg>=190)lon_deg-=190;
    else if(lon_deg>=180)lon_deg-=80;
    unsigned lon_min=p[1]-28u;
    if(lon_min>=60)lon_min-=60;
    lon_min=lon_min*100u+(p[2]-28u);

    const int32_t lat=degrees_e5(lat_deg,centered_minutes(lat_min,ambiguity));
    const int32_t lon=degrees_e5(lon_deg,centered_minutes(lon_min,ambiguity));
    out.latitude_e5=north?lat:-lat;
    out.longitude_e5=west?-lon:lon;
    out.ambiguity=uint8_t(ambiguity);
    return true;
}

inline bool named_position_header(const uint8_t *frame,size_t size,size_t &at,Position &out) {
    const bool object=out.kind==PositionKind::Object;
    unsigned length=0;
    while(length<9 && at<size) {
        const uint8_t ch=frame[at];
        if(!object && (ch=='!' || ch=='_'))break;
        if(ch<' ' || ch>'~')return false;
        out.name[length++]=char(ch);
        ++at;
    }
    if(at==size || (object?length!=9:length<3))return false;
    const uint8_t action=frame[at++];
    if(action!='_' && action!=(object?'*':'!'))return false;
    out.removed=action=='_';
    if(object) {
        if(size-at<7 || !position_timestamp(frame+at))return false;
        at+=7;
    }
    while(length && out.name[length-1]==' ')out.name[--length]=0;
    return length!=0;
}

// APRS Protocol Reference, chapters 6 and 8-11. Coordinates for the local
// station list; AX.25 export retains the complete original payload.
inline bool position(const uint8_t *frame,size_t size,Position &out) {
    size_t at=0;
    if(!ui_payload(frame,size,at))return false;
    const uint8_t type=frame[at++];
    Position candidate;
    if(type==0x60 || type==0x27 || type==0x1c || type==0x1d) {
        if(!mic_e_position(frame,frame+at,size-at,candidate))return false;
    } else {
        if(type=='/' || type=='@') {
            if(size-at<7 || !position_timestamp(frame+at))return false;
            at+=7;
        } else if(type==';' || type==')') {
            candidate.kind=type==';'?PositionKind::Object:PositionKind::Item;
            if(!named_position_header(frame,size,at,candidate))return false;
        } else if(type!='!' && type!='=')return false;
        if(at==size)return false;
        if(frame[at]>='0' && frame[at]<='9') {
            if(size-at<19 || !coordinate(frame+at,2,true,candidate.latitude_e5,&candidate.ambiguity) ||
               !coordinate(frame+at+9,3,false,candidate.longitude_e5,&candidate.ambiguity))return false;
        } else if(!compressed_position(frame+at,size-at,candidate))return false;
    }
    for(unsigned i=0;i<6 && (frame[7+i]>>1)!=' ';++i)
        candidate.callsign[i]=char(frame[7+i]>>1);
    candidate.ssid=(frame[13]>>1)&0x0f;
    out=candidate;
    return true;
}
}
