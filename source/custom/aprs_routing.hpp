// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_tx.hpp"

namespace rt950::aprs {
struct Address {
    char callsign[7]{};
    uint8_t ssid=0;
};
enum class BeaconPath : uint8_t { Direct, FillIn, Wide, Custom };
enum class RelayMode : uint8_t { Addressed, FillIn, Wide };
enum class BeaconMode : uint8_t { Timed, Manual, Smart };
enum class BeaconFormat : uint8_t { Normal, Compressed, OffDuty, EnRoute, InService, Returning, Committed, Special, Priority, Count };
inline constexpr unsigned BeaconOverlayCount=37;
constexpr char overlay_character(unsigned index) {
    return index==0?'\\':char(index<=10?'0'+index-1:'A'+index-11);
}
constexpr bool beacon_table(char table) {
    return table=='/' || table=='\\' || (table>='0' && table<='9') || (table>='A' && table<='Z');
}
inline constexpr uint16_t relay_delays[]={100,250,500,1000,2000};
inline constexpr uint8_t tx_flags[]={16,32,48,64};
struct Routing {
    Address hops[2]{};
    BeaconPath beacon=BeaconPath::Direct;
    RelayMode relay=RelayMode::Addressed;
    bool relay_use_beacon=false;
    uint8_t delay=2,preamble=1;
};

inline bool valid_address(const Address &address,bool allow_empty=false) {
    if(address.ssid>15 || address.callsign[6])return false;
    bool ended=false;
    for(char ch:address.callsign) {
        if(!ch){ended=true;continue;}
        if(ended || !((ch>='A' && ch<='Z') || (ch>='0' && ch<='9')))return false;
    }
    return address.callsign[0] || (allow_empty && !address.ssid);
}

inline void write_address(uint8_t *out,const Address &address,bool repeated=false,bool last=false) {
    bool ended=false;
    for(unsigned i=0;i<6;++i) {
        ended=ended || !address.callsign[i];
        out[i]=uint8_t((ended?' ':address.callsign[i])<<1);
    }
    out[6]=uint8_t(0x60u|(address.ssid<<1)|(repeated?0x80u:0)|(last?1u:0));
}

inline bool same_address(const uint8_t *on_air,const Address &address) {
    uint8_t encoded[7];
    write_address(encoded,address);
    for(unsigned i=0;i<6;++i)if(on_air[i]!=encoded[i])return false;
    return (on_air[6]&0x1eu)==(encoded[6]&0x1eu);
}

// The payload and source/destination identify a repeat even when its path
// changes. Fixed cache entries are never evicted during the suppression window.
class RelayQueue final {
public:
    static constexpr uint32_t DuplicateMs=30000;
    void cancel() {size_=0;}
    bool pending() const {return size_!=0;}

    bool receive(const uint8_t *frame,size_t size,const Address &local,RelayMode mode,
                 uint32_t now,uint16_t delay) {
        size_t payload=0;
        if(pending() || size>FrameTransmitter::MaxFrame || !valid_address(local) ||
           !ui_payload(frame,size,payload) || same_address(frame+7,local) || frame[payload]=='}')return false;
        const size_t end=payload-2;
        size_t next=end;
        for(size_t at=14;at<end;at+=7) {
            const bool repeated=frame[at+6]&0x80u;
            if(repeated) {
                if(next!=end || same_address(frame+at,local))return false;
            } else if(next==end)next=at;
        }
        if(next==end)return false;
        for(size_t at=next+7;at<end;at+=7)if(same_address(frame+at,local))return false;
        const uint8_t *hop=frame+next;
        const unsigned ssid=(hop[6]>>1)&15u;
        constexpr Address wide1{"WIDE1",1},wide2_1{"WIDE2",1},wide2_2{"WIDE2",2};
        const bool addressed=same_address(hop,local);
        const bool wide1_ok=mode!=RelayMode::Addressed && same_address(hop,wide1);
        const bool wide2_ok=mode==RelayMode::Wide &&
            (same_address(hop,wide2_1) || same_address(hop,wide2_2));
        if(!addressed && !wide1_ok && !wide2_ok)return false;
        const bool insert=!addressed && wide2_ok && ssid==2;
        constexpr size_t max_address_bytes=10*7;
        if(insert && (end==max_address_bytes || size+7>FrameTransmitter::MaxFrame))return false;

        const uint64_t key=fingerprint(frame,payload,size);
        Entry *available=nullptr;
        for(auto &entry:seen_) {
            if(!entry.used || uint32_t(now-entry.at)>=DuplicateMs) {
                if(!available)available=&entry;
            } else if(entry.key==key)return false;
        }
        if(!available)return false;
        *available={key,now,true};

        const bool last=hop[6]&1u;
        for(size_t i=0;i<next;++i)data_[i]=frame[i];
        write_address(data_+next,local,true,last && !insert);
        if(insert)write_address(data_+next+7,wide2_1,false,last);
        const size_t extra=insert?7:0;
        for(size_t i=next+7;i<size;++i)data_[i+extra]=frame[i];
        size_=size+extra;due_=now+delay;until_=due_+2000u;
        return true;
    }

    bool take(FrameTransmitter &out,uint32_t now,bool authorized,bool clear,unsigned preamble) {
        if(!authorized || (pending() && int32_t(now-until_)>=0))cancel();
        if(!pending() || !clear || int32_t(now-due_)<0)return false;
        const auto size=size_;
        cancel();
        return out.encode(data_,size,preamble);
    }

private:
    // FNV-1a over addresses without command/extension bits and full information.
    static uint64_t fingerprint(const uint8_t *frame,size_t payload,size_t size) {
        uint64_t hash=14695981039346656037ull;
        for(unsigned i=0;i<14;++i) {
            const uint8_t value=i==6 || i==13?frame[i]&0x1eu:frame[i];
            hash=(hash^value)*1099511628211ull;
        }
        for(size_t i=payload;i<size;++i)hash=(hash^frame[i])*1099511628211ull;
        return hash;
    }
    struct Entry {uint64_t key=0;uint32_t at=0;bool used=false;};
    Entry seen_[16]{};
    uint8_t data_[FrameTransmitter::MaxFrame]{};
    size_t size_=0;
    uint32_t due_=0,until_=0;
};
}
