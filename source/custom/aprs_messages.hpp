// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_rx.hpp"

namespace rt950::aprs {
class RecentMessages final {
public:
    static constexpr unsigned Capacity=3;
    struct Entry {
        char source[7]{},destination[10]{},body[68]{},id[6]{};
        uint8_t ssid=0;
    };
    unsigned count() const {return size_;}
    const Entry *at(unsigned index) const {return index<size_?&entries_[index]:nullptr;}

    void record(const uint8_t *frame,size_t size) {
        size_t at=0;
        if(!ui_payload(frame,size,at) || size-at<11 || frame[at]!=':' || frame[at+10]!=':')return;
        Entry entry;
        unsigned address_size=9;
        while(address_size && frame[at+address_size]==' ')--address_size;
        if(!address_size)return;
        for(unsigned i=0;i<address_size;++i) {
            const uint8_t ch=frame[at+1+i];
            if(ch<' ' || ch>'~' || ch==':')return;
            entry.destination[i]=char(ch);
        }
        at+=11;
        unsigned length=0;
        while(at<size && frame[at]!='{') {
            const uint8_t ch=frame[at++];
            if(length==67 || ch<' ' || ch>'~' || ch=='|' || ch=='~')return;
            entry.body[length++]=char(ch);
        }
        if(!length)return;
        if(at<size) {
            ++at;
            const size_t id_size=size-at;
            if(!id_size || id_size>5)return;
            for(unsigned i=0;i<id_size;++i) {
                const uint8_t ch=frame[at+i];
                if(!((ch>='0' && ch<='9') || (ch>='A' && ch<='Z') || (ch>='a' && ch<='z')))return;
                entry.id[i]=char(ch);
            }
        }
        for(unsigned i=0;i<6 && (frame[7+i]>>1)!=' ';++i)
            entry.source[i]=char(frame[7+i]>>1);
        entry.ssid=(frame[13]>>1)&15u;

        // Repeated transmissions should not evict other messages.
        for(unsigned i=0;i<size_;++i) {
            const auto &old=entries_[i];
            if(old.ssid==entry.ssid && equal(old.source,entry.source) &&
               equal(old.destination,entry.destination) && equal(old.id,entry.id) &&
               equal(old.body,entry.body))return;
        }
        if(size_<Capacity)++size_;
        for(unsigned i=size_-1;i>0;--i)entries_[i]=entries_[i-1];
        entries_[0]=entry;
    }
private:
    static bool equal(const char *a,const char *b) {
        while(*a && *a==*b){++a;++b;}
        return *a==*b;
    }
    Entry entries_[Capacity]{};
    unsigned size_=0;
};
}
