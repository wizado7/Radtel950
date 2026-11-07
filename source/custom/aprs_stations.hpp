// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_rx.hpp"

namespace rt950::aprs {
class RecentStations final {
public:
    static constexpr unsigned Capacity=3;
    struct Entry {
        Position position{};
        uint32_t received_at=0;
    };

    const Entry *at(unsigned index) const {
        return index<size_?&entries_[index]:nullptr;
    }

    void clear() {size_=0;}

    void record(const uint8_t *frame,size_t size,uint32_t now) {
        Position station;
        if(!position(frame,size,station))return;
        unsigned index=0;
        while(index<size_ && !same(entries_[index].position,station))++index;
        if(station.removed) {
            if(index==size_)return;
            --size_;
            for(;index<size_;++index)entries_[index]=entries_[index+1];
            return;
        }
        if(index==size_) {
            if(size_<Capacity)++size_;
            else index=Capacity-1;
        }
        for(;index;--index)entries_[index]=entries_[index-1];
        entries_[0]={station,now};
    }

private:
    static bool same(const Position &a,const Position &b) {
        if(a.kind!=b.kind)return false;
        if(a.kind!=PositionKind::Station) {
            for(unsigned i=0;i<sizeof a.name;++i)
                if(a.name[i]!=b.name[i])return false;
            return true;
        }
        if(a.ssid!=b.ssid)return false;
        for(unsigned i=0;i<sizeof a.callsign;++i)
            if(a.callsign[i]!=b.callsign[i])return false;
        return true;
    }

    Entry entries_[Capacity]{};
    unsigned size_=0;
};
}
