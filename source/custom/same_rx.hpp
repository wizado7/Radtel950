// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
class SameRx {
public:
    void reset() {size_=0;active_=false;}
    void sync() {size_=0;active_=true;}
    bool active() const {return active_;}

    bool push(uint16_t word,uint32_t now,char (&event)[4]) {
        if(!active_)return false;
        // The first dash distinguishes the supported FIFO byte/bit orders.
        if(!size_) {
            for(order_=0;order_<4;++order_)
                if(byte(word,0)=='-')break;
            if(order_==4){reset();return false;}
        }
        for(unsigned i=0;i<2;++i) {
            const char c=byte(word,i);
            if(c<32 || c>126 || size_==sizeof header_){reset();return false;}
            header_[size_++]=c;
            if(c!='-' || size_<34 || header_[size_-10]!='-')continue;
            int at=8;
            if(header_[0]!='-' || header_[4]!='-' || !letters(1) || !letters(5)) {
                reset();return false;
            }
            do {
                if(header_[at++]!='-' || !digits(at,6)){at=-1;break;}
                at+=6;
            } while(unsigned(at)<size_ && header_[at]=='-');
            if(at<0 || unsigned(at+23)!=size_ || header_[at]!='+' ||
               !digits(at+1,4) || header_[at+5]!='-' || !digits(at+6,7) ||
               header_[at+13]!='-')continue;
            const int day=(header_[at+6]-'0')*100+(header_[at+7]-'0')*10+header_[at+8]-'0';
            if(day==0 || day>366 || header_[at+3]>'5' || header_[at+11]>'5' ||
               (header_[at+9]-'0')*10+header_[at+10]-'0'>23) {
                reset();return false;
            }
            uint32_t hash=2166136261u;
            for(unsigned j=0;j<size_;++j)hash=(hash^uint8_t(header_[j]))*16777619u;
            reset();
            for(const auto &recent:recent_)
                if(recent.hash==hash && uint32_t(now-recent.time)<300000u)return false;
            recent_[next_]={hash,now};
            next_=uint8_t((next_+1)&3);
            for(unsigned j=0;j<3;++j)event[j]=header_[5+j];
            event[3]=0;
            return true;
        }
        return false;
    }
private:
    struct Seen {uint32_t hash=0,time=0;};
    Seen recent_[4]{};
    uint8_t next_=0,order_=0;
    unsigned size_=0;
    bool active_=false;
    char header_[256]{};

    char byte(uint16_t word,unsigned n) const {
        const uint8_t b=(n^(order_&1))?uint8_t(word):uint8_t(word>>8);
        if(!(order_&2))return char(b);
        uint32_t reverse;
        __asm("rbit %0,%1":"=r"(reverse):"r"(uint32_t(b)));
        return char(reverse>>24);
    }
    bool letters(unsigned at) const {
        for(unsigned i=0;i<3;++i)if(header_[at+i]<'A' || header_[at+i]>'Z')return false;
        return true;
    }
    bool digits(unsigned at,unsigned count) const {
        if(at+count>size_)return false;
        for(unsigned i=0;i<count;++i)if(header_[at+i]<'0' || header_[at+i]>'9')return false;
        return true;
    }
};
}
