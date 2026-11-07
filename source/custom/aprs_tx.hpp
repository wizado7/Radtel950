// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_rx.hpp"

namespace rt950::aprs {
// Encode one FCS-free AX.25 UI frame to packed NRZI levels. The radio
// transport consumes one level per symbol and chooses how to modulate it.
class FrameTransmitter final {
public:
    static constexpr size_t MaxFrame=330;
    static constexpr unsigned MaxPreamble=64;

    bool encode(const uint8_t *frame,size_t size,unsigned preamble) {
        count_=0;
        size_t payload=0;
        if(size>MaxFrame || !preamble || preamble>MaxPreamble ||
           !ui_payload(frame,size,payload))return false;
        level_=false;
        ones_=0;
        for(unsigned i=0;i<preamble;++i)byte(0x7e,false);
        uint16_t fcs=0xffff;
        for(size_t i=0;i<size;++i) {
            fcs=fcs_byte(fcs,frame[i]);
            byte(frame[i],true);
        }
        fcs=uint16_t(~fcs);
        byte(uint8_t(fcs),true);
        byte(uint8_t(fcs>>8),true);
        byte(0x7e,false);
        return true;
    }

    size_t size() const {return count_;}
    bool level(size_t at) const {return at<count_ && ((levels_[at/8]>>(at%8))&1u);}
private:
    void append(bool bit) {
        if(!bit)level_=!level_;
        const uint8_t mask=uint8_t(1u<<(count_%8));
        if(count_%8==0)levels_[count_/8]=0;
        if(level_)levels_[count_/8]|=mask;
        ++count_;
    }
    void byte(uint8_t value,bool stuff) {
        for(unsigned i=0;i<8;++i) {
            const bool bit=(value>>i)&1u;
            append(bit);
            if(!stuff)continue;
            ones_=bit?ones_+1:0;
            if(ones_==5) {append(false);ones_=0;}
        }
    }
    static constexpr size_t FrameBits=(MaxFrame+2)*8;
    uint8_t levels_[MaxPreamble+1+(FrameBits+FrameBits/5+7)/8]{};
    size_t count_=0;
    unsigned ones_=0;
    bool level_=false;
};
}
