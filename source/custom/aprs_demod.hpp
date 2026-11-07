// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_rx.hpp"

namespace rt950::aprs {
// Bell 202 detector. Feed one unsigned 12-bit audio sample per 9600 Hz tick.
// The source must reset the detector whenever the audio stream is interrupted.
class AfskReceiver final {
public:
    static constexpr unsigned SampleRate=9600;
    void reset() {*this=AfskReceiver{};}

    void sample(uint16_t value,FrameReceiver::Handler handler,void *context) {
        samples_[head_]=int16_t((value&0xfffu)-2048);
        head_=(head_+1)&7u;
        if(filled_<8 && ++filled_<8)return;

        const auto mark=energy(MarkI,MarkQ);
        const auto space=energy(SpaceI,SpaceQ);
        // Ignore silence. This threshold is in ADC counts and must be
        // confirmed with the actual analog gain on the radio.
        if(mark<256 && space<256) {
            if(quiet_<16 && ++quiet_==16) {
                frame_=FrameReceiver{};
                locked_=have_symbol_=false;
            }
            return;
        }
        quiet_=0;
        const bool tone=mark>space;
        if(!locked_) {
            locked_=true;tone_=tone;phase_=0;
        } else {
            phase_=(phase_+1)&7u;
            if(tone!=tone_) {
                tone_=tone;
                // The moving correlation delays every transition. Recover
                // clock phase relative to that delay and sample mid-symbol.
                if(phase_<=2 || phase_>=6)phase_=0;
                else if(phase_<4)--phase_;
                else phase_=(phase_+1)&7u;
            }
        }
        if(phase_!=4)return;
        if(have_symbol_)frame_.bit(tone==symbol_,handler,context);
        have_symbol_=true;
        symbol_=tone;
    }

private:
    // Zero-mean, Q6 quadrature references for an eight-sample window.
    inline static constexpr int8_t MarkI[]={64,45,0,-45,-64,-45,0,45};
    inline static constexpr int8_t MarkQ[]={0,45,64,45,0,-45,-64,-45};
    inline static constexpr int8_t SpaceI[]={66,10,-60,-23,57,41,-43,-48};
    inline static constexpr int8_t SpaceQ[]={-6,58,11,-65,-38,45,40,-45};

    int32_t energy(const int8_t (&i)[8],const int8_t (&q)[8]) const {
        int32_t a=0,b=0;
        for(unsigned n=0;n<8;++n) {
            const int32_t value=samples_[(head_+n)&7u];
            a+=value*i[n];b+=value*q[n];
        }
        a/=64;b/=64;
        return a*a+b*b;
    }

    FrameReceiver frame_{};
    int16_t samples_[8]{};
    unsigned head_=0,filled_=0,phase_=0,quiet_=0;
    bool locked_=false,tone_=false,have_symbol_=false,symbol_=false;
};
}
