// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
struct FrequencyCopy {
    enum class State : uint8_t { Starting, Searching, Found, Timeout, Fault };
    State state=State::Starting;
    uint32_t hz=0;
};

class FrequencyCounter {
public:
    void begin(uint32_t now,bool uhf) {
        result_={};uhf_=uhf;first_=0;deadline_=now+10000u;
    }

    template<class Registers>
    void cancel(Registers &regs) {
        if(running_)regs.write(0x32,0x8244);
        running_=false;
    }

    template<class Registers>
    FrequencyCopy poll(Registers &regs,uint32_t now,bool ready,bool fault) {
        using State=FrequencyCopy::State;
        if(result_.state!=State::Starting && result_.state!=State::Searching)return result_;
        if(fault)return finish(regs,State::Fault);
        if(int32_t(now-deadline_)>=0)return finish(regs,State::Timeout);
        if(!ready)return result_;
        if(!running_) {
            // OEM 0.29 0x0801E94C starts a fresh measurement.
            if(!start(regs,now))return finish(regs,State::Fault);
            return result_;
        }
        if(int32_t(now-next_)<0)return result_;
        next_=now+15u;
        uint16_t high=0,low=0;
        // OEM 0.29 0x0801E5F0: busy bit and 27-bit result in 10 Hz units.
        if(!regs.read(0x0d,high) || high==0xffff)return finish(regs,State::Fault);
        if(high&0x8000)return result_;
        if(!regs.read(0x0e,low))return finish(regs,State::Fault);
        const uint32_t word=(uint32_t(high&0x07ff)<<16)|low;
        if(!word)return result_;

        if(!regs.write(0x32,0x8244))return finish(regs,State::Fault);
        running_=false;
        const auto delta=word>first_?word-first_:first_-word;
        // OEM 0.29 0x08022596..0x080225AC and 0x08022658..0x08022668.
        if(first_ && delta<50u) {
            const uint32_t hz=(((first_+word)/2u+13u)/25u)*250u;
            const uint32_t low_hz=uhf_?400000000u:136000000u;
            const uint32_t high_hz=uhf_?470000000u:174000000u;
            if(hz>=low_hz && hz<=high_hz) {
                result_={State::Found,hz};
                return result_;
            }
            first_=0;
        } else first_=word;
        if(!start(regs,now))return finish(regs,State::Fault);
        return result_;
    }

private:
    template<class Registers>
    bool start(Registers &regs,uint32_t now) {
        running_=true;
        result_.state=FrequencyCopy::State::Searching;
        next_=now+15u;
        return regs.write(0x32,0x8245);
    }

    template<class Registers>
    FrequencyCopy finish(Registers &regs,FrequencyCopy::State state) {
        cancel(regs);
        result_={state,0};
        return result_;
    }

    FrequencyCopy result_{};
    uint32_t first_=0,next_=0,deadline_=0;
    bool uhf_=false,running_=false;
};
}
