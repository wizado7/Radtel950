// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950 {
class Spectrum {
public:
    static constexpr unsigned Radius=10,Count=Radius*2+1;
    static constexpr uint16_t Unmeasured=0xffff;

    void begin(uint32_t center,uint32_t step,uint32_t low,uint32_t high) {
        const unsigned below=(center-low)/step,above=(high-center)/step;
        const unsigned left=below<Radius?below:Radius,right=above<Radius?above:Radius;
        first_=center-left*step;step_=step;count_=left+right+1;
        selected_=left;next_=0;tuning_=true;
        for(auto &sample:samples_)sample=Unmeasured;
    }
    unsigned count() const {return count_;}
    unsigned selected() const {return selected_;}
    uint32_t frequency(unsigned point) const {return first_+point*step_;}
    uint16_t sample(unsigned point) const {return samples_[point];}
    bool sweeping() const {return next_<count_;}
    void select(bool right) {
        if(right && selected_+1<count_)++selected_;
        if(!right && selected_)--selected_;
    }
    uint32_t take_tune(uint32_t now) {
        if(!sweeping() || !tuning_)return 0;
        started_=now;tuning_=false;
        return frequency(next_);
    }
    void observe(uint32_t now,uint32_t hz,bool sampled,uint16_t rssi) {
        if(!sweeping() || tuning_)return;
        if(sampled && hz==frequency(next_))samples_[next_]=rssi;
        else if(uint32_t(now-started_)<250u)return;
        ++next_;tuning_=true;
    }
private:
    uint16_t samples_[Count]{};
    uint32_t first_=0,step_=0,started_=0;
    unsigned count_=0,selected_=0,next_=0;
    bool tuning_=false;
};
}
