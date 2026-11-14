// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
extern "C" {
#include "../mvp/core.h"
}

namespace rt950::voice {
// A single receive path. The backend must gate audio and report a valid
// squelch result only after settling the most recent tuning request.
class Watch final {
public:
    template<class Receiver>
    void poll(Receiver &receiver,const rt_config &config,uint32_t now,bool enabled,bool ptt) {
        poll(receiver,config,now,enabled,ptt,[&](uint8_t,uint32_t frequency) {
            if(frequency)receiver.tune(frequency,now);
            else receiver.stop();
        });
    }

    template<class Receiver,class Select>
    void poll(Receiver &receiver,const rt_config &config,uint32_t now,bool enabled,bool ptt,Select select) {
        if(!enabled || config.active>2) {
            if(running_)receiver.stop();
            running_=false;locked_=false;watch_={};return;
        }
        const bool same_config=running_ && config.active==watch_.last_active &&
            config.watch_mode==mode_ && config.watch_mask==mask_ &&
            config.vfo[slot_].rx_hz==frequency_;
        if(!same_config)watch_={};
        const bool open=same_config && frequency_ && !ptt && receiver.squelch_open(now);
        const uint8_t slot=rt_watch_tick(&watch_,&config,now,open,ptt);
        const uint32_t frequency=config.vfo[slot].rx_hz;
        if(!running_ || slot!=slot_ || frequency!=frequency_) {
            select(slot,frequency);
        }
        locked_=open && slot==slot_;
        running_=true;slot_=slot;frequency_=frequency;
        mode_=config.watch_mode;mask_=config.watch_mask;
    }
    uint8_t slot() const {return slot_;}
    bool receiving() const {return locked_;}
private:
    rt_watch watch_{};
    uint32_t frequency_=0;
    uint8_t slot_=0,mode_=0,mask_=0;
    bool running_=false,locked_=false;
};
}
