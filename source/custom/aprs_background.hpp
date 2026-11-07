// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::aprs {
class BackgroundListen final {
public:
    static constexpr uint32_t WindowMs=1200;
    explicit BackgroundListen(uint32_t now):since_(now){}

    uint32_t windows() const {return windows_;}
    uint32_t cancelled() const {return cancelled_;}

    bool poll(uint32_t now,bool allowed,bool voice_idle,uint32_t rest_ms) {
        if(!allowed || !rest_ms || rest_ms!=rest_ms_) {
            if(listening_)++cancelled_;
            listening_=false;since_=now;rest_ms_=rest_ms;
        } else if(listening_) {
            if(uint32_t(now-since_)>=WindowMs) {
                listening_=false;since_=now;
            }
        } else if(voice_idle && uint32_t(now-since_)>=rest_ms) {
            listening_=true;since_=now;++windows_;
        }
        return listening_;
    }

private:
    uint32_t since_,rest_ms_=10000;
    uint32_t windows_=0,cancelled_=0;
    bool listening_=false;
};
}
