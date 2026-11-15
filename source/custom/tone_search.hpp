// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "radio_settings.hpp"

namespace rt950 {
class ToneSearch {
public:
    enum class Result : uint8_t { Pending, Found, Exhausted, Unavailable };

    void begin(ToneKind kind) {
        const auto setting=kind==ToneKind::Ctcss?RadioSetting::RxCtcss:RadioSetting::RxDcs;
        candidate_={kind,1};
        count_=radio_option_count(setting);
        tuning_=true;
    }
    void cancel() {candidate_={};}
    bool active() const {return candidate_.kind!=ToneKind::Off;}
    Tone candidate() const {return candidate_;}
    bool take_tune(uint32_t now) {
        if(!active() || !tuning_)return false;
        tuning_=false;
        started_=now;
        return true;
    }
    Result poll(uint32_t now,bool sampled,bool matched,bool fault) {
        if(!active() || tuning_)return Result::Pending;
        if(fault)return Result::Unavailable;
        if(sampled && matched)return Result::Found;
        const uint32_t dwell=candidate_.kind==ToneKind::Ctcss?150u:400u;
        if(uint32_t(now-started_)<dwell)return Result::Pending;
        if(!sampled)return Result::Unavailable;
        if(unsigned(candidate_.index)+1==count_)return Result::Exhausted;
        ++candidate_.index;
        tuning_=true;
        return Result::Pending;
    }
private:
    Tone candidate_{};
    unsigned count_=0;
    uint32_t started_=0;
    bool tuning_=false;
};
}
