// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
constexpr uint8_t ptt_source(uint8_t mask) {return mask&1u?1u:mask&4u?4u:mask&2u?2u:0u;}
constexpr uint8_t ptt_slot(uint8_t source,uint8_t active) {return source==4u?2u:source==2u?1u:active;}
constexpr bool tail_inputs_released(uint8_t mask,uint8_t enabled) {return (mask&enabled)==0;}

struct TxRequest {
    uint32_t frequency=0;
    uint16_t timeout_seconds=0;
    uint8_t slot=0;
    bool authorized=false,busy_lock=true,tail=false;
};

// Backend contract: begin leaves RF output disabled; poll_ready checks the
// selected TX channel and reports Ready only when all hardware is prepared.
// stop disables RF first and unwinds both prepared and keyed states.
class Ptt final {
public:
    enum class Preparation : uint8_t { Pending, Ready, Failed, Busy };
    enum class State : uint8_t { WaitRelease, Idle, Preparing, Transmitting, Ending };
    enum class ReleaseError : uint8_t { None, Permission, Changed, Timeout, Radio, Signalling };
    State state() const {return state_;}
    uint8_t target_slot() const {return target_.slot;}
    ReleaseError release_error() const {return release_error_;}
    bool blocked_by_carrier() const {return state_==State::WaitRelease && blocked_by_carrier_;}

    template<class Backend>
    void poll(Backend &backend,const TxRequest &request,uint32_t now,bool pressed) {
        if(state_==State::Ending) {
            if(!request.authorized)release_error_=ReleaseError::Permission;
            else if(!same_target(request))release_error_=ReleaseError::Changed;
            else if(!backend.healthy())release_error_=ReleaseError::Radio;
            if(pressed || !request.authorized || !same_target(request) || !backend.healthy() ||
               uint32_t(now-since_)>=ending_ms_)
                finish(backend,pressed?State::WaitRelease:State::Idle);
            return;
        }
        if(!pressed) {
            blocked_by_carrier_=false;
            if(state_==State::Transmitting && target_.tail) {
                if(!request.authorized)release_error_=ReleaseError::Permission;
                else if(!same_target(request))release_error_=ReleaseError::Changed;
                else if(uint32_t(now-since_)>=uint32_t(target_.timeout_seconds)*1000u)
                    release_error_=ReleaseError::Timeout;
                else if(!backend.healthy())release_error_=ReleaseError::Radio;
                else if(!backend.start_tail())release_error_=ReleaseError::Signalling;
                else {
                    const uint32_t remaining=uint32_t(target_.timeout_seconds)*1000u-uint32_t(now-since_);
                    ending_ms_=remaining<TailMilliseconds?remaining:TailMilliseconds;
                    since_=now;state_=State::Ending;return;
                }
            }
            finish(backend,State::Idle);
            return;
        }
        if(state_==State::WaitRelease)return;
        if(!request.authorized || !request.frequency || request.slot>2 ||
           !request.timeout_seconds || request.timeout_seconds>300) {
            finish(backend,State::WaitRelease);
            return;
        }
        if(state_==State::Idle) {
            release_error_=ReleaseError::None;
            if(!backend.begin(request)) {
                backend.stop();
                state_=State::WaitRelease;
                return;
            }
            target_=request;since_=now;state_=State::Preparing;
        }
        if(!same_target(request)) {
            finish(backend,State::WaitRelease);
            return;
        }
        if(state_==State::Transmitting) {
            if(uint32_t(now-since_)>=uint32_t(target_.timeout_seconds)*1000u || !backend.healthy())
                finish(backend,State::WaitRelease);
            return;
        }
        if(uint32_t(now-since_)>=250u) {
            finish(backend,State::WaitRelease);
            return;
        }
        const auto ready=backend.poll_ready(now);
        if(ready==Preparation::Pending)return;
        blocked_by_carrier_=ready==Preparation::Busy ||
            (ready==Preparation::Ready && target_.busy_lock && !backend.channel_clear());
        if(ready!=Preparation::Ready || blocked_by_carrier_ || !backend.key()) {
            finish(backend,State::WaitRelease);
            return;
        }
        since_=now;state_=State::Transmitting;
    }
    static constexpr uint32_t TailMilliseconds=400;
private:
    bool same_target(const TxRequest &request) const {
        return target_.frequency==request.frequency && target_.slot==request.slot &&
            target_.timeout_seconds==request.timeout_seconds && target_.busy_lock==request.busy_lock &&
            target_.tail==request.tail;
    }
    template<class Backend> void finish(Backend &backend,State next) {
        if(state_==State::Preparing || state_==State::Transmitting || state_==State::Ending)backend.stop();
        state_=next;
    }
    State state_=State::WaitRelease;
    TxRequest target_{};
    ReleaseError release_error_=ReleaseError::None;
    uint32_t since_=0,ending_ms_=0;
    bool blocked_by_carrier_=false;
};
}
