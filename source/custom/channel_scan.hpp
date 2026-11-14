// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "application.hpp"
#include "voice_rx.hpp"

namespace rt950::voice {
class ChannelScan final {
public:
    static constexpr unsigned None=Application::MemoryCount;

    unsigned poll(Application &app,uint32_t now,bool enabled,bool sampled,bool open,bool suspended=false) {
        if(!enabled){channel_=None;frequency_=0;started_=suspended_=false;return channel_;}
        if(suspended){suspended_=true;return channel_;}
        const bool resuming=suspended_;
        suspended_=false;
        const bool vfo=app.page()==Page::VfoScan;
        const bool manual_step=app.take_scan_step();
        if(!started_ || vfo!=vfo_) {
            started_=true;vfo_=vfo;channel_=None;frequency_=0;
            advance(app,now);return channel_;
        }
        if(manual_step) {
            advance(app,now);return channel_;
        }
        if(resuming) {
            // The shared receiver is settling again after owning another RX/TX target.
            tuned_at_=now;holding_=false;
            return channel_;
        }
        if(!frequency_ || app.scan_paused())return channel_;
        if(sampled && open) {
            if(app.scan_mode()==ScanMode::Search)app.pause_scan();
            if(!holding_ || app.scan_mode()==ScanMode::Carrier)last_open_=now;
            holding_=true;
        }
        const auto &config=app.config();
        if(holding_) {
            const bool carrier=app.scan_mode()==ScanMode::Carrier;
            const uint32_t limit=carrier?config.hold_ms:5000u;
            if((carrier && sampled && open) || uint32_t(now-last_open_)<limit)return channel_;
        }
        const auto *memory=app.memory(channel_);
        const auto &profile=memory?memory->radio:app.scan_vfo_profile();
        // Leave one extra sampling interval for loop scheduling before advancing.
        constexpr uint32_t tone_dwell=RxRestartMs+RxSettleMs+RxToneSamples*RxSampleMs;
        const uint32_t minimum=!sampled?250u:profile.rx_tone.kind==ToneKind::Off?0u:tone_dwell;
        const uint32_t dwell=config.dwell_ms<minimum?minimum:config.dwell_ms;
        if(uint32_t(now-tuned_at_)<dwell)return channel_;
        advance(app,now);
        return channel_;
    }

    uint32_t frequency() const {return frequency_;}

private:
    void advance(const Application &app,uint32_t now) {
        holding_=false;tuned_at_=now;
        if(vfo_) {
            advance_vfo(app);
            return;
        }
        const bool reverse=app.scan_reverse();
        const unsigned start=channel_==None?(reverse?None-1:0):
            (channel_+(reverse?None-1:1))%None;
        channel_=None;frequency_=0;
        for(unsigned offset=0;offset<None;++offset) {
            const unsigned index=(start+(reverse?None-offset:offset))%None;
            if(app.scan_eligible(index)) {
                channel_=index;frequency_=app.memory(index)->rx_hz;break;
            }
        }
    }

    void advance_vfo(const Application &app) {
        const uint32_t anchor=app.scan_vfo_frequency();
        uint32_t first=0,last=0;
        if(!app.scan_limits(anchor,first,last))return;
        const uint32_t step=Application::vfo_steps[app.applied_vfo_step()];
        if(!frequency_) {
            frequency_=anchor>=first && anchor<=last?anchor:first;
            return;
        }
        // Wrap on the same step grid, even if the upper limit is between steps.
        first+=(frequency_-first)%step;
        last-=(last-first)%step;
        if(app.scan_reverse())frequency_=frequency_-first<step?last:frequency_-step;
        else frequency_=last-frequency_<step?first:frequency_+step;
    }

    unsigned channel_=None;
    uint32_t frequency_=0;
    uint32_t tuned_at_=0,last_open_=0;
    bool started_=false,holding_=false,vfo_=false,suspended_=false;
};
}
