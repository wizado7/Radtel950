// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "fm_probe.hpp"
namespace rt950::fm {
class BoardProbe final {
public:
    void poll(bool open,uint32_t now,Band band=Band::Fm);
    void listen(uint16_t frequency,bool seek,bool audio,uint32_t now,bool seek_up);
    bool seek_finished() const {return seek_finished_;}
    uint16_t seek_result() const {return seek_result_;}
    bool audio_enabled() const {return audio_enabled_;}
    bool audio_error() const {return audio_error_;}
    const Probe &state() const {return probe_;}
private:
    Probe probe_{};
    uint8_t saved_[3]{};
    bool levels_[3]{};
    uint32_t audio_deadline_=0;
    uint16_t seek_result_=0;
    uint8_t audio_step_=0;
    bool active_=false,audio_enabled_=false,audio_error_=false,seek_finished_=false;
    void mute();
};
}
