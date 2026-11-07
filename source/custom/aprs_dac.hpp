// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_afsk.hpp"
#include "aprs_tx.hpp"

namespace rt950::aprs {
// Owns TIM6, DMA2 channel 3 and DAC1 only during a burst. The radio
// session must stop RF if this transport faults or is cancelled.
class DacBurst final {
public:
    enum class State : uint8_t { Idle, Running, Complete, Fault };
    DacBurst()=default;
    DacBurst(const DacBurst &)=delete;
    DacBurst &operator=(const DacBurst &)=delete;
    bool begin(const FrameTransmitter &frame,uint32_t now);
    State poll(uint32_t now);
    void cancel();
private:
    static constexpr unsigned Half=1024;
    void fill(unsigned half);
    void release(State state);

    alignas(4) uint16_t samples_[2*Half]{};
    FrameTransmitter frame_;
    AfskWave wave_;
    size_t symbol_=0;
    unsigned sample_=0,next_half_=0;
    bool final_[2]{};
    State state_=State::Idle;
    uint32_t last_event_=0;
    uint32_t timer_[5]{},dma_[4]{},dac_control_=0,dac_sample_=0;
    uint8_t pa4_mode_=0,pc12_mode_=0;
    bool pb8_high_=false,pc12_high_=false,timer_clock_=false,dac_clock_=false,dma_clock_=false;
};
}
