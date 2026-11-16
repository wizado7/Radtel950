// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "voice_rx.hpp"
#include "voice_uhf_tx.hpp"

namespace rt950::voice {
// Requires exclusive ownership of both RF paths and the PA DAC while keyed.
// The caller suspends Watch before begin and re-arms it after restore_rx.
class BoardUhfTx final {
public:
    enum class TailFailure : uint8_t { None, State, Level, Write };
    using Channel=UhfTransmitter<BoardUhfTx>::Channel;
    BoardUhfTx(BoardReceiver &receiver,const UhfCalibration &factory,uint8_t selector,
               bool (*hold_check)()=nullptr,const VhfPaCalibration *vhf=nullptr,
               const CbPaCalibration *cb=nullptr,bool packet=false)
        :receiver_(receiver),factory_(factory),hold_check_(hold_check),vhf_(vhf),cb_(cb),selector_(selector),packet_(packet){}
    bool acquire();
    bool listen(uint32_t hz);
    Channel channel(uint32_t now);
    void mute_receiver();
    void set(TxRoutePin pin,bool high);
    bool read(uint8_t address,uint16_t &value);
    bool write(uint8_t address,uint16_t value);
    void delay_us(unsigned microseconds);
    void bias(uint8_t value);
    bool configure_voice(const UhfTxValues &values);
    bool finish_voice();
    bool start_tail();
    TailFailure tail_failure() const {return tail_failure_;}
    bool healthy() const {return acquired_ && !fault_ && (!hold_check_ || hold_check_());}
    void restore_rx();
    bool tx_values(uint16_t mic,UhfTxValues &out) const;
    void select_power(uint8_t selector) {if(!acquired_)selector_=selector;}
    void select_measured_bias(uint32_t hz,uint8_t code) {if(!acquired_){bias_hz_=hz;bias_code_=code;}}
    uint8_t measured_bias() const {return bias_code_;}
    uint32_t measured_frequency() const {return bias_hz_;}
    void select_channel_check(bool narrow,uint8_t sql) {
        if(!acquired_) {narrow_=narrow;sql_=sql;}
    }
    void select_tone(Tone tone) {if(!acquired_)tone_=tone;}
    void select_microphone(MicGain gain) {
        if(!packet_ && !acquired_ && unsigned(gain)<=unsigned(MicGain::High))mic_gain_=gain;
    }
    MicGain microphone_gain() const {return mic_gain_;}
    bool permits_tone(Tone tone) const;
    uint8_t power_selector() const {return selector_;}
private:
    BoardReceiver &receiver_;
    const UhfCalibration &factory_;
    bool (*hold_check_)();
    const VhfPaCalibration *vhf_;
    const CbPaCalibration *cb_;
    uint8_t chip_=1;
    uint32_t frequency_=0,settled_at_=0;
    uint32_t bias_hz_=0;
    uint8_t bias_code_=0;
    Tone tone_{};
    MicGain mic_gain_=MicGain::High;
    uint16_t reg52_=0;
    bool reg52_valid_=false;
    TailFailure tail_failure_=TailFailure::None;
    uint8_t selector_,sql_=3;
    bool packet_;
    bool narrow_=true;
    bool acquired_=false,tuned_=false,fault_=false,chip_started_=false;
};
}
