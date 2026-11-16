// SPDX-License-Identifier: GPL-3.0-or-later
#include "voice_rx.hpp"
#include "voice_bus.hpp"
#include "voice_tx_board.hpp"
#include "voice_tx_registers.hpp"
extern "C" {
#include "drivers/gpio.h"
#include "../mvp/core.h"
}
namespace rt950::voice {
namespace {
void pin(GPIO_TypeDef *port,uint16_t mask,bool high) {
    if(high)gpio_set_pin(port,mask);
    else gpio_clear_pin(port,mask);
}
void output(uint16_t mask,bool high) {
    pin(GPIOE,mask,high);
    gpio_config_pin(GPIOE,mask,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
}
struct Pins {
    void select(uint8_t chip,bool active){pin(GPIOE,chip?GPIO_PIN_15:GPIO_PIN_8,!active);}
    void clock(bool high){pin(GPIOE,GPIO_PIN_10,high);}
    void drive(bool high){output(GPIO_PIN_11,high);}
    void input(){
        gpio_set_pin(GPIOE,GPIO_PIN_11);
        gpio_config_pin(GPIOE,GPIO_PIN_11,GPIO_MODE_INPUT,GPIO_CNF_PULL);
    }
    bool sample() const {return gpio_read_pin(GPIOE,GPIO_PIN_11);}
    void delay() const {for(volatile unsigned n=0;n<80;++n)__asm volatile("nop");}
};
Pins pins;
RegisterBus<Pins> bus(pins);
#ifdef RT950_MAIN_TX
struct FrequencyRegisters {
    uint8_t chip;
    bool write(uint8_t address,uint16_t value) {return bus.write(chip,address,value);}
    bool read(uint8_t address,uint16_t &value) {return bus.read(chip,address,value);}
};
#endif
// The stock AF helper uses 0xB00F | (level << 4). Start below the
// previous maximum to leave headroom for the analog volume control.
constexpr uint8_t VoiceAfLevel=35;
constexpr uint16_t VoiceAfGain=0xb00f | (VoiceAfLevel<<4);
}
void BoardReceiver::audio(bool enabled,bool speaker) {
    speaker=enabled && speaker;
    if(speaker_ && !speaker)gpio_set_pin(GPIOE,GPIO_PIN_1);
    if(enabled && !audio_) {
        // OEM 0.29 AF selector 1 routes demodulated FM to the audio output.
        bus.write(rx_chip_,0x47,0x6142);
        bus.write(rx_chip_,0x48,VoiceAfGain);
        gpio_set_pin(GPIOE,GPIO_PIN_4);
        gpio_set_pin(GPIOB,GPIO_PIN_8);
    } else if(!enabled && audio_) {
        gpio_clear_pin(GPIOB,GPIO_PIN_8);
        gpio_clear_pin(GPIOE,GPIO_PIN_4);
        bus.write(rx_chip_,0x47,0x6042);
    }
    if(speaker && !speaker_)gpio_clear_pin(GPIOE,GPIO_PIN_1);
    speaker_=speaker;audio_=enabled;
}
void BoardReceiver::enable(bool enabled) {
    if(active_==enabled)return;
    if(!enabled){stop();active_=false;status_={};return;}
    // This diagnostic leaves all external transmit switch lines at idle.
    output(GPIO_PIN_8,true);output(GPIO_PIN_15,true);
    output(GPIO_PIN_10,false);output(GPIO_PIN_11,false);
    active_=true;init_step_=0;
}
void BoardReceiver::stop() {
#ifdef RT950_MAIN_TX
    FrequencyRegisters regs{rx_chip_};
    frequency_counter_.cancel(regs);
#endif
    audio(false);
    if(status_.ready)bus.write(rx_chip_,0x30,0);
    gpio_set_pin(GPIOE,GPIO_PIN_12|GPIO_PIN_13);
    gpio_set_pin(GPIOB,GPIO_PIN_0);
    status_.frequency=0;status_.receiving=false;status_.rssi=0;status_.status=0;
    restart_=false;status_.sampled=false;carrier_=false;sql_open_=false;tone_matches_=0;
}
#ifdef RT950_MAIN_TX
void BoardReceiver::start_frequency_copy(uint32_t hz,uint32_t now,bool narrow) {
    tune_profile(hz,{},0,now,narrow,0);
    frequency_counter_.begin(now,uhf_tx_frequency(hz));
}
FrequencyCopy BoardReceiver::poll_frequency_copy(uint32_t now) {
    FrequencyRegisters regs{rx_chip_};
    return frequency_counter_.poll(regs,now,status_.sampled,status_.fault);
}
#endif
void BoardReceiver::tune(uint32_t hz,uint32_t now) {
    stop();
#ifdef RT950_MAIN_TX
    const bool cb=rt_low_fm_rx_frequency(hz);
#else
    const bool cb=cb_fm_frequency(hz);
#endif
    if(!(cb || vhf_tx_frequency(hz) || uhf_tx_frequency(hz)) || hz%10)return;
    const uint8_t chip=cb?1:0;
    if(chip!=rx_chip_) {
        rx_chip_=chip;status_={};init_step_=0;
    }
    status_.frequency=hz;
    status_.sampled=false;
    if(!status_.ready)return;
    const uint32_t word=hz/10;
    bus.write(rx_chip_,0x38,uint16_t(word));
    bus.write(rx_chip_,0x39,uint16_t(word>>16));
#ifdef RT950_MAIN_TX
    bus.write(rx_chip_,0x43,fm_filter_word(cb || narrow_));
#endif
    due_=now+RxRestartMs;restart_=true;
}
void BoardReceiver::tune_profile(uint32_t hz,Tone tone,uint8_t level,uint32_t now,bool narrow,uint8_t sql) {
    tone_=tone;tone_level_=level;narrow_=narrow;sql_=sql;
    tune(hz,now);
}
void BoardReceiver::set_profile(Tone tone,uint8_t level,uint32_t now,bool narrow,uint8_t sql) {
    const bool sql_changed=sql<=9 && sql_!=sql;
    if(sql_changed) {
        sql_=sql;sql_open_=false;tone_matches_=0;
        status_.receiving=false;status_.sampled=false;
        audio(false);next_sample_=now;
    }
    if(tone.kind==tone_.kind && tone.index==tone_.index && level==tone_level_ && narrow==narrow_)return;
    tone_=tone;tone_level_=level;narrow_=narrow;
    if(status_.frequency)tune(status_.frequency,now);
}

void BoardReceiver::poll(uint32_t now,bool ptt,bool monitor,bool packet_audio,WeatherAudio weather) {
    if(!active_ || status_.fault)return;
    if(!status_.ready) {
        constexpr unsigned prefix=sizeof rx_startup/sizeof rx_startup[0];
        constexpr unsigned suffix=sizeof rx_finish/sizeof rx_finish[0];
        if(init_step_<prefix) {
            const auto reg=rx_startup[init_step_];
            bus.write(rx_chip_,reg.address,reg.value);
        } else if(init_step_==prefix) {
            uint16_t existing=0;
            bus.read(rx_chip_,0x40,existing);
            if(existing==0xffff){status_.fault=true;return;}
            bus.write(rx_chip_,0x40,uint16_t((existing&0xf000)|0x04d2));
        } else {
            const auto reg=rx_finish[init_step_-prefix-1];
            bus.write(rx_chip_,reg.address,reg.value);
        }
        ++init_step_;
        status_.ready=init_step_==prefix+1+suffix;
        if(status_.ready && status_.frequency)tune(status_.frequency,now);
        return;
    }
    if(!status_.frequency)return;
    if(restart_) {
        if(int32_t(now-due_)<0)return;
        // OEM path 1 for UHF (0x0801D658), path 0 for VHF (0x0801D620).
        if(rx_chip_==1) {
            // OEM route 4 selects the low-band RF path.
            gpio_clear_pin(GPIOE,GPIO_PIN_12);
            gpio_clear_pin(GPIOB,GPIO_PIN_0);
            gpio_set_pin(GPIOE,GPIO_PIN_13);
        } else {
            gpio_clear_pin(GPIOE,GPIO_PIN_13);
            if(status_.frequency>=400000000u)gpio_clear_pin(GPIOB,GPIO_PIN_0);
            else gpio_clear_pin(GPIOE,GPIO_PIN_12);
        }
#ifdef RT950_MAIN_TX
        struct RxRegisters {
            uint8_t chip;
            bool write(uint8_t address,uint16_t value) {return bus.write(chip,address,value);}
        } registers{rx_chip_};
        tone_ready_=tone_.kind==ToneKind::Off;
        if(tone_.kind!=ToneKind::Off) {
            const bool calibrated=tone_level_>0 && tone_level_<=127;
            tone_ready_=calibrated && (tone_.kind==ToneKind::Ctcss?
                configure_ctcss(registers,ctcss_frequency(tone_),tone_level_,false):
                configure_dcs(registers,dcs_code(tone_),tone_level_,dcs_inverted(tone_)));
            tone_programmed_|=uint8_t(1u<<rx_chip_);
        } else if(tone_programmed_&(1u<<rx_chip_)) {
            tone_ready_=bus.write(rx_chip_,0x51,0);
            if(tone_ready_)tone_programmed_&=uint8_t(~(1u<<rx_chip_));
        }
#endif
#ifdef RT950_MAIN_TX
        if(rx_chip_==0 && !narrow_)bus.write(rx_chip_,0x30,fm_start_word(false));
#endif
        bus.write(rx_chip_,0x30,0xbff1);
        restart_=false;next_sample_=now+RxSettleMs;
    }
    if(int32_t(now-next_sample_)>=0) {
        next_sample_=now+RxSampleMs;
        uint16_t rssi=0;
        bus.read(rx_chip_,0x0c,status_.status);
        bus.read(rx_chip_,0x67,rssi);
        if(status_.status==0xffff || rssi==0xffff){stop();status_.fault=true;return;}
        status_.sampled=true;
        status_.rssi=rssi&0x01ff;
        // Provisional diagnostic thresholds between the measured idle and P7 levels.
        constexpr uint16_t open_rssi=110,close_rssi=90;
        carrier_=status_.rssi>=(carrier_?close_rssi:open_rssi);
#ifdef RT950_MAIN_TX
        const bool calibrated=sql_ && sql_calibration_ && (vhf_tx_frequency(status_.frequency) ||
            uhf_tx_frequency(status_.frequency));
        const auto threshold=calibrated?sql_calibration_->thresholds(status_.frequency,sql_):fallback_sql(sql_);
        sql_open_=sql_==0 || status_.rssi>=(sql_open_?threshold.close:threshold.open);
        if(sql_open_ && tone_ready_ && rx_tone_detected(tone_,status_.status)) {
            if(tone_matches_<RxToneSamples)++tone_matches_;
        } else tone_matches_=0;
        status_.receiving=(sql_==0?carrier_:sql_open_) && (tone_.kind==ToneKind::Off || tone_matches_>=RxToneSamples);
#else
        status_.receiving=carrier_;
#endif
    }
    if(weather!=WeatherAudio::Off) {
        status_.receiving=status_.sampled && hardware_sql_open(status_.status);
        audio(!ptt && weather==WeatherAudio::Speaker && status_.receiving,true);
        return;
    }
    audio(!ptt && (status_.receiving || ((monitor || packet_audio || (sql_==0 && tone_.kind==ToneKind::Off)) && status_.sampled)),!packet_audio);
}
bool BoardReceiver::squelch_open(uint32_t now) {
    return !restart_ && status_.receiving && int32_t(now-next_sample_)<int32_t(RxSampleMs);
}

bool BoardUhfTx::acquire() {
    if(acquired_ || selector_>=3 || !receiver_.status().ready || receiver_.status().fault)
        return false;
    prepare_board_pa_bias();
    acquired_=true;tuned_=false;fault_=false;chip_started_=false;
    reg52_valid_=false;tail_failure_=TailFailure::None;
    return true;
}

bool BoardUhfTx::listen(uint32_t hz) {
    if(!healthy() || hz%10)return false;
    const bool vhf=vhf_tx_frequency(hz);
    if(cb_fm_frequency(hz)) {
        if(!cb_ || !cb_->bias(hz,selector_))return false;
    } else if(vhf ? (!vhf_ || !vhf_->bias(hz,selector_)) : !uhf_tx_frequency(hz))return false;
    chip_=vhf?0:uhf_tx_chip;
    frequency_=hz;
    return true;
}

BoardUhfTx::Channel BoardUhfTx::channel(uint32_t now) {
    if(!healthy())return Channel::Failed;
    if(!tuned_) {
        receiver_.tune_profile(frequency_,{},0,now,packet_?chip_!=0:narrow_,sql_);
        settled_at_=now+45u;tuned_=true;
    }
    receiver_.poll(now,true);
    if(receiver_.status().fault || receiver_.status().frequency!=frequency_)return Channel::Failed;
    if(!receiver_.status().ready || int32_t(now-settled_at_)<0 || !receiver_.has_sample())
        return Channel::Pending;
    const bool occupied=packet_?receiver_.carrier():receiver_.carrier_at_squelch();
    return occupied?Channel::Busy:Channel::Clear;
}

void BoardUhfTx::mute_receiver(){receiver_.stop();}

void BoardUhfTx::set(TxRoutePin route,bool high) {
    switch(route) {
    case TxRoutePin::Pe7:pin(GPIOE,GPIO_PIN_7,high);break;
    case TxRoutePin::Pe12:pin(GPIOE,GPIO_PIN_12,high);break;
    case TxRoutePin::Pe13:pin(GPIOE,GPIO_PIN_13,high);break;
    case TxRoutePin::Pe14:pin(GPIOE,GPIO_PIN_14,high);break;
    case TxRoutePin::Pb0:pin(GPIOB,GPIO_PIN_0,high);break;
    case TxRoutePin::Pb1:pin(GPIOB,GPIO_PIN_1,high);break;
    }
}

bool BoardUhfTx::read(uint8_t address,uint16_t &value) {
    // Initialize the TX chip before the PTT backend reads the initial mic value.
    if(!healthy())return false;
    if(!chip_started_) {
        chip_started_=true;
        for(const auto &reg:rx_startup)bus.write(chip_,reg.address,reg.value);
        uint16_t old=0;
        bus.read(chip_,0x40,old);
        if(old==0xffff){fault_=true;return false;}
        bus.write(chip_,0x40,(old&0xf000)|0x04d2);
        for(const auto &reg:rx_finish)bus.write(chip_,reg.address,reg.value);
    }
    if(bus.read(chip_,address,value) && value!=0xffff)return true;
    fault_=true;return false;
}

bool BoardUhfTx::write(uint8_t address,uint16_t value) {
    if(!healthy() || !bus.write(chip_,address,value))return false;
    if(address==0x52){reg52_=value;reg52_valid_=true;}
    return true;
}

void BoardUhfTx::delay_us(unsigned microseconds) {
    // HCLK is 120 MHz. Even at one cycle per iteration this waits at
    // least the requested interval, with a fixed bounded loop.
    for(unsigned i=0;i<microseconds*120u;++i)__asm volatile("nop");
}

void BoardUhfTx::bias(uint8_t value) {
    if(!value)clear_board_pa_bias();
    else if(healthy())DAC->DHR8R2=value;
}

bool BoardUhfTx::tx_values(uint16_t mic,UhfTxValues &out) const {
    const bool cb=cb_fm_frequency(frequency_);
    if(chip_!=0 && !cb) {
        if(!uhf_tx_values(factory_,frequency_,selector_,mic,out))return false;
        if(!packet_)out.mic=microphone_word(mic,uint8_t(mic_gain_),factory_.microphone);
        if(bias_hz_ || bias_code_) {
            // Custom numeric targets use a per-unit measurement on this exact
            // UHF frequency. They cannot change PA code above factory Low.
            if(packet_ || selector_!=2 || bias_hz_!=frequency_ || !bias_code_ || bias_code_>out.pa_bias)return false;
            out.pa_bias=bias_code_;
        }
        return true;
    }
    if(bias_hz_ || bias_code_)return false;
    if(factory_.microphone>63)return false;
    const uint8_t bias=cb?(cb_?cb_->bias(frequency_,selector_):0):
        (vhf_?vhf_->bias(frequency_,selector_):0);
    if(!bias)return false;
    const uint32_t word=corrected_frequency_word(frequency_/10,factory_.frequency_correction);
    if(cb?!cb_fm_frequency(word*10u):!vhf_tx_frequency(word*10u))return false;
    out={uint16_t(word),uint16_t(word>>16),microphone_word(mic,packet_?2:uint8_t(mic_gain_),factory_.microphone),bias};
    return true;
}

bool BoardUhfTx::permits_tone(Tone tone) const {
    if(tone_.kind!=tone.kind || tone_.index!=tone.index)return false;
    switch(tone.kind) {
    case ToneKind::Off:return true;
    case ToneKind::Ctcss:return ctcss_frequency(tone) && factory_.ctcss_level && factory_.ctcss_level<=127;
    case ToneKind::Dcs:return dcs_code(tone) && factory_.dcs_level && factory_.dcs_level<=127;
    }
    return false;
}

bool BoardUhfTx::configure_voice(const UhfTxValues &values) {
    const uint8_t level=tone_.kind==ToneKind::Dcs?factory_.dcs_level:factory_.ctcss_level;
    if(!healthy() || !permits_tone(tone_))return false;
    auto profile=values;
    if(packet_)profile.mic=uint16_t((profile.mic&~31u)|0x34u);
    return prepare_voice_tx(*this,profile,packet_?chip_!=0:narrow_,tone_,level);
}

bool BoardUhfTx::finish_voice(){return healthy() && finish_voice_tx(*this,selector_);}

bool BoardUhfTx::start_tail() {
    tail_failure_=TailFailure::State;
    if(packet_ || cb_fm_frequency(frequency_) || !healthy() || !reg52_valid_)return false;
    tail_failure_=TailFailure::Level;
    if(tone_.kind!=ToneKind::Dcs && (!factory_.ctcss_level || factory_.ctcss_level>127))return false;
    // No other RF client runs while acquired. Use the control word written
    // during preparation instead of a fresh serial read with the PA active.
    tail_failure_=TailFailure::Write;
    if(!prepare_tail_tx(*this,tone_,factory_.ctcss_level,reg52_))return false;
    tail_failure_=TailFailure::None;
    return true;
}

void BoardUhfTx::restore_rx() {
    clear_board_pa_bias();
    if(chip_started_) {
        bus.write(chip_,0x30,0);
        bus.write(chip_,0x37,0x1d00);
    }
    gpio_set_pin(GPIOE,GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
    gpio_set_pin(GPIOB,GPIO_PIN_0|GPIO_PIN_1);
    if(chip_started_ && (chip_==0 || cb_fm_frequency(frequency_))) {
        receiver_.enable(false);
        receiver_.enable(true);
    } else receiver_.stop();
    acquired_=false;tuned_=false;chip_started_=false;reg52_valid_=false;
}
}
