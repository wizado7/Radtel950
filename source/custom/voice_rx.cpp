// SPDX-License-Identifier: GPL-3.0-or-later
#include "voice_rx.hpp"
#include "voice_bus.hpp"
#include "voice_tx_board.hpp"
#include "voice_tx_registers.hpp"
extern "C" {
#include "drivers/gpio.h"
#include "board.h"
#include "../mvp/core.h"
uint32_t get_tick(void);
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
    void delay() const {
        const unsigned loops=custom_clock_low?16u:80u;
        for(volatile unsigned n=0;n<loops;++n)__asm volatile("nop");
    }
};
Pins pins;
RegisterBus<Pins> bus(pins);
void wait_us(unsigned microseconds) {
    const unsigned cycles=microseconds*(custom_clock_low?24u:120u);
    for(unsigned i=0;i<cycles;++i)__asm volatile("nop");
}
bool rf_transmitting() {
    return (GPIOE->ODR&(GPIO_PIN_7|GPIO_PIN_14)) || (GPIOB->ODR&GPIO_PIN_1);
}

void rf_select(GPIO_TypeDef *port, uint16_t mask) {
    gpio_clear_pin(GPIOE,GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
    gpio_clear_pin(GPIOB,GPIO_PIN_0|GPIO_PIN_1);
    if(mask)gpio_set_pin(port,mask);
}

void disable_rx_route() {
    if(!rf_transmitting())rf_select(GPIOE,0);
}

void sleep_chip(uint8_t chip) {
    bus.write(chip,0x47,0x6042);
    bus.write(chip,0x30,0);
    wait_us(1000);
    bus.write(chip,0x37,RfLdoSleep);
}
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
    if(active_==enabled || rf_transmitting())return;
    if(!enabled){standby();active_=false;return;}
    // This diagnostic leaves all external transmit switch lines at idle.
    output(GPIO_PIN_8,true);output(GPIO_PIN_15,true);
    output(GPIO_PIN_10,false);output(GPIO_PIN_11,false);
    sleep_chip(uint8_t(rx_chip_^1u));
    active_=true;idle_requested_=false;init_step_=0;
}
void BoardReceiver::standby() {
    if(rf_transmitting())return;
    const bool initialized=init_step_!=0;
    stop();
    if(initialized)sleep_chip(rx_chip_);
#ifdef RT950_MAIN_TX
    asleep_=false;sleep_eligible_=false;initialized_=0;
#endif
    status_={};init_step_=0;idle_requested_=true;
}
void BoardReceiver::stop() {
    if(rf_transmitting())return;
#ifdef RT950_MAIN_TX
    FrequencyRegisters regs{rx_chip_};
    frequency_counter_.cancel(regs);
    disable_decoder();
    packet_gate_=false;
#endif
    audio(false);
    if(status_.ready)bus.write(rx_chip_,0x30,0);
    disable_rx_route();
    status_.frequency=0;status_.receiving=false;status_.rssi=0;status_.status=0;
    restart_=false;status_.sampled=false;carrier_=false;sql_open_=false;tone_matches_=0;
}
void BoardReceiver::mute_for_tx(uint8_t tx_chip) {
    stop();
    sleep_chip(tx_chip^1u);
#ifdef RT950_MAIN_TX
    if(rx_chip_!=tx_chip)asleep_=true;
#endif
    gpio_set_pin(GPIOE,GPIO_PIN_1);
    gpio_clear_pin(GPIOE,GPIO_PIN_4);
    gpio_clear_pin(GPIOB,GPIO_PIN_8);
}
#ifdef RT950_MAIN_TX
void BoardReceiver::wake_receiver(uint32_t now) {
    if(!asleep_ || rf_transmitting())return;
    bus.write(rx_chip_,0x37,RfLdoOn);
    asleep_=false;restart_=true;due_=now+RfWakeMs;
}

void BoardReceiver::idle_activity(uint32_t now) {
    wake_receiver(now);
    sleep_at_=now+10000u;
    recent_until_=sleep_at_+60000u;
}

uint32_t BoardReceiver::next_poll(uint32_t now) const {
    if(asleep_)return sleep_at_;
    if(!active_ || idle_requested_ || status_.fault)return now+25u;
    if(!status_.ready)return now+1u;
    if(!status_.frequency)return now+25u;
    return restart_?due_:next_sample_;
}

bool BoardReceiver::idle_sleep(uint32_t now,bool allowed,bool packet_window) {
    if(rf_transmitting())return false;
    // Background APRS borrows the receiver without restarting the idle hold.
    if(allowed && packet_window) {
        wake_receiver(now);
        return false;
    }
    if(!low_power_ || !allowed || !status_.ready || !status_.frequency || !sql_ || tone_.kind!=ToneKind::Off ||
       status_.fault || sql_open_ || status_.receiving) {
        sleep_eligible_=false;
        wake_receiver(now);
    } else if(!sleep_eligible_) {
        idle_activity(now);sleep_eligible_=true;
    } else if(int32_t(now-sleep_at_)>=0) {
        if(asleep_) {
            wake_receiver(now);
            sleep_at_=now+(clock_due(now,recent_until_)?180u:300u);
        } else if(!restart_ && status_.sampled) {
            disable_decoder();
            audio(false);
            sleep_chip(rx_chip_);
            // The external receive selects have their own power path.
            // The regular restart sequence restores the band after LDO settle.
            disable_rx_route();
            status_.sampled=false;status_.rssi=0;status_.status=0;
            tone_matches_=0;asleep_=true;sleep_at_=now+(clock_due(now,recent_until_)?800u:300u);
        }
    }
    return asleep_;
}

void BoardReceiver::disable_decoder() {
    if(decoder_==Decoder::Off)return;
    uint16_t irq=0;
    bus.read(rx_chip_,0x3f,irq);
    const bool dtmf=decoder_==Decoder::Dtmf;
    if(irq!=0xffff)bus.write(rx_chip_,0x3f,uint16_t(irq&~(dtmf?0x0800u:0x3002u)));
    if(dtmf)bus.write(rx_chip_,0x24,0);
    else {bus.write(rx_chip_,0x59,0);bus.write(rx_chip_,0x58,0);}
    bus.write(rx_chip_,0x70,0);
    bus.write(rx_chip_,0x02,0);
    decoder_=Decoder::Off;
    same_.reset();
}

char BoardReceiver::decode(Decoder mode,uint32_t now,char (&event)[4]) {
    if(mode==Decoder::Off || !status_.sampled || status_.fault || restart_ || asleep_) {
        disable_decoder();
        return 0;
    }
    if(decoder_!=mode) {
        disable_decoder();
        bus.write(rx_chip_,0x02,0);
        uint16_t irq=0;
        bus.read(rx_chip_,0x3f,irq);
        if(irq==0xffff)return 0;
        if(mode==Decoder::Dtmf) {
            constexpr uint8_t taps[]={0x6f,0x6b,0x67,0x62,0x50,0x47,0x3a,0x2c,
                                      0x41,0x37,0x25,0x17,0xe4,0xcb,0xb5,0x9f};
            for(unsigned i=0;i<sizeof taps;++i)
                bus.write(rx_chip_,0x09,uint16_t((i<<12)|taps[i]));
            bus.write(rx_chip_,0x24,0x87ff);
            bus.write(rx_chip_,0x70,0xc5c5);
            bus.write(rx_chip_,0x3f,uint16_t(irq|0x0800));
        } else {
            constexpr RegisterValue fsk[]={
                {0x58,0x03c5},{0x70,0x0080},{0x72,5377},{0x5a,0x5ac2},{0x5b,0x5ac2},
                {0x5c,0xa730},{0x5d,0xff00},{0x59,0x40f8},{0x59,0x10f8},
            };
            for(const auto &reg:fsk)bus.write(rx_chip_,reg.address,reg.value);
            bus.write(rx_chip_,0x3f,uint16_t((irq&~0xc000u)|0x3002));
        }
        decoder_=mode;decode_at_=now;
        return 0;
    }
    if(int32_t(now-decode_at_)<0)return 0;
    decode_at_=now+8u;
    uint16_t flags=0;
    bus.read(rx_chip_,0x0c,flags);
    if(flags==0xffff || !(flags&1))return 0;
    bus.write(rx_chip_,0x02,0);
    bus.read(rx_chip_,0x02,flags);
    if(flags==0xffff)return 0;
    if(mode==Decoder::Dtmf) {
        if(!(flags&0x0800) || !status_.receiving)return 0;
        uint16_t code=0;
        bus.read(rx_chip_,0x0b,code);
        return code==0xffff?0:"0123456789ABCD*#"[(code>>8)&15];
    }
    if(flags&2)same_.sync();
    unsigned words=(flags&0x2000)?8u:(flags&0x1000)?4u:0u;
    while(words-- && same_.active()) {
        uint16_t word=0;
        bus.read(rx_chip_,0x5f,word);
        if(same_.push(word,now,event)){disable_decoder();return 1;}
    }
    if((flags&0x2000) || ((flags&0x1000) && !same_.active()))disable_decoder();
    return 0;
}

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
    if(rf_transmitting())return;
    stop();
#ifdef RT950_MAIN_TX
    const bool cb=rt_low_fm_rx_frequency(hz);
#else
    const bool cb=cb_fm_frequency(hz);
#endif
    if(!(cb || vhf_tx_frequency(hz) || uhf_tx_frequency(hz)) || hz%10)return;
    const uint8_t chip=cb?1:0;
    if(chip!=rx_chip_) {
        if(init_step_)sleep_chip(rx_chip_);
        rx_chip_=chip;status_={};init_step_=0;
#ifdef RT950_MAIN_TX
        // A chip in LDO standby retains the initialized register set.
        asleep_=status_.ready=(initialized_&(1u<<chip))!=0;
        if(asleep_)init_step_=1;
#endif
    }
    idle_requested_=false;status_.frequency=hz;
    status_.sampled=false;
    if(!status_.ready)return;
#ifdef RT950_MAIN_TX
    const bool was_asleep=asleep_;
    wake_receiver(now);
#endif
    const uint32_t word=hz/10;
    bus.write(rx_chip_,0x38,uint16_t(word));
    bus.write(rx_chip_,0x39,uint16_t(word>>16));
#ifdef RT950_MAIN_TX
    bus.write(rx_chip_,0x43,fm_filter_word(cb || narrow_));
#endif
    restart_=true;
#ifdef RT950_MAIN_TX
    if(!was_asleep)due_=now+RxRestartMs;
#else
    due_=now+RxRestartMs;
#endif
}
void BoardReceiver::tune_profile(uint32_t hz,Tone tone,uint8_t level,uint32_t now,bool narrow,uint8_t sql) {
    tone_=tone;tone_level_=level;narrow_=narrow;sql_=sql;
    tune(hz,now);
}
void BoardReceiver::set_profile(Tone tone,uint8_t level,uint32_t now,bool narrow,uint8_t sql) {
    if(rf_transmitting())return;
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
    if(rf_transmitting())return;
    if(!active_ || idle_requested_ || status_.fault)return;
#ifdef RT950_MAIN_TX
    if(asleep_) {
        if(!ptt && !monitor && !packet_audio && weather==WeatherAudio::Off)return;
        wake_receiver(now);
    }
#endif
    if(!status_.ready) {
        if(init_step_==3 && int32_t(now-due_)<0)return;
        constexpr unsigned prefix=sizeof rx_startup/sizeof rx_startup[0];
        constexpr unsigned suffix=sizeof rx_finish/sizeof rx_finish[0];
        if(init_step_<prefix) {
            const auto reg=rx_startup[init_step_];
            bus.write(rx_chip_,reg.address,reg.value);
            if(reg.address==0x37)due_=now+RfWakeMs;
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
        if(status_.ready) {
#ifdef RT950_MAIN_TX
            initialized_|=uint8_t(1u<<rx_chip_);
#endif
            if(status_.frequency)tune(status_.frequency,now);
        }
        return;
    }
    if(!status_.frequency)return;
    if(restart_) {
        if(int32_t(now-due_)<0)return;
        // Retuning and sleep both clear the receive selects before restart.
        // OEM 0.29 mode 1: one active-high receive path.
        if(rx_chip_==1)rf_select(GPIOE,GPIO_PIN_13);
        else if(status_.frequency>=400000000u)rf_select(GPIOE,GPIO_PIN_12);
        else rf_select(GPIOB,GPIO_PIN_0);
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
        status_.sampled=true;status_.sampled_at=now;
        status_.rssi=rssi&0x01ff;
        // Provisional diagnostic thresholds between the measured idle and P7 levels.
        constexpr uint16_t open_rssi=110,close_rssi=90;
        carrier_=status_.rssi>=(carrier_?close_rssi:open_rssi);
#ifdef RT950_MAIN_TX
        const bool calibrated=sql_ && sql_calibration_ && (vhf_tx_frequency(status_.frequency) ||
            uhf_tx_frequency(status_.frequency));
        const auto threshold=calibrated?sql_calibration_->thresholds(status_.frequency,sql_):fallback_sql(sql_);
        sql_open_=sql_==0 || status_.rssi>=(sql_open_?threshold.close:threshold.open);
        const bool signal=sql_==0?carrier_:sql_open_;
        if(!signal || (tone_.kind!=ToneKind::Off && !tone_ready_)) {
            tone_matches_=0;
            status_.receiving=false;
        } else if(tone_.kind==ToneKind::Off)status_.receiving=true;
        else if(rx_tone_detected(tone_,status_.status)) {
            if(status_.receiving)tone_matches_=RxToneSamples;
            else status_.receiving=++tone_matches_==RxToneSamples;
        } else if(status_.receiving) {
            // Brief detector dropouts must not mute an already qualified carrier.
            status_.receiving=--tone_matches_!=0;
        } else tone_matches_=0;
#else
        status_.receiving=carrier_;
#endif
    }
    if(weather!=WeatherAudio::Off) {
        status_.receiving=status_.sampled && hardware_sql_open(status_.status);
        audio(!ptt && weather==WeatherAudio::Speaker && status_.receiving,true);
        return;
    }
#ifdef RT950_MAIN_TX
    if(packet_audio) {
        if(status_.sampled && hardware_sql_open(status_.status)) {
            packet_gate_=true;packet_until_=now+120u;
        }
        if(ptt || !status_.sampled || int32_t(now-packet_until_)>=0)packet_gate_=false;
        audio(packet_gate_);
        return;
    }
    packet_gate_=false;
    audio(!ptt && (status_.receiving || ((monitor || (sql_==0 && tone_.kind==ToneKind::Off)) && status_.sampled)),true);
#else
    audio(!ptt && (status_.receiving || ((monitor || packet_audio || (sql_==0 && tone_.kind==ToneKind::Off)) && status_.sampled)),!packet_audio);
#endif
}
bool BoardReceiver::squelch_open(uint32_t now) {
    return !restart_ && status_.receiving && int32_t(now-next_sample_)<int32_t(RxSampleMs);
}

bool BoardUhfTx::acquire() {
    if(acquired_ || selector_>=3 || !receiver_.status().ready || receiver_.status().fault)
        return false;
    prepare_board_pa_bias();
    accessory_=!packet_ && !(GPIOC->IDR&GPIO_PIN_15);
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

void BoardUhfTx::mute_receiver() {
    receiver_.mute_for_tx(chip_);
    if(packet_)return;
    // Confirm the jack again after the channel-check dwell. Hold the
    // microphone route for the entire keyed transmission.
    accessory_=accessory_ && !(GPIOC->IDR&GPIO_PIN_15);
    pin(GPIOE,GPIO_PIN_1,accessory_);
    pin(GPIOB,GPIO_PIN_8,accessory_);
}

void BoardUhfTx::set(TxRoutePin route,bool high) {
    static constexpr uint16_t masks[]={GPIO_PIN_7,GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_0,GPIO_PIN_1};
    const unsigned index=unsigned(route);
    auto *const port=index<unsigned(TxRoutePin::Pb0)?GPIOE:GPIOB;
    if(high)rf_select(port,masks[index]);
    else gpio_clear_pin(port,masks[index]);
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
    for(unsigned i=0;i<microseconds*clock_mhz(clock_mult());++i)__asm volatile("nop");
}

void BoardUhfTx::bias(uint8_t value) {
    if(!value)clear_board_pa_bias();
    else if(healthy())board_pa_bias_write(value);
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
    if(!read(0x3b,xtal_))return false;
    auto profile=values;
    if(packet_) {
        if(!read(0x2b,packet_filter_) || !read(0x19,packet_agc_) || !read(0x7d,packet_mic_))return false;
        packet_regs_=true;
        profile.mic=uint16_t((profile.mic&~0x7fu)|0x74u);
    }
    if(!prepare_voice_tx(*this,profile,packet_?chip_!=0:narrow_,tone_,level))return false;
    return !packet_ || (write(0x2b,(packet_filter_&~7u)|4u) && write(0x19,packet_agc_|0x8000u));
}

bool BoardUhfTx::verify_crystal(uint8_t &failed) {
    uint16_t value;
    if(read(0x3b,value) && value==xtal_)return true;
    failed=0x3b;
    return false;
}

bool BoardUhfTx::finish_voice(){return healthy() && finish_voice_tx(*this,selector_);}
bool BoardUhfTx::configure_signal(const TxSignal &signal) {
    if(!healthy() || packet_ || cb_fm_frequency(frequency_))return false;
    signal_used_=true;
    return configure_keyed_signal(*this,signal,narrow_);
}

bool BoardUhfTx::start_tail() {
    tail_failure_=TailFailure::State;
    if(packet_ || cb_fm_frequency(frequency_) || !healthy() || !reg52_valid_)return false;
    // No other RF client runs while acquired. Use the control word written
    // during preparation instead of a fresh serial read with the PA active.
    tail_failure_=TailFailure::Write;
    if(!prepare_tail_tx(*this,tone_,reg52_,factory_.ctcss_level))return false;
    tail_failure_=TailFailure::None;
    return true;
}

void BoardUhfTx::restore_rx() {
    release_board_pa_bias();
    if(guard_ && !fault_ && rf_transmitting()) {
        gpio_set_pin(GPIOE,GPIO_PIN_1);
        gpio_clear_pin(GPIOE,GPIO_PIN_4);
        gpio_clear_pin(GPIOB,GPIO_PIN_8);
        settled_at_=get_tick()+100u;
        releasing_=true;
        return;
    }
    finish_release();
}

void BoardUhfTx::poll_release(uint32_t now,bool force) {
    if(releasing_ && (force || clock_due(now,settled_at_)))finish_release();
}

void BoardUhfTx::finish_release() {
    if(chip_started_) {
#ifdef RT950_MAIN_TX
        receiver_.invalidate_chip(chip_);
#endif
        bus.write(chip_,0x30,0);
        bus.write(chip_,0x51,0);
        if(reg52_valid_)bus.write(chip_,0x52,reg52_&0x1fffu);
        if(signal_used_)bus.write(chip_,0x70,0);
        if(packet_regs_) {
            bus.write(chip_,0x2b,packet_filter_);
            bus.write(chip_,0x19,packet_agc_);
            bus.write(chip_,0x7d,packet_mic_);
            packet_regs_=false;
        }
        bus.write(chip_,0x37,RfLdoSleep);
    }
    rf_select(GPIOE,0);
    gpio_set_pin(GPIOE,GPIO_PIN_1);
    if(accessory_)gpio_clear_pin(GPIOB,GPIO_PIN_8);
    if(chip_started_ && (chip_==0 || cb_fm_frequency(frequency_))) {
        receiver_.enable(false);
        receiver_.enable(true);
    } else receiver_.stop();
    releasing_=false;signal_used_=false;acquired_=false;tuned_=false;chip_started_=false;reg52_valid_=false;
}
}
