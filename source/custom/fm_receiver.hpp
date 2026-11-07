// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rt950::fm {
constexpr uint8_t Address=0x11;
constexpr uint16_t MinFrequency=6500, MaxFrequency=10800; // units of 10 kHz
constexpr uint16_t SwMinFrequency=2300, SwMaxFrequency=30000; // kHz
constexpr uint8_t Cts=0x80, Error=0x40, TuneComplete=0x01;

enum class Quality : uint8_t { NoStation, Weak, Fair, Good };
struct Signal {
    uint8_t rssi=0,snr=0;
    bool valid=false,stereo=false,afc_railed=false;
};
inline bool decode_signal(const uint8_t *reply,size_t size,Signal &out) {
    out={};
    if(!reply || size!=8 || (reply[0]&(Cts|Error))!=Cts)return false;
    out={reply[4],reply[5],bool(reply[2]&1u),bool(reply[3]&0x80u),bool(reply[2]&2u)};
    return true;
}

// Preliminary UI thresholds. Keep raw readings available for hardware calibration.
inline Quality quality(const Signal &signal) {
    if(!signal.valid || signal.afc_railed)return Quality::NoStation;
    if(signal.rssi>=40 && signal.snr>=18)return Quality::Good;
    if(signal.rssi>=25 && signal.snr>=10)return Quality::Fair;
    return Quality::Weak;
}

// A supplied bus performs one bounded transaction. No GPIO operations are
// implied here. Board startup must establish powered FM mode first.
class Receiver final {
public:
    enum class Failure : uint8_t { None, Bus, Chip, Timeout, Frequency };
    enum class State : uint8_t { Idle, Tuning, Tuned, Fault };
    bool tune(uint16_t frequency,uint32_t now,bool am=false) {
        if(frequency<(am?SwMinFrequency:MinFrequency) ||
           frequency>(am?SwMaxFrequency:MaxFrequency) || state_==State::Tuning)return false;
        am_=am;
        requested_=frequency;tuned_=0;signal_={};signal_ready_=false;
        failure_=Failure::None;state_=State::Tuning;phase_=Phase::BeforeTune;started_=next_=now;
        return true;
    }
    Failure failure() const {return failure_;}
    State state() const {return state_;}
    uint16_t frequency() const {return tuned_;}
    bool signal(uint32_t now,Signal &out) const {
        out={};
        if(!signal_ready_ || uint32_t(now-sampled_)>=1500u)return false;
        out=signal_;return true;
    }
    template<class Bus> void poll(Bus &bus,uint32_t now) {
        if(state_==State::Idle || state_==State::Fault)return;
        if(state_==State::Tuning && uint32_t(now-started_)>=1000u){fail(Failure::Timeout);return;}
        if(int32_t(now-next_)<0)return;
        next_=now+5;
        uint8_t reply[8]{};
        switch(phase_) {
        case Phase::BeforeTune:
            if(ready(bus))phase_=Phase::SendTune;
            break;
        case Phase::SendTune: {
            const uint8_t command[]={uint8_t(am_?0x40:0x20),uint8_t(am_?0x40:3),
                                     uint8_t(requested_>>8),uint8_t(requested_),0,1};
            if(!bus.write(Address,command,am_?6:5)){fail(Failure::Bus);break;}
            phase_=Phase::QueryInterrupt;next_=now+50;
            break;
        }
        case Phase::QueryInterrupt: {
            if(!ready(bus))break;
            const uint8_t command=0x14;
            if(!bus.write(Address,&command,1)){fail(Failure::Bus);break;}
            phase_=Phase::ReadInterrupt;
            break;
        }
        case Phase::ReadInterrupt:
            if(!read_reply(bus,reply,1))break;
            phase_=(reply[0]&TuneComplete)?Phase::SendStatus:Phase::QueryInterrupt;
            break;
        case Phase::SendStatus: {
            const uint8_t command[]={uint8_t(am_?0x42:0x22),3}; // OEM CANCEL | INTACK after STC
            if(!bus.write(Address,command,sizeof command)){fail(Failure::Bus);break;}
            phase_=Phase::ReadStatus;
            break;
        }
        case Phase::ReadStatus:
            if(!read_reply(bus,reply,am_?8:7))break;
            if((uint16_t(reply[2])<<8 | reply[3])!=requested_){fail(Failure::Frequency);break;}
            tuned_=requested_;state_=State::Tuned;phase_=Phase::SendSignal;
            break;
        case Phase::SendSignal: {
            const uint8_t command[]={uint8_t(am_?0x43:0x23),1};
            if(!bus.write(Address,command,sizeof command)){fail(Failure::Bus);break;}
            phase_=Phase::ReadSignal;request_at_=now;
            break;
        }
        case Phase::ReadSignal:
            if(!read_reply(bus,reply,am_?6:8)) {
                if(state_!=State::Fault && uint32_t(now-request_at_)>=100u)fail(Failure::Timeout);
                break;
            }
            if(am_) {
                signal_={reply[4],reply[5],bool(reply[2]&1u),false,bool(reply[2]&2u)};
                signal_ready_=true;
            } else signal_ready_=decode_signal(reply,8,signal_);
            sampled_=now;phase_=Phase::SendSignal;next_=now+500;
            break;
        }
    }
private:
    enum class Phase : uint8_t { BeforeTune, SendTune, QueryInterrupt, ReadInterrupt, SendStatus, ReadStatus, SendSignal, ReadSignal };
    template<class Bus> bool ready(Bus &bus) {
        uint8_t status=0;
        return read_reply(bus,&status,1);
    }
    template<class Bus> bool read_reply(Bus &bus,uint8_t *reply,size_t size) {
        if(!bus.read(Address,reply,size)){fail(Failure::Bus);return false;}
        if(reply[0]&Error){fail(Failure::Chip);return false;}
        return reply[0]&Cts;
    }
    void fail(Failure why){failure_=why;state_=State::Fault;tuned_=0;signal_ready_=false;signal_={};}
    Failure failure_=Failure::None;
    State state_=State::Idle;
    Phase phase_=Phase::BeforeTune;
    uint32_t started_=0,next_=0,request_at_=0,sampled_=0;
    uint16_t requested_=0,tuned_=0;
    Signal signal_{};
    bool signal_ready_=false,am_=false;
};
}
