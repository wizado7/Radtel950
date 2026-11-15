// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rt950::fm {
constexpr uint8_t Address=0x11;
constexpr uint16_t MinFrequency=6500, MaxFrequency=10800; // units of 10 kHz
enum class Band : uint8_t { Fm, Shortwave, Longwave, Mediumwave };
struct BandPlan {uint16_t low,high,initial,seek_spacing;};
constexpr BandPlan band_plan(Band band) {
    switch(band) {
    case Band::Longwave:return {153,279,198,9};
    case Band::Mediumwave:return {520,1710,999,9};
    case Band::Shortwave:return {2300,30000,7100,5};
    case Band::Fm:return {MinFrequency,MaxFrequency,10030,0};
    }
    return {};
}
constexpr bool in_band(unsigned frequency,Band band) {
    const auto plan=band_plan(band);
    return frequency>=plan.low && frequency<=plan.high;
}
constexpr Band am_band(unsigned frequency) {
    if(in_band(frequency,Band::Longwave))return Band::Longwave;
    if(in_band(frequency,Band::Mediumwave))return Band::Mediumwave;
    if(in_band(frequency,Band::Shortwave))return Band::Shortwave;
    return Band::Fm;
}
constexpr uint8_t Cts=0x80, Error=0x40, TuneComplete=0x01;
constexpr uint8_t FairRssi=25,FairSnr=10;

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
    if(signal.rssi>=FairRssi && signal.snr>=FairSnr)return Quality::Fair;
    return Quality::Weak;
}

// A supplied bus performs one bounded transaction. No GPIO operations are
// implied here. Board startup must establish powered FM mode first.
class Receiver final {
public:
    enum class Failure : uint8_t { None, Bus, Chip, Timeout, Frequency };
    enum class State : uint8_t { Idle, Tuning, Tuned, Fault };
    bool tune(uint16_t frequency,uint32_t now,Band band=Band::Fm) {
        if(!in_band(frequency,band) || state_==State::Tuning)return false;
        band_=band;operation_=Operation::Tune;seek_done_=false;
        requested_=frequency;tuned_=0;signal_={};signal_ready_=false;
        failure_=Failure::None;state_=State::Tuning;phase_=Phase::BeforeTune;started_=next_=now;
        return true;
    }
    bool seek(uint32_t now,bool up=true) {
        if(state_!=State::Tuned)return false;
        requested_=tuned_;tuned_=0;signal_={};signal_ready_=false;
        seek_done_=false;seek_up_=up;operation_=Operation::Seek;
        state_=State::Tuning;phase_=Phase::BeforeTune;started_=next_=now;
        return true;
    }
    bool seeking() const {return state_==State::Tuning && operation_!=Operation::Tune;}
    void cancel_seek(uint32_t now,bool report=false) {
        if(!seeking() || operation_==Operation::Cancel)return;
        operation_=Operation::Cancel;
        report_cancel_=report;
        phase_=Phase::BeforeCancel;started_=next_=now;
    }
    bool take_seek_result(uint16_t &frequency) {
        if(!seek_done_)return false;
        seek_done_=false;frequency=tuned_;return true;
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
        const bool am=band_!=Band::Fm;
        if(state_==State::Tuning && uint32_t(now-started_)>=
           (operation_==Operation::Seek?(am?180000u:30000u):1000u)) {
            if(operation_!=Operation::Seek){fail(Failure::Timeout);return;}
            cancel_seek(now,true);
        }
        if(int32_t(now-next_)<0)return;
        next_=now+5;
        uint8_t reply[8]{};
        switch(phase_) {
        case Phase::BeforeTune:
            if(ready(bus))phase_=Phase::SendTune;
            break;
        case Phase::SendTune: {
            if(operation_==Operation::Seek) {
                // OEM 0.29 sends 41 0C. AN332 specifies SEEKUP at bit 3 and ANTCAP=1 for SW.
                const uint8_t command[]={uint8_t(am?0x41:0x21),uint8_t(seek_up_?0x0c:0x04),0,0,0,1};
                if(!bus.write(Address,command,band_==Band::Shortwave?6:2)){
                    fail(Failure::Bus);break;
                }
            } else {
                const uint8_t command[]={uint8_t(am?0x40:0x20),uint8_t(am?0x40:3),
                                         uint8_t(requested_>>8),uint8_t(requested_),0,1};
                if(!bus.write(Address,command,am?6:5)){fail(Failure::Bus);break;}
            }
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
        case Phase::BeforeCancel:
            if(ready(bus))phase_=Phase::SendStatus;
            break;
        case Phase::SendStatus: {
            // INTACK completes a seek. CANCEL | INTACK aborts it or finishes a manual tune.
            const uint8_t command[]={uint8_t(am?0x42:0x22),uint8_t(operation_==Operation::Seek?1:3)};
            if(!bus.write(Address,command,sizeof command)){fail(Failure::Bus);break;}
            phase_=Phase::ReadStatus;
            break;
        }
        case Phase::ReadStatus: {
            if(!read_reply(bus,reply,am?8:7))break;
            const uint16_t frequency=uint16_t(reply[2])<<8 | reply[3];
            if(operation_==Operation::Tune) {
                if(frequency!=requested_){fail(Failure::Frequency);break;}
                tuned_=frequency;
            } else {
                constexpr uint8_t valid=1,band_limit=0x80;
                if(operation_==Operation::Seek && (reply[1]&(valid|band_limit))==valid &&
                   frequency!=requested_ && in_band(frequency,band_))tuned_=frequency;
                seek_done_=operation_==Operation::Seek || report_cancel_;
            }
            state_=tuned_?State::Tuned:State::Idle;phase_=Phase::SendSignal;
            break;
        }
        case Phase::SendSignal: {
            const uint8_t command[]={uint8_t(am?0x43:0x23),1};
            if(!bus.write(Address,command,sizeof command)){fail(Failure::Bus);break;}
            phase_=Phase::ReadSignal;request_at_=now;
            break;
        }
        case Phase::ReadSignal:
            if(!read_reply(bus,reply,am?6:8)) {
                if(state_!=State::Fault && uint32_t(now-request_at_)>=100u)fail(Failure::Timeout);
                break;
            }
            if(am) {
                signal_={reply[4],reply[5],bool(reply[2]&1u),false,bool(reply[2]&2u)};
                signal_ready_=true;
            } else signal_ready_=decode_signal(reply,8,signal_);
            sampled_=now;phase_=Phase::SendSignal;next_=now+500;
            break;
        }
    }
private:
    enum class Phase : uint8_t { BeforeTune, SendTune, QueryInterrupt, ReadInterrupt, BeforeCancel, SendStatus, ReadStatus, SendSignal, ReadSignal };
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
    enum class Operation : uint8_t { Tune, Seek, Cancel };
    Operation operation_=Operation::Tune;
    Band band_=Band::Fm;
    bool signal_ready_=false,seek_done_=false,report_cancel_=false,seek_up_=true;
};
}
