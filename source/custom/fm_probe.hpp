// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "fm_receiver.hpp"

namespace rt950::fm {
class Probe final {
public:
    enum class Phase : uint8_t { Off, Reset, InitialCts, PowerWait, PowerCts, Revision, Configure, Receiver, Fault };
    enum class Failure : uint8_t { None, Bus, Chip, NoReply, Timeout, TuneTimeout, TuneFrequency };
    void begin(uint32_t now,Band band=Band::Fm) {
        band_=band;
        phase_=Phase::Reset;failure_=Failure::None;due_=now+10u;deadline_=now+1000u;
        part_=0;property_=0;heard_=false;receiver_=Receiver{};
    }
    void end(){phase_=Phase::Off;receiver_=Receiver{};}
    Phase phase() const {return phase_;}
    Failure failure() const {return failure_;}
    Band band() const {return band_;}
    uint8_t part() const {return part_;}
    bool retune(uint16_t frequency,uint32_t now) {
        return phase_==Phase::Receiver && receiver_.tune(frequency,now,band_);
    }
    bool seek(uint32_t now,bool up=true) {return phase_==Phase::Receiver && receiver_.seek(now,up);}
    bool seeking() const {return receiver_.seeking();}
    void cancel_seek(uint32_t now) {receiver_.cancel_seek(now);}
    bool take_seek_result(uint16_t &frequency) {return receiver_.take_seek_result(frequency);}
    uint16_t frequency() const {return receiver_.frequency();}
    bool signal(uint32_t now,Signal &out) const {return receiver_.signal(now,out);}

    template<class Bus,class Reset> void poll(Bus &bus,Reset reset,uint32_t now) {
        if(phase_==Phase::Off || phase_==Phase::Fault)return;
        if(phase_==Phase::Receiver) {
            receiver_.poll(bus,now);
            switch(receiver_.failure()) {
            case Receiver::Failure::Bus:fail(Failure::Bus);break;
            case Receiver::Failure::Chip:fail(Failure::Chip);break;
            case Receiver::Failure::Timeout:fail(Failure::TuneTimeout);break;
            case Receiver::Failure::Frequency:fail(Failure::TuneFrequency);break;
            case Receiver::Failure::None:break;
            }
            return;
        }
        if(int32_t(now-deadline_)>=0) {
            if(phase_==Phase::InitialCts && !heard_) {
                // 0.29 sends POWER_UP even after the bounded initial CTS poll.
                power_up(bus,now,Failure::NoReply);
            } else fail(Failure::Timeout);
            return;
        }
        if(int32_t(now-due_)<0)return;
        due_=now+5u;
        if(phase_==Phase::Reset){reset(true);phase_=Phase::InitialCts;return;}
        if(phase_==Phase::PowerWait){phase_=Phase::PowerCts;return;}
        uint8_t reply[9]{};
        if(!bus.read(Address,reply,phase_==Phase::Revision?9:1)) {
            // A device may not acknowledge immediately after reset.
            if(phase_!=Phase::InitialCts)fail(Failure::Bus);
            return;
        }
        heard_=true;
        if(reply[0]&Error){fail(Failure::Chip);return;}
        if(!(reply[0]&Cts))return;
        if(phase_==Phase::InitialCts) {
            power_up(bus,now,Failure::Bus);
        } else if(phase_==Phase::PowerCts) {
            const uint8_t command[]={0x10};
            if(!bus.write(Address,command,sizeof command)){fail(Failure::Bus);return;}
            phase_=Phase::Revision;
        } else if(phase_==Phase::Revision) {
            part_=reply[1];phase_=Phase::Configure;deadline_=now+1000u;
        } else if(phase_==Phase::Configure) {
            // 0.29 configures STCIEN and the two FM band edges.
            constexpr uint16_t fm_properties[][2]={{0x0001,1},{0x1400,MinFrequency},{0x1401,MaxFrequency}};
            const auto plan=band_plan(band_);
            const uint16_t am_properties[][2]={{0x0001,1},{0x3202,0},{0x3103,0x7800},
                {0x3400,plan.low},{0x3401,plan.high},{0x3402,plan.seek_spacing}};
            const bool am=band_!=Band::Fm;
            if(property_==(am?6:3)) {
                receiver_.tune(plan.initial,now,band_);phase_=Phase::Receiver;return;
            }
            const auto &property=am?am_properties[property_]:fm_properties[property_];
            const uint16_t id=property[0],value=property[1];
            const uint8_t command[]={0x12,0,uint8_t(id>>8),uint8_t(id),uint8_t(value>>8),uint8_t(value)};
            if(!bus.write(Address,command,sizeof command)){fail(Failure::Bus);return;}
            ++property_;
        }
    }
private:
    template<class Bus> void power_up(Bus &bus,uint32_t now,Failure failure) {
        const uint8_t command[]={0x01,uint8_t(band_!=Band::Fm?0x11:0x10),0x05};
        if(!bus.write(Address,command,sizeof command)){fail(failure);return;}
        due_=now+500u;deadline_=now+1000u;phase_=Phase::PowerWait;
    }
    void fail(Failure why){failure_=why;phase_=Phase::Fault;}
    Receiver receiver_{};
    Phase phase_=Phase::Off;
    Failure failure_=Failure::None;
    uint32_t due_=0,deadline_=0;
    uint8_t part_=0,property_=0;
    bool heard_=false;
    Band band_=Band::Fm;
};
}
