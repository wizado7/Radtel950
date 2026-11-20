// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "board.h"

namespace rt950 {
enum class SupplyFault : uint8_t { None, Sensor, Low, High };
constexpr SupplyFault classify_supply(bool fresh,uint8_t raw,uint8_t low,uint8_t high) {
    return !fresh?SupplyFault::Sensor:raw<low?SupplyFault::Low:raw>high?SupplyFault::High:SupplyFault::None;
}
constexpr bool tx_supply_ready(uint32_t age_ms) {
    return age_ms<=BOARD_BATTERY_FAST_MS;
}
constexpr bool tx_supply_live(uint32_t age_ms) {
    // Fast interval plus time for the discard and measured conversions.
    return age_ms<BOARD_BATTERY_FAST_MS+2u*BOARD_BATTERY_CONVERSION_TIMEOUT_MS;
}
class TxBatteryWait final {
public:
    bool waiting(uint32_t now,bool requested,bool ready) {
        if(!requested){active_=false;return false;}
        if(!active_){active_=true;since_=now;}
        return !ready && uint32_t(now-since_)<150u;
    }
private:
    uint32_t since_=0;
    bool active_=false;
};
inline const char *supply_fault_text(SupplyFault fault) {
    switch(fault) {
    case SupplyFault::Sensor:return "TX BATTERY SENSOR";
    case SupplyFault::Low:return "TX LOW BATTERY";
    case SupplyFault::High:return "TX HIGH VOLTAGE";
    default:return "";
    }
}
// A volatile history of actual battery ADC samples around the last TX.
// It neither measures RF power nor controls the transmitter.
class TxSupplyTrace final {
public:
    enum class Owner : uint8_t { Voice, Aprs };
    void observe(uint32_t now,bool transmitting,uint32_t hz,Owner owner,uint8_t code,
                 unsigned mv,uint32_t age,bool fresh) {
        const bool usable=fresh && mv && age<2000u;
        if(!transmitting) {
            active_=false;
            idle_valid_=usable;
            if(usable){idle_mv_=uint16_t(mv);idle_stamp_=now-age;}
            return;
        }
        if(!active_ || frequency_!=hz || owner_!=owner) {
            active_=true;has_session_=true;frequency_=hz;owner_=owner;started_=now;
            before_=idle_valid_ && uint32_t(now-idle_stamp_)<2000u?idle_mv_:0;
            idle_valid_=false;
            minimum_=maximum_=0;samples_=0;sample_seen_=false;fault_=SupplyFault::None;
        }
        code_=code;
        // A sample from before keying is not evidence of voltage under TX.
        if(!usable || age>=uint32_t(now-started_))return;
        const uint32_t stamp=now-age;
        if(sample_seen_ && stamp==sample_stamp_)return;
        sample_seen_=true;sample_stamp_=stamp;
        if(!samples_)minimum_=maximum_=uint16_t(mv);
        else {
            if(mv<minimum_)minimum_=uint16_t(mv);
            if(mv>maximum_)maximum_=uint16_t(mv);
        }
        if(samples_<65535u)++samples_;
    }
    bool has_session() const {return has_session_;}
    uint32_t frequency() const {return frequency_;}
    Owner owner() const {return owner_;}
    unsigned before() const {return before_;}
    unsigned minimum() const {return minimum_;}
    unsigned maximum() const {return maximum_;}
    unsigned samples() const {return samples_;}
    unsigned code() const {return code_;}
    void trip(SupplyFault fault) {if(active_ && fault_==SupplyFault::None)fault_=fault;}
    SupplyFault fault() const {return fault_;}
    bool sag_valid() const {return before_ && samples_;}
    int sag() const {return int(before_)-int(minimum_);}
private:
    uint32_t frequency_=0,started_=0,idle_stamp_=0,sample_stamp_=0;
    uint16_t idle_mv_=0,before_=0,minimum_=0,maximum_=0,samples_=0;
    uint8_t code_=0;
    Owner owner_=Owner::Voice;
    SupplyFault fault_=SupplyFault::None;
    bool active_=false,has_session_=false,idle_valid_=false,sample_seen_=false;
};
// Time attributed to the previous loop state, in a rolling uint32_t timebase.
// These overlapping counters report work scheduled by firmware, not current.
class WorkloadTime final {
public:
    enum Activity : uint8_t { SingleRx, WatchRx, Gps, Ble, AprsRx, Tx, Count };
    void observe(uint32_t now,uint8_t mask) {
        const uint32_t elapsed=now-last_;
        for(unsigned i=0;i<Count;++i)if(mask_&(1u<<i)) {
            const uint32_t room=UINT32_MAX-active_[i];
            active_[i]+=elapsed<room?elapsed:room;
        }
        last_=now;mask_=mask;
    }
    uint32_t seconds(Activity activity) const {return active_[activity]/1000u;}
private:
    uint32_t active_[Count]{},last_=0;
    uint8_t mask_=0;
};
struct PowerDiagnostics {
    TxSupplyTrace trace{};
    WorkloadTime workload{};
    uint32_t adc_age=0xffffffffu,adc_errors=0;
    uint16_t battery_mv=0,raw12=0;
    uint8_t raw8=0,low=0,high=0;
    uint8_t reset_flags=0;
    int mcu_degrees=-32768;
    bool fresh=false,factory_limits=false;
    const char *profile_check="NO DATA";
};
}
