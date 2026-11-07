// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "radio_settings.hpp"

namespace rt950 {
constexpr bool numeric_power(TxPower power) {
    return power==TxPower::Lpd10mW || power==TxPower::Pmr500mW;
}
constexpr unsigned target_mw(TxPower power) {
    return power==TxPower::Lpd10mW?10:power==TxPower::Pmr500mW?500:0;
}
constexpr bool power_channel(TxPower power,uint32_t hz) {
    if(power==TxPower::Lpd10mW)
        return hz>=433075000u && hz<=434750000u && (hz-433075000u)%25000u==0;
    if(power==TxPower::Pmr500mW)
        return hz>=446006250u && hz<=446193750u && (hz-446006250u)%12500u==0;
    return false;
}
// Russian quick-selection profiles. Keep the wider technical calibration
// reader above compatible with already saved records from Acceptance 11.
// Source: GKRCh 04-03-04-001 and 05-10-02-001, frequency grids.
constexpr unsigned preset_channels(TxPower power) {
    return power==TxPower::Lpd10mW?68:power==TxPower::Pmr500mW?8:0;
}
constexpr uint32_t preset_frequency(TxPower power,unsigned channel) {
    if(!channel || channel>preset_channels(power))return 0;
    return power==TxPower::Lpd10mW?433075000u+(channel-1u)*25000u:
                                  446006250u+(channel-1u)*12500u;
}
constexpr unsigned preset_channel(TxPower power,uint32_t hz) {
    const uint32_t first=preset_frequency(power,1),step=power==TxPower::Lpd10mW?25000u:12500u;
    if(!first || hz<first || (hz-first)%step)return 0;
    const unsigned channel=(hz-first)/step+1u;
    return channel<=preset_channels(power)?channel:0;
}
struct MeasuredPowerPoint {
    uint32_t hz=0;
    uint16_t battery_mv=0,measured_mw=0;
    uint8_t code=0;
    TxPower power=TxPower::Low;
};
// These are operator-entered measurement records, not an RF power sensor.
// No frequency interpolation or extrapolation across battery voltage is made.
class MeasuredPower final {
public:
    static constexpr unsigned Capacity=16,VoltageWindow=100;
    MeasuredPowerPoint points[Capacity]{};
    void clear() {for(auto &point:points)point=MeasuredPowerPoint{};}
    static unsigned distance(unsigned a,unsigned b) {return a>b?a-b:b-a;}
    static bool valid(const MeasuredPowerPoint &point) {
        if(!point.hz)return !point.battery_mv && !point.measured_mw && !point.code && point.power==TxPower::Low;
        return power_channel(point.power,point.hz) && point.code && point.code<255 &&
            point.battery_mv>=6400 && point.battery_mv<=8400 && point.measured_mw &&
            point.measured_mw<=target_mw(point.power);
    }
    const MeasuredPowerPoint *find(TxPower power,uint32_t hz,unsigned mv) const {
        const MeasuredPowerPoint *best=nullptr;
        for(const auto &point:points)
            if(point.hz && valid(point) && point.hz==hz && point.power==power &&
               distance(point.battery_mv,mv)<=VoltageWindow &&
               (!best || distance(point.battery_mv,mv)<distance(best->battery_mv,mv)))best=&point;
        return best;
    }
    bool store(MeasuredPowerPoint point) {
        if(!point.hz || !valid(point))return false;
        MeasuredPowerPoint *empty=nullptr,*replacement=nullptr;
        for(auto &old:points) {
            if(!old.hz && !empty)empty=&old;
            if(old.hz==point.hz && old.power==point.power &&
               distance(old.battery_mv,point.battery_mv)<=VoltageWindow &&
               (!replacement || distance(old.battery_mv,point.battery_mv)<distance(replacement->battery_mv,point.battery_mv)))replacement=&old;
        }
        if(!replacement)replacement=empty;
        if(!replacement)return false; // Never silently evict another measured point.
        *replacement=point;return true;
    }
    unsigned count() const {unsigned n=0;for(const auto &p:points)if(p.hz)++n;return n;}
};
struct TargetPowerDecision {bool ready=true;uint8_t code=0;};
inline TargetPowerDecision resolve_target_power(const MeasuredPower &table,const RadioProfile &profile,
    uint32_t rx,uint32_t tx,unsigned resting_mv,uint8_t factory_low,bool probe,uint8_t probe_code) {
    if(!numeric_power(profile.power))return {};
    if(!power_channel(profile.power,tx) || !profile.narrow || profile.reverse ||
       profile.shift!=Shift::None || rx!=tx || !factory_low)return {false,0};
    const auto *point=table.find(profile.power,tx,resting_mv);
    const uint8_t code=probe?probe_code:point?point->code:0;
    return {code!=0 && code<=factory_low,code};
}
}
