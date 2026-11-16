// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace rt950 {
enum class RadioSetting : uint8_t { Squelch, Bandwidth, Power, BusyLock, Timeout, RxCtcss, TxCtcss, RxDcs, TxDcs, Shift, Reverse, TxInhibit, Count };
enum class Shift : uint8_t { None, Plus, Minus };
enum class TxPower : uint8_t { Low, Middle, High, Lpd10mW, Pmr500mW };
// OEM global function setting +0x32, menu 0x08019390 / setter 0x08020EDC.
enum class MicGain : uint8_t { Low, Middle, High };
enum class ToneKind : uint8_t { Off, Ctcss, Dcs };
struct Tone {ToneKind kind=ToneKind::Off;uint8_t index=0;};
struct RadioProfile {
    uint32_t offset_hz=0;
    Shift shift=Shift::None;
    bool reverse=false,tx_inhibit=false;
    Tone rx_tone{},tx_tone{};
    uint16_t timeout_seconds=60;
    uint8_t squelch=3;
    TxPower power=TxPower::Low;
    bool narrow=true,busy_lock=true;
    Tone heard_tone(bool reversed) const {return reversed?tx_tone:rx_tone;}
    Tone sent_tone(bool reversed) const {return reversed?rx_tone:tx_tone;}
};
constexpr uint16_t tx_timeouts[]={0,15,30,60,120,180,300};
uint16_t ctcss_frequency(Tone tone);
uint16_t dcs_code(Tone tone);
bool dcs_inverted(Tone tone);
bool radio_is_tone(RadioSetting setting);
unsigned radio_option_count(RadioSetting setting);
inline constexpr unsigned RadioOptionLabelSize=16;
const char *radio_option_label(RadioSetting setting,unsigned option,char *buffer,bool russian=false);
const char *radio_setting_label(RadioSetting setting);
unsigned radio_profile_option(const RadioProfile &profile,RadioSetting setting);
void radio_profile_apply(RadioProfile &profile,RadioSetting setting,unsigned option);
}
