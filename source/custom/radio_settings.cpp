// SPDX-License-Identifier: GPL-3.0-or-later
#include "radio_settings.hpp"
namespace rt950 {
namespace {
// OEM 0.29 raw tables 0x29A5A and 0x2994E. DCS words are octal codes.
constexpr uint16_t ctcss[]={
    670, 693, 719, 744, 770, 797, 825, 854, 885, 915,
    948, 974, 1000, 1035, 1072, 1109, 1148, 1188, 1230, 1273,
    1318, 1365, 1413, 1462, 1514, 1567, 1598, 1622, 1655, 1679,
    1713, 1738, 1773, 1799, 1835, 1862, 1899, 1928, 1966, 1995,
    2035, 2065, 2107, 2181, 2257, 2291, 2336, 2418, 2503, 2541,
};
constexpr uint16_t dcs[]={
    0023, 0025, 0026, 0031, 0032, 0036, 0043, 0047, 0051, 0053,
    0054, 0065, 0071, 0072, 0073, 0074, 0114, 0115, 0116, 0122,
    0125, 0131, 0132, 0134, 0143, 0145, 0152, 0155, 0156, 0162,
    0165, 0172, 0174, 0205, 0212, 0223, 0225, 0226, 0243, 0244,
    0245, 0246, 0251, 0252, 0255, 0261, 0263, 0265, 0266, 0271,
    0274, 0306, 0311, 0315, 0325, 0331, 0332, 0343, 0346, 0351,
    0356, 0364, 0365, 0371, 0411, 0412, 0413, 0423, 0431, 0432,
    0445, 0446, 0452, 0454, 0455, 0462, 0464, 0465, 0466, 0503,
    0506, 0516, 0523, 0526, 0532, 0546, 0565, 0606, 0612, 0624,
    0627, 0631, 0632, 0645, 0654, 0662, 0664, 0703, 0712, 0723,
    0731, 0732, 0734, 0743, 0754,
};
constexpr unsigned ctcss_count=sizeof(ctcss)/sizeof(ctcss[0]);
constexpr unsigned dcs_count=sizeof(dcs)/sizeof(dcs[0]);
constexpr const char *sql[]={"0","1","2","3","4","5","6","7","8","9"};
constexpr const char *bandwidth[]={"Wide","Narrow"};
constexpr const char *power[]={"Low","Middle","High","10 mW target","0.5 W target"};
constexpr const char *busy[]={"OFF","ON"};
constexpr const char *shift[]={"OFF","+","-"};
constexpr const char *tot[]={"300 s (limit)","15 s","30 s","60 s","120 s","180 s","300 s"};
struct Setting {const char *name;const char *const *options;unsigned count;};
template<unsigned N>
constexpr Setting setting(const char *name,const char *const (&options)[N]) {return {name,options,N};}
constexpr Setting settings[]={
    setting("Squelch (SQL)",sql),setting("FM bandwidth",bandwidth),
    setting("TX power",power),setting("Busy lockout",busy),setting("TX timeout (TOT)",tot),
    {"RX CTCSS",nullptr,1+ctcss_count},{"TX CTCSS",nullptr,1+ctcss_count},
    {"RX DCS",nullptr,1+2*dcs_count},{"TX DCS",nullptr,1+2*dcs_count},
    setting("Shift direction",shift),setting("Reverse",busy),setting("TX inhibit",busy)
};
static_assert(sizeof(settings)/sizeof(settings[0])==unsigned(RadioSetting::Count));
static_assert(sizeof(tot)/sizeof(tot[0])==sizeof(tx_timeouts)/sizeof(tx_timeouts[0]));
}
uint16_t ctcss_frequency(Tone tone) {
    return tone.kind==ToneKind::Ctcss && tone.index && tone.index<=ctcss_count?ctcss[tone.index-1]:0;
}
uint16_t dcs_code(Tone tone) {
    return tone.kind==ToneKind::Dcs && tone.index && tone.index<=2*dcs_count?
        dcs[(tone.index-1)%dcs_count]:0;
}
bool dcs_inverted(Tone tone) {
    return tone.kind==ToneKind::Dcs && tone.index>dcs_count && tone.index<=2*dcs_count;
}
bool radio_is_tone(RadioSetting s) {return s>=RadioSetting::RxCtcss && s<=RadioSetting::TxDcs;}
unsigned radio_option_count(RadioSetting s) {return settings[unsigned(s)].count;}
const char *radio_option_label(RadioSetting s,unsigned option,char *buffer) {
    if(!radio_is_tone(s))return settings[unsigned(s)].options[option];
    if(!option)return "OFF";
    if(s==RadioSetting::RxCtcss || s==RadioSetting::TxCtcss) {
        const unsigned tone=ctcss[option-1];
        char *out=buffer;
        if(tone>=1000)*out++=char('0'+tone/1000);
        *out++=char('0'+tone/100%10);
        *out++=char('0'+tone/10%10);
        *out++='.';
        *out++=char('0'+tone%10);
        *out=0;
    } else {
        const unsigned code=dcs[(option-1)%dcs_count];
        buffer[0]='D';
        buffer[1]=char('0'+(code>>6&7));
        buffer[2]=char('0'+(code>>3&7));
        buffer[3]=char('0'+(code&7));
        buffer[4]=option>dcs_count?'I':'N';
        buffer[5]=0;
    }
    return buffer;
}
const char *radio_setting_label(RadioSetting s) {return settings[unsigned(s)].name;}
unsigned radio_profile_option(const RadioProfile &p,RadioSetting s) {
    switch(s) {
    case RadioSetting::Squelch:return p.squelch;
    case RadioSetting::Bandwidth:return p.narrow;
    case RadioSetting::Power:return unsigned(p.power);
    case RadioSetting::BusyLock:return p.busy_lock;
    case RadioSetting::Timeout:
        for(unsigned i=0;i<radio_option_count(s);++i)if(tx_timeouts[i]==p.timeout_seconds)return i;
        break;
    case RadioSetting::RxCtcss:return p.rx_tone.kind==ToneKind::Ctcss?p.rx_tone.index:0;
    case RadioSetting::TxCtcss:return p.tx_tone.kind==ToneKind::Ctcss?p.tx_tone.index:0;
    case RadioSetting::RxDcs:return p.rx_tone.kind==ToneKind::Dcs?p.rx_tone.index:0;
    case RadioSetting::TxDcs:return p.tx_tone.kind==ToneKind::Dcs?p.tx_tone.index:0;
    case RadioSetting::Shift:return unsigned(p.shift);
    case RadioSetting::Reverse:return p.reverse;
    case RadioSetting::TxInhibit:return p.tx_inhibit;
    case RadioSetting::Count:break;
    }
    return 0;
}
void radio_profile_apply(RadioProfile &p,RadioSetting s,unsigned option) {
    if(s==RadioSetting::Count || option>=radio_option_count(s))return;
    if(radio_is_tone(s)) {
        auto &tone=(s==RadioSetting::RxCtcss || s==RadioSetting::RxDcs)?p.rx_tone:p.tx_tone;
        const auto kind=(s==RadioSetting::RxCtcss || s==RadioSetting::TxCtcss)?ToneKind::Ctcss:ToneKind::Dcs;
        if(option)tone={kind,uint8_t(option)};
        else if(tone.kind==kind)tone={};
        return;
    }
    switch(s) {
    case RadioSetting::Squelch:p.squelch=uint8_t(option);break;
    case RadioSetting::Bandwidth:p.narrow=option!=0;break;
    case RadioSetting::Power:p.power=TxPower(option);break;
    case RadioSetting::BusyLock:p.busy_lock=option!=0;break;
    case RadioSetting::Timeout:p.timeout_seconds=tx_timeouts[option];break;
    case RadioSetting::Shift:p.shift=Shift(option);break;
    case RadioSetting::Reverse:p.reverse=option!=0;break;
    case RadioSetting::TxInhibit:p.tx_inhibit=option!=0;break;
    default:break;
    }
}
}
