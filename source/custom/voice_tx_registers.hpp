// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "voice_tx_values.hpp"
#include "voice_signal.hpp"

namespace rt950::voice {
// OEM 0.29 0x080168C8 selects chip 1 and route 1 for UHF 400-470 MHz.
constexpr uint8_t uhf_tx_chip=1;

// OEM 0.29 0x0801E510 and 0x0801ED50 take the slot bandwidth, not the chip.
constexpr uint16_t fm_filter_word(bool narrow) {return narrow?0x4008:0x3028;}
inline constexpr uint16_t TxActiveWord=0xc1fe,TxSignalWord=0xc3fa;
constexpr uint16_t fm_start_word(bool narrow) {return narrow?0:0x0200;}


// OEM 0.29 0x0800A584 CTCSS branch, factory level from 0xF211.
template<class Radio>
bool configure_ctcss(Radio &radio,uint16_t tenths_hz,uint8_t level,bool extra_level) {
    if(tenths_hz<670 || tenths_hz>2541 || level>127)return false;
    const uint16_t gain=uint16_t((unsigned(level)+(extra_level?15u:0u))&127u);
    const uint16_t word=uint16_t(uint32_t(tenths_hz)*206489u/100000u);
    return radio.write(0x51,0x9000|gain) && radio.write(0x07,word) &&
           radio.write(0x07,0x21cd) && radio.write(0x52,0x0292);
}
// OEM 0.29 0x08015178 builds the normal-polarity DCS word.
constexpr uint32_t dcs_normal_word(uint16_t code) {
    const uint32_t payload=0x800u|code;
    uint32_t parity=0;
    for(unsigned bit=0;bit<12;++bit) {
        const bool feedback=bool(parity&2u)!=bool(payload&(1u<<bit));
        parity=(parity>>1)^(feedback?0xc75u:0u);
    }
    return payload|((parity&0xfffu)<<11);
}

// OEM 0.29 0x0800D78C selects polarity in 0x51, leaving the code word unchanged.
template<class Radio>
bool configure_dcs(Radio &radio,uint16_t code,uint8_t level,bool inverted) {
    if(!code || code>0x1ff || level>127)return false;
    const uint32_t word=dcs_normal_word(code);
    return radio.write(0x51,(inverted?0x8000:0xa000)|level) && radio.write(0x07,0x0ad7) &&
           radio.write(0x08,word&0xfff) && radio.write(0x08,0x8000|(word>>12));
}

// OEM 0.29 0x0801F274 in ordinary RX without a special signalling mode.
inline bool rx_tone_detected(Tone tone,uint16_t status) {
    if(tone.kind==ToneKind::Off)return true;
    if(tone.kind==ToneKind::Ctcss)return (status&0x0400)!=0;
    if(tone.kind==ToneKind::Dcs)return (status&(dcs_inverted(tone)?0x8000:0x4000))!=0;
    return false;
}

// Reference branch: OEM 0.29 0x0801F2A8, normal simplex,
// normal (non-APRS) voice. The caller keeps PA bias at zero.
template<class Radio>
bool prepare_voice_tx(Radio &radio,const UhfTxValues &values,bool narrow,
                      Tone tone={},uint8_t tone_level=0) {
    if(!radio.write(0x37,0x9d1f) || !radio.write(0x30,0))return false;
    radio.delay_us(50);
    if(!radio.write(0x47,0x6142) ||
       !radio.write(0x38,values.frequency_low) ||
       !radio.write(0x39,values.frequency_high) ||
       !radio.write(0x43,fm_filter_word(narrow)) ||
       !radio.write(0x50,0x3b18))return false;
    uint16_t reg52=0,reg31=0,reg40=0;
    if(!radio.read(0x52,reg52) || !radio.write(0x51,0) ||
       !radio.write(0x52,reg52&(narrow?0x0fff:0x1fff)) || !radio.write(0x51,0) ||
       (tone.kind==ToneKind::Ctcss && !configure_ctcss(radio,ctcss_frequency(tone),tone_level,false)) ||
       (tone.kind==ToneKind::Dcs && !configure_dcs(radio,dcs_code(tone),tone_level,dcs_inverted(tone))) ||
       !radio.write(0x7d,values.mic) ||
       !radio.read(0x31,reg31) || !radio.write(0x31,reg31&~2u) ||
       !radio.read(0x40,reg40) || !radio.write(0x40,(reg40&0xf000)|0x050a) ||
       !radio.write(0x30,fm_start_word(narrow)))return false;
    radio.delay_us(50);
    return radio.write(0x30,TxActiveWord);
}

// OEM 0.29 0x0801D7B4 with its extra signalling flag clear.
template<class Radio>
bool prepare_tail_tx(Radio &radio,Tone tone,uint8_t ctcss_level,uint16_t value) {
    value&=0x1fff;
    if(tone.kind==ToneKind::Dcs)value|=0x8000;
    else {
        if(!ctcss_level || ctcss_level>127 ||
           !radio.write(0x51,uint16_t(0x9000u|ctcss_level)) ||
           !radio.write(0x07,0x0471))return false;
    }
    return radio.write(0x52,value);
}

template<class Radio>
bool prepare_tail_tx(Radio &radio,Tone tone,uint8_t ctcss_level) {
    uint16_t value=0;
    return radio.read(0x52,value) && prepare_tail_tx(radio,tone,ctcss_level,value);
}

// OEM 0x0801F114. Selectors are in factory order.
constexpr uint16_t pa_control_word(uint8_t selector) {return selector==0?0x007f:0x0057;}

template<class Radio>
bool finish_voice_tx(Radio &radio,uint8_t selector) {
    return selector<3 && radio.write(0x36,pa_control_word(selector));
}

// Keep the DAC at zero throughout readback. Verify the registers set here.
template<class Radio>
bool verify_voice_tx(Radio &radio,const UhfTxValues &values,uint8_t selector,uint16_t mode=TxActiveWord) {
    const auto matches=[&radio](uint8_t address,uint16_t expected) {
        uint16_t actual=0;
        return radio.read(address,actual) && actual==expected;
    };
    return selector<3 && matches(0x30,mode) && matches(0x38,values.frequency_low) &&
           matches(0x39,values.frequency_high) && matches(0x36,pa_control_word(selector));
}

// BK4829 26 MHz family, rounded to the nearest register unit.
constexpr uint16_t generator_word(uint16_t hz) {
    return uint16_t((uint32_t(hz)*1032444u+50000u)/100000u);
}

// Voice/tone modulation transition. The board keeps external PA bias at zero.
template<class Radio>
bool configure_keyed_signal(Radio &radio,const TxSignal &signal,bool narrow) {
    uint16_t deviation=0;
    if(!radio.read(0x40,deviation) || !radio.write(0x30,fm_start_word(narrow)))return false;
    radio.delay_us(50);
    if(signal && (!radio.write(0x71,generator_word(signal.first)) ||
                  !radio.write(0x72,generator_word(signal.second))))return false;
    return radio.write(0x70,signal?(signal.second?0xc5c5:0x9700):0) &&
           radio.write(0x40,(deviation&0xf000u)|(signal?0x043c:0x050a)) &&
           radio.write(0x47,signal?0x6042:0x6142) &&
           radio.write(0x30,signal?TxSignalWord:TxActiveWord);
}
}
