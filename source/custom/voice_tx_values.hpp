// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#include "voice_calibration.hpp"
#include "radio_settings.hpp"

namespace rt950::voice {
// OEM 0.29 Trans Power table, 0x200004E4 after startup decompression.
constexpr uint8_t factory_power_selector(TxPower power) {
    switch(power) {
    case TxPower::High:return 0;
    case TxPower::Middle:return 1;
    case TxPower::Low:case TxPower::Lpd10mW:case TxPower::Pmr500mW:return 2;
    }
    return 0xff;
}

// OEM 0.29, 0x0801E4D0. The OR intentionally preserves bit 5 from
// the old register and from the selected gain, as the stock code does.
constexpr uint16_t microphone_word(uint16_t old,uint8_t source,uint8_t calibrated) {
    const unsigned gain=source==2?(calibrated&63u):source==1?45u:36u;
    return uint16_t((old&0xffe0u)|gain);
}

// OEM 0.29, 0x0801E510. Valid calibration is per radio; callers
// must validate the stored record before allowing RF output.
constexpr uint32_t corrected_frequency_word(uint32_t word,int16_t calibration) {
    constexpr uint32_t divisor=435500;
    const uint32_t magnitude=uint32_t(calibration<0?-int(calibration):calibration);
    const uint64_t product=uint64_t(word/100u)*magnitude;
    uint32_t remainder=uint32_t(product>>32),low=uint32_t(product),adjustment=0;
    // Base-256 division keeps each remainder below 2^32 on Cortex-M4.
    for(unsigned byte=0;byte<4;++byte) {
        remainder=(remainder<<8)|(low>>24);
        low<<=8;
        adjustment=(adjustment<<8)|(remainder/divisor);
        remainder%=divisor;
    }
    return calibration<0?word-adjustment:word+adjustment;
}

// OEM 0.29, UHF branch of 0x0801EA60. This selects a byte
// in the factory table for the requested power level, not a power value.
constexpr bool uhf_tx_frequency(uint32_t hz) {
    return hz>=400000000u && hz<=470000000u;
}

constexpr unsigned uhf_pa_index(uint32_t frequency_hz) {
    if(!uhf_tx_frequency(frequency_hz))return 0xff;
    return (frequency_hz/1000000u-400u)/10u;
}

struct UhfTxValues {
    uint16_t frequency_low=0,frequency_high=0,mic=0;
    uint8_t pa_bias=0;
};

constexpr bool vhf_tx_frequency(uint32_t hz) {
    return hz>=136000000u && hz<=174000000u && hz%10==0;
}

// Compute without touching the board. The caller supplies the verified OEM
// selector and must apply the result only in a valid, authorized TX session.
inline bool uhf_tx_values(const UhfCalibration &cal,uint32_t hz,uint8_t selector,
                          uint16_t old_mic,UhfTxValues &out) {
    const unsigned index=uhf_pa_index(hz);
    if(selector>=3 || index>=13 || hz%10 || cal.microphone>63)return false;
    const uint8_t bias=cal.pa[selector][index];
    if(!bias || bias==0xff)return false;
    const uint32_t word=corrected_frequency_word(hz/10,cal.frequency_correction);
    if(word<40000000u || word>47000000u)return false;
    out={uint16_t(word),uint16_t(word>>16),microphone_word(old_mic,2,cal.microphone),bias};
    return true;
}
}
