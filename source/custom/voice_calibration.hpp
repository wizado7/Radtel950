// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rt950::voice {
enum class CalibrationStatus : uint8_t { Ready, SpiUnavailable, ReadFailed, InvalidMarker, InvalidMic, InvalidTrim };

// Plausibility limit for a marked crystal trim: 871 ten-Hz units at 435.5 MHz (20 ppm).
inline constexpr int16_t MaxCrystalTrim=871;

template<size_t N>
constexpr bool valid_pa_column(const uint8_t (&pa)[3][N],unsigned index) {
    return pa[2][index]!=0 && pa[2][index]<pa[1][index] &&
           pa[1][index]<pa[0][index] && pa[0][index]!=0xff;
}

constexpr uint8_t factory_tone_level(uint8_t value) {return value&0x7fu;}

struct UhfCalibration {
    uint8_t pa[3][13]{};
    uint8_t microphone=0,ctcss_level=0xff,dcs_level=0xff;
    uint8_t ctcss_raw=0xff,dcs_raw=0xff;
    int16_t frequency_correction=0;

    uint8_t bias(unsigned index,uint8_t selector) const {
        if(index>=13 || selector>=3)return 0;
        const uint8_t high=pa[0][index],mid=pa[1][index],low=pa[2][index];
        const auto programmed=[](uint8_t code){return code && code!=0xff;};
        if(!programmed(low))return 0;
        if((programmed(mid) && mid<low) || (programmed(high) && high<low))return 0;
        // An absent higher level does not invalidate a programmed Low.
        if(selector==2)return low;
        if(!programmed(mid) || !programmed(high) || high<mid)return 0;
        return pa[selector][index];
    }
};

// OEM 0.29 0x08024424: read opcode, big-endian 24-bit address,
// then data. The transport must enforce a per-byte timeout.
template<class Spi>
bool read_factory_flash(Spi &spi,uint32_t address,uint8_t *data,size_t length) {
    if(!data || !length || length>16 || address>=0x400000u || length>0x400000u-address)
        return false;
    const uint8_t header[]={3,uint8_t(address>>16),uint8_t(address>>8),uint8_t(address)};
    uint8_t ignored=0;
    spi.select(true);
    bool ok=true;
    for(uint8_t byte:header)if(!(ok=spi.transfer(byte,ignored)))break;
    for(size_t i=0;ok && i<length;++i)ok=spi.transfer(0,data[i]);
    spi.select(false);
    return ok;
}

// OEM loader 0x080121FC. Read every row; cells are checked when TX selects
// its frequency and power. The table order is the OEM selector order.
template<class Reader>
CalibrationStatus inspect_uhf_calibration(Reader read,UhfCalibration &out) {
    uint8_t marker=0;
    if(!read(0xf0e0,&marker,1))return CalibrationStatus::ReadFailed;
    if(!marker || marker==0xff)return CalibrationStatus::InvalidMarker;
    UhfCalibration candidate;
    for(unsigned level=0;level<3;++level) {
        auto &table=candidate.pa[level];
        if(!read(0xf000u+level*0x40u,table,sizeof table))return CalibrationStatus::ReadFailed;
    }
    uint8_t settings[16]{};
    if(!read(0xf210,settings,sizeof settings))return CalibrationStatus::ReadFailed;
    if(settings[0]>63)return CalibrationStatus::InvalidMic;
    candidate.microphone=settings[0];
    candidate.ctcss_raw=settings[1];
    candidate.dcs_raw=settings[2];
    candidate.ctcss_level=factory_tone_level(settings[1]);
    candidate.dcs_level=factory_tone_level(settings[2]);
    if(settings[11]==0xa0) {
        const unsigned encoded=unsigned(settings[12])|(unsigned(settings[13])<<8);
        candidate.frequency_correction=int16_t(encoded<0x8000?int(encoded):int(encoded)-65536);
        if(candidate.frequency_correction < -MaxCrystalTrim || candidate.frequency_correction > MaxCrystalTrim)
            return CalibrationStatus::InvalidTrim;
    }
    out=candidate;
    return CalibrationStatus::Ready;
}

template<class Reader>
bool load_uhf_calibration(Reader read,UhfCalibration &out) {
    return inspect_uhf_calibration(read,out)==CalibrationStatus::Ready;
}

struct VhfPaCalibration {
    uint8_t pa[3][16]{};

    uint8_t bias(uint32_t hz,uint8_t selector) const {
        if(hz<136000000u || hz>174000000u || hz%10 || selector>=3)return 0;
        const unsigned index=(hz/1000000u-130u)/3u;
        return valid_pa_column(pa,index)?pa[selector][index]:0;
    }
};

// OEM 0.29 loader 0x080121FC. VHF pointers at 0x200000D4+116
// resolve to the second 16-byte block of each factory power level.
template<class Reader>
bool load_vhf_pa_calibration(Reader read,VhfPaCalibration &out) {
    out={};
    uint8_t marker=0;
    if(!read(0xf0e0,&marker,1) || !marker || marker==0xff)return false;
    VhfPaCalibration candidate;
    for(unsigned level=0;level<3;++level) {
        auto &table=candidate.pa[level];
        if(!read(0xf010u+level*0x40u,table,sizeof table))return false;
    }
    out=candidate;
    return true;
}

constexpr bool cb_fm_frequency(uint32_t hz) {
    return hz>=26000000u && hz<28000000u && hz%10==0;
}

// OEM 0.29 0x0801EAEE indexes the low-band PA table in 5 MHz steps.
struct CbPaCalibration {
    uint8_t pa[3][16]{};

    uint8_t bias(uint32_t hz,uint8_t selector) const {
        if(!cb_fm_frequency(hz) || selector>=3)return 0;
        const unsigned index=(hz/1000000u-15u)/5u;
        return valid_pa_column(pa,index)?pa[selector][index]:0;
    }
};

template<class Reader>
bool load_cb_pa_calibration(Reader read,CbPaCalibration &out) {
    out={};
    uint8_t marker=0;
    if(!read(0xf0e0,&marker,1) || !marker || marker==0xff)return false;
    CbPaCalibration candidate;
    for(unsigned level=0;level<3;++level)
        if(!read(0xf190u+level*0x10u,candidate.pa[level],16))return false;
    out=candidate;
    return true;
}

// Call once at startup before peripheral clients begin.
void initialize_factory_spi();
bool read_board_uhf_calibration(UhfCalibration &out,CalibrationStatus *status=nullptr);
bool read_board_vhf_calibration(VhfPaCalibration &out);
bool read_board_cb_calibration(CbPaCalibration &out);
void prepare_board_pa_bias();
void clear_board_pa_bias();
void release_board_pa_bias();
}
