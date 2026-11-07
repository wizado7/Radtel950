// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#include <stddef.h>
namespace rt950 {
// OEM 0.29 0x0800D478: polynomial 0x1021, init 0, MSB first.
// This is the control-packet checksum, NOT AX.25's reflected FCS.
inline uint16_t oem_control_crc(const uint8_t *data,size_t count) {
    uint16_t crc=0;
    for(size_t i=0;i<count;++i){crc^=uint16_t(uint16_t(data[i])<<8);for(unsigned bit=0;bit<8;++bit)crc=uint16_t(crc&0x8000u?(uint32_t(crc)<<1)^0x1021u:uint32_t(crc)<<1);}
    return crc;
}
}
