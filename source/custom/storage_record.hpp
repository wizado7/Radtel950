// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rt950::storage {
// Byte layout, independent of MCU ABI: magic, schema, sequence, length,
// checksum, commit word, payload. The commit word is programmed last.
constexpr size_t HeaderSize=24, CommitOffset=20;
constexpr uint32_t Magic=0x53303952u, Commit=0x54494D43u, Erased=0xFFFFFFFFu;

inline uint32_t word(const uint8_t *p) {
    return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
}
inline void put(uint8_t *p,uint32_t value) {
    for(unsigned i=0;i<4;++i)p[i]=uint8_t(value>>(8*i));
}
inline uint32_t crc_bytes(uint32_t crc,const uint8_t *p,size_t n) {
    while(n--) {
        crc^=*p++;
        for(unsigned bit=0;bit<8;++bit)
            crc=(crc>>1)^(0xEDB88320u & (0u-(crc&1u)));
    }
    return crc;
}
inline uint32_t checksum(const uint8_t *header,const uint8_t *payload,size_t size) {
    return ~crc_bytes(crc_bytes(0xFFFFFFFFu,header,16),payload,size);
}
inline void prepare(uint8_t *header,uint32_t schema,uint32_t sequence,
                    const uint8_t *payload,uint32_t size) {
    put(header,Magic);put(header+4,schema);put(header+8,sequence);put(header+12,size);
    put(header+16,checksum(header,payload,size));put(header+CommitOffset,Erased);
}
struct Record {
    const uint8_t *payload=nullptr;
    uint32_t size=0,sequence=0;
};
inline Record inspect(const uint8_t *slot,size_t capacity,uint32_t schema) {
    if(!slot || capacity<HeaderSize || word(slot)!=Magic || word(slot+4)!=schema ||
       word(slot+CommitOffset)!=Commit)return {};
    const uint32_t size=word(slot+12);
    if(!size || size>capacity-HeaderSize || word(slot+16)!=checksum(slot,slot+HeaderSize,size))return {};
    return {slot+HeaderSize,size,word(slot+8)};
}
inline Record newest(Record first,Record second) {
    if(!first.payload)return second;
    if(!second.payload)return first;
    const uint32_t distance=second.sequence-first.sequence;
    return distance && distance<0x80000000u?second:first;
}
}
