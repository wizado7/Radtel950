// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "storage_record.hpp"

namespace rt950::storage {
enum class SaveResult : uint8_t { Saved, Unchanged, Invalid, Failed };

// Backend exposes exactly two slots. view() returns a stable mapped view.
// erase(slot) must not touch the other slot. program() only clears bits.
// The payload must reside outside both slots.
template<class Backend>
SaveResult save(Backend &backend,uint32_t schema,const uint8_t *data,uint32_t size,uint8_t usable=3) {
    const size_t capacity=backend.capacity();
    if(!data || !size || size%4 || capacity<HeaderSize || size>capacity-HeaderSize)
        return SaveResult::Invalid;

    for(unsigned i=0;i<2;++i) {
        const auto *slot=backend.view(i);
        if(!slot)return SaveResult::Invalid;
        const uint32_t stored_schema=word(slot+4);
        if(stored_schema!=schema && inspect(slot,capacity,stored_schema).payload)
            return SaveResult::Invalid;
    }
    const auto first=(usable&1u)?inspect(backend.view(0),capacity,schema):Record{};
    const auto second=(usable&2u)?inspect(backend.view(1),capacity,schema):Record{};
    const auto current=newest(first,second);
    if(current.payload && current.size==size) {
        uint32_t i=0;
        while(i<size && current.payload[i]==data[i])++i;
        if(i==size)return SaveResult::Unchanged;
    }
    const unsigned target=current.payload && current.payload==first.payload?1u:0u;
    const uint32_t sequence=current.payload?current.sequence+1u:0u;
    uint8_t header[HeaderSize];
    prepare(header,schema,sequence,data,size);
    if(!backend.erase(target) ||
       !backend.program(target,0,header,CommitOffset) ||
       !backend.program(target,HeaderSize,data,size))return SaveResult::Failed;

    const uint8_t *slot=backend.view(target);
    if(!slot || word(slot+CommitOffset)!=Erased)return SaveResult::Failed;
    for(size_t i=0;i<CommitOffset;++i)if(slot[i]!=header[i])return SaveResult::Failed;
    for(uint32_t i=0;i<size;++i)if(slot[HeaderSize+i]!=data[i])return SaveResult::Failed;

    uint8_t marker[4];
    put(marker,Commit);
    if(!backend.program(target,CommitOffset,marker,sizeof marker))return SaveResult::Failed;
    const auto written=inspect(backend.view(target),capacity,schema);
    return written.payload && written.sequence==sequence?SaveResult::Saved:SaveResult::Failed;
}
}
