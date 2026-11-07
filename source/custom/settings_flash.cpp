// SPDX-License-Identifier: GPL-3.0-or-later
#include "application.hpp"
#include "board.h"
#include "settings_flash.hpp"
#include "storage_journal.hpp"

extern "C" {
extern const uint8_t _storage_start[],_storage_end[],_storage_slot_size[];
uint32_t get_tick(void);
}

namespace rt950 {
namespace {
constexpr uintptr_t SlotStart=0x08010000u;
constexpr size_t SlotSize=8192, SectorSize=2048;
constexpr uint32_t Busy=1u, ProgramError=4u, ProtectionError=16u, Done=32u;
constexpr uint32_t Program=1u, SectorErase=2u, StartErase=64u, Lock=128u;
constexpr uint32_t Faults=ProgramError|ProtectionError;
constexpr uint32_t EraseDeadline=36000000u; // 300 ms at the configured 120 MHz
uint8_t snapshot[Application::SnapshotCapacity];
constexpr char SavedMessage[]="SETTINGS SAVED";

#define REG32(address) (*reinterpret_cast<volatile uint32_t *>(address))
#define FLASH_KEY REG32(0x40022004u)
#define FLASH_STATUS REG32(0x4002200cu)
#define FLASH_CONTROL REG32(0x40022010u)
#define FLASH_ADDRESS REG32(0x40022014u)
#define CYCLE_COUNTER REG32(0xe0001004u)

// Bank 1 cannot supply instructions while busy. No calls or flash data here.
// noipa keeps the independent address checks even if all current callers are bounded.
__attribute__((section(".ramfunc"),noipa,optimize("no-if-conversion","no-if-conversion2")))
bool flash_operation(uintptr_t address,uint32_t value,bool erase) {
    // Check the whole write/erase span before touching any controller register.
    const size_t span=erase?SectorSize:sizeof(uint32_t);
    if(address<SlotStart || address>SlotStart+2*SlotSize-span || (address&(span-1u)))
        return false;
    uint32_t primask;
    __asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask) :: "memory");
    bool ok=false;
    if(!(FLASH_STATUS&Busy) && FLASH_CONTROL==Lock) {
        FLASH_KEY=0x45670123u;FLASH_KEY=0xcdef89abu;
        if(FLASH_CONTROL==0) {
            FLASH_STATUS=Faults|Done;
            if(erase) {
                FLASH_ADDRESS=address;
                FLASH_CONTROL=SectorErase;
                FLASH_CONTROL=SectorErase|StartErase;
            } else {
                FLASH_CONTROL=Program;
                REG32(address)=value;
            }
            const uint32_t began=CYCLE_COUNTER;
            unsigned spins=4000000;
            while((FLASH_STATUS&Busy) && uint32_t(CYCLE_COUNTER-began)<EraseDeadline && --spins) {}
            const uint32_t status=FLASH_STATUS;
            if(status&Busy) {
                // Abort by reset. A flash fetch could stall until the operation ends.
                REG32(0xe000ed0cu)=0x05fa0004u;
                __asm volatile("dsb" ::: "memory");
            } else {
                FLASH_STATUS=Faults|Done;
                FLASH_CONTROL=Lock;
                ok=(status&(Faults|Done))==Done;
            }
        }
    }
    __asm volatile("msr primask, %0" :: "r"(primask) : "memory");
    return ok;
}

bool layout_confirmed() {
    return board_flash_kib()==1024 &&
        reinterpret_cast<uintptr_t>(_storage_start)==SlotStart &&
        reinterpret_cast<uintptr_t>(_storage_end)==SlotStart+2*SlotSize &&
        reinterpret_cast<uintptr_t>(_storage_slot_size)==SlotSize;
}

struct FlashSlots {
    bool shutting_down=false;
    size_t capacity() const {return SlotSize;}
    const uint8_t *view(unsigned slot) const {
        return slot<2?reinterpret_cast<const uint8_t *>(SlotStart+slot*SlotSize):nullptr;
    }
    bool erase(unsigned slot) {
        const auto *data=view(slot);
        if(!data)return false;
        for(size_t offset=0;offset<SlotSize;offset+=SectorSize) {
            if(!shutting_down && board_power_switch_off())return false;
            bool blank=true;
            for(size_t i=0;i<SectorSize;i+=4)if(storage::word(data+offset+i)!=storage::Erased){blank=false;break;}
            if(!blank && !flash_operation(reinterpret_cast<uintptr_t>(data)+offset,0,true))return false;
            custom_heartbeat=get_tick();
        }
        return true;
    }
    bool program(unsigned slot,size_t offset,const uint8_t *data,size_t size) {
        const auto *target=view(slot);
        if(!target || offset>SlotSize || size>SlotSize-offset || (offset%4) || (size%4))return false;
        for(size_t i=0;i<size;i+=4) {
            if(!shutting_down && board_power_switch_off())return false;
            if(storage::word(target+offset+i)!=storage::Erased)return false;
            const uint32_t word=storage::word(data+i);
            if(word!=storage::Erased &&
               !flash_operation(reinterpret_cast<uintptr_t>(target)+offset+i,word,false))return false;
            custom_heartbeat=get_tick();
        }
        return true;
    }
};
}

void load_settings(Application &app) {
    if(!layout_confirmed())return;
    FlashSlots flash;
    const auto first=storage::inspect(flash.view(0),SlotSize,Application::SnapshotSchema);
    const auto second=storage::inspect(flash.view(1),SlotSize,Application::SnapshotSchema);
    const auto selected=storage::newest(first,second);
    if(selected.payload && !app.restore_snapshot(selected.payload,selected.size)) {
        const auto older=selected.payload==first.payload?second:first;
        if(older.payload)app.restore_snapshot(older.payload,older.size);
    }
}

const char *save_settings(const Application &app,bool shutting_down) {
    if(!layout_confirmed())return "STORAGE UNAVAILABLE";
    if(board_battery_mv()<6800u || (!shutting_down && board_power_switch_off()))return "CHECK BATTERY";
    REG32(0xe000edfcu)|=1u<<24; // CoreDebug DEMCR.TRCENA
    REG32(0xe0001000u)|=1u;     // DWT CYCCNTENA
    FlashSlots flash{shutting_down};
    // Exclude semantically invalid records from target selection, even if
    // their outer CRC is valid. Keep the last loadable copy until commit.
    uint8_t usable=0;
    {
        Application probe(false);
        for(unsigned i=0;i<2;++i) {
            const auto record=storage::inspect(flash.view(i),SlotSize,Application::SnapshotSchema);
            if(record.payload && !Application::snapshot_format_supported(record.payload,record.size))return "STORAGE FORMAT NEWER";
            if(record.payload && probe.restore_snapshot(record.payload,record.size))usable|=uint8_t(1u<<i);
        }
    }
    const auto size=app.save_compact_snapshot(snapshot,sizeof snapshot);
    switch(storage::save(flash,Application::SnapshotSchema,snapshot,uint32_t(size),usable)) {
    case storage::SaveResult::Saved:
    case storage::SaveResult::Unchanged:return SavedMessage;
    case storage::SaveResult::Invalid:return "STORAGE UNAVAILABLE";
    case storage::SaveResult::Failed:return "STORAGE WRITE FAILED";
    }
    return "STORAGE WRITE FAILED";
}
namespace {
uint32_t fingerprint(const Application &app) {
    // Observe every current field. The legacy v1 export API intentionally
    // omits new bindings and must never be used for autosave change detection.
    const size_t size=app.save_portable_snapshot(snapshot,sizeof snapshot);
    return ~storage::crc_bytes(0xffffffffu,snapshot,size);
}
}
void SettingsAutosave::begin(const Application &app,uint32_t now) {
    saved_=observed_=fingerprint(app);
    scanned_=changed_=now;
    attempted_=false;state_=SaveState::Ready;
}
const char *SettingsAutosave::poll(const Application &app,uint32_t now,bool shutting_down,bool ptt) {
    if(!shutting_down && uint32_t(now-scanned_)<250u)return nullptr;
    scanned_=now;
    const uint32_t current=fingerprint(app);
    if(current!=observed_) {observed_=current;changed_=now;state_=SaveState::Pending;}
    if(current==saved_) {
        state_=attempted_?SaveState::Saved:SaveState::Ready;
        return nullptr;
    }
    if(ptt || board_battery_mv()<6800u){state_=SaveState::Deferred;return nullptr;}
    if(state_==SaveState::Deferred)state_=SaveState::Pending;
    if(!shutting_down && (uint32_t(now-changed_)<2000u ||
       (attempted_ && uint32_t(now-last_attempt_)<15000u)))return nullptr;
    attempted_=true;last_attempt_=now;
    const char *result=save_settings(app,shutting_down);
    if(result==SavedMessage){saved_=current;state_=SaveState::Saved;}
    else state_=SaveState::Failed;
    return result;
}

}
