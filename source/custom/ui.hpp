// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "application.hpp"
#include "settings_flash.hpp"
#include "settings_export.hpp"
#include "settings_import.hpp"
#include "fm_probe.hpp"
#include "voice_rx.hpp"
#include "aprs_stations.hpp"
#include "aprs_messages.hpp"
#include "power_diagnostics.hpp"
namespace rt950 {
struct InputStatus {
#ifdef RT950_BENCH_TX_HZ
    const char *bench_ptt="";
#endif
    uint8_t sampled=255,last=255,encoder_ab=0;
    uint8_t ptt_mask=0;
    uint8_t voice_tx_slot=0xff;
    bool monitor_active=false;
    const char *power_status="";
    uint8_t factory_ctcss_raw=0xff,factory_ctcss_level=0xff;
    uint32_t presses=0,cw=0,ccw=0,edges=0,invalid_edges=0,encoder_lost=0;
    SaveState save_state=SaveState::Ready;
    fm::Probe::Phase fm_phase=fm::Probe::Phase::Off;
    fm::Probe::Failure fm_failure=fm::Probe::Failure::None;
    uint16_t fm_frequency=0;
    uint8_t fm_part=0;
    bool fm_signal_ready=false,fm_audio_enabled=false,fm_audio_error=false;
    fm::Signal fm_signal{};
    voice::VoiceStatus voice{};
    struct SignalLevel {
        uint32_t frequency=0,sampled_at=0;
        uint16_t raw=0;
        unsigned bars(uint32_t hz,uint32_t now) const {
            if(!frequency || frequency!=hz || uint32_t(now-sampled_at)>=2500u || raw<=65)return 0;
            return raw>=200?8u:1u+(unsigned(raw)-66u)*7u/134u;
        }
    };
    SignalLevel slot_signal[3]{};
    struct Digits {
        char text[9]{};
        uint32_t frequency=0,last=0;
        unsigned size=0;
        const char *heard(uint32_t hz,uint32_t now) const {
            return frequency==hz && uint32_t(now-last)<6000u?text:"";
        }
        void append(char digit,uint32_t hz,uint32_t now) {
            if(!*heard(hz,now))size=0;
            if(size==8) {
                for(unsigned i=1;i<8;++i)text[i-1]=text[i];
                --size;
            }
            text[size++]=digit;text[size]=0;
            last=now;frequency=hz;
        }
    } dtmf[3]{};
    void observe_signal(unsigned slot,const voice::VoiceStatus &rx,uint32_t hz,uint32_t now) {
        if(slot<3 && rx.ready && !rx.fault && rx.sampled && rx.sampled_at==now && rx.frequency==hz)
            slot_signal[slot]={hz,now,rx.rssi};
    }
    aprs::RecentStations aprs_stations{};
    aprs::RecentMessages aprs_messages{};
    uint32_t aprs_frames=0,aprs_background_frames=0,aprs_windows=0,aprs_cancelled=0;
    uint32_t aprs_export_dropped=0;
    bool aprs_receiving=false;
    PowerDiagnostics power{};
    BackupPhase backup_phase=BackupPhase::Off;
    uint32_t backup_sent=0,backup_total=0;
    ImportPhase import_phase=ImportPhase::Off;
    uint32_t import_received=0;uint16_t import_channels=0;
};
void ui_boot_screen(const Application &);
void ui_invalidate();
bool ui_transitioning();
void ui_draw(Application &,const InputStatus &,uint32_t);
}
