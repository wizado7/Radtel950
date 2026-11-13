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
void ui_boot_screen();
void ui_invalidate();
void ui_draw(Application &,const InputStatus &,uint32_t);
}
