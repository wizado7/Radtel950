// SPDX-License-Identifier: GPL-3.0-or-later
#include "application.hpp"
#include "fm_receiver.hpp"
#include "voice_calibration.hpp"
#include "voice_tx_values.hpp"
namespace rt950 {
namespace {
constexpr char callsign_chars[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
constexpr char name_chars[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-";
void text(char *dst, const char *src) {
    unsigned n=0; while (n<29 && src[n]) {dst[n]=src[n]; ++n;} dst[n]=0;
}
void frequency(char *dst, uint32_t hz) {
    if (!hz) {text(dst,"UNSET"); return;}
    // Fixed MHz.ddddd, precision 10 Hz, no printf/libc dependency.
    dst[0]=char('0'+hz/100000000u%10); dst[1]=char('0'+hz/10000000u%10);
    dst[2]=char('0'+hz/1000000u%10); dst[3]='.';
    uint32_t div=100000;
    for(unsigned i=4;i<9;++i,div/=10) dst[i]=char('0'+hz/div%10);
    dst[9]=0;
}
}
Application::Application(bool presets) {
 rt_defaults(&config_);
 for(unsigned i=0;i<ZoneCount;++i) {
  text(zone_names_[i],"ZONE00");zone_names_[i][4]=char('0'+(i+1)/10);zone_names_[i][5]=char('0'+(i+1)%10);
 }
 if(presets) for(unsigned i=0;i<MemoryCount;++i) {
  auto &m=memories_[i];const bool pmr=i<16;const unsigned ch=pmr?i+1:i-15;
  m.rx_hz=pmr?446006250u+i*12500u:433075000u+(i-16)*25000u;
  m.tx_hz=0;m.occupied=true;m.radio.narrow=pmr;
  text(m.name,pmr?"PMR00":"LPD00");m.name[3]=char('0'+ch/10);m.name[4]=char('0'+ch%10);
 }
 if(presets) {
  constexpr unsigned initial_channels[]={0,1,16};
  for(unsigned slot=0;slot<3;++slot) {
   const unsigned channel=initial_channels[slot];
   config_.vfo[slot].memory_channel=uint16_t(channel);
   config_.vfo[slot].rx_hz=vfo_hz_[slot]=memories_[channel].rx_hz;
   vfo_profiles_[slot]=memories_[channel].radio;
  }
 }
}
// Reset logical state only. Persistence uses the existing guarded A/B autosave;
// neither reset mode has a raw flash/calibration/bootloader erase path.
void Application::apply_defaults(bool all) {
 Application defaults(all);
 defaults.language_=defaults.language_draft_=language_;
 defaults.measured_power_=measured_power_; // Preserve the owner's measured RF points.
 if(!all) {
  for(unsigned i=0;i<MemoryCount;++i)defaults.memories_[i]=memories_[i];
  for(unsigned i=0;i<ZoneCount;++i)
   for(unsigned j=0;j<9;++j)defaults.zone_names_[i][j]=zone_names_[i][j];
  for(unsigned i=0;i<BroadcastCount;++i) {
   defaults.fm_presets_[i]=fm_presets_[i];defaults.am_presets_[i]=am_presets_[i];
  }
  for(unsigned i=0;i<2;++i)defaults.scan_excluded_[i]=scan_excluded_[i];
  defaults.fm_memory_=fm_memory_;defaults.waypoint_=waypoint_;
 }
 defaults.now_=defaults.last_activity_=now_;
 defaults.page_=Page::Settings;defaults.menu_group_=MenuGroup::System;
 *this=defaults;
 notify(all?"USER DATA RESET":"SETTINGS RESET");
}
void Application::user_key(unsigned which,bool held) {
 if(which>1 || keypad_locked_ || editing() || (which==0 && side_ptt_enabled()))return;
 const auto action=key_actions_[which*2u+unsigned(held)];
 if((page_==Page::ChannelScan && action==KeyAction::ScanChannels) ||
    (page_==Page::VfoScan && action==KeyAction::ScanVfo)) {
  key(Key::Back);return;
 }
 if((page_==Page::Weather && action==KeyAction::Weather) ||
    (page_==Page::Spectrum && action==KeyAction::Spectrum)) {
  key(Key::Back);return;
 }
 if(page_!=Page::Radio && page_!=Page::FmProbe) {
  if(which==1 && !held)key(Key::Back);
  return;
 }
 if(page_==Page::FmProbe && action!=KeyAction::Fm)return;
 switch(action) {
 case KeyAction::Monitor:
    if(held)monitor_held_=uint8_t(which);
    else {monitor_enabled_=!monitor_enabled_;notify(monitor_enabled_?"MONITOR ON":"MONITOR OFF");}
    break;
 case KeyAction::Fm:key(Key::FmRadio);break;
 case KeyAction::NextSlot:key(Key::NextSlot);break;
 case KeyAction::Mode:key(Key::Mode);break;
 case KeyAction::Gps:key(Key::Gps);break;
 case KeyAction::Aprs:key(Key::Aprs);break;
 case KeyAction::Channels:key(Key::Channels);break;
 case KeyAction::ScanChannels:key(Key::ChannelScan);break;
 case KeyAction::ScanVfo:key(Key::VfoScan);break;
 case KeyAction::Weather:key(Key::Weather);break;
 case KeyAction::Spectrum:key(Key::Spectrum);break;
 default:break;
 }
}
void Application::notify(const char *message) {notice_=message;notice_since_=now_;}
void Application::aprs_message_counts(unsigned all,unsigned personal) {
    aprs_all_count_=uint8_t(all);aprs_personal_count_=uint8_t(personal);
    const unsigned count=aprs_personal_?personal:all;
    if(aprs_message_>=count)aprs_message_=0;
}
void Application::tick(uint32_t now) {
 now_=now;
 if(screen_seconds_ && uint32_t(now-last_activity_)>=screen_seconds_*1000u)screen_awake_=false;
 if(notice_[0] && uint32_t(now-notice_since_)>=2000u){notice_="";invalid_=false;}
 // Status, receiver and diagnostic pages remain open. A timeout cancels
 // unapplied editor drafts; it never confirms an edit or a destructive action.
 bool menu_page=true;
 switch(page_) {
 case Page::Radio:case Page::Gps:case Page::Aprs:case Page::AprsMessages:
 case Page::Debug:case Page::Battery:case Page::LcdTest:case Page::FlashInfo:
 case Page::FmProbe:case Page::VoiceRx:case Page::ChannelScan:case Page::VfoScan:case Page::PowerStatus:case Page::About:case Page::Distance:case Page::Backup:case Page::Import:case Page::Weather:case Page::Spectrum:
     menu_page=false;break;
 default:break;
 }
 const uint32_t timeout=uint32_t(menu_timeouts[menu_timeout_])*1000u;
 if(menu_page && !tone_searching() && timeout && uint32_t(now-last_activity_)>=timeout) {
     page_=Page::Radio;parent_=Page::Settings;menu_group_=MenuGroup::Root;
     power_test_=false;editor_value_=0;editor_digits_=0;notify("");invalid_=false;
     // Start a fresh inactivity period after leaving an unattended editor.
     // Do not immediately lock because time was spent inside settings.
     last_activity_=now;
 }
 const uint32_t lock_timeout=uint32_t(menu_timeouts[auto_lock_])*1000u;
 if(!keypad_locked_ && lock_timeout && page_==Page::Radio && !editing() &&
    uint32_t(now-last_activity_)>=lock_timeout) {
     keypad_locked_=true;notify("KEYPAD LOCKED");
 }
}
bool Application::frequency_valid(uint32_t hz) {
#ifdef RT950_MAIN_TX
    if(rt_low_fm_rx_frequency(hz))return true;
#endif
    return (hz>=136000000u && hz<=174000000u) ||
           (hz>=400000000u && hz<=470000000u);
}
bool Application::valid_scan_bounds(uint32_t first,uint32_t last) {
    if((first && !frequency_valid(first)) || (last && !frequency_valid(last)))return false;
    return !first || !last || (first<=last && (first<30000000u)==(last<30000000u) &&
                               (first>=400000000u)==(last>=400000000u));
}
bool Application::scan_limits(uint32_t anchor,uint32_t &first,uint32_t &last) const {
    const uint32_t band=scan_start_?scan_start_:scan_end_?scan_end_:anchor;
    if(!frequency_valid(band))return false;
    first=scan_start_?scan_start_:band<30000000u?18000000u:band>=400000000u?400000000u:136000000u;
    last=scan_end_?scan_end_:band<30000000u?29999990u:band>=400000000u?470000000u:174000000u;
    return true;
}
void Application::cancel_entry() {entry_khz_=0; digits_=0; invalid_=false;notify("");}
bool Application::set_frequency(uint32_t hz) {
    if(!frequency_valid(hz)) return false;
    const unsigned slot=config_.active;
    const uint32_t previous=vfo_hz_[slot];
    if(!vfo_tx_hz_[slot] || vfo_tx_hz_[slot]==previous ||
       (hz<30000000u)!=(previous<30000000u))vfo_tx_hz_[slot]=hz;
    if(rt_low_fm_rx_frequency(hz) && !voice::cb_fm_frequency(hz))vfo_tx_hz_[slot]=0;
    vfo_hz_[slot]=hz;
    config_.vfo[slot].channel_mode=false;
    sync_frequencies();
    return true;
}
bool Application::set_aprs_frequency(uint32_t hz) {
    if(hz<30000000u || !frequency_valid(hz)) return false;
    config_.aprs_hz=hz; return true;
}
bool Application::store(unsigned index) {
    if(index>=MemoryCount) return false;
    const uint32_t rx=base_rx(config_.active),tx=base_tx(config_.active);
    if(!frequency_valid(rx)||(tx && !frequency_valid(tx))) return false;
    const RadioProfile profile=active_radio_profile();
    auto &m=memories_[index];
    if(!m.occupied) {
        scan_excluded_[index/32]&=~(1u<<(index%32));
        text(m.name,"USER00");m.name[4]=char('0'+(index+1)/10);m.name[5]=char('0'+(index+1)%10);
    }
    m.rx_hz=rx;m.tx_hz=tx;m.occupied=true;m.radio=profile;m.zone=uint8_t(active_zone_);
    reconcile_channels();
    return true;
}
bool Application::recall(unsigned index) {
    if(!channel_visible(index)) return false;
    auto &v=config_.vfo[config_.active];
    v.memory_channel=uint16_t(index);v.channel_mode=true;sync_frequencies();return true;
}
bool Application::scan_eligible(unsigned i) const {
    return channel_visible(i) && !(scan_excluded_[i/32]&(1u<<(i%32)));
}
bool Application::channel_visible(unsigned i) const {
    return i<MemoryCount && memories_[i].occupied && memories_[i].zone==active_zone_;
}
void Application::reconcile_channels() {
    for(auto &v:config_.vfo)if(v.channel_mode && !channel_visible(v.memory_channel))v.channel_mode=false;
    sync_frequencies();
}
void Application::select_channel_for_list() {
    channel_=config_.vfo[config_.active].memory_channel;
    if(channel_visible(channel_))return;
    channel_=0;
    for(unsigned i=0;i<MemoryCount;++i)if(channel_visible(i)){channel_=i;break;}
}
void Application::open_name_editor(const char *name) {
    for(char &c:name_draft_)c=0;
    text(name_draft_,name);name_cursor_=0;page_=Page::EditName;notify("");
}
uint32_t Application::base_rx(unsigned slot) const {
    const auto &v=config_.vfo[slot];
    return v.channel_mode?memories_[v.memory_channel].rx_hz:vfo_hz_[slot];
}
uint32_t Application::base_tx(unsigned slot) const {
    const auto &v=config_.vfo[slot];
    return v.channel_mode?memories_[v.memory_channel].tx_hz:vfo_tx_hz_[slot];
}
uint32_t Application::profile_tx_frequency() const {return base_tx(config_.active);}
bool Application::reverse_active(unsigned slot) const {
    return config_.vfo[slot].rx_hz!=base_rx(slot);
}
uint32_t Application::candidate_tx(unsigned slot) const {
    const auto &v=config_.vfo[slot];
    const auto &profile=v.channel_mode?memories_[v.memory_channel].radio:vfo_profiles_[slot];
    const uint32_t rx=base_rx(slot);
    uint32_t tx=base_tx(slot);
    if(tx && profile.shift!=Shift::None) {
        if(!profile.offset_hz)return 0;
        tx=profile.shift==Shift::Plus?rx+profile.offset_hz:rx-profile.offset_hz;
    }
    if(rx<30000000u || tx<30000000u)
        return tx==rx && profile.shift==Shift::None && voice::cb_fm_frequency(rx)?tx:0;
    return frequency_valid(rx) && frequency_valid(tx)?tx:0;
}
const char *Application::tx_status() const {
    const unsigned slot=config_.active;
    const auto &profile=active_radio_profile();
    if(profile.tx_inhibit)return "TX INHIBITED";
    if(rt_low_fm_rx_frequency(base_rx(slot)) && !voice::cb_fm_frequency(base_rx(slot)))return "RX ONLY";
    if(!base_tx(slot))return "SET TX FREQUENCY";
    if(profile.shift!=Shift::None && !profile.offset_hz)return "SET SHIFT AMOUNT";
    if(!candidate_tx(slot))return "TX OUT OF BAND";
    return "PROFILE READY";
}
void Application::sync_frequencies() {
    for(unsigned slot=0;slot<3;++slot) {
        auto &v=config_.vfo[slot];
        const auto &profile=v.channel_mode?memories_[v.memory_channel].radio:vfo_profiles_[slot];
        const uint32_t rx=base_rx(slot),tx=candidate_tx(slot);
        v.rx_hz=profile.reverse && tx?tx:rx;
        v.tx_hz=profile.tx_inhibit || !tx?0:profile.reverse?rx:tx;
    }
}
const RadioProfile &Application::active_radio_profile() const {
    return radio_profile(config_.active);
}
const RadioProfile &Application::radio_profile(unsigned slot) const {
    const auto &v=config_.vfo[slot];
    return v.channel_mode?memories_[v.memory_channel].radio:vfo_profiles_[slot];
}
RadioProfile &Application::editable_radio_profile() {
    const auto &v=config_.vfo[config_.active];
    return v.channel_mode?memories_[v.memory_channel].radio:vfo_profiles_[config_.active];
}
bool Application::tone_search_available() const {
#ifdef RT950_MAIN_TX
    const auto hz=config_.vfo[config_.active].rx_hz;
    return (radio_setting_==RadioSetting::RxCtcss || radio_setting_==RadioSetting::RxDcs) &&
        (voice::vhf_tx_frequency(hz) || voice::uhf_tx_frequency(hz)) && hz%10==0;
#else
    return false;
#endif
}
void Application::poll_tone_search(uint32_t now,bool sampled,bool matched,bool fault) {
    if(!tone_searching())return;
    const auto result=tone_search_.poll(now,sampled,matched,fault);
    if(result==ToneSearch::Result::Pending)return;
    if(result==ToneSearch::Result::Found)radio_draft_=tone_candidate().index;
    tone_search_.cancel();
    activity(now);
    notify(result==ToneSearch::Result::Found?"TONE FOUND / OK SAVE":
        result==ToneSearch::Result::Exhausted?"NO TONE FOUND":"TONE SCAN UNAVAILABLE");
}
void Application::tone_search_ptt() {
    if(!tone_searching())return;
    tone_search_.cancel();
    page_=Page::Radio;
    notify("");
}
unsigned Application::watch_slots_selection() const {
    if(config_.watch_mode==RT_WATCH_SINGLE)return 0;
    if(config_.watch_mode==RT_WATCH_ABC)return 1;
    for(unsigned i=2;i<sizeof watch_masks/sizeof watch_masks[0];++i)
        if(watch_masks[i]==config_.watch_mask)return i;
    return 1;
}
bool Application::enabled(Option o) const {
    switch(o) {
    case Option::SidePtt:return side_ptt_enabled();
    case Option::SecondPtt:return second_ptt_enabled();
    case Option::Watch:return config_.watch_mode!=RT_WATCH_SINGLE;
    case Option::Gps:return config_.gps_on;
    case Option::AprsRx:return config_.aprs_rx;
    case Option::Tail:return tx_tail_;
    case Option::Beacon:return config_.beacon_on;
    case Option::Ble:return config_.ble_on;
    case Option::Export:return config_.ble_export;
    default:return false;
    }
}
bool Application::set_option(Option o,bool on) {
    if(on && o==Option::Beacon &&
       !frequency_valid(config_.aprs_hz)) return false;
    if(on && o==Option::Beacon && (!beacon_identity_.callsign[0] ||
       !(beacon_saved_point_?waypoint_.valid:config_.gps_on)))return false;
    if(on && o==Option::Export && !config_.ble_on) return false;
    switch(o) {
    case Option::SidePtt:aux_ptt_=uint8_t((aux_ptt_&2u)|(on?1u:0u));break;
    case Option::SecondPtt:aux_ptt_=uint8_t((aux_ptt_&1u)|(on?2u:0u));break;
    case Option::Watch:config_.watch_mode=!on?RT_WATCH_SINGLE:config_.watch_mask==7?RT_WATCH_ABC:RT_WATCH_MASK;break;
    case Option::Gps:config_.gps_on=on;if(!on && !beacon_saved_point_)config_.beacon_on=false;break;
    case Option::AprsRx:config_.aprs_rx=on;break;
    case Option::Tail:tx_tail_=on;break;
    case Option::Beacon:config_.beacon_on=on;break;
    case Option::Ble:config_.ble_on=on;if(!on)config_.ble_export=false;break;
    case Option::Export:config_.ble_export=on;break;
    default:return false;
    }
    return true;
}
void Application::fm_seek_result(uint16_t found) {
    if(found && fm::in_band(found,broadcast_band_)) {
        (sw_am()?sw_request_:fm_request_)=found;
        notify("");
    } else notify(sw_am()?"AM no station found":"FM no station found");
    fm_seek_=false;
}
void Application::observe_voice_slot(uint8_t slot,uint32_t hz) {
    if(page_!=Page::Radio || slot>=3 || !hz || config_.vfo[slot].rx_hz!=hz)return;
    last_voice_slot_=slot;last_voice_hz_=hz;last_voice_at_=now_;
}
int Application::recent_voice_slot() const {
    if(last_voice_slot_>=3 || uint32_t(now_-last_voice_at_)>=5000u ||
       config_.vfo[last_voice_slot_].rx_hz!=last_voice_hz_)return -1;
    return last_voice_slot_;
}
void Application::aprs_distance_targets(const GeoPoint *targets,unsigned count) {
    count=count>AprsDistanceCapacity?AprsDistanceCapacity:count;
    for(unsigned i=0;i<count;++i)aprs_distance_targets_[i]=targets[i];
    aprs_distance_count_=uint8_t(count);
    if(aprs_distance_cursor_>=count)aprs_distance_cursor_=0;
}
void Application::change_waypoint(GeoPoint point) {
    waypoint_=point;
    if(beacon_saved_point_){beacon_saved_point_=false;config_.beacon_on=false;}
}
void Application::write_waypoint_digit(unsigned digit) {
    static constexpr uint32_t powers[]={1,10,100,1000,10000,100000,1000000,10000000,100000000};
    const unsigned row=waypoint_edit_cursor_>=WaypointLatitudeDigits;
    const unsigned end=row?WaypointDigits:WaypointLatitudeDigits;
    const auto scale=powers[end-1u-waypoint_edit_cursor_];
    auto &value=waypoint_edit_value_[row];
    value=value-value/scale%10u*scale+digit*scale;
    if(waypoint_edit_cursor_+1u<WaypointDigits)++waypoint_edit_cursor_;
    notify("");
}
void Application::digit(unsigned d) {
    last_activity_=now_;
    if(keypad_locked_){notify("HOLD # TO UNLOCK");return;}
    if(d>9)return;
    if(page_==Page::EditWaypoint) {write_waypoint_digit(d);return;}
    if(page_==Page::Radio && config_.vfo[config_.active].channel_mode)key(Key::Channels);
    if(page_==Page::Channels || page_==Page::StoreChannel) {
        if(editor_digits_==2){editor_digits_=0;editor_value_=0;}
        editor_value_=editor_value_*10u+d;++editor_digits_;notify("");return;
    }
    if(page_==Page::FmPresets || page_==Page::AmPresets) {
        if(!broadcast_confirm() && d>=1 && d<=BroadcastCount)broadcast_selection_=uint8_t(d-1);
        return;
    }
    if(page_==Page::BandProfile) {
        if(d>9)return;
        if(editor_digits_>=2){notify("TOO MANY DIGITS");return;}
        editor_value_=editor_value_*10u+d;++editor_digits_;notify("");return;
    }
    if(page_==Page::PowerCalibration || page_==Page::PowerMeasured) {
        power_test_=false;
        if(d>9)return;
        if(editor_digits_>=3){notify("TOO MANY DIGITS");return;}
        editor_value_=editor_value_*10u+d;++editor_digits_;notify("");return;
    }
    if(page_==Page::FmProbe && fm_audio_) {
        if(d<=9 && fm_digits_<5){fm_seek_=false;fm_entry_[fm_digits_++]=char('0'+d);fm_entry_[fm_digits_]=0;}
        return;
    }
    if(page_==Page::EditName) {
        if(d<=9)name_draft_[name_cursor_]=char('0'+d);
        return;
    }
    if(page_==Page::BeaconIdentity) {
        if(d<=9 && identity_cursor_<6)identity_draft_.callsign[identity_cursor_]=char('0'+d);
        return;
    }
    if(page_==Page::ScreenTimer || page_==Page::BeaconInterval || page_==Page::AprsScanInterval || watch_timer_editor() || frequency_editor()) {
        if(d>9)return;
        const unsigned limit=frequency_editor()?8:page_==Page::ScreenTimer?4:5;
        if(editor_digits_>=limit){notify("TOO MANY DIGITS");return;}
        editor_value_=editor_value_*10u+d;++editor_digits_;notify("");return;
    }
    if(page_!=Page::Radio || config_.vfo[config_.active].channel_mode || d>9) return;
    if(digits_>=8) {notify("ENTRY FULL: OK OR DELETE");return;}
    entry_khz_=entry_khz_*10u+d;++digits_;invalid_=false;notify("");
}
void Application::step(int direction) {
    auto &v=config_.vfo[config_.active];
    if(v.channel_mode) {
        for(unsigned n=1;n<=MemoryCount;++n) {
            unsigned idx=unsigned((int(v.memory_channel)+direction*int(n)+int(MemoryCount))%int(MemoryCount));
            if(channel_visible(idx)) {recall(idx);return;}
        }
    } else if(base_rx(config_.active)) {
        const uint32_t step_hz=vfo_steps[applied_vfo_step()];
        const unsigned slot=config_.active;
        const uint32_t previous=vfo_hz_[slot];
        const uint32_t hz=direction>0?previous+step_hz:previous-step_hz;
        if(!frequency_valid(hz))notify("BAND LIMIT");
        else {
            vfo_hz_[slot]=hz;
            if(rt_low_fm_rx_frequency(hz) && !voice::cb_fm_frequency(hz))vfo_tx_hz_[slot]=0;
            else if(vfo_tx_hz_[slot]==previous)vfo_tx_hz_[slot]=hz;
            sync_frequencies();
            notify("");
        }
    }
}
namespace {
constexpr MenuEntry root_menu[]={
    {"Radio",Key::Menu,Option::Count,MenuGroup::Radio},
    {"Channels & zones",Key::Menu,Option::Count,MenuGroup::Channels},
    {"GPS",Key::Menu,Option::Count,MenuGroup::Gps},
    {"APRS",Key::Menu,Option::Count,MenuGroup::Aprs},
    {"Bluetooth",Key::Menu,Option::Count,MenuGroup::Bluetooth},
    {"Display & power",Key::Menu,Option::Count,MenuGroup::Display},
    {"System",Key::Menu,Option::Count,MenuGroup::System},
    {"Diagnostics",Key::Menu,Option::Count,MenuGroup::Diagnostics},
};
constexpr MenuEntry radio_menu[]={
    {"Watch A/B/C",Key::Menu,Option::Watch},{"VFO step",Key::VfoStep},
    {"Squelch (SQL)",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::Squelch},
    {"FM bandwidth",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::Bandwidth},
    {"TX power",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::Power},
    {"Busy lockout",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::BusyLock},
    {"TX timeout (TOT)",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::Timeout},
    {"TX tail suppression",Key::Menu,Option::Tail},
    {"RX CTCSS",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::RxCtcss},
    {"TX CTCSS",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::TxCtcss},
    {"RX DCS",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::RxDcs},
    {"TX DCS",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::TxDcs},
    {"FM Radio",Key::FmRadio},
#ifdef RT950_MAIN_TX
    {"Shortwave AM",Key::SwAm},
    {"Side key PTT C",Key::Menu,Option::SidePtt},
    {"Second PTT B",Key::Menu,Option::SecondPtt},
#endif
    {"Watch slots",Key::WatchSlots},{"Watch slot time",Key::WatchDwell},{"Watch resume delay",Key::WatchHold},
    {"LPD / PMR profiles (RU)",Key::BandProfile},
    {"FM presets",Key::FmPresets},{"AM presets",Key::AmPresets},
    {"FM tuning step",Key::FmStep},{"AM tuning step",Key::AmStep},
    {"Mic Gain",Key::MicGain},
#ifdef RT950_MAIN_TX
    {"Longwave AM",Key::LongwaveAm},{"Mediumwave AM",Key::MediumwaveAm},
#endif
    {"NOAA",Key::Weather},{"NOAA Alert",Key::WeatherAlert},
    {"Monitor",Key::Monitor},
    {"Repeater",Key::Menu,Option::Count,MenuGroup::Repeater}
};
constexpr MenuEntry repeater_menu[]={
    {"TX frequency",Key::TxFrequency},
    {"Shift direction",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::Shift},
    {"Repeater offset",Key::RepeaterOffset},
    {"Reverse",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::Reverse},
    {"TX inhibit",Key::RadioOptions,Option::Count,MenuGroup::Root,RadioSetting::TxInhibit}
};
constexpr MenuEntry channel_menu[]={
    {"Channels",Key::Channels},{"Save channel",Key::Save},
    {"Rename channel",Key::RenameChannel},{"Delete channel",Key::DeleteChannel},
    {"Select zone",Key::SelectZone},{"Rename zone",Key::RenameZone},{"Move channel",Key::MoveChannel},
#ifdef RT950_MAIN_TX
    {"Scan zone channels",Key::ChannelScan},{"Scan mode",Key::ScanMode},{"Scan VFO",Key::VfoScan},
    {"VFO scan lower limit",Key::ScanStart},{"VFO scan upper limit",Key::ScanEnd},
    {"RX spectrum",Key::Spectrum},
#endif
};
constexpr MenuEntry gps_menu[]={{"GPS status",Key::Gps},{"GPS receiver",Key::Menu,Option::Gps},{"Time zone",Key::Timezone},{"Coordinate format",Key::CoordinateFormat},{"Speed units",Key::SpeedUnits},{"Altitude units",Key::AltitudeUnits},{"Distance units",Key::DistanceUnits},{"Saved GPS point",Key::Waypoint},{"Distance to point",Key::Distance}};
constexpr MenuEntry aprs_menu[]={
    {"APRS status",Key::Aprs},
    {"APRS messages",Key::AprsMessages},
    {"RX Setup",Key::Menu,Option::Count,MenuGroup::AprsRx},
    {"Beacon Setup",Key::Menu,Option::Count,MenuGroup::Beacon},
    {"APRS receive",Key::Menu,Option::AprsRx},
    {"APRS RF beacon",Key::Menu,Option::Beacon},
    {"APRS phone export",Key::Menu,Option::Export}
};
constexpr MenuEntry aprs_rx_menu[]={{"APRS receive",Key::Menu,Option::AprsRx},{"RX frequency",Key::AprsRxFrequency},{"Background scan interval",Key::AprsScanInterval},{"Clear APRS list",Key::ClearAprs}};
constexpr MenuEntry beacon_menu[]={{"APRS beacon",Key::Menu,Option::Beacon},{"Beacon interval",Key::BeaconInterval},{"Beacon frequency",Key::BeaconFrequency},{"Callsign / SSID",Key::BeaconIdentity},{"Beacon symbol",Key::BeaconSymbol},{"Beacon position source",Key::BeaconSource}};
constexpr MenuEntry ble_menu[]={{"Bluetooth",Key::Menu,Option::Ble},{"BLE export",Key::Menu,Option::Export}};
constexpr MenuEntry display_menu[]={{"LCD brightness",Key::Brightness},{"Screen timeout",Key::ScreenTimer},{"Battery status",Key::Battery}};
constexpr MenuEntry system_menu[]={{"CPU power save",Key::Mode},{"Language",Key::LanguageSelect},{"Menu timeout",Key::MenuTimeout},{"About firmware",Key::About},{"Export settings",Key::Backup},{"Auto keypad lock",Key::AutoLock},{"Reset settings",Key::ResetSettings},{"Reset all user data",Key::ResetAll},{"Import settings",Key::Import},{"User keys",Key::Menu,Option::Count,MenuGroup::UserKeys},{"Lock keypad",Key::KeypadLock}};
constexpr MenuEntry diagnostics_menu[]={{"Keys / encoder",Key::Debug},{"LCD field test",Key::LcdTest},{"Flash info",Key::FlashInfo},{"FM tuner test",Key::FmProbe},{"RX / Watch monitor",Key::VoiceRx},{"TX power calibration",Key::PowerCalibration},{"Power / TX status",Key::PowerStatus}};

constexpr MenuEntry user_keys_menu[]={{"PF1 short",Key::Pf1Short},{"PF1 hold",Key::Pf1Hold},{"PF2 short",Key::Pf2Short},{"PF2 hold",Key::Pf2Hold}};
struct MenuList {const MenuEntry *entries;unsigned count;};
template<unsigned N>
constexpr MenuList list(const MenuEntry (&entries)[N]) {return {entries,N};}
constexpr MenuList menus[]={
    list(root_menu),list(radio_menu),list(channel_menu),list(gps_menu),list(aprs_menu),
    list(ble_menu),list(display_menu),list(system_menu),list(diagnostics_menu),list(aprs_rx_menu),list(beacon_menu),list(repeater_menu),list(user_keys_menu)
};
static_assert(sizeof(menus)/sizeof(menus[0])==unsigned(MenuGroup::Count));
}
unsigned Application::menu_count() const {return menus[unsigned(menu_group_)].count;}
const MenuEntry &Application::menu_entry(unsigned index) const {return menus[unsigned(menu_group_)].entries[index];}

void Application::key(Key k) {
    last_activity_=now_;
    if(k==Key::KeypadLock) {
        if(keypad_locked_) {keypad_locked_=false;notify("KEYPAD UNLOCKED");return;}
        if((page_==Page::Radio && !editing()) ||
           (page_==Page::Settings && menu_group_==MenuGroup::System)) {
            page_=Page::Radio;keypad_locked_=true;notify("KEYPAD LOCKED");
        }
        return;
    }
    if(keypad_locked_){notify("HOLD # TO UNLOCK");return;}
    if(tone_searching()) {
        if(k==Key::Mode || k==Key::Back || k==Key::Menu) {
            tone_search_.cancel();notify("");
        }
        return;
    }
    if(k>=Key::Pf1Short && k<=Key::Pf2Hold) {
        binding_selection_=uint8_t(unsigned(k)-unsigned(Key::Pf1Short));
        binding_draft_=uint8_t(key_actions_[binding_selection_]);page_=Page::KeyBinding;notify("");return;
    }
    if(page_==Page::KeyBinding) {
        if(k==Key::Up || k==Key::Down)binding_draft_=uint8_t((binding_draft_+(k==Key::Down?1u:unsigned(KeyAction::Count)-1u))%unsigned(KeyAction::Count));
        if(k==Key::Back || k==Key::Menu){page_=Page::Settings;notify("");}
        if(k==Key::Ok){key_actions_[binding_selection_]=KeyAction(binding_draft_);page_=Page::Settings;notify("SETTING APPLIED");}
        return;
    }
    if(page_==Page::Weather) {
        if(k==Key::Back || k==Key::Menu)page_=parent_;
        if(k==Key::Up || k==Key::Down || k==Key::Left || k==Key::Right)
            weather_.step(k==Key::Up || k==Key::Right,now_);
        if(k==Key::Ok)weather_.confirm(now_);
        return;
    }
    if(k==Key::Weather || k==Key::WeatherAlert) {
        if((page_==Page::Radio && !editing()) || page_==Page::Settings) {
            parent_=page_;page_=Page::Weather;weather_.begin(k==Key::WeatherAlert,now_);notify("");
        }
        return;
    }
    if(k==Key::Import){page_=Page::Import;import_ready_=import_confirm_=import_request_=import_apply_=false;notify("");return;}
    if(page_==Page::Import) {
        if(k==Key::Back || k==Key::Menu || (k==Key::Ok && import_ready_ && !import_confirm_)) {
            page_=Page::Settings;import_request_=import_apply_=import_confirm_=import_ready_=false;notify("");return;
        }
        if(import_ready_ && (k==Key::Up || k==Key::Down))import_confirm_=!import_confirm_;
        if(k==Key::Ok) {if(import_ready_)import_apply_=true;else import_request_=true;}
        return;
    }
    if(k==Key::ResetSettings || k==Key::ResetAll) {
        page_=Page::Reset;reset_all_=k==Key::ResetAll;reset_confirm_=false;notify("");return;
    }
    if(page_==Page::Reset) {
        if(k==Key::Up || k==Key::Down)reset_confirm_=!reset_confirm_;
        if(k==Key::Back || k==Key::Menu || (k==Key::Ok && !reset_confirm_)) {
            page_=Page::Settings;reset_confirm_=false;notify("");return;
        }
        if(k==Key::Ok && reset_confirm_)apply_defaults(reset_all_);
        return;
    }
    if(k==Key::Backup){page_=Page::Backup;backup_request_=false;notify("");return;}
    if(page_==Page::Backup) {
        if(k==Key::Ok){backup_request_=true;notify("");}
        if(k==Key::Back || k==Key::Menu){backup_request_=false;page_=Page::Settings;notify("");}
        return;
    }
    if(k==Key::Timezone) {
        parent_=page_==Page::Settings?Page::Settings:Page::Radio;
        page_=Page::Timezone;timezone_draft_=timezone_minutes_;notify("");
        return;
    }
    if(page_==Page::Timezone) {
        int delta=0;
        switch(k) {
        case Key::Up:delta=15;break;
        case Key::Down:delta=-15;break;
        case Key::Right:delta=30;break;
        case Key::Left:delta=-30;break;
        case Key::NextSlot:delta=60;break;
        case Key::Ok:
            timezone_minutes_=timezone_draft_;page_=parent_;notify("SETTING APPLIED");return;
        case Key::Back:case Key::Menu:
            page_=parent_;notify("");return;
        default:return;
        }
        const int next=timezone_draft_+delta;
        if(next>=-720 && next<=840)timezone_draft_=next;
        return;
    }
    if(k==Key::DistanceUnits) {
        page_=Page::DistanceUnits;distance_draft_=distance_unit_;notify("");return;
    }
    if(page_==Page::DistanceUnits) {
        if(k==Key::Up || k==Key::Down)
            distance_draft_=DistanceUnit((unsigned(distance_draft_)+(k==Key::Down?1u:2u))%3u);
        if(k==Key::Back || k==Key::Menu){page_=Page::Settings;notify("");}
        if(k==Key::Ok){distance_unit_=distance_draft_;page_=Page::Settings;notify("SETTING APPLIED");}
        return;
    }
    if(page_==Page::EditWaypoint) {
        const unsigned row=waypoint_edit_cursor_>=WaypointLatitudeDigits;
        switch(k) {
        case Key::Left:if(waypoint_edit_cursor_)--waypoint_edit_cursor_;break;
        case Key::Right:if(waypoint_edit_cursor_+1u<WaypointDigits)++waypoint_edit_cursor_;break;
        case Key::Up:waypoint_edit_cursor_=0;break;
        case Key::Down:waypoint_edit_cursor_=WaypointLatitudeDigits;break;
        case Key::NextSlot:waypoint_edit_negative_[row]=!waypoint_edit_negative_[row];break;
        case Key::Erase:waypoint_edit_value_[row]=0;break;
        case Key::Back:case Key::Menu:page_=Page::Waypoint;break;
        case Key::Ok: {
            const GeoPoint point{int32_t(waypoint_edit_value_[0])*(waypoint_edit_negative_[0]?-1:1),
                int32_t(waypoint_edit_value_[1])*(waypoint_edit_negative_[1]?-1:1),true};
            if(!geo_valid(point)){notify("COORDINATES OUT OF RANGE");return;}
            change_waypoint(point);page_=Page::Waypoint;notify("GPS POINT SAVED");return;
        }
        default:return;
        }
        notify("");return;
    }
    if(k==Key::Waypoint) {
        page_=Page::Waypoint;waypoint_clear_=false;waypoint_confirm_=false;notify("");return;
    }
    if(page_==Page::Waypoint) {
        if(k==Key::Back || k==Key::Menu) {
            if(waypoint_confirm_)waypoint_confirm_=false;else page_=Page::Settings;
            notify("");return;
        }
        if(!waypoint_confirm_ && (k==Key::Up || k==Key::Down))waypoint_clear_=!waypoint_clear_;
        if(k==Key::Right && !waypoint_confirm_) {
            const int32_t coordinates[]={waypoint_.latitude,waypoint_.longitude};
            for(unsigned i=0;i<2;++i) {
                waypoint_edit_negative_[i]=coordinates[i]<0;
                waypoint_edit_value_[i]=uint32_t(coordinates[i]<0?-coordinates[i]:coordinates[i]);
            }
            waypoint_edit_cursor_=0;page_=Page::EditWaypoint;notify("");return;
        }
        if(k==Key::Ok) {
            if(waypoint_clear_ && !waypoint_.valid){notify("NO SAVED GPS POINT");return;}
            GeoPoint current{};
            if(!waypoint_clear_ && (!config_.gps_on || !gps_current_point(now_,current))) {
                waypoint_confirm_=false;notify("FRESH GPS FIX REQUIRED");return;
            }
            if(!waypoint_confirm_){waypoint_confirm_=true;notify("");return;}
            change_waypoint(waypoint_clear_?GeoPoint{}:current);waypoint_confirm_=false;
            notify(waypoint_clear_?"GPS POINT CLEARED":"GPS POINT SAVED");
        }
        return;
    }
    if(k==Key::Distance){aprs_distance_=false;page_=Page::Distance;notify("");return;}
    if(page_==Page::Distance) {
        if(k==Key::Back || k==Key::Menu) {
            page_=aprs_distance_?Page::Aprs:Page::Settings;
            if(aprs_distance_)aprs_distance_targets(nullptr,0);
            notify("");
        }
        return;
    }
    if(page_==Page::Aprs && aprs_distance_count_) {
        if(k==Key::Up && aprs_distance_cursor_)--aprs_distance_cursor_;
        if(k==Key::Down && aprs_distance_cursor_+1u<aprs_distance_count_)++aprs_distance_cursor_;
        if(k==Key::Ok) {
            aprs_distance_target_=aprs_distance_targets_[aprs_distance_cursor_];
            aprs_distance_=true;page_=Page::Distance;notify("");return;
        }
    }
    if(k==Key::FmStep || k==Key::AmStep) {
        parent_=Page::Settings;page_=k==Key::AmStep?Page::AmStep:Page::FmStep;
        tuner_step_draft_=uint8_t(tuner_step_selection(k==Key::AmStep));notify("");return;
    }
    if(page_==Page::FmStep || page_==Page::AmStep) {
        const bool am=page_==Page::AmStep;const unsigned count=am?4u:3u;
        if(k==Key::Up || k==Key::Down)
            tuner_step_draft_=uint8_t((tuner_step_draft_+(k==Key::Down?1u:count-1u))%count);
        if(k==Key::Back || k==Key::Menu){page_=parent_;notify("");}
        if(k==Key::Ok){(am?am_step_:fm_step_)=tuner_step_draft_;page_=parent_;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::About) {page_=Page::About;notify("");return;}
    if(page_==Page::About) {
        if(k==Key::Back || k==Key::Menu){page_=Page::Settings;notify("");}
        return;
    }
    if(k==Key::FmPresets || k==Key::AmPresets) {
        broadcast_parent_=page_==Page::FmProbe?Page::FmProbe:Page::Settings;
        if(broadcast_parent_==Page::Settings)parent_=Page::Settings;
        page_=k==Key::AmPresets?Page::AmPresets:Page::FmPresets;
        broadcast_selection_=0;broadcast_action_=0;notify("");return;
    }
    if(page_==Page::FmPresets || page_==Page::AmPresets) {
        const bool am=broadcast_am();
        auto &selected=(am?am_presets_:fm_presets_)[broadcast_selection_];
        if(broadcast_confirm()) {
            if(k==Key::Back || k==Key::Menu){broadcast_action_=0;notify("");return;}
            if(k==Key::Ok) {
                selected=broadcast_deleting()?0:broadcast_source();
                if(!am && broadcast_selection_==0)fm_memory_=selected;
                broadcast_action_=0;notify(selected?"PRESET SAVED":"PRESET DELETED");
            }
            return;
        }
        if(k==Key::Back || k==Key::Menu){page_=broadcast_parent_;notify("");return;}
        if(k==Key::Up || k==Key::Down)
            broadcast_selection_=uint8_t((broadcast_selection_+(k==Key::Down?1u:BroadcastCount-1u))%BroadcastCount);
        if(k==Key::Left){broadcast_action_=1;notify("");}
        if(k==Key::Erase){if(selected){broadcast_action_=2;notify("");}else notify("PRESET EMPTY");}
        if(k==Key::Ok) {
            if(!selected){notify("PRESET EMPTY");return;}
            broadcast_band_=am?fm::am_band(selected):fm::Band::Fm;
            if(am)sw_request_=selected;else fm_request_=selected;
            fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;fm_audio_=true;
            page_=Page::FmProbe;notify("");
        }
        return;
    }
    if(k==Key::AutoLock) {
        parent_=Page::Settings;page_=Page::AutoLock;auto_lock_draft_=auto_lock_;notify("");return;
    }
    if(page_==Page::AutoLock) {
        constexpr unsigned count=sizeof menu_timeouts/sizeof menu_timeouts[0];
        if(k==Key::Up || k==Key::Down)
            auto_lock_draft_=uint8_t((auto_lock_draft_+(k==Key::Down?1u:count-1u))%count);
        if(k==Key::Back || k==Key::Menu){page_=parent_;notify("");}
        if(k==Key::Ok){auto_lock_=auto_lock_draft_;page_=parent_;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::Monitor) {
        parent_=Page::Settings;page_=Page::Monitor;monitor_draft_=monitor_enabled_;notify("");return;
    }
    if(page_==Page::Monitor) {
        if(k==Key::Up || k==Key::Down)monitor_draft_=!monitor_draft_;
        if(k==Key::Back || k==Key::Menu){page_=parent_;notify("");return;}
        if(k==Key::Ok){monitor_enabled_=monitor_draft_;page_=Page::Radio;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::MicGain) {
        page_=Page::MicGain;parent_=Page::Settings;mic_gain_draft_=mic_gain_;notify("");return;
    }
    if(page_==Page::MicGain) {
        if(k==Key::Up || k==Key::Down)
            mic_gain_draft_=MicGain((unsigned(mic_gain_draft_)+(k==Key::Down?1u:2u))%3u);
        if(k==Key::Back || k==Key::Menu){page_=Page::Settings;notify("");return;}
        if(k==Key::Ok){mic_gain_=mic_gain_draft_;page_=Page::Settings;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::PowerStatus) {
        power_test_=false;page_=Page::PowerStatus;parent_=Page::Settings;
        power_status_page_=0;notify("");return;
    }
    if(page_==Page::PowerStatus) {
        if(k==Key::Up || k==Key::Down)
            power_status_page_=uint8_t((power_status_page_+(k==Key::Down?1u:3u))%4u);
        if(k==Key::Back || k==Key::Menu){page_=Page::Settings;notify("");}
        return;
    }
    if(k==Key::BandProfile) {
        const uint32_t hz=config_.vfo[config_.active].rx_hz;
        band_power_=preset_channel(TxPower::Lpd10mW,hz)?TxPower::Lpd10mW:TxPower::Pmr500mW;
        const unsigned channel=preset_channel(band_power_,hz);
        band_channel_=uint8_t(channel?channel:1);editor_value_=0;editor_digits_=0;
        power_test_=false;parent_=Page::Settings;page_=Page::BandProfile;notify("");return;
    }
    if(page_==Page::BandProfile) {
        if(k==Key::Back || k==Key::Menu){page_=Page::Settings;notify("");return;}
        if(k==Key::Erase){editor_value_/=10u;if(editor_digits_)--editor_digits_;notify("");return;}
        if(k==Key::Left || k==Key::Right) {
            band_power_=band_power_==TxPower::Lpd10mW?TxPower::Pmr500mW:TxPower::Lpd10mW;
            if(band_channel_>preset_channels(band_power_))band_channel_=1;
            editor_value_=0;editor_digits_=0;notify("");
        }
        if(k==Key::Up || k==Key::Down) {
            const unsigned count=preset_channels(band_power_);
            band_channel_=uint8_t((band_channel_-1u+(k==Key::Down?1u:count-1u))%count+1u);
            editor_value_=0;editor_digits_=0;notify("");
        }
        if(k==Key::Ok) {
            const unsigned channel=editor_digits_?editor_value_:band_channel_;
            const uint32_t hz=preset_frequency(band_power_,channel);
            if(!hz){notify("CHANNEL OUT OF RANGE");return;}
            const unsigned slot=config_.active;
            // Apply an explicit VFO profile without overwriting a saved channel.
            const auto previous=active_radio_profile();
            vfo_profiles_[slot]=previous;
            auto &profile=vfo_profiles_[slot];
            profile.power=band_power_;profile.narrow=true;
            profile.shift=Shift::None;profile.offset_hz=0;profile.reverse=false;
            vfo_hz_[slot]=hz;vfo_tx_hz_[slot]=hz;config_.vfo[slot].channel_mode=false;
            cancel_entry();sync_frequencies();page_=Page::Radio;notify("CHANNEL PROFILE APPLIED");
        }
        return;
    }
    if(page_==Page::PowerClear) {
        power_test_=false;
        if(k==Key::Ok){measured_power_.clear();page_=Page::PowerCalibration;notify("POWER RECORDS CLEARED");}
        if(k==Key::Back){page_=Page::PowerCalibration;notify("");}
        if(k==Key::Menu){page_=Page::Settings;notify("");}
        return;
    }
    if(k==Key::PowerCalibration) {
        const auto &profile=active_radio_profile();
        const uint32_t hz=config_.vfo[config_.active].tx_hz;
        if(!numeric_power(profile.power) || !profile.narrow ||
           profile.shift!=Shift::None || profile.reverse || hz!=config_.vfo[config_.active].rx_hz) {
            notify("SELECT TARGET AND SIMPLEX");return;
        }
        if(hz!=calibration_frequency(profile.power)) {
            notify(profile.power==TxPower::Lpd10mW?"SET VFO TO 433.00000":"SET VFO TO 446.00000");return;
        }
        const auto *point=measured_power_.find(profile.power,hz,power_battery_mv_);
        power_code_=point?point->code:1;power_hz_=hz;power_kind_=profile.power;
        power_test_=false;parent_=Page::Settings;page_=Page::PowerCalibration;
        editor_value_=0;editor_digits_=0;notify("");return;
    }
    if(page_==Page::PowerCalibration || page_==Page::PowerMeasured) {
        if(k==Key::Back || k==Key::Menu) {
            power_test_=false;editor_value_=0;editor_digits_=0;
            page_=k==Key::Back && page_==Page::PowerMeasured?Page::PowerCalibration:Page::Settings;
            notify("");return;
        }
        if(k==Key::Erase) {
            power_test_=false;editor_value_/=10u;if(editor_digits_)--editor_digits_;notify("");return;
        }
        if(page_==Page::PowerMeasured) {
            if(k==Key::Ok) {
                if(!editor_digits_){notify("ENTER MEASURED POWER");return;}
                if(!editor_value_){notify("MEASUREMENT MUST BE POSITIVE");return;}
                if(editor_value_>target_mw(power_kind_)) {
                    notify("MEASUREMENT ABOVE TARGET");return;
                }
                const MeasuredPowerPoint point{power_hz_,power_battery_mv_,uint16_t(editor_value_),power_code_,power_kind_};
                if(!MeasuredPower::valid(point)){notify("POWER BATTERY INVALID");return;}
                if(!measured_power_.store(point)){notify("POWER TABLE FULL");return;}
                page_=Page::PowerCalibration;editor_value_=0;editor_digits_=0;notify("MEASUREMENT SAVED");
            }
            return;
        }
        if(k==Key::Right) {
            if(!power_limit_ || power_code_>power_limit_){notify("POWER CODE EXCEEDS LOW");return;}
            power_test_=false;page_=Page::PowerMeasured;editor_value_=0;editor_digits_=0;notify("");return;
        }
        if(k==Key::Left) {
            power_test_=false;page_=Page::PowerClear;notify("");return;
        }
        if(k==Key::Up || k==Key::Down) {
            power_test_=false;editor_value_=0;editor_digits_=0;
            if(k==Key::Up && power_code_<power_limit_)++power_code_;
            if(k==Key::Down && power_code_>1)--power_code_;
        }
        if(k==Key::Ok) {
            if(editor_digits_) {
                power_test_=false;
                if(!editor_value_ || editor_value_>power_limit_){notify("POWER CODE EXCEEDS LOW");return;}
                power_code_=uint8_t(editor_value_);editor_value_=0;editor_digits_=0;
            } else {
                if(!power_limit_ || power_code_>power_limit_){notify("POWER CODE EXCEEDS LOW");return;}
                power_test_=!power_test_;
            }
            notify("");
        }
        return;
    }
    if(k==Key::MenuTimeout) {
        parent_=Page::Settings;page_=Page::MenuTimeout;
        menu_timeout_draft_=menu_timeout_;notify("");return;
    }
    if(page_==Page::MenuTimeout && k!=Key::Back && k!=Key::Menu) {
        constexpr unsigned count=sizeof menu_timeouts/sizeof menu_timeouts[0];
        if(k==Key::Up || k==Key::Down)
            menu_timeout_draft_=uint8_t((menu_timeout_draft_+(k==Key::Down?1:count-1))%count);
        if(k==Key::Ok) {
            menu_timeout_=menu_timeout_draft_;page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(k==Key::AprsMessages) {
        parent_=Page::Settings;page_=Page::AprsMessages;aprs_message_=0;notify("");return;
    }
    if(page_==Page::AprsMessages) {
        if(k==Key::Erase){page_=Page::ClearAprsMessages;notify("");return;}
        if(k==Key::Mode){aprs_personal_=!aprs_personal_;aprs_message_=0;return;}
        const unsigned count=aprs_personal_?aprs_personal_count_:aprs_all_count_;
        if(count && (k==Key::Down || k==Key::Up))
            aprs_message_=uint8_t((aprs_message_+(k==Key::Down?1:count-1))%count);
    }
    if(page_==Page::ClearAprsMessages) {
        if(k==Key::Back || k==Key::Menu)page_=Page::AprsMessages;
        if(k==Key::Ok) {
            aprs_messages_clear_=true;aprs_message_=0;
            page_=Page::AprsMessages;notify("APRS MESSAGES CLEARED");
        }
        return;
    }
    if(k==Key::BeaconSource) {
        parent_=Page::Settings;page_=Page::BeaconSource;
        beacon_source_draft_=beacon_saved_point_;notify("");return;
    }
    if(page_==Page::BeaconSource) {
        if(k==Key::Up || k==Key::Down)beacon_source_draft_=!beacon_source_draft_;
        if(k==Key::Back || k==Key::Menu){page_=parent_;notify("");}
        if(k==Key::Ok) {
            if(beacon_source_draft_ && !waypoint_.valid){notify("NO SAVED GPS POINT");return;}
            if(beacon_saved_point_!=beacon_source_draft_) {
                config_.beacon_on=false;beacon_saved_point_=beacon_source_draft_;
                notify("SOURCE SET / BEACON OFF");
            } else notify("SETTING APPLIED");
            page_=parent_;
        }
        return;
    }
    if(k==Key::ClearAprs) {
        parent_=Page::Settings;page_=Page::ClearAprs;notify("");return;
    }
    if(page_==Page::ClearAprs && k==Key::Ok) {
        aprs_clear_=true;page_=parent_;notify("APRS LIST CLEARED");return;
    }
    if(k==Key::BeaconSymbol) {
        parent_=Page::Settings;page_=Page::BeaconSymbol;
        beacon_symbol_draft_=beacon_symbol_;notify("");return;
    }
    if(page_==Page::BeaconSymbol && k!=Key::Back && k!=Key::Menu) {
        constexpr unsigned count=sizeof beacon_symbols/sizeof beacon_symbols[0];
        if(k==Key::Up || k==Key::Down)
            beacon_symbol_draft_=uint8_t((beacon_symbol_draft_+(k==Key::Down?1:count-1))%count);
        if(k==Key::Ok) {
            beacon_symbol_=beacon_symbol_draft_;page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(k==Key::VoiceRx){page_=Page::VoiceRx;notify("");return;}
    if(page_==Page::VoiceRx) {
        if(k==Key::Back){page_=Page::Settings;return;}
        return;
    }
    if(k==Key::FmRadio || k==Key::FmProbe || k==Key::SwAm || k==Key::LongwaveAm || k==Key::MediumwaveAm) {
        if(page_==Page::FmProbe){page_=parent_;return;}
        if(page_!=Page::Radio && page_!=Page::Settings)return;
        parent_=page_;page_=Page::FmProbe;fm_audio_=k!=Key::FmProbe;
        broadcast_band_=k==Key::SwAm?fm::Band::Shortwave:k==Key::LongwaveAm?fm::Band::Longwave:
            k==Key::MediumwaveAm?fm::Band::Mediumwave:fm::Band::Fm;
        if(sw_am() && !fm::in_band(sw_request_,broadcast_band_))
            sw_request_=fm::band_plan(broadcast_band_).initial;
        fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;notify("");return;
    }
    if(page_==Page::FmProbe) {
        if(k==Key::Back){page_=parent_;return;}
        if(!fm_audio_)return;
        if(k==Key::Menu){fm_seek_=false;key(sw_am()?Key::AmPresets:Key::FmPresets);return;}
        if(sw_am()) {
            const auto plan=fm::band_plan(broadcast_band_);
            if(k==Key::Erase && fm_digits_)fm_entry_[--fm_digits_]=0;
            else if(k==Key::Ok && fm_digits_) {
                unsigned value=0;
                for(unsigned i=0;i<fm_digits_;++i)value=value*10u+unsigned(fm_entry_[i]-'0');
                if(fm::in_band(value,broadcast_band_)) {
                    sw_request_=uint16_t(value);fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;notify("");
                } else notify(broadcast_band_==fm::Band::Longwave?"AM 153-279 kHz":
                    broadcast_band_==fm::Band::Mediumwave?"AM 520-1710 kHz":"AM 2300-30000 kHz");
            } else if(k==Key::Up || k==Key::Down) {
                const int step=int(tuner_step(true));
                const int next=int(sw_request_)+(k==Key::Up?step:-step);
                sw_request_=uint16_t(next>plan.high?plan.low:next<plan.low?plan.high:next);
                fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;notify("");
            } else if(k==Key::Right || k==Key::Left) {
                fm_seek_down_=k==Key::Left;
                fm_seek_=!fm_seek_;fm_digits_=0;fm_entry_[0]=0;notify("");
            }
            return;
        }
        if(k==Key::Erase && fm_digits_)fm_entry_[--fm_digits_]=0;
        else if(k==Key::Ok && fm_digits_) {
            unsigned value=0;
            for(unsigned i=0;i<fm_digits_;++i)value=value*10u+unsigned(fm_entry_[i]-'0');
            if(value>=6500 && value<=10800) {
                fm_request_=uint16_t(value);fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;notify("");
            } else notify("FM 65.00-108.00 MHz");
        } else if(k==Key::Up || k==Key::Down) {
            const int step=int(tuner_step(false));
            const int next=int(fm_request_)+(k==Key::Up?step:-step);
            fm_request_=uint16_t(next>10800?6500:next<6500?10800:next);
            fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;
        } else if(k==Key::Right) {
            fm_seek_down_=false;fm_seek_=!fm_seek_;fm_digits_=0;fm_entry_[0]=0;notify("");
        }
        else if(k==Key::NextSlot) {fm_memory_=fm_request_;fm_presets_[0]=fm_request_;notify("FM preset saved");}
        else if(k==Key::Left && fm_memory_) {fm_request_=fm_memory_;fm_seek_=false;fm_digits_=0;fm_entry_[0]=0;notify("");}
        return;
    }
    if(k==Key::FlashInfo) {
        parent_=Page::Settings;page_=Page::FlashInfo;notify("");return;
    }
    if(k==Key::Gps && page_==Page::Gps)return;
    if(k==Key::RenameChannel || k==Key::DeleteChannel || k==Key::MoveChannel) {
        channel_context_=false;channel_action_=k;page_=Page::ManageChannel;select_channel_for_list();notify("");return;
    }
    if(k==Key::SelectZone || k==Key::RenameZone) {
        channel_context_=false;channel_action_=k;zone_selection_=active_zone_;page_=Page::Zones;notify("");return;
    }
    const Page channel_return=channel_context_?Page::Channels:Page::ManageChannel;
    if(page_==Page::ChannelCommands) {
        if(k==Key::Back || k==Key::Menu){page_=Page::Channels;notify("");}
        else if(k==Key::Up || k==Key::Down)
            channel_command_=uint8_t((channel_command_+(k==Key::Down?1u:ChannelCommandCount-1u))%ChannelCommandCount);
        else if(k==Key::Ok && channel_visible(channel_)) {
            if(channel_command_==ChannelCommandCount-1u) {
                const auto &memory=memories_[channel_];
                const unsigned slot=config_.active;
                vfo_hz_[slot]=memory.rx_hz;
                vfo_tx_hz_[slot]=memory.tx_hz;
                vfo_profiles_[slot]=memory.radio;
                config_.vfo[slot].channel_mode=false;
                sync_frequencies();
                cancel_entry();page_=Page::Radio;notify("CHANNEL COPIED TO VFO");return;
            }
            constexpr Key commands[]={Key::RenameChannel,Key::MoveChannel,Key::DeleteChannel};
            channel_action_=commands[channel_command_];
            if(channel_action_==Key::RenameChannel)open_name_editor(memories_[channel_].name);
            else if(channel_action_==Key::MoveChannel){zone_selection_=active_zone_;page_=Page::Zones;}
            else page_=Page::ConfirmDelete;
        }
        return;
    }
    if(page_==Page::EditName && k!=Key::Menu) {
        if(k==Key::Back){page_=channel_action_==Key::RenameZone?Page::Zones:channel_return;notify("");}
        else if(k==Key::Left && name_cursor_)--name_cursor_;
        else if(k==Key::Right && name_cursor_<7 && name_draft_[name_cursor_])++name_cursor_;
        else if(k==Key::Erase) {
            for(unsigned i=name_cursor_;i<8;++i)name_draft_[i]=name_draft_[i+1];
        } else if(k==Key::Up || k==Key::Down) {
            char &c=name_draft_[name_cursor_];unsigned index=0;
            while(index<37 && name_chars[index]!=c)++index;
            c=name_chars[index==37?(k==Key::Up?0:36):(index+(k==Key::Up?1:36))%37];
        } else if(k==Key::Ok) {
            if(!name_draft_[0]){notify("ENTER A NAME");return;}
            if(channel_action_==Key::RenameZone){text(zone_names_[zone_selection_],name_draft_);page_=Page::Zones;}
            else {text(memories_[channel_].name,name_draft_);page_=channel_return;}
            notify("SETTING APPLIED");
        }
        return;
    }
    if(page_==Page::ConfirmDelete && k!=Key::Menu) {
        if(k==Key::Ok) {
            scan_excluded_[channel_/32]&=~(1u<<(channel_%32));
            memories_[channel_]={};reconcile_channels();select_channel_for_list();
            page_=channel_return;notify("CHANNEL DELETED");
        } else if(k==Key::Back){page_=channel_return;notify("");}
        return;
    }
    if(page_==Page::Zones && k!=Key::Menu) {
        if(k==Key::Up || k==Key::Down)zone_selection_=(zone_selection_+(k==Key::Down?1:ZoneCount-1))%ZoneCount;
        else if(k==Key::Back){page_=channel_action_==Key::MoveChannel?channel_return:
            channel_context_?Page::Channels:Page::Settings;notify("");}
        else if(k==Key::Ok) {
            if(channel_action_==Key::RenameZone)open_name_editor(zone_names_[zone_selection_]);
            else if(channel_action_==Key::MoveChannel) {
                memories_[channel_].zone=uint8_t(zone_selection_);reconcile_channels();
                select_channel_for_list();page_=channel_return;notify("CHANNEL MOVED");
            } else {
                active_zone_=zone_selection_;reconcile_channels();
                if(channel_context_)select_channel_for_list();
                page_=channel_context_?Page::Channels:Page::Settings;notify("ZONE SELECTED");
            }
        }
        return;
    }
    if(page_==Page::RadioOptions && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Mode && tone_search_available()) {
            tone_search_.begin(radio_setting_==RadioSetting::RxCtcss?ToneKind::Ctcss:ToneKind::Dcs);
            notify("");return;
        }
        const unsigned count=radio_option_count(radio_setting_);
        if(k==Key::Up || k==Key::Down)radio_draft_=(radio_draft_+(k==Key::Down?1:count-1))%count;
        if(radio_is_tone(radio_setting_) && (k==Key::Left || k==Key::Right))
            radio_draft_=(radio_draft_+(k==Key::Right?10:count-10))%count;
        if(k==Key::Ok) {
            radio_profile_apply(editable_radio_profile(),radio_setting_,radio_draft_);
            sync_frequencies();
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(k==Key::LanguageSelect || k==Key::VfoStep) {
        parent_=Page::Settings;
        page_=k==Key::LanguageSelect?Page::LanguageSelect:Page::VfoStep;
        language_draft_=language_;vfo_step_draft_=uint8_t(applied_vfo_step());notify("");return;
    }
    if((page_==Page::LanguageSelect || page_==Page::VfoStep) && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Up || k==Key::Down) {
            if(page_==Page::LanguageSelect)
                language_draft_=language_draft_==Language::English?Language::Russian:Language::English;
            else vfo_step_draft_=uint8_t((vfo_step_draft_+(k==Key::Down?1:vfo_step_count-1))%vfo_step_count);
        }
        if(k==Key::Ok) {
            if(page_==Page::LanguageSelect)language_=language_draft_;
            else vfo_step_[config_.active]=vfo_step_draft_;
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(k==Key::AltitudeUnits) {
        parent_=Page::Settings;page_=Page::AltitudeUnits;
        altitude_draft_feet_=altitude_feet_;notify("");return;
    }
    if(page_==Page::AltitudeUnits && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Up || k==Key::Down)altitude_draft_feet_=!altitude_draft_feet_;
        if(k==Key::Ok){altitude_feet_=altitude_draft_feet_;page_=parent_;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::SpeedUnits) {
        parent_=Page::Settings;page_=Page::SpeedUnits;
        speed_draft_=speed_unit_;notify("");return;
    }
    if(page_==Page::SpeedUnits && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Up || k==Key::Down)
            speed_draft_=SpeedUnit((unsigned(speed_draft_)+(k==Key::Up?1:2))%3);
        if(k==Key::Ok){speed_unit_=speed_draft_;page_=parent_;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::BeaconIdentity || k==Key::CoordinateFormat) {
        parent_=Page::Settings;
        page_=k==Key::BeaconIdentity?Page::BeaconIdentity:Page::CoordinateFormat;
        identity_draft_=beacon_identity_;identity_cursor_=0;
        coordinate_draft_=uint8_t(coordinate_selection());notify("");return;
    }
    if(page_==Page::BeaconIdentity && k!=Key::Back) {
        auto &id=identity_draft_;
        if(k==Key::NextSlot)identity_cursor_=identity_cursor_==6?0:6;
        else if(k==Key::Left && identity_cursor_>0) {
            if(identity_cursor_==6)identity_cursor_=0;
            else --identity_cursor_;
        }
        else if(k==Key::Right && identity_cursor_<6 && id.callsign[identity_cursor_])++identity_cursor_;
        else if(k==Key::Erase && identity_cursor_<6) {
            for(unsigned i=identity_cursor_;i<6;++i)id.callsign[i]=id.callsign[i+1];
        } else if(k==Key::Up || k==Key::Down) {
            const bool up=k==Key::Up;
            if(identity_cursor_==6)id.ssid=uint8_t((id.ssid+(up?1:15))%16);
            else {
                char &c=id.callsign[identity_cursor_];
                unsigned index=0;
                while(index<36 && callsign_chars[index]!=c)++index;
                c=callsign_chars[index==36?(up?0:35):(index+(up?1:35))%36];
            }
        } else if(k==Key::Ok) {
            if(!id.callsign[0]){notify("ENTER A CALLSIGN");return;}
            beacon_identity_=id;page_=parent_;notify("SETTING APPLIED");
        } else if(k==Key::Menu) {
            page_=Page::Settings;menu_group_=MenuGroup::Root;notify("");
        }
        return;
    }
    if(page_==Page::CoordinateFormat && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Up || k==Key::Down)coordinate_draft_=uint8_t((coordinate_draft_+(k==Key::Down?1u:2u))%3u);
        if(k==Key::Ok){decimal_coordinates_=coordinate_draft_==1;dms_coordinates_=coordinate_draft_==2;page_=parent_;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::ScanMode) {
        parent_=Page::Settings;page_=Page::ScanMode;scan_mode_draft_=scan_mode_;notify("");return;
    }
    if(page_==Page::ScanMode && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Up || k==Key::Down)
            scan_mode_draft_=static_cast<ScanMode>((unsigned(scan_mode_draft_)+(k==Key::Down?1:2))%3);
        if(k==Key::Ok){scan_mode_=scan_mode_draft_;page_=parent_;notify("SETTING APPLIED");}
        return;
    }
    if(k==Key::Spectrum || page_==Page::Spectrum) {
        if(page_==Page::Spectrum && (k==Key::Back || k==Key::Menu)){page_=parent_;return;}
        if(page_==Page::Spectrum && k==Key::NextSlot) {
            const unsigned point=spectrum_.selected();
            if(spectrum_.sample(point)==Spectrum::Unmeasured){notify("SELECT A MEASURED FREQUENCY");return;}
            const auto hz=spectrum_.frequency(point);
            const auto &rx=active_radio_profile();
            auto &profile=vfo_profiles_[config_.active];
            profile.narrow=rx.narrow;profile.squelch=rx.squelch;
            profile.rx_tone={};profile.reverse=false;profile.shift=Shift::None;profile.offset_hz=0;
            profile.tx_inhibit=true;
            vfo_tx_hz_[config_.active]=0;
            vfo_hz_[config_.active]=hz;
            config_.vfo[config_.active].channel_mode=false;
            sync_frequencies();
            page_=Page::Radio;notify("FREQUENCY APPLIED");return;
        }
        if(k==Key::Left || k==Key::Right){spectrum_.select(k==Key::Right);notify("");}
        if(k!=Key::Spectrum && k!=Key::Ok)return;
        const uint32_t center=config_.vfo[config_.active].rx_hz;
        const bool vhf=voice::vhf_tx_frequency(center);
        if(!vhf && !(voice::uhf_tx_frequency(center) && center%10==0)) {
            notify("SPECTRUM REQUIRES VHF/UHF");return;
        }
        if(page_!=Page::Spectrum){parent_=page_;page_=Page::Spectrum;}
        spectrum_.begin(center,vfo_steps[applied_vfo_step()],vhf?136000000u:400000000u,vhf?174000000u:470000000u);
        notify("");return;
    }
    if(k==Key::ChannelScan || k==Key::VfoScan) {
        parent_=page_==Page::Settings?Page::Settings:Page::Radio;
        page_=k==Key::ChannelScan?Page::ChannelScan:Page::VfoScan;
        scan_channel(MemoryCount);scan_reverse_=false;scan_step_=false;notify("");return;
    }
    if(scan_page() && k!=Key::Back && k!=Key::Menu) {
        if(k==Key::Left && page_==Page::ChannelScan && scan_eligible(scan_channel_)) {
            scan_excluded_[scan_channel_/32]|=1u<<(scan_channel_%32);
            scan_channel(MemoryCount);scan_step_=true;
            notify("CHANNEL EXCLUDED");return;
        }
        if(k==Key::Up || k==Key::Down)scan_reverse_=k==Key::Down;
        if(k==Key::Right || k==Key::Up || k==Key::Down){scan_paused_=false;scan_step_=true;}
        if(k==Key::Ok && scan_frequency_) {
            if(!scan_paused_)scan_paused_=true;
            else if(page_==Page::ChannelScan && recall(scan_channel_)) {
                page_=Page::Radio;notify("CHANNEL SELECTED");
            } else if(page_==Page::VfoScan && frequency_valid(scan_frequency_)) {
                vfo_profiles_[config_.active].reverse=false;
                set_frequency(scan_frequency_);
                page_=Page::Radio;notify("FREQUENCY APPLIED");
            }
        }
        return;
    }
    if(k==Key::WatchSlots) {
        parent_=page_==Page::Radio?Page::Radio:Page::Settings;page_=Page::WatchSlots;
        watch_slots_draft_=uint8_t(watch_slots_selection());notify("");return;
    }
    if(page_==Page::WatchSlots && k!=Key::Back && k!=Key::Menu) {
        constexpr unsigned count=sizeof watch_masks/sizeof watch_masks[0];
        if(k==Key::Up || k==Key::Down)
            watch_slots_draft_=uint8_t((watch_slots_draft_+(k==Key::Down?1:count-1))%count);
        if(k==Key::Ok) {
            const uint8_t mask=watch_masks[watch_slots_draft_];
            if(mask)config_.watch_mask=mask;
            set_option(Option::Watch,mask!=0);
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(k==Key::ScanStart || k==Key::ScanEnd) {
        parent_=Page::Settings;page_=k==Key::ScanStart?Page::ScanStart:Page::ScanEnd;
        editor_value_=0;editor_digits_=0;notify("");return;
    }
    if(k==Key::WatchDwell || k==Key::WatchHold || k==Key::AprsScanInterval || k==Key::BeaconInterval || k==Key::AprsRxFrequency || k==Key::BeaconFrequency || k==Key::TxFrequency || k==Key::RepeaterOffset) {
        parent_=Page::Settings;
        page_=k==Key::WatchDwell?Page::WatchDwell:k==Key::WatchHold?Page::WatchHold:k==Key::AprsScanInterval?Page::AprsScanInterval:k==Key::BeaconInterval?Page::BeaconInterval:k==Key::AprsRxFrequency?Page::AprsRxFrequency:k==Key::BeaconFrequency?Page::BeaconFrequency:
              k==Key::TxFrequency?Page::TxFrequency:Page::RepeaterOffset;
        editor_value_=0;editor_digits_=0;notify("");return;
    }
    if(k==Key::Gps || k==Key::Aprs || k==Key::Debug || k==Key::Channels || k==Key::Battery || k==Key::Brightness || k==Key::ScreenTimer || k==Key::LcdTest) {
        parent_=page_==Page::Settings?Page::Settings:Page::Radio;
        page_=k==Key::Gps?Page::Gps:k==Key::Aprs?Page::Aprs:k==Key::Debug?Page::Debug:k==Key::Channels?Page::Channels:k==Key::Battery?Page::Battery:k==Key::Brightness?Page::Brightness:k==Key::ScreenTimer?Page::ScreenTimer:Page::LcdTest;
        if(page_==Page::Aprs)aprs_distance_targets(nullptr,0);
        if(k==Key::LcdTest)lcd_test_gray_=false;
        editor_value_=0;editor_digits_=0;
        if(k==Key::Channels)select_channel_for_list();
        return;
    }
    if(page_==Page::ConfirmStore) {
        if(k==Key::Back || k==Key::Menu){page_=Page::StoreChannel;notify("");}
        if(k==Key::Ok && channel_visible(channel_) && store(channel_)){
            page_=Page::Radio;notify("CHANNEL UPDATED");
        }
        return;
    }
    if(k==Key::Erase) {
        if((page_==Page::Channels || page_==Page::StoreChannel) && editor_digits_) {
            editor_value_/=10u;--editor_digits_;notify("");return;
        }
        if(page_==Page::ScreenTimer || page_==Page::BeaconInterval || page_==Page::AprsScanInterval || watch_timer_editor() || frequency_editor()){if(editor_digits_){editor_value_/=10u;--editor_digits_;notify("");}return;}
        k=Key::Back;
    }
    if(k==Key::Back) {
        if(page_==Page::ManageChannel){page_=Page::Settings;notify("");return;}
        if(page_==Page::Settings && menu_group_!=MenuGroup::Root) {
            menu_group_=menu_group_==MenuGroup::Repeater?MenuGroup::Radio:(menu_group_==MenuGroup::Beacon || menu_group_==MenuGroup::AprsRx)?MenuGroup::Aprs:MenuGroup::Root;
            notify("");return;
        }
        if(digits_ && page_==Page::Radio) {entry_khz_/=10;--digits_;invalid_=false;notify("");}
        else {page_=(page_==Page::Gps || page_==Page::Aprs || page_==Page::AprsMessages || page_==Page::Debug || page_==Page::Channels || page_==Page::Battery || page_==Page::StoreChannel || page_==Page::Brightness || page_==Page::ScreenTimer || page_==Page::LcdTest || page_==Page::BeaconInterval || page_==Page::AprsScanInterval || watch_timer_editor() || page_==Page::WatchSlots || scan_page() || page_==Page::ScanMode || page_==Page::BeaconSymbol || page_==Page::ClearAprs || frequency_editor() || page_==Page::BeaconIdentity || page_==Page::CoordinateFormat || page_==Page::SpeedUnits || page_==Page::AltitudeUnits || page_==Page::LanguageSelect || page_==Page::VfoStep || page_==Page::MenuTimeout || page_==Page::RadioOptions || page_==Page::FlashInfo || page_==Page::FmProbe)?parent_:Page::Radio;invalid_=false;notify("");}
        return;
    }
    if(page_==Page::TxFrequency && k==Key::Left) {
        editor_value_=base_rx(config_.active)/10u;
        editor_digits_=8;notify("");return;
    }
    if(page_==Page::TxFrequency && k==Key::Right) {
        editor_value_=0;editor_digits_=1;notify("");return;
    }
    if(frequency_editor()) {
        const bool tx_editor=page_==Page::TxFrequency;
        if(k==Key::Ok) {
            if(editor_digits_) {
                const bool unset=!editor_value_ && (scan_bound_editor() || tx_editor);
                if(editor_digits_<6 && !unset){notify("ENTER 6 TO 8 DIGITS");return;}
                uint32_t hz=editor_value_*10u;
                for(unsigned i=editor_digits_;i<8;++i)hz*=10u;
                if(scan_bound_editor()) {
                    const bool upper=page_==Page::ScanEnd;
                    if(!valid_scan_bounds(upper?scan_start_:hz,upper?hz:scan_end_)) {
                        notify("INVALID SCAN LIMITS");return;
                    }
                    (upper?scan_end_:scan_start_)=hz;
                } else if(page_==Page::RepeaterOffset) {
                    if(hz>99999990u){notify("OFFSET TOO LARGE");return;}
                    editable_radio_profile().offset_hz=hz;
                    sync_frequencies();
                } else {
                    if(!unset && (!frequency_valid(hz) ||
                       (hz<30000000u && (!tx_editor || !voice::cb_fm_frequency(hz))))) {
                        notify("FREQUENCY OUT OF BAND");return;
                    }
                    if(page_==Page::AprsRxFrequency)config_.aprs_rx_hz=hz;
                    else if(page_==Page::BeaconFrequency)set_aprs_frequency(hz);
                    else {
                        const unsigned slot=config_.active;
                        if(!frequency_valid(base_rx(slot))){notify("SET FREQUENCY FIRST");return;}
                        auto &v=config_.vfo[slot];
                        if(v.channel_mode)memories_[v.memory_channel].tx_hz=hz;
                        else vfo_tx_hz_[slot]=hz;
                        auto &profile=editable_radio_profile();
                        profile.shift=Shift::None;profile.reverse=false;
                        sync_frequencies();
                    }
                }
            }
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(watch_timer_editor()) {
        if(k==Key::Ok) {
            const bool dwell=page_==Page::WatchDwell;
            if(editor_digits_) {
                if(dwell && (editor_value_<100 || editor_value_>2000)) {
                    notify("USE 100 TO 2000 MS");return;
                }
                if(!dwell && editor_value_>10000){notify("USE 0 TO 10000 MS");return;}
                if(dwell)config_.dwell_ms=uint16_t(editor_value_);
                else config_.hold_ms=uint16_t(editor_value_);
            }
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(page_==Page::BeaconInterval || page_==Page::AprsScanInterval) {
        if(k==Key::Ok) {
            if(editor_digits_) {
                if(page_==Page::AprsScanInterval) {
                    if(editor_value_ && (editor_value_<5 || editor_value_>600)) {
                        notify("USE 0 OR 5 TO 600 SECONDS");return;
                    }
                    config_.aprs_scan_seconds=uint16_t(editor_value_);
                } else {
                    if(editor_value_<10 || editor_value_>86400){notify("USE 10 TO 86400 SECONDS");return;}
                    config_.beacon_seconds=editor_value_;
                }
            }
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(page_==Page::LcdTest) {
        if(k==Key::Ok)lcd_test_gray_=!lcd_test_gray_;
        if(k==Key::Down)brightness_=1;
        if(k==Key::Up)brightness_=4;
        return;
    }
    if(page_==Page::Brightness || page_==Page::ScreenTimer) {
        if(k==Key::Up || k==Key::Down) {
            int dir=k==Key::Up?1:-1;
            if(page_==Page::Brightness){int n=int(brightness_)+dir;if(n>=1 && n<=4)brightness_=unsigned(n);}
        } else if(k==Key::Ok) {
            if(page_==Page::ScreenTimer && editor_digits_) {
                if(editor_value_>3600){notify("USE 0 TO 3600 SECONDS");return;}
                screen_seconds_=editor_value_;
            }
            page_=parent_;notify("SETTING APPLIED");
        }
        return;
    }
    if(k==Key::Menu) {page_=Page::Settings;menu_group_=MenuGroup::Root;notify("");return;}
    if(k==Key::Save) {
        if(digits_){notify("APPLY OR CANCEL INPUT FIRST");return;}
        if(!frequency_valid(config_.vfo[config_.active].rx_hz)){notify("SET FREQUENCY FIRST");return;}
        parent_=page_==Page::Settings?Page::Settings:Page::Radio;
        select_channel_for_list();
        if(!config_.vfo[config_.active].channel_mode) {
            for(unsigned i=0;i<MemoryCount;++i)if(!memories_[i].occupied){channel_=i;break;}
        }
        if(memories_[channel_].occupied && !channel_visible(channel_)){notify("NO CHANNEL SPACE");return;}
        editor_value_=0;editor_digits_=0;page_=Page::StoreChannel;notify("");return;
    }
    if(page_==Page::Channels && !editor_digits_ && k==Key::Right && channel_visible(channel_)) {
        scan_excluded_[channel_/32]^=1u<<(channel_%32);
        notify("");return;
    }
    if(page_==Page::Channels && !editor_digits_ && k==Key::Left && channel_visible(channel_)) {
        channel_context_=true;channel_command_=0;page_=Page::ChannelCommands;notify("");return;
    }
    if(page_==Page::Channels && !editor_digits_ && k==Key::NextSlot) {
        channel_context_=true;channel_action_=Key::SelectZone;
        zone_selection_=active_zone_;page_=Page::Zones;notify("");return;
    }
    if(page_==Page::Channels || page_==Page::StoreChannel || page_==Page::ManageChannel) {
        if(k==Key::Up || k==Key::Down) {
            if(page_!=Page::ManageChannel){editor_value_=0;editor_digits_=0;notify("");}
            for(unsigned n=1;n<=MemoryCount;++n) {
                unsigned i=unsigned((int(channel_)+(k==Key::Down?1:-1)*int(n)+int(MemoryCount))%int(MemoryCount));
                if(channel_visible(i) || (page_==Page::StoreChannel && !memories_[i].occupied)){channel_=i;break;}
            }
        } else if(k==Key::Ok) {
            if(page_!=Page::ManageChannel && editor_digits_) {
                const unsigned index=editor_value_-1u;
                if(index>=MemoryCount || (!channel_visible(index) &&
                   (page_!=Page::StoreChannel || memories_[index].occupied))) {
                    notify("CHANNEL NOT IN LIST");return;
                }
                channel_=index;editor_value_=0;editor_digits_=0;
            }
            if(page_==Page::ManageChannel) {
                if(!channel_visible(channel_)){notify("NO SAVED CHANNELS");return;}
                if(channel_action_==Key::RenameChannel)open_name_editor(memories_[channel_].name);
                else if(channel_action_==Key::DeleteChannel){page_=Page::ConfirmDelete;notify("");}
                else {zone_selection_=active_zone_;page_=Page::Zones;notify("");}
            } else if(page_==Page::StoreChannel) {
                if(channel_visible(channel_)){page_=Page::ConfirmStore;notify("");}
                else if(!memories_[channel_].occupied && store(channel_)){page_=Page::Radio;notify("CHANNEL UPDATED");}
                else notify("CANNOT SAVE CHANNEL");
            }
            else if(recall(channel_)){page_=Page::Radio;notify("CHANNEL SELECTED");}else notify("NO SAVED CHANNELS");
        }
        return;
    }
    if(page_==Page::Settings) {
        if(k==Key::Up || k==Key::Down) {
            const unsigned n=menu_count();
            auto &selection=menu_selection_[unsigned(menu_group_)];
            selection=uint8_t((selection+(k==Key::Up?n-1:1))%n);
            invalid_=false;
        } else if(k==Key::Ok) {
            const auto &entry=menu_entry(menu_selection());
            if(entry.group!=MenuGroup::Root) {menu_group_=entry.group;notify("");}
            else if(entry.toggle!=Option::Count) {
                invalid_=!set_option(entry.toggle,!enabled(entry.toggle));notify(invalid_?"OPTION UNAVAILABLE":"");
            } else if(entry.action==Key::RadioOptions) {
                radio_setting_=entry.radio_setting;
                radio_draft_=radio_profile_option(active_radio_profile(),radio_setting_);
                parent_=Page::Settings;page_=Page::RadioOptions;notify("");
            } else if(entry.action==Key::Mode) power_save_=!power_save_;
            else key(entry.action);
        }
        return;
    }
    if(page_!=Page::Radio) return;
    if(!digits_ && k==Key::Left){key(Key::WatchSlots);return;}
    if(!digits_ && k==Key::Right) {
        const int heard=recent_voice_slot();
        if(heard>=0 && heard!=config_.active){config_.active=uint8_t(heard);notify("RX SLOT SELECTED");}
        return;
    }
    if(k==Key::NextSlot) {cancel_entry();config_.active=uint8_t((config_.active+1)%3);}
    else if(k==Key::Ok && digits_) {
        uint32_t hz=entry_khz_;
        for(unsigned i=digits_;i<9;++i) hz*=10u;
        if(digits_<3) {invalid_=true;notify("ENTER AT LEAST 3 DIGITS");}
        else if(!set_frequency(hz)) {invalid_=true;notify("FREQUENCY OUT OF BAND");}
        else {cancel_entry();notify("FREQUENCY APPLIED");}
    } else if(k==Key::Mode) {
        cancel_entry();auto &v=config_.vfo[config_.active];
        if(v.channel_mode) {
            v.channel_mode=false;
            sync_frequencies();
        }
        else {bool found=recall(v.memory_channel);if(!found)for(unsigned i=0;i<MemoryCount;++i) if(recall(i)) {found=true;break;}
            notify(found?"CHANNEL MODE":"NO SAVED CHANNELS");}
    } else if((k==Key::Up || k==Key::Down) && !digits_) step(k==Key::Up?1:-1);
}
void Application::entry_text(char *out) const {
    // Fixed positions make MHz and the unfilled decimal places explicit.
    char digits[8];uint32_t value=entry_khz_;
    for(unsigned i=digits_;i>0;--i){digits[i-1]=char('0'+value%10);value/=10;}
    unsigned n=0;
    for(unsigned i=0;i<8;++i){if(i==3)out[n++]='.';out[n++]=i<digits_?digits[i]:'_';}
    out[n]=0;
}
void Application::render(Display &out) const {
    out={};text(out.title,"RT950 CUSTOM / DEV");
    if(page_==Page::Settings) {
        for(unsigned i=0;i<menu_count() && i<9;++i) {
            out.line[i][0]=i==menu_selection()?'>':' ';
            text(out.line[i]+1,menu_entry(i).name);
        }
    } else if(page_==Page::Gps) {
        text(out.line[0],"GPS status from board port");
    } else {
        for(unsigned i=0;i<3;++i) {
            auto &line=out.line[i];const auto &v=config_.vfo[i];
            line[0]=config_.active==i?'>':' ';line[1]=char('A'+i);line[2]=' ';
            line[3]=v.channel_mode?'M':'V';line[4]=' ';frequency(line+5,v.rx_hz);
        }
        text(out.line[3],config_.watch_mode==RT_WATCH_ABC?"Watch ABC":"Watch selected");
        if(digits_) {text(out.line[4],"Enter MHz: ");frequency(out.line[4]+11,entry_khz_*1000u);}
        text(out.line[5],"Channel profiles");
    }
    if(invalid_) text(out.line[6],"Invalid / unavailable");
    text(out.line[7],"RF / APRS / BLE NOT READY");
}
} // namespace rt950
