// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#include "locale.hpp"
#include "gps_navigation.hpp"
#include "radio_settings.hpp"
#include "power_calibration.hpp"
#include "weather_rx.hpp"
extern "C" {
#include "../mvp/core.h"
}

namespace rt950 {
enum class Key : uint8_t { NextSlot, Mode, Up, Down, Ok, Back, Menu, Save, Gps, Aprs, Debug, Channels, Battery, StoreChannel, Brightness, ScreenTimer, Timezone, Erase, LcdTest, BeaconInterval, AprsRxFrequency, BeaconFrequency, BeaconIdentity, CoordinateFormat, Left, Right, SpeedUnits, AltitudeUnits, LanguageSelect, VfoStep, RadioOptions, TxFrequency, RepeaterOffset, RenameChannel, DeleteChannel, SelectZone, RenameZone, MoveChannel, FlashInfo, FmProbe, FmRadio, VoiceRx, SwAm, AprsScanInterval, WatchDwell, WatchHold, WatchSlots, ChannelScan, ScanMode, VfoScan, ScanStart, ScanEnd, BeaconSymbol, ClearAprs, AprsMessages, MenuTimeout, PowerCalibration, BandProfile, PowerStatus, MicGain, KeypadLock, Monitor, AutoLock, FmPresets, AmPresets, About, FmStep, AmStep, DistanceUnits, Waypoint, Distance, Backup, ResetSettings, ResetAll, Import, Pf1Short, Pf1Hold, Pf2Short, Pf2Hold, Weather, WeatherAlert };
enum class Page : uint8_t { Radio, Settings, Gps, Aprs, Debug, Channels, Battery, StoreChannel, Brightness, ScreenTimer, Timezone, LcdTest, BeaconInterval, AprsRxFrequency, BeaconFrequency, BeaconIdentity, CoordinateFormat, SpeedUnits, AltitudeUnits, LanguageSelect, VfoStep, RadioOptions, TxFrequency, RepeaterOffset, ManageChannel, EditName, ConfirmDelete, Zones, FlashInfo, FmProbe, VoiceRx, AprsScanInterval, WatchDwell, WatchHold, WatchSlots, ChannelScan, ScanMode, VfoScan, ScanStart, ScanEnd, BeaconSymbol, ClearAprs, AprsMessages, MenuTimeout, PowerCalibration, PowerMeasured, PowerClear, BandProfile, PowerStatus, MicGain, Monitor, AutoLock, FmPresets, AmPresets, About, FmStep, AmStep, DistanceUnits, Waypoint, Distance, Backup, Reset, Import, KeyBinding, ConfirmStore, ChannelCommands, Weather };
enum class KeyAction : uint8_t { None, Monitor, Fm, NextSlot, Mode, Gps, Aprs, Channels, ScanChannels, ScanVfo, Weather, Count };
enum class SpeedUnit : uint8_t { Kmh, Mph, Knots };
enum class Option : uint8_t { Watch, Gps, AprsRx, Beacon, Ble, Export, SidePtt, SecondPtt, Tail, Count };
enum class MenuGroup : uint8_t { Root, Radio, Channels, Gps, Aprs, Bluetooth, Display, System, Diagnostics, AprsRx, Beacon, Repeater, UserKeys, Count };
struct MenuEntry {
    const char *name;
    Key action;
    Option toggle=Option::Count;
    MenuGroup group=MenuGroup::Root;
    RadioSetting radio_setting=RadioSetting::Count;
};
inline constexpr uint8_t watch_masks[]={0,7,3,5,6};

struct BeaconSymbol {const char *label;char code;};
inline constexpr BeaconSymbol beacon_symbols[]={{"Car",'>'},{"Person",'['},{"House",'-'},{"Bicycle",'b'}};
struct BeaconIdentity {char callsign[7]{};uint8_t ssid=0;};
enum class ScanMode : uint8_t { Time, Carrier, Search };
struct Memory {
    uint32_t rx_hz=0,tx_hz=0;
    bool occupied=false;
    char name[9]{};
    RadioProfile radio{};
    uint8_t zone=0;
};
struct Display {
    char title[30];
    char line[9][30];
};

// UI state is independent of GPIO, the vendor firmware and the radio driver.
// All storage is bounded and owned by this object; no dynamic allocation.
class Application final {
public:
    static constexpr unsigned MemoryCount = 64, ZoneCount = 15;
    explicit Application(bool presets=true);
    void key(Key key);
    void tick(uint32_t now);
    void activity(uint32_t now) {last_activity_=now;screen_awake_=true;}
    bool screen_awake() const { return screen_awake_; }
    bool keypad_locked() const {return keypad_locked_;}
    bool lcd_test_gray() const { return lcd_test_gray_; }
    unsigned brightness() const { return brightness_; }
    unsigned screen_seconds() const { return screen_seconds_; }
    inline static constexpr uint16_t menu_timeouts[]={0,10,30,60,120};
    inline static constexpr const char *menu_timeout_labels[]={"OFF","10 seconds","30 seconds","60 seconds","120 seconds"};
    MicGain mic_gain() const {return mic_gain_;}
    MicGain mic_gain_draft() const {return mic_gain_draft_;}
    unsigned power_status_page() const {return power_status_page_;}
    TxPower band_power() const {return band_power_;}
    unsigned band_channel() const {return band_channel_;}
    unsigned band_draft_channel() const {return editor_digits_?editor_value_:band_channel_;}
    uint32_t band_frequency() const {return preset_frequency(band_power_,band_draft_channel());}
    const MeasuredPower &measured_power() const {return measured_power_;}
    void power_battery(unsigned mv) {power_battery_mv_=uint16_t(mv);}
    unsigned power_battery() const {return power_battery_mv_;}
    uint8_t power_code() const {return power_code_;}
    unsigned power_code_limit() const {return power_limit_;}
    void power_code_limit(unsigned limit) {power_limit_=uint8_t(limit<255?limit:0);}
    bool power_test(unsigned slot) const {
        return page_==Page::PowerCalibration && power_test_ && slot==config_.active &&
            power_hz_==config_.vfo[slot].tx_hz && power_kind_==radio_profile(slot).power;
    }
    unsigned menu_timeout_selection() const {return menu_timeout_;}
    unsigned menu_timeout_draft() const {return menu_timeout_draft_;}
    unsigned auto_lock_selection() const {return auto_lock_;}
    unsigned auto_lock_draft() const {return auto_lock_draft_;}
    int timezone_minutes() const { return timezone_minutes_; }
    int timezone_draft() const { return timezone_draft_; }
    unsigned aprs_message_index() const {return aprs_message_;}
    unsigned beacon_symbol_index() const {return beacon_symbol_;}
    unsigned beacon_symbol_draft() const {return beacon_symbol_draft_;}
    bool take_aprs_clear() {const bool pending=aprs_clear_;aprs_clear_=false;return pending;}
    const BeaconIdentity &beacon_identity() const {return beacon_identity_;}
    const BeaconIdentity &identity_draft() const {return identity_draft_;}
    unsigned identity_cursor() const {return identity_cursor_;}
    bool import_confirm() const {return import_confirm_;}
    bool import_ready() const {return import_ready_;}
    void import_ready(bool ready){if(ready!=import_ready_)import_confirm_=false;import_ready_=ready;}
    bool take_import_request(){const bool value=import_request_;import_request_=false;return value;}
    bool take_import_apply(){const bool value=import_apply_;import_apply_=false;return value;}
    bool import_snapshot(const uint8_t *data,size_t size,uint32_t now,unsigned schema=1);
    void user_key(unsigned which,bool held);
    KeyAction key_action(unsigned which) const {return which<4?key_actions_[which]:KeyAction::None;}
    unsigned binding_selection() const {return binding_selection_;}
    unsigned binding_draft() const {return binding_draft_;}
    static constexpr const char *key_action_labels[]={"OFF","Monitor","FM radio","Next slot","VFO / Channel","GPS status","APRS status","Channel list","Scan zone channels","Scan VFO","NOAA"};
    static constexpr const char *key_binding_labels[]={"PF1 short","PF1 hold","PF2 short","PF2 hold"};
    static_assert(sizeof(key_action_labels)/sizeof(key_action_labels[0])==unsigned(KeyAction::Count));
    static constexpr unsigned PortableSchema=2,PortableSize=8008;
    size_t save_portable_snapshot(uint8_t *data,size_t capacity) const;
    bool restore_portable_snapshot(const uint8_t *data,size_t size,unsigned schema);
    bool reset_all() const {return reset_all_;}
    bool reset_confirm() const {return reset_confirm_;}
    bool take_backup_request(){const bool requested=backup_request_;backup_request_=false;return requested;}
    void backup_ptt(){if(page_==Page::Backup || page_==Page::Import){backup_request_=import_request_=import_apply_=false;import_ready_=import_confirm_=false;page_=Page::Radio;notify("TRANSFER CANCELLED");}}
    DistanceUnit distance_unit() const {return distance_unit_;}
    DistanceUnit distance_draft() const {return distance_draft_;}
    const GeoPoint &waypoint() const {return waypoint_;}
    bool waypoint_clear() const {return waypoint_clear_;}
    bool waypoint_confirm() const {return waypoint_confirm_;}
    bool decimal_coordinates() const {return decimal_coordinates_;}
    bool decimal_draft() const {return coordinate_draft_==1;}
    bool dms_coordinates() const {return dms_coordinates_;}
    unsigned coordinate_selection() const {return dms_coordinates_?2u:unsigned(decimal_coordinates_);}
    unsigned coordinate_draft() const {return coordinate_draft_;}
    SpeedUnit speed_unit() const {return speed_unit_;}
    SpeedUnit speed_draft() const {return speed_draft_;}
    bool altitude_feet() const {return altitude_feet_;}
    bool altitude_draft_feet() const {return altitude_draft_feet_;}
    Language language() const {return language_;}
    Language language_draft() const {return language_draft_;}
    static constexpr uint32_t vfo_steps[]={2500,5000,6250,10000,12500,20000,25000,50000,100000};
    static constexpr unsigned vfo_step_count=sizeof(vfo_steps)/sizeof(vfo_steps[0]);
    unsigned vfo_step_selection() const {return vfo_step_draft_;}
    unsigned applied_vfo_step() const {return vfo_step_[config_.active];}
    const RadioProfile &active_radio_profile() const;
    const RadioProfile &radio_profile(unsigned slot) const;
    RadioSetting radio_setting() const {return radio_setting_;}
    inline static constexpr uint16_t fm_steps[]={5,10,20}; // 10 kHz units
    inline static constexpr uint16_t am_steps[]={1,5,9,10}; // kHz
    unsigned tuner_step_draft() const {return tuner_step_draft_;}
    unsigned tuner_step_selection(bool am) const {return am?am_step_:fm_step_;}
    unsigned tuner_step(bool am) const {return am?am_steps[am_step_]:fm_steps[fm_step_];}
    static constexpr unsigned BroadcastCount=8;
    unsigned broadcast_selection() const {return broadcast_selection_;}
    bool broadcast_am() const {return page_==Page::AmPresets;}
    bool broadcast_confirm() const {return broadcast_action_!=0;}
    bool broadcast_deleting() const {return broadcast_action_==2;}
    uint16_t broadcast_frequency(unsigned i) const {
        return i<BroadcastCount?(broadcast_am()?am_presets_[i]:fm_presets_[i]):0;
    }
    uint16_t broadcast_source() const {return broadcast_am()?sw_request_:fm_request_;}
    uint16_t fm_request() const {return sw_am_?sw_request_:fm_request_;}
    WeatherRx &weather() {return weather_;}
    const WeatherRx &weather() const {return weather_;}
    void weather_ptt() {if(page_==Page::Weather)page_=Page::Radio;}
    bool sw_am() const {return sw_am_;}
    const char *fm_entry() const {return fm_entry_;}
    bool monitor_enabled() const {return monitor_enabled_;}
    bool monitor_draft() const {return monitor_draft_;}
    bool monitor_active() const {return monitor_enabled_ && page_==Page::Radio && !editing();}
    bool fm_audio() const {return fm_audio_;}
    bool fm_seek() const {return fm_seek_;}
    void fm_seek_result(uint16_t found);
    unsigned radio_draft() const {return radio_draft_;}
    unsigned editor_value() const { return editor_value_; }
    unsigned watch_slots_draft() const {return watch_slots_draft_;}
    unsigned watch_slots_selection() const;
    bool side_ptt_enabled() const {return (aux_ptt_&1u)!=0;}
    bool second_ptt_enabled() const {return (aux_ptt_&2u)!=0;}
    bool watch_timer_editor() const {return page_==Page::WatchDwell || page_==Page::WatchHold;}
    bool frequency_editor() const {
        return page_==Page::AprsRxFrequency || page_==Page::BeaconFrequency ||
               page_==Page::TxFrequency || page_==Page::RepeaterOffset || scan_bound_editor();
    }
    uint32_t profile_tx_frequency() const;
    const char *tx_status() const;
    bool reverse_active(unsigned slot) const;
    unsigned editor_digits() const { return editor_digits_; }
    unsigned active_zone() const {return active_zone_;}
    unsigned zone_selection() const {return zone_selection_;}
    const char *zone_name(unsigned i) const {return zone_names_[i];}
    bool channel_visible(unsigned i) const;
    bool scan_eligible(unsigned i) const;
    bool scan_page() const {return page_==Page::ChannelScan || page_==Page::VfoScan;}
    bool voice_tx_page(bool probe=false) const {
        return (page_==Page::Radio && !editing()) || page_==Page::Aprs || page_==Page::AprsMessages ||
            scan_page() || (page_==Page::PowerCalibration && probe);
    }
    uint32_t scan_vfo_frequency() const {return vfo_hz_[config_.active];}
    const RadioProfile &scan_vfo_profile() const {return vfo_profiles_[config_.active];}
    unsigned scan_channel() const {return scan_channel_;}
    uint32_t scan_frequency() const {return scan_frequency_;}
    bool scan_bound_editor() const {return page_==Page::ScanStart || page_==Page::ScanEnd;}
    uint32_t scan_bound(bool upper) const {return upper?scan_end_:scan_start_;}
    bool scan_limits(uint32_t anchor,uint32_t &first,uint32_t &last) const;
    bool scan_reverse() const {return scan_reverse_;}
    bool take_scan_step() {const bool requested=scan_step_;scan_step_=false;return requested;}
    ScanMode scan_mode() const {return scan_mode_;}
    ScanMode scan_mode_draft() const {return scan_mode_draft_;}
    void pause_scan() {scan_paused_=true;}
    bool scan_paused() const {return scan_paused_;}
    void scan_channel(unsigned index) {scan_target(index,index<MemoryCount?memories_[index].rx_hz:0);}
    void scan_target(unsigned index,uint32_t hz) {
        scan_channel_=index;scan_frequency_=hz;
        if(!hz)scan_paused_=false;
    }
    Key channel_action() const {return channel_action_;}
    unsigned channel_command() const {return channel_command_;}
    static constexpr const char *channel_commands[]={"Rename channel","Move channel","Delete channel","Copy to VFO"};
    static constexpr unsigned ChannelCommandCount=sizeof channel_commands/sizeof channel_commands[0];
    const char *name_draft() const {return name_draft_;}
    unsigned name_cursor() const {return name_cursor_;}
    unsigned channel_selection() const { return channel_; }
    void digit(unsigned digit);
    void observe_voice_slot(uint8_t slot,uint32_t hz);
    int recent_voice_slot() const;
    bool set_frequency(uint32_t hz);
    bool store(unsigned index);
    bool recall(unsigned index);
    bool set_aprs_frequency(uint32_t hz);
    bool set_option(Option option, bool enabled);
    static constexpr unsigned SnapshotSchema=1;
    static constexpr size_t SnapshotCapacity=8192;
    size_t save_snapshot(uint8_t *data,size_t capacity) const; // Legacy canonical words, retained for migration/export.
    size_t save_compact_snapshot(uint8_t *data,size_t capacity) const;
    size_t compact_snapshot_size() const;
    size_t canonical_snapshot_size() const;
    static bool snapshot_format_supported(const uint8_t *data,size_t size);
    bool restore_snapshot(const uint8_t *data,size_t size);
    void report_save(const char *message) {notify(message);}
    void render(Display &out) const;
    const rt_config &config() const { return config_; }
    const Memory *memory(unsigned i) const { return i < MemoryCount ? &memories_[i] : nullptr; }
    Page page() const { return page_; }
    bool power_save() const { return power_save_; }
    const char *feedback() const { return notice_; }
    void entry_text(char *out) const;
    unsigned menu_selection() const { return menu_selection_[unsigned(menu_group_)]; }
    MenuGroup menu_group() const { return menu_group_; }
    unsigned menu_count() const;
    const MenuEntry &menu_entry(unsigned index) const;
    bool enabled(Option option) const;
    bool editing() const { return digits_ != 0; }
    uint32_t entered_khz() const { return entry_khz_; }
    static bool frequency_valid(uint32_t hz);
    static bool valid_scan_bounds(uint32_t first,uint32_t last);
private:
    rt_config config_{};
    WeatherRx weather_{};
    RadioProfile vfo_profiles_[3]{};
    RadioSetting radio_setting_=RadioSetting::Squelch;
    unsigned radio_draft_=0;
    Language language_=Language::English,language_draft_=Language::English;
    uint8_t vfo_step_[3]={4,4,4},vfo_step_draft_=4;
    bool altitude_feet_=false,altitude_draft_feet_=false;
    SpeedUnit speed_unit_=SpeedUnit::Kmh,speed_draft_=SpeedUnit::Kmh;
    uint32_t scan_excluded_[2]{};
    unsigned scan_channel_=MemoryCount;
    uint32_t scan_frequency_=0,scan_start_=0,scan_end_=0;
    bool scan_paused_=false,scan_reverse_=false,scan_step_=false;
    uint32_t last_voice_hz_=0,last_voice_at_=0;
    uint8_t last_voice_slot_=3;
    ScanMode scan_mode_=ScanMode::Carrier,scan_mode_draft_=ScanMode::Carrier;
    uint8_t watch_slots_draft_=0;
    uint8_t aux_ptt_=0;
    MicGain mic_gain_=MicGain::High,mic_gain_draft_=MicGain::High;
    MeasuredPower measured_power_{};
    TxPower band_power_=TxPower::Pmr500mW;
    uint8_t band_channel_=1,power_status_page_=0;
    uint16_t power_battery_mv_=0;
    uint32_t power_hz_=0;
    TxPower power_kind_=TxPower::Low;
    uint8_t power_code_=1;
    uint8_t power_limit_=0;
    bool power_test_=false;
    uint8_t menu_timeout_=2,menu_timeout_draft_=2;
    uint8_t beacon_symbol_=0,beacon_symbol_draft_=0;
    bool aprs_clear_=false,tx_tail_=true;
    uint8_t aprs_message_=0;
    BeaconIdentity beacon_identity_{},identity_draft_{};
    unsigned identity_cursor_=0;
    bool decimal_coordinates_=false,dms_coordinates_=false;
    DistanceUnit distance_unit_=DistanceUnit::Kilometres,distance_draft_=DistanceUnit::Kilometres;
    GeoPoint waypoint_{};
    bool waypoint_clear_=false,waypoint_confirm_=false;
    KeyAction key_actions_[4]={KeyAction::None,KeyAction::None,KeyAction::Fm,KeyAction::None};
    uint8_t binding_selection_=0,binding_draft_=0;
    bool backup_request_=false;
    bool import_ready_=false,import_confirm_=false,import_request_=false,import_apply_=false;
    bool reset_all_=false,reset_confirm_=false; // Runtime only; never restored armed.
    void apply_defaults(bool all);
    uint8_t coordinate_draft_=0;
    uint8_t fm_step_=1,am_step_=1,tuner_step_draft_=1;
    Memory memories_[MemoryCount]{};
    uint32_t vfo_hz_[3]{};
    uint32_t vfo_tx_hz_[3]{};
    Page page_ = Page::Radio;
    uint16_t fm_request_=10030,fm_memory_=0,sw_request_=7100;
    uint16_t fm_presets_[BroadcastCount]{},am_presets_[BroadcastCount]{};
    uint8_t broadcast_selection_=0,broadcast_action_=0;
    Page broadcast_parent_=Page::Settings;
    char fm_entry_[6]{};
    uint8_t fm_digits_=0;
    bool monitor_enabled_=false,monitor_draft_=false;
    bool fm_audio_=false,fm_seek_=false,sw_am_=false;
    Page parent_=Page::Settings;
    const char *notice_="";
    uint32_t now_=0,notice_since_=0;
    unsigned channel_=0;
    unsigned active_zone_=0,zone_selection_=0,name_cursor_=0;
    char zone_names_[ZoneCount][9]{},name_draft_[9]{};
    Key channel_action_=Key::Channels;
    uint8_t channel_command_=0;
    bool channel_context_=false;
    void notify(const char *message);
    uint8_t menu_selection_[unsigned(MenuGroup::Count)]{};
    MenuGroup menu_group_=MenuGroup::Root;
    uint32_t entry_khz_ = 0;
    uint8_t digits_ = 0;
    bool invalid_ = false, lcd_test_gray_ = false;
    bool power_save_ = true,screen_awake_=true;
    bool keypad_locked_=false; // Runtime lock; reboot always starts unlocked.
    uint8_t auto_lock_=0,auto_lock_draft_=0; // Same duration list as menu timeout; default OFF.
    unsigned brightness_=4,screen_seconds_=60,editor_value_=0,editor_digits_=0;
    int timezone_minutes_=180,timezone_draft_=180;
    uint32_t last_activity_=0;
    RadioProfile &editable_radio_profile();
    uint32_t base_rx(unsigned slot) const;
    uint32_t base_tx(unsigned slot) const;
    uint32_t candidate_tx(unsigned slot) const;
    void sync_frequencies();
    void reconcile_channels();
    void select_channel_for_list();
    void open_name_editor(const char *name);
    void step(int direction);
    void cancel_entry();
    template<class Archive,class App> static void snapshot_fields(Archive &,App &);
    template<class Archive,class App> static void full_snapshot_fields(Archive &,App &);
    template<class Archive,class App> static void portable_snapshot_fields(Archive &,App &);
    bool valid_snapshot() const;
};
} // namespace rt950
