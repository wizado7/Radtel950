// SPDX-License-Identifier: GPL-3.0-or-later
#include "application.hpp"
#include "storage_record.hpp"

namespace rt950 {
namespace {
struct Writer {
    uint8_t *data;size_t capacity,pos=0;bool good=true;
    template<class T> void field(T value) {
        if(!good || capacity-pos<4){good=false;return;}
        const uint32_t word=uint32_t(value);
        for(unsigned i=0;i<4;++i)data[pos++]=uint8_t(word>>(8*i));
    }
};
struct Reader {
    const uint8_t *data;size_t capacity,pos=0;bool good=true;
    template<class T> void field(T &value) {
        if(!good || capacity-pos<4){good=false;return;}
        uint32_t word=0;
        for(unsigned i=0;i<4;++i)word|=uint32_t(data[pos++])<<(8*i);
        value=static_cast<T>(word);
        if(uint32_t(value)!=word)good=false;
    }
};
// Compact container v1: magic, version, canonical word count, encoded length,
// CRC32 over the first 16 header bytes and encoded bytes, then zero alignment.
constexpr uint32_t PackedMagic=0x32535452u,PackedVersion=10;
constexpr size_t PackedHeader=20;
// Changing array capacities requires a separate historical decoder, not a
// silent reinterpretation of the released v1/v2 canonical field order.
static_assert(Application::MemoryCount==64 && Application::ZoneCount==15 && Application::BroadcastCount==8);
static_assert(Application::PortableSize==8008+31*4);
struct WordCounter {
    size_t words=0;
    template<class T> void field(T){++words;}
};
struct PackedWriter {
    uint8_t *data;size_t capacity,pos=PackedHeader,words=0;bool good=true;
    template<class T> void field(T input) {
        if(!good)return;
        uint32_t value=uint32_t(input);++words;
        do {
            if(pos>=capacity){good=false;return;}
            uint8_t byte=uint8_t(value&127u);value>>=7;
            if(value)byte|=128u;
            if(data)data[pos]=byte;
            ++pos;
        }while(value);
    }
};
struct PackedReader {
    const uint8_t *data;size_t end,pos=PackedHeader,words=0;bool good=true;
    template<class T> void field(T &output) {
        if(!good)return;
        uint32_t value=0;
        for(unsigned i=0;i<5;++i) {
            if(pos>=end){good=false;return;}
            const uint8_t byte=data[pos++];
            if(i==4 && (byte&0xf0u)){good=false;return;}
            value|=uint32_t(byte&127u)<<(7u*i);
            if(!(byte&128u)) {
                if(i && !byte){good=false;return;} // No overlong encodings.
                output=static_cast<T>(value);
                if(uint32_t(output)!=value)good=false;
                ++words;return;
            }
        }
        good=false;
    }
};
uint32_t packed_checksum(const uint8_t *data,size_t encoded) {
    return ~storage::crc_bytes(storage::crc_bytes(0xffffffffu,data,16),data+PackedHeader,encoded);
}
template<class Archive,class Profile>
void profile_fields(Archive &archive,Profile &p) {
    archive.field(p.offset_hz);archive.field(p.shift);archive.field(p.reverse);archive.field(p.tx_inhibit);
    archive.field(p.rx_tone.kind);archive.field(p.rx_tone.index);
    archive.field(p.tx_tone.kind);archive.field(p.tx_tone.index);
    archive.field(p.timeout_seconds);archive.field(p.squelch);archive.field(p.power);
    archive.field(p.narrow);archive.field(p.busy_lock);
}
bool valid_tone(Tone t) {
    switch(t.kind) {
    case ToneKind::Off:return t.index==0;
    case ToneKind::Ctcss:return t.index>0 && t.index<radio_option_count(RadioSetting::RxCtcss);
    case ToneKind::Dcs:return t.index>0 && t.index<radio_option_count(RadioSetting::RxDcs);
    }
    return false;
}
bool valid_profile(const RadioProfile &p) {
    bool timeout=false;
    for(auto seconds:tx_timeouts)if(seconds==p.timeout_seconds)timeout=true;
    return p.offset_hz<=99999990u && unsigned(p.shift)<=unsigned(Shift::Minus) &&
           p.squelch<=9 && unsigned(p.power)<=unsigned(TxPower::Pmr500mW) && timeout &&
           valid_tone(p.rx_tone) && valid_tone(p.tx_tone);
}
bool valid_name(const char *name,size_t size,bool dash,bool empty=false) {
    for(size_t i=0;i<size;++i) {
        const char c=name[i];
        if(!c)return i || empty;
        if((c<'A' || c>'Z') && (c<'0' || c>'9') && (!dash || c!='-'))return false;
    }
    return false;
}
}

template<class Archive,class App>
void Application::snapshot_fields(Archive &a,App &s) {
    a.field(s.language_);a.field(s.brightness_);a.field(s.screen_seconds_);
    a.field(s.timezone_minutes_);a.field(s.power_flags_);a.field(s.active_zone_);
    a.field(s.altitude_feet_);a.field(s.speed_unit_);a.field(s.decimal_coordinates_);
    auto &c=s.config_;
    a.field(c.active);a.field(c.watch_mode);a.field(c.watch_mask);
    a.field(c.dwell_ms);a.field(c.hold_ms);a.field(c.gps_on);
    a.field(c.aprs_rx);a.field(c.ble_on);a.field(c.ble_export);
    a.field(c.aprs_rx_hz);a.field(c.aprs_hz);a.field(c.beacon_seconds);
    // Automatic TX permissions are deliberately excluded from the snapshot.
    for(auto &ch:s.beacon_identity_.callsign)a.field(ch);
    a.field(s.beacon_identity_.ssid);
    for(unsigned i=0;i<3;++i) {
        a.field(s.vfo_hz_[i]);a.field(s.vfo_tx_hz_[i]);a.field(s.vfo_step_[i]);
        a.field(c.vfo[i].memory_channel);a.field(c.vfo[i].channel_mode);
        profile_fields(a,s.vfo_profiles_[i]);
    }
    for(auto &name:s.zone_names_)for(auto &ch:name)a.field(ch);
    for(auto &m:s.memories_) {
        a.field(m.occupied);a.field(m.rx_hz);a.field(m.tx_hz);a.field(m.zone);
        for(auto &ch:m.name)a.field(ch);
        profile_fields(a,m.radio);
    }
}

template<class Archive,class App>
void Application::full_snapshot_fields(Archive &archive,App &s) {
    snapshot_fields(archive,s);
    archive.field(s.fm_memory_);
    archive.field(s.config_.aprs_scan_seconds);
    archive.field(s.aux_ptt_);
    for(auto &excluded:s.scan_excluded_)archive.field(excluded);
    archive.field(s.scan_mode_);archive.field(s.scan_start_);archive.field(s.scan_end_);
    archive.field(s.beacon_symbol_);archive.field(s.tx_tail_);
    archive.field(s.menu_timeout_);
    for(auto &point:s.measured_power_.points) {
        archive.field(point.hz);archive.field(point.battery_mv);archive.field(point.measured_mw);
        archive.field(point.code);archive.field(point.power);
    }
    archive.field(s.mic_gain_);
    archive.field(s.auto_lock_);
    for(auto &hz:s.fm_presets_)archive.field(hz);
    for(auto &hz:s.am_presets_)archive.field(hz);
    archive.field(s.fm_step_);archive.field(s.am_step_);archive.field(s.dms_coordinates_);
    archive.field(s.distance_unit_);archive.field(s.waypoint_.latitude);archive.field(s.waypoint_.longitude);archive.field(s.waypoint_.valid);
}
template<class Archive,class App>
void Application::portable_snapshot_fields(Archive &archive,App &s) {
    full_snapshot_fields(archive,s);
    for(auto &action:s.key_actions_)archive.field(action);
}
template<class Archive,class App>
void Application::aprs_routing_fields(Archive &archive,App &s) {
    auto &routing=s.aprs_routing_;
    for(auto &hop:routing.hops) {
        for(auto &ch:hop.callsign)archive.field(ch);
        archive.field(hop.ssid);
    }
    archive.field(routing.beacon);archive.field(routing.relay);
    archive.field(routing.relay_use_beacon);
    archive.field(routing.delay);archive.field(routing.preamble);
}
template<class Archive,class App>
void Application::beacon_fields(Archive &archive,App &s) {
    archive.field(s.beacon_format_);archive.field(s.beacon_overlay_);archive.field(s.beacon_mode_);
}
template<class Archive,class App>
void Application::vox_fields(Archive &archive,App &s) {
    archive.field(s.vox_level_);archive.field(s.vox_delay_);
}
template<class Archive,class App>
void Application::display_fields(Archive &a,App &s) {
    a.field(s.light_theme_);a.field(s.show_voltage_);a.field(s.slot_ratio_);
}
size_t Application::save_portable_snapshot(uint8_t *data,size_t capacity) const {
    if(!data)return 0;
    Writer writer{data,capacity};portable_snapshot_fields(writer,*this);aprs_routing_fields(writer,*this);
    writer.field(beacon_altitude_code_);beacon_fields(writer,*this);vox_fields(writer,*this);display_fields(writer,*this);writer.field(large_font_);
    return writer.good?writer.pos:0;
}
bool Application::restore_portable_snapshot(const uint8_t *data,size_t size,unsigned schema) {
    if(!data || !portable_layout_supported(size,schema) || storage::word(data)>1u)return false;
    Application candidate(false);
    if(!candidate.restore_snapshot(data,7992))return false;
    if(schema>=2) {
        Reader reader{data+7992,size-7992};
        for(auto &action:candidate.key_actions_)reader.field(action);
        if(schema>=3)aprs_routing_fields(reader,candidate);
        if(schema>=4)reader.field(candidate.beacon_altitude_code_);
        if(schema>=6)beacon_fields(reader,candidate);
        if(schema>=7)vox_fields(reader,candidate);
        if(schema>=9)display_fields(reader,candidate);
        if(schema>=10)reader.field(candidate.large_font_);
        if(!reader.good || reader.pos!=reader.capacity || !candidate.valid_snapshot())return false;
    }
    if(schema<8)candidate.upgrade_pmr_presets();
    *this=candidate;return true;
}
size_t Application::save_snapshot(uint8_t *data,size_t capacity) const {
    if(!data)return 0;
    Writer writer{data,capacity};full_snapshot_fields(writer,*this);
    return writer.good?writer.pos:0;
}
size_t Application::canonical_snapshot_size() const {
    WordCounter counter;portable_snapshot_fields(counter,*this);aprs_routing_fields(counter,*this);
    counter.field(beacon_altitude_code_);beacon_fields(counter,*this);vox_fields(counter,*this);display_fields(counter,*this);counter.field(large_font_);return counter.words*4u;
}
size_t Application::compact_snapshot_size() const {
    PackedWriter writer{nullptr,SnapshotCapacity};portable_snapshot_fields(writer,*this);aprs_routing_fields(writer,*this);
    writer.field(beacon_altitude_code_);beacon_fields(writer,*this);vox_fields(writer,*this);display_fields(writer,*this);writer.field(large_font_);
    const size_t size=(writer.pos+3u)&~size_t(3);
    return writer.good && size<=SnapshotCapacity?size:0;
}
size_t Application::save_compact_snapshot(uint8_t *data,size_t capacity) const {
    if(!data || capacity<PackedHeader)return 0;
    PackedWriter writer{data,capacity};portable_snapshot_fields(writer,*this);aprs_routing_fields(writer,*this);
    writer.field(beacon_altitude_code_);beacon_fields(writer,*this);vox_fields(writer,*this);display_fields(writer,*this);writer.field(large_font_);
    const size_t size=(writer.pos+3u)&~size_t(3);
    if(!writer.good || size>capacity)return 0;
    storage::put(data,PackedMagic);storage::put(data+4,PackedVersion);
    storage::put(data+8,uint32_t(writer.words));storage::put(data+12,uint32_t(writer.pos-PackedHeader));
    storage::put(data+16,packed_checksum(data,writer.pos-PackedHeader));
    for(size_t i=writer.pos;i<size;++i)data[i]=0;
    return size;
}

bool Application::snapshot_format_supported(const uint8_t *data,size_t size) {
    return data && size>=4 && (storage::word(data)!=PackedMagic ||
        (size>=8 && storage::word(data+4)>=1 && storage::word(data+4)<=PackedVersion));
}
bool Application::restore_snapshot(const uint8_t *data,size_t size) {
    if(!data || size>SnapshotCapacity)return false;
    Application candidate(false);
    if(size>=4 && storage::word(data)==PackedMagic) {
        if(size<PackedHeader || size%4 || !snapshot_format_supported(data,size))return false;
        const auto version=storage::word(data+4);
        const size_t encoded=storage::word(data+12);
        if(encoded>size-PackedHeader || size!=((PackedHeader+encoded+3u)&~size_t(3)) ||
           storage::word(data+16)!=packed_checksum(data,encoded))return false;
        for(size_t i=PackedHeader+encoded;i<size;++i)if(data[i])return false;
        PackedReader reader{data,PackedHeader+encoded};
        if(version==1)full_snapshot_fields(reader,candidate);
        else portable_snapshot_fields(reader,candidate);
        if(version>=3)aprs_routing_fields(reader,candidate);
        if(version>=4)reader.field(candidate.beacon_altitude_code_);
        if(version>=6)beacon_fields(reader,candidate);
        if(version>=7)vox_fields(reader,candidate);
        if(version>=9)display_fields(reader,candidate);
        if(version>=10)reader.field(candidate.large_font_);
        if(!reader.good || reader.pos!=reader.end || reader.words!=storage::word(data+8) || !candidate.valid_snapshot())return false;
        candidate.config_.ble_export=false;candidate.reconcile_channels();
        if(version<8)candidate.upgrade_pmr_presets();
        if(!rt_config_valid(&candidate.config_))return false;
        *this=candidate;return true;
    }
    Reader reader{data,size};
    snapshot_fields(reader,candidate);
    const size_t remaining=reader.capacity-reader.pos;
    const size_t broadcast_size=52+MeasuredPower::Capacity*20+BroadcastCount*8;
    const bool has_navigation=remaining==broadcast_size+28;
    const bool has_tuner_steps=remaining==broadcast_size+12 || has_navigation;
    const bool has_broadcast=remaining==broadcast_size || has_tuner_steps;
    const bool has_auto_lock=remaining==52+MeasuredPower::Capacity*20 || has_broadcast;
    const bool has_mic=remaining==48+MeasuredPower::Capacity*20 || has_auto_lock;
    const bool has_power=remaining==44+MeasuredPower::Capacity*20 || has_mic;
    const bool has_menu_timeout=remaining==44 || has_power;
    const bool has_tail=remaining==40 || has_menu_timeout;
    const bool has_symbol=remaining==36 || has_tail;
    const size_t tail=has_symbol?32:remaining;
    if(reader.good && (tail==4 || tail==8 || tail==12 || tail==20 || tail==24 || tail==32))reader.field(candidate.fm_memory_);
    if(reader.good && (tail==8 || tail==12 || tail==20 || tail==24 || tail==32))reader.field(candidate.config_.aprs_scan_seconds);
    if(reader.good && (tail==12 || tail==20 || tail==24 || tail==32))reader.field(candidate.aux_ptt_);
    if(reader.good && (tail==20 || tail==24 || tail==32))
        for(auto &excluded:candidate.scan_excluded_)reader.field(excluded);
    if(reader.good && (tail==24 || tail==32))reader.field(candidate.scan_mode_);
    if(reader.good && tail==32){reader.field(candidate.scan_start_);reader.field(candidate.scan_end_);}
    if(reader.good && has_symbol)reader.field(candidate.beacon_symbol_);
    if(reader.good && has_tail)reader.field(candidate.tx_tail_);
    if(reader.good && has_menu_timeout)reader.field(candidate.menu_timeout_);
    if(reader.good && has_power)for(auto &point:candidate.measured_power_.points) {
        reader.field(point.hz);reader.field(point.battery_mv);reader.field(point.measured_mw);
        reader.field(point.code);reader.field(point.power);
    }
    if(reader.good && has_mic)reader.field(candidate.mic_gain_);
    if(reader.good && has_auto_lock)reader.field(candidate.auto_lock_);
    if(reader.good && has_broadcast) {
        for(auto &hz:candidate.fm_presets_)reader.field(hz);
        for(auto &hz:candidate.am_presets_)reader.field(hz);
    } else candidate.fm_presets_[0]=candidate.fm_memory_;
    if(reader.good && has_tuner_steps) {
        reader.field(candidate.fm_step_);reader.field(candidate.am_step_);reader.field(candidate.dms_coordinates_);
    }
    if(reader.good && has_navigation) {
        reader.field(candidate.distance_unit_);reader.field(candidate.waypoint_.latitude);
        reader.field(candidate.waypoint_.longitude);reader.field(candidate.waypoint_.valid);
    }
    if(!reader.good || reader.pos!=size || !candidate.valid_snapshot())return false;
    // Old snapshots may contain the previous, automatically enabled export default.
    candidate.config_.ble_export=false;
    candidate.reconcile_channels();
    if(!rt_config_valid(&candidate.config_))return false;
    *this=candidate;
    return true;
}

// Validate into a separate object before atomically replacing live state.
// Device-specific measured RF points never come from an uploaded file.
bool Application::import_snapshot(const uint8_t *data,size_t size,uint32_t now,unsigned schema) {
    Application candidate(false);
    if(!candidate.restore_portable_snapshot(data,size,schema))return false;
    candidate.measured_power_=measured_power_;
    candidate.config_.beacon_on=false;candidate.config_.ble_export=false;
    if(!candidate.compact_snapshot_size())return false;
    candidate.page_=Page::Settings;candidate.menu_group_=MenuGroup::System;
    candidate.now_=candidate.last_activity_=now;
    *this=candidate;notify("SETTINGS IMPORTED");return true;
}

bool Application::valid_snapshot() const {
    if(power_flags_>5 || slot_ratio_<66 || slot_ratio_>160 || config_.gps_on>RT_GPS_ON)return false;
    for(auto action:key_actions_)if(unsigned(action)>=unsigned(KeyAction::Count))return false;
    for(const auto &hop:aprs_routing_.hops)if(!aprs::valid_address(hop,true))return false;
    if(aprs_routing_.beacon>aprs::BeaconPath::Custom || aprs_routing_.relay>aprs::RelayMode::Wide ||
       aprs_routing_.delay>=sizeof aprs::relay_delays/sizeof *aprs::relay_delays ||
       aprs_routing_.preamble>=sizeof aprs::tx_flags/sizeof *aprs::tx_flags)return false;
    if(unsigned(distance_unit_)>unsigned(DistanceUnit::NauticalMiles) || !geo_valid(waypoint_) ||
       beacon_altitude_code_>MaxBeaconAltitudeMetres+1u || beacon_format_>=aprs::BeaconFormat::Count ||
       beacon_overlay_>=aprs::BeaconOverlayCount || beacon_mode_>aprs::BeaconMode::Smart)return false;
    if(fm_step_>=sizeof fm_steps/sizeof fm_steps[0] || am_step_>=sizeof am_steps/sizeof am_steps[0] ||
       (decimal_coordinates_ && dms_coordinates_))return false;
    for(auto hz:fm_presets_)if(hz && (hz<6500 || hz>10800))return false;
    for(auto hz:am_presets_)if(hz && fm::am_band(hz)==fm::Band::Fm)return false;
    if(auto_lock_>=sizeof menu_timeouts/sizeof menu_timeouts[0])return false;
    if(unsigned(mic_gain_)>unsigned(MicGain::High) || vox_level_<1 || vox_level_>9 ||
       vox_delay_>=voice::VoxDelayCount)return false;
    for(const auto &point:measured_power_.points)if(!MeasuredPower::valid(point))return false;
    if(menu_timeout_>=sizeof menu_timeouts/sizeof menu_timeouts[0])return false;
    if(beacon_symbol_>=BeaconSymbolCount)return false;
    if(aux_ptt_>3 || scan_mode_>ScanMode::Search || !valid_scan_bounds(scan_start_,scan_end_))return false;
    if(fm_memory_ && (fm_memory_<6500 || fm_memory_>10800))return false;
    if(unsigned(language_)>unsigned(Language::Russian) || brightness_<1 || brightness_>BrightnessCount ||
       screen_seconds_>3600 || timezone_minutes_<-720 || timezone_minutes_>840 ||
       timezone_minutes_%15 || active_zone_>=ZoneCount ||
       unsigned(speed_unit_)>unsigned(SpeedUnit::Knots) || beacon_identity_.ssid>15 ||
       !valid_name(beacon_identity_.callsign,sizeof beacon_identity_.callsign,false,true))return false;
    if((config_.aprs_rx_hz && !frequency_valid(config_.aprs_rx_hz)) ||
       (config_.aprs_hz && !frequency_valid(config_.aprs_hz)) ||
       (config_.ble_export && !config_.ble_on))return false;
    for(unsigned i=0;i<3;++i) {
        if(vfo_step_[i]>=vfo_step_count || config_.vfo[i].memory_channel>=MemoryCount ||
           !valid_profile(vfo_profiles_[i]) ||
           (vfo_hz_[i] && !frequency_valid(vfo_hz_[i])) ||
           (vfo_tx_hz_[i] && (!vfo_hz_[i] || !frequency_valid(vfo_tx_hz_[i]))))return false;
    }
    for(const auto &name:zone_names_)if(!valid_name(name,sizeof name,true))return false;
    for(const auto &m:memories_) {
        if(m.zone>=ZoneCount || !valid_profile(m.radio))return false;
        if(m.occupied && (!frequency_valid(m.rx_hz) || (m.tx_hz && !frequency_valid(m.tx_hz)) ||
                         !valid_name(m.name,sizeof m.name,true)))return false;
    }
    return true;
}
}
