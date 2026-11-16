// SPDX-License-Identifier: GPL-3.0-or-later
#include "application.hpp"
#include "settings_flash.hpp"
#include "settings_export.hpp"
#include "settings_import.hpp"
#include "fm_board.hpp"
#include "voice_rx.hpp"
#include "voice_watch.hpp"
#ifdef RT950_BENCH_TX_HZ
#include "voice_tx_board.hpp"
#ifdef RT950_MAIN_TX
#include "aprs_beacon.hpp"
#include "aprs_dac.hpp"
#include "aprs_adc.hpp"
#include "mcu_diagnostics.hpp"
#include "aprs_background.hpp"
#include "channel_scan.hpp"
#include "voice_battery_limits.hpp"
#include "aprs_ble.hpp"
#endif
#endif
#include "controls.hpp"
#include "board.h"
#include "ui.hpp"
extern "C" {
#include "drivers/lcd.h"
#include "app/encoder.h"
#include "../diagnostic/nmea_status.h"
#include "drivers/gpio.h"
uint32_t get_tick(void);
void delay_ms(uint32_t);
extern uint8_t custom_reset_flags;
}
using namespace rt950;
#ifdef RT950_MAIN_TX
static uint8_t held_ptt_source,enabled_ptt_inputs;
static uint16_t session_power_mv;
static bool tail_hold;
#endif
extern "C" int main(void) {
#ifdef RT950_MAIN_TX
    McuTemperature mcu_temperature;
#endif
    SCB->SCR &= ~6u; // shallow sleep; never inherit SLEEPDEEP/SLEEPONEXIT
    board_init();board_battery_init(get_tick());lcd_init();encoder_init();
    Application app;load_settings(app);
    board_display_init();ui_boot_screen();board_display_brightness((uint8_t)app.brightness());delay_ms(400);
    SettingsAutosave autosave;autosave.begin(app,get_tick());Controls keys;PowerSwitch power;WakeGuard wake_keys;
    // Recovery route to diagnostics even if the matrix labels are wrong.
    // A held key at boot always enters diagnostics.
    if(board_ptt_mask()) app.key(Key::Debug);
    board_gps_enable(app.config().gps_on);
    uint32_t last_input=get_tick(),last_draw=last_input;
    uint32_t last_encoder=last_input;
    bool keys_awake=true;
    fm::BoardProbe fm_probe;
    voice::BoardReceiver voice_rx;voice::Watch voice_watch;
#ifdef RT950_BENCH_TX_HZ
    voice::initialize_factory_spi();
    voice::UhfCalibration factory;
    voice::CalibrationStatus factory_status;
    const bool factory_valid=voice::read_board_uhf_calibration(factory,&factory_status);
#ifdef RT950_BENCH_MATRIX
    voice::VhfPaCalibration vhf;
    const bool vhf_valid=factory_valid && voice::read_board_vhf_calibration(vhf);
#endif
#ifdef RT950_MAIN_TX
    voice::SqlCalibration sql_calibration;
    if(voice::read_board_sql_calibration(sql_calibration))voice_rx.use_sql_calibration(sql_calibration);
    voice::BatteryTxLimits battery_limits;
    const bool battery_limits_factory=factory_valid && voice::read_board_battery_limits(battery_limits);
    voice::CbPaCalibration cb;
    const bool cb_valid=factory_valid && voice::read_board_cb_calibration(cb);
#endif
    voice::BoardUhfTx tx_board(voice_rx,factory,voice::factory_power_selector(TxPower::Low),[] {
#ifdef RT950_MAIN_TX
        if(board_power_switch_off())return false;
        if(tail_hold)return voice::tail_inputs_released(board_ptt_mask(),enabled_ptt_inputs);
        return voice::ptt_source(board_ptt_mask()&enabled_ptt_inputs)==held_ptt_source &&
            held_ptt_source;
#else
        return (board_ptt_mask()&1u) && !board_power_switch_off();
#endif
    }
#ifdef RT950_BENCH_MATRIX
    ,vhf_valid?&vhf:nullptr
#endif
#ifdef RT950_MAIN_TX
    ,cb_valid?&cb:nullptr
#endif
    );
    voice::UhfTransmitter<voice::BoardUhfTx> transmitter(tx_board);
    voice::Ptt ptt;
#ifdef RT950_MAIN_TX
    voice::BoardUhfTx packet_board(voice_rx,factory,voice::factory_power_selector(TxPower::Low),[] {
        return !board_ptt_mask() && !board_power_switch_off();
    },vhf_valid?&vhf:nullptr,nullptr,true);
    voice::UhfTransmitter<voice::BoardUhfTx> packet_rf(packet_board);
    aprs::DacBurst packet_audio;
    aprs::AdcCapture packet_rx;
    aprs::BleExport ble_export;
    aprs::BackgroundListen aprs_background(get_tick());
    voice::ChannelScan channel_scan;
    bool scan_rx_owned=false;
    uint32_t last_gps_export=get_tick()-2000u;
    aprs::FrameTransmitter packet_frame;
    aprs::BeaconSession<decltype(packet_rf),aprs::DacBurst> packet_tx(packet_rf,packet_audio);
    aprs::RelayQueue relay;
    bool packet_is_relay=false,packet_battery_pending=false;
    uint32_t last_beacon=get_tick(),packet_tx_hz=0;
#endif
#endif
    static SettingsExport<BackupUart> settings_export;
    static SettingsImport<BackupUart> settings_import;
    InputStatus input;bool gps=app.config().gps_on,display_awake=true,wake_encoder=false;
#ifdef RT950_MAIN_TX
    input.power.reset_flags=custom_reset_flags;
#endif
#ifdef RT950_BENCH_TX_HZ
    input.factory_ctcss_raw=factory.ctcss_raw;
    input.factory_ctcss_level=factory.ctcss_level;
#endif
#ifdef RT950_MAIN_TX
    uint32_t release_notice_since=0;
#endif
#ifdef RT950_MAIN_TX
    bool weather_previous=false,frequency_copy_previous=false;
#endif
    app.activity(get_tick());
    custom_heartbeat=get_tick();custom_watchdog_armed=1;
    for(;;) {
        const uint32_t now=get_tick();custom_heartbeat=now;
        // A held PTT is activity even at the exact auto-lock deadline.
        const uint8_t raw_ptt=board_ptt_mask();
        if(raw_ptt)app.activity(now);
#ifdef RT950_MAIN_TX
        const bool rf_session=packet_tx.active() || ptt.state()==voice::Ptt::State::Preparing ||
            ptt.state()==voice::Ptt::State::Transmitting || ptt.state()==voice::Ptt::State::Ending;
        board_battery_fast(raw_ptt || rf_session || packet_battery_pending);
#endif
        app.tick(now);
        board_battery_poll(now);
        const bool switch_off=board_power_switch_off()!=0;
        if(power.poll(switch_off,now)) {
            settings_export.cancel();settings_import.cancel();
#ifdef RT950_BENCH_TX_HZ
#ifdef RT950_MAIN_TX
            ble_export.enable(false,now);
            packet_rx.stop();
            packet_tx.stop(now);relay.cancel();
#endif
            ptt.poll(transmitter,{},now,false);
#endif
            fm_probe.poll(false,now);
            voice_rx.enable(false);
            autosave.poll(app,now,true,board_ptt_mask()!=0);
            board_power_cut();
        }
        if(uint32_t(now-last_input)>=10u) {
            board_gps_poll(now);
            last_input=now;input.sampled=board_scan_key();const uint8_t code=keys.poll(input.sampled,now);
            app.update_monitor_hold(keys.accepted());
            if(input.sampled!=255){if(!app.screen_awake())wake_keys.begin();app.activity(now);}
            keys_awake=wake_keys.poll(input.sampled,now);
            if(code!=255) {
                input.last=uint8_t(code&127u);++input.presses;
                const bool select_during_ptt=raw_ptt && code==5 && app.page()==Page::Radio;
                if(keys_awake && !select_during_ptt)dispatch_key(app,code);
            }
        }
        if(uint32_t(now-last_encoder)>=1u) {
            last_encoder=now;const int8_t dir=encoder_poll();
            input.encoder_ab=encoder_state();const uint32_t edges=encoder_edges();
            if(edges!=input.edges){if(!app.screen_awake())wake_encoder=true;app.activity(now);}
            input.edges=edges;input.invalid_edges=encoder_invalid_edges();input.encoder_lost=encoder_overruns();
            if(dir) {if(dir>0)++input.cw;else ++input.ccw;if(!wake_encoder)app.key(dir>0?Key::Up:Key::Down);wake_encoder=false;}
        }
        input.ptt_mask=board_ptt_mask();
        if(!app.side_ptt_enabled())input.ptt_mask&=3u;
        if(input.ptt_mask) {
#ifdef RT950_MAIN_TX
            if(input.ptt_mask&uint8_t(1u|(app.second_ptt_enabled()?2u:0u)|(app.side_ptt_enabled()?4u:0u))) {
                app.weather_ptt();
                app.tone_search_ptt();
                app.spectrum_ptt();
            }
#endif
            app.backup_ptt();settings_export.cancel();settings_import.cancel();
            app.activity(now);
            if(app.page()==Page::FmProbe && app.fm_audio())app.key(Key::Back);
        }
        if(app.page()==Page::Debug) {
            if(input.sampled==10)board_radio_led(1,0);
            else if(input.sampled==11)board_radio_led(0,1);
            else if(input.sampled==17)board_radio_led(0,0);
        }
        if(!switch_off) {
            bool rf_busy=input.ptt_mask!=0;
#ifdef RT950_MAIN_TX
            rf_busy=rf_busy || packet_tx.active();
#endif
            if(const char *result=autosave.poll(app,now,false,rf_busy || settings_export.active() || settings_import.active() || app.page()==Page::Backup || app.page()==Page::Import))app.report_save(result);
        }
        input.save_state=autosave.state();
        if(app.take_aprs_clear())input.aprs_stations.clear();
        if(app.take_aprs_messages_clear())input.aprs_messages.clear();
        const Page page=app.page();
        const bool fm_open=page==Page::FmProbe;
#ifdef RT950_MAIN_TX
        const bool weather_open=page==Page::Weather;
        const bool spectrum_open=page==Page::Spectrum;
        const bool frequency_copy_open=page==Page::FrequencyCopy;
        if(frequency_copy_previous && !frequency_copy_open) {
            if(voice_rx.status().fault)voice_rx.enable(false);
            else voice_rx.stop();
            voice_watch.poll(voice_rx,app.config(),now,false,false);
        }
        frequency_copy_previous=frequency_copy_open;
        if(weather_open!=weather_previous) {
            voice_watch.poll(voice_rx,app.config(),now,false,false);
            if(voice_rx.status().fault)voice_rx.enable(false);
            else voice_rx.stop();
            weather_previous=weather_open;
        }
#endif
        const bool voice_enabled=!switch_off && !fm_open &&
            page!=Page::Debug && page!=Page::LcdTest && page!=Page::FlashInfo;
        input.monitor_active=app.monitor_active();
        const bool monitor=input.monitor_active;
        bool tx_owns_radio=false,transmitting=false;
#ifdef RT950_BENCH_TX_HZ
        const auto &config=app.config();
#ifdef RT950_MAIN_TX
        enabled_ptt_inputs=uint8_t(1u|(app.second_ptt_enabled()?2u:0u)|(app.side_ptt_enabled()?4u:0u));
        const uint8_t source=voice::ptt_source(input.ptt_mask&enabled_ptt_inputs);
        const bool tx_session=ptt.state()==voice::Ptt::State::Preparing ||
            ptt.state()==voice::Ptt::State::Transmitting || ptt.state()==voice::Ptt::State::Ending;
        const uint8_t tx_slot=!source && tx_session?ptt.target_slot():voice::ptt_slot(source,config.active);
#else
        const uint8_t tx_slot=config.active;
#endif
        const auto &slot=config.vfo[tx_slot];
        const auto &profile=app.radio_profile(tx_slot);
#ifdef RT950_MAIN_TX
        ble_export.enable(config.ble_on && !switch_off,now);
        const bool export_allowed=config.ble_on && config.ble_export && !switch_off;
        ble_export.poll(export_allowed,now);
        const bool ptt_down=board_ptt_mask()!=0;
        const bool power_sample_ok=board_battery_sample_fresh(now);
        const uint8_t battery_raw=board_battery_raw();
        const auto battery_age=board_battery_sample_age(now);
        const bool supply_ready=tx_supply_ready(battery_age);
        const auto supply_fault=classify_supply(power_sample_ok && (!rf_session || tx_supply_live(battery_age)),
            battery_raw,battery_limits.low,battery_limits.High);
        const bool power_tx_ok=supply_fault==SupplyFault::None;
        // Capture the reason before a voice or APRS stop removes TX ownership.
        if(!power_tx_ok)input.power.trace.trip(supply_fault);
        app.power_battery(power_sample_ok?board_battery_mv():0);
        const bool aprs_allowed=config.aprs_rx && voice_enabled && !weather_open && !ptt_down && !monitor &&
            page!=Page::PowerCalibration && page!=Page::PowerMeasured &&
            ptt.state()==voice::Ptt::State::Idle && !packet_tx.active();
        const bool background=aprs_background.poll(now,aprs_allowed && page==Page::Radio,
            voice_rx.status().ready && voice_rx.has_sample() && !voice_rx.carrier() &&
            !voice_rx.status().receiving,uint32_t(config.aprs_scan_seconds)*1000u);
        const bool aprs_page=page==Page::Aprs || page==Page::AprsMessages;
        const bool relay_page=aprs_page || (page==Page::Radio && !app.editing());
        bool aprs_listen=background || (aprs_allowed && (aprs_page || (relay_page && relay.pending())));
        if(!aprs_listen)packet_rx.stop();
        // Release ADC1 before the APRS capture below may start in this same loop.
        mcu_temperature.poll(page==Page::PowerStatus && app.power_status_page()==2 &&
            !aprs_listen && !ptt_down && !packet_tx.active() && !switch_off,now);
        input.power.mcu_degrees=mcu_temperature.reading(now);
        const auto &identity=app.beacon_identity();
        input.aprs_messages.select_identity(identity.callsign,identity.ssid);
        const uint32_t packet_hz=config.aprs_hz;
        const bool saved_position=app.beacon_saved_point();
        const bool position_ready=saved_position?app.waypoint().valid:config.gps_on && nav_fix_fresh(now);
        const auto calibrated_packet=[&](uint32_t hz) {
            return hz%10u==0 && (voice::vhf_tx_frequency(hz)?
                vhf_valid && vhf.bias(hz,voice::factory_power_selector(TxPower::Low)):
                factory_valid && voice::uhf_tx_frequency(hz));
        };
        const auto &routing=app.aprs_routing();
        const uint32_t relay_hz=routing.relay_use_beacon?packet_hz:config.aprs_rx_hz;
        const bool packet_permitted=voice_enabled && !ptt_down && !monitor && power_tx_ok &&
            ptt.state()==voice::Ptt::State::Idle;
        const bool beacon_allowed=packet_permitted && page==Page::Radio && config.beacon_on &&
            position_ready && identity.callsign[0] && calibrated_packet(packet_hz);
        const bool relay_allowed=packet_permitted && relay_page && config.aprs_rx &&
            app.enabled(Option::AprsRelay) && calibrated_packet(relay_hz);
        const auto begin_packet=[&](uint32_t hz,bool is_relay) {
            packet_rx.stop();
            aprs_listen=false;
            voice_watch.poll(voice_rx,config,now,false,false);
            packet_tx_hz=hz;packet_is_relay=is_relay;
            packet_tx.start(hz,now);
        };
        if(packet_tx.active() && (packet_is_relay?
            !relay_allowed || packet_tx_hz!=relay_hz:!beacon_allowed || packet_tx_hz!=packet_hz))
            packet_tx.stop(now);
        const auto &current_rx=voice_rx.status();
        const bool relay_channel_clear=aprs_listen && current_rx.sampled && !current_rx.fault &&
            current_rx.frequency==config.aprs_rx_hz && !voice_rx.carrier();
        if(relay.take(packet_frame,now,relay_allowed && !packet_tx.active(),relay_channel_clear && supply_ready,
            aprs::tx_flags[routing.preamble]))begin_packet(relay_hz,true);
        const bool beacon_due=!packet_tx.active() && !relay.pending() && beacon_allowed &&
            !aprs_listen && !voice_rx.carrier() && current_rx.ready &&
            uint32_t(now-last_beacon)>=config.beacon_seconds*1000u;
        packet_battery_pending=!packet_tx.active() && !supply_ready &&
            (beacon_due || (relay_allowed && relay.pending()));
        if(beacon_due && supply_ready) {
            last_beacon=now;
            const auto &nav=*nav_get();
            const char symbol=beacon_symbols[app.beacon_symbol_index()].code;
            const bool encoded=saved_position?
                aprs::encode_beacon(packet_frame,identity.callsign,identity.ssid,app.waypoint(),symbol,routing):
                aprs::encode_beacon(packet_frame,identity.callsign,identity.ssid,
                    nav.lat,nav.ns[0],nav.lon,nav.ew[0],symbol,aprs::beacon_altitude(nav,now),routing);
            if(encoded)begin_packet(packet_hz,false);
        }
        if(packet_tx.active())packet_tx.poll(packet_frame,now,packet_is_relay?relay_allowed:beacon_allowed);
#endif
#ifdef RT950_MAIN_TX
        const bool vhf_tx=voice::vhf_tx_frequency(slot.tx_hz);
        const bool cb_tx=voice::cb_fm_frequency(slot.tx_hz);
        const bool frequency_allowed=cb_tx?
            slot.rx_hz==slot.tx_hz && profile.shift==Shift::None && profile.power==TxPower::Low &&
            profile.tx_tone.kind==ToneKind::Off && profile.rx_tone.kind==ToneKind::Off:
            vhf_tx?voice::vhf_tx_frequency(slot.rx_hz):
            voice::uhf_tx_frequency(slot.tx_hz) && voice::uhf_tx_frequency(slot.rx_hz);
        const uint8_t selector=voice::factory_power_selector(profile.power);
        const bool numeric=numeric_power(profile.power);
        const unsigned pa_index=voice::uhf_pa_index(slot.tx_hz);
        const uint8_t factory_low=factory_valid && pa_index<13?factory.pa[2][pa_index]:0;
        app.power_code_limit(factory_low);
        // Select a record at the unkeyed battery voltage. Normal battery
        // protection remains live during TX; load sag must not select another
        // PA code in the middle of a session.
        if(ptt.state()==voice::Ptt::State::Idle)session_power_mv=uint16_t(app.power_battery());
        const bool power_probe=app.power_test(tx_slot) && (source==1 || (!source && tx_session));
        const auto measured=resolve_target_power(app.measured_power(),profile,slot.rx_hz,slot.tx_hz,
            session_power_mv,factory_low,power_probe,app.power_code());
        const uint8_t measured_code=measured.code;
        const bool numeric_ready=measured.ready;
        if(ptt.state()==voice::Ptt::State::Idle) {
            tx_board.select_power(selector);
            tx_board.select_measured_bias(numeric?slot.tx_hz:0,measured_code);
        }
        const bool power_allowed=selector<3 && tx_board.power_selector()==selector && numeric_ready &&
            tx_board.measured_bias()==measured_code && tx_board.measured_frequency()==(numeric?slot.tx_hz:0);
        input.power_status=!numeric?"":!numeric_ready?"POWER CALIBRATION NEEDED":
            profile.power==TxPower::Lpd10mW?"10 mW TARGET / RECORD":"0.5 W TARGET / RECORD";
        const bool calibration_valid=cb_tx?cb_valid && cb.bias(slot.tx_hz,selector):vhf_tx?
            vhf_valid && vhf.bias(slot.tx_hz,selector):factory_valid;
#else
        const bool simplex=slot.tx_hz==slot.rx_hz && profile.shift==Shift::None;
        bool frequency_allowed=simplex && slot.rx_hz==RT950_BENCH_TX_HZ;
        bool power_allowed=profile.power==TxPower::Low;
#ifdef RT950_BENCH_MATRIX
        frequency_allowed=frequency_allowed ||
            (simplex && (slot.rx_hz==430025000u || slot.rx_hz==460025000u)) ||
            (slot.rx_hz==430025000u && slot.tx_hz==430625000u &&
             profile.shift==Shift::Plus && profile.power==TxPower::Low);
        if(slot.rx_hz==145500000u)
            frequency_allowed=simplex && profile.power==TxPower::Low;
        const uint8_t selector=voice::factory_power_selector(profile.power);
        if(ptt.state()==voice::Ptt::State::Idle)tx_board.select_power(selector);
        power_allowed=selector<3 && tx_board.power_selector()==selector;
#endif
        bool calibration_valid=factory_valid;
#ifdef RT950_BENCH_MATRIX
        if(slot.tx_hz==145500000u)
            calibration_valid=vhf_valid && vhf.bias(slot.tx_hz,tx_board.power_selector());
#endif
#endif
#ifdef RT950_MAIN_TX
        if(ptt.state()==voice::Ptt::State::Idle) {
            tx_board.select_tone(profile.tx_tone);
            tx_board.select_bandwidth(profile.narrow);
            tx_board.select_microphone(app.mic_gain());
        }
#endif
        const bool tone_allowed=tx_board.permits_tone(profile.tx_tone)
#ifndef RT950_MAIN_TX
            && profile.rx_tone.kind==ToneKind::Off
#endif
            ;
        const bool allowed=calibration_valid && voice_enabled &&
#ifdef RT950_MAIN_TX
            app.voice_tx_page(power_probe) && power_tx_ok &&
#else
            page==Page::Radio &&
#endif
            frequency_allowed &&
#ifdef RT950_MAIN_TX
            (!cb_tx || profile.narrow) &&
#else
            profile.narrow &&
#endif
            !profile.tx_inhibit &&
            !profile.reverse && tone_allowed &&
#ifdef RT950_MAIN_TX
            tx_board.microphone_gain()==app.mic_gain() &&
#endif
            power_allowed;
#ifdef RT950_MAIN_TX
        const bool pressed=source!=0;
        if(ptt.state()==voice::Ptt::State::Idle)held_ptt_source=source;
        // Keep a finite software limit even when the configured TOT is off.
        const uint16_t timeout=power_probe?5:profile.timeout_seconds?profile.timeout_seconds:300;
        const bool tail_enabled=app.enabled(Option::Tail) && !power_probe && !cb_tx && held_ptt_source!=0;
        tail_hold=!pressed && allowed && tail_enabled &&
            (ptt.state()==voice::Ptt::State::Transmitting || ptt.state()==voice::Ptt::State::Ending);
        const voice::TxRequest request{slot.tx_hz,timeout,tx_slot,
            allowed && (source==held_ptt_source || tail_hold),profile.busy_lock,tail_enabled};
#else
        const bool pressed=(input.ptt_mask&1u)!=0;
        const voice::TxRequest request{slot.tx_hz,15,tx_slot,allowed,true};
#endif
#ifdef RT950_MAIN_TX
        const bool waiting_for_battery=pressed && allowed &&
            ptt.state()==voice::Ptt::State::Idle && !supply_ready;
#else
        constexpr bool waiting_for_battery=false;
#endif
        if(pressed && allowed && ptt.state()==voice::Ptt::State::Idle && !waiting_for_battery)
            voice_watch.poll(voice_rx,config,now,false,false);
        // After a preempted packet the shared receiver may still be restarting.
#ifdef RT950_MAIN_TX
        const bool was_transmitting=ptt.state()==voice::Ptt::State::Transmitting ||
            ptt.state()==voice::Ptt::State::Ending;
#endif
        if(!waiting_for_battery && (!pressed || !allowed || ptt.state()!=voice::Ptt::State::Idle || voice_rx.status().ready))
            ptt.poll(transmitter,request,now,pressed);
#ifdef RT950_MAIN_TX
        if(!pressed && was_transmitting && ptt.release_error()!=voice::Ptt::ReleaseError::None)
            release_notice_since=now;
#endif
        transmitting=ptt.state()==voice::Ptt::State::Transmitting || ptt.state()==voice::Ptt::State::Ending;
        input.voice_tx_slot=transmitting?ptt.target_slot():0xff;
        tx_owns_radio=transmitting || ptt.state()==voice::Ptt::State::Preparing;
#ifdef RT950_MAIN_TX
        constexpr const char *tx_labels[]={"TX SLOT A","TX SLOT B","TX SLOT C"};
        const char *tx_on=power_probe?"POWER TEST / UNMEASURED":tx_labels[tx_slot];
#else
        constexpr const char *tx_on="BENCH TX ON";
#endif
#ifdef RT950_MAIN_TX
        if(packet_tx.active()) {
            tx_owns_radio=true;
            transmitting=packet_tx.transmitting();
        }
#endif
        const char *calibration_error="TX NO CALIBRATION";
#ifdef RT950_MAIN_TX
        constexpr const char *factory_errors[]={"TX NO CALIBRATION","CAL SPI UNAVAILABLE",
            "CAL SPI READ FAILED","CAL MARKER INVALID","CAL PA TABLE INVALID","CAL MIC INVALID"};
        if(!factory_valid)calibration_error=factory_errors[unsigned(factory_status)];
#endif
        input.bench_ptt=ptt.state()==voice::Ptt::State::Ending?"TX ENDING":!pressed?"":transmitting?tx_on:tx_owns_radio?"TX PREPARING":
#ifdef RT950_MAIN_TX
            page==Page::Radio && app.editing()?"TX: FINISH FREQ INPUT":
#endif
            profile.tx_inhibit?"TX INHIBITED":
            !slot.tx_hz?"TX FREQ UNSET":
            !calibration_valid?calibration_error:
#ifdef RT950_MAIN_TX
            !power_tx_ok?supply_fault_text(supply_fault):
            waiting_for_battery?"TX CHECKING BATTERY":
            numeric && !numeric_ready?"POWER CALIBRATION NEEDED":
#endif
            !allowed?"TX PROFILE BLOCKED":ptt.blocked_by_carrier()?"TX CHANNEL BUSY":"TX BLOCKED / RELEASE";
#ifdef RT950_MAIN_TX
        constexpr const char *release_errors[]={"","TX END NOT PERMITTED","TX END PROFILE CHANGED",
            "TX END TIME LIMIT","TX END RADIO BLOCKED","TX END SIGNAL FAILED"};
        if(!pressed && ptt.state()==voice::Ptt::State::Idle &&
           ptt.release_error()!=voice::Ptt::ReleaseError::None && uint32_t(now-release_notice_since)<2000u) {
            input.bench_ptt=release_errors[unsigned(ptt.release_error())];
            if(ptt.release_error()==voice::Ptt::ReleaseError::Signalling) {
                constexpr const char *tail_errors[]={"TX END SIGNAL FAILED","TX END NO RF STATE",
                    "TX END LEVEL INVALID","TX END WRITE BLOCKED"};
                input.bench_ptt=tail_errors[unsigned(tx_board.tail_failure())];
            }
        }
        if(packet_tx.transmitting())input.bench_ptt=packet_is_relay?"APRS RELAY TX":"APRS TX";
        auto &diagnostics=input.power;
        diagnostics.low=battery_limits.low;diagnostics.high=battery_limits.High;
        diagnostics.factory_limits=battery_limits_factory;
        diagnostics.profile_check=profile.tx_inhibit?"TX INHIBITED":!slot.tx_hz?"TX FREQ UNSET":
            !calibration_valid?calibration_error:!power_tx_ok?supply_fault_text(supply_fault):
            !frequency_allowed?"TX OUT OF BAND":profile.reverse?"TX PROFILE BLOCKED":
            !tone_allowed?"TX TONE INVALID":!power_allowed?"POWER CALIBRATION NEEDED":
            cb_tx && !profile.narrow?"TX PROFILE BLOCKED":"STATIC CHECKS PASS";
        const bool packet_transmitting=packet_tx.transmitting();
        const uint8_t commanded_code=(CRM->APB1EN&CRM_APB1EN_DACEN)?uint8_t(DAC->DHR8R2):0;
        diagnostics.trace.observe(now,transmitting,packet_transmitting?packet_tx_hz:slot.tx_hz,
            packet_transmitting?TxSupplyTrace::Owner::Aprs:TxSupplyTrace::Owner::Voice,
            commanded_code,board_battery_mv(),board_battery_sample_age(now),power_sample_ok);
#endif
#endif
        input.power.battery_mv=board_battery_mv();
        input.power.raw12=board_battery_raw12();input.power.raw8=board_battery_raw();
        input.power.adc_age=board_battery_sample_age(now);input.power.adc_errors=board_battery_errors();
        input.power.fresh=board_battery_sample_fresh(now)!=0;
        if(!voice_enabled) {
            voice_watch.poll(voice_rx,app.config(),now,false,false);
            voice_rx.enable(false);
        }
        fm_probe.poll(fm_open,now,app.broadcast_band());
        if(app.page()==Page::FmProbe) {
            fm_probe.listen(app.fm_request(),app.fm_seek(),app.fm_audio(),now,app.fm_seek_up());
            if(app.fm_seek() && fm_probe.seek_finished())app.fm_seek_result(fm_probe.seek_result());
            input.fm_audio_enabled=fm_probe.audio_enabled();
            input.fm_audio_error=fm_probe.audio_error();
            const auto &state=fm_probe.state();
            input.fm_phase=state.phase();input.fm_failure=state.failure();
            input.fm_frequency=state.frequency();input.fm_part=state.part();
            input.fm_signal_ready=state.signal(now,input.fm_signal);
        }
#ifdef RT950_MAIN_TX
        const bool scanning=app.scan_page() && voice_enabled && !ptt_down && !tx_owns_radio;
        const bool scan_sample=scan_rx_owned && channel_scan.frequency() &&
            voice_rx.status().frequency==channel_scan.frequency() && voice_rx.has_sample();
        const unsigned previous_channel=app.scan_channel();
        const auto *previous_memory=app.memory(previous_channel);
        const auto &previous_profile=previous_memory?previous_memory->radio:app.scan_vfo_profile();
        const bool scan_open=previous_profile.squelch==0 && previous_profile.rx_tone.kind==ToneKind::Off?
            voice_rx.carrier():voice_rx.squelch_open(now);
        const unsigned scanned=channel_scan.poll(app,now,app.scan_page() && voice_enabled,
            scan_sample,scan_open,!scanning);
        app.scan_target(scanned,channel_scan.frequency());
        const bool scan_has_target=scanning && channel_scan.frequency()!=0;
        const bool tone_search=app.tone_searching() && !tx_owns_radio;
#endif
        if(voice_enabled && !tx_owns_radio) {
            voice_rx.enable(true);
#ifdef RT950_MAIN_TX
            const bool voice_target=!weather_open && !scan_has_target && !aprs_listen && !tone_search && !spectrum_open && !frequency_copy_open;
            const auto tone_level=[&](Tone tone) {
                if(tone.kind==ToneKind::Ctcss)return factory.ctcss_level;
                if(tone.kind==ToneKind::Dcs)return factory.dcs_level;
                return uint8_t(0);
            };
            if(voice_target) {
                const auto &current=app.radio_profile(voice_watch.slot());
                voice_rx.set_profile(current.rx_tone,tone_level(current.rx_tone),
                    now,current.narrow,current.squelch);
            }
            // Switch the frequency and RX profile together, before polling audio.
            voice_watch.poll(voice_rx,config,now,voice_target && !voice_rx.status().fault,
                ptt_down || monitor,[&](uint8_t rx_slot,uint32_t hz) {
                    const auto &rx_profile=app.radio_profile(rx_slot);
                    if(hz)voice_rx.tune_profile(hz,rx_profile.rx_tone,tone_level(rx_profile.rx_tone),
                        now,rx_profile.narrow,rx_profile.squelch);
                    else voice_rx.stop();
                });
            bool tone_scan_fault=false;
            if(tone_search) {
                const auto tone=app.tone_candidate();
                const auto level=tone_level(tone);
                const auto &rx_profile=app.active_radio_profile();
                tone_scan_fault=!level || level>127;
                if(app.take_tone_tune(now) && !tone_scan_fault)
                    voice_rx.tune_profile(config.vfo[config.active].rx_hz,tone,level,now,
                        rx_profile.narrow,rx_profile.squelch);
            }
            if(scan_has_target) {
                const auto *memory=app.memory(scanned);
                const auto &rx_profile=memory?memory->radio:app.scan_vfo_profile();
                if(!scan_rx_owned || voice_rx.status().frequency!=channel_scan.frequency() || scanned!=previous_channel)
                    voice_rx.tune_profile(channel_scan.frequency(),rx_profile.rx_tone,tone_level(rx_profile.rx_tone),
                        now,rx_profile.narrow,rx_profile.squelch);
            }
            if(spectrum_open) {
                if(const auto hz=app.spectrum().take_tune(now))
                    voice_rx.tune_profile(hz,{},0,now,app.active_radio_profile().narrow,0);
            }
            if(frequency_copy_open) {
                if(const auto hz=app.take_frequency_copy_start()) {
                    if(voice_rx.status().fault){voice_rx.enable(false);voice_rx.enable(true);}
                    voice_rx.start_frequency_copy(hz,now,app.active_radio_profile().narrow);
                }
            }
            if(weather_open && voice_rx.status().frequency!=app.weather().frequency())
                voice_rx.tune_profile(app.weather().frequency(),{},0,now,true,3);
            if(aprs_listen) {
                if(voice_rx.status().frequency!=config.aprs_rx_hz) {
                    packet_rx.stop();
                    voice_rx.tune_profile(config.aprs_rx_hz,{},0,now,true,3);
                } else voice_rx.set_profile({},0,now,true,3);
            }
            using WeatherAudio=voice::BoardReceiver::WeatherAudio;
            const auto wx_audio=!weather_open?WeatherAudio::Off:
                app.weather().scanning()?WeatherAudio::Muted:WeatherAudio::Speaker;
            voice_rx.poll(now,ptt_down || tone_search || spectrum_open || frequency_copy_open,monitor,aprs_listen,wx_audio);
#else
            voice_watch.poll(voice_rx,app.config(),now,!voice_rx.status().fault,input.ptt_mask!=0 || monitor);
            voice_rx.poll(now,input.ptt_mask!=0,monitor);
#endif
#ifdef RT950_MAIN_TX
            const auto &rx=voice_rx.status();
            if(frequency_copy_open) {
                app.frequency_copy_result(voice_rx.poll_frequency_copy(now));
            } else if(spectrum_open) {
                app.spectrum().observe(now,rx.frequency,rx.sampled && !rx.fault,rx.rssi);
            } else if(tone_search) {
                app.poll_tone_search(now,rx.sampled,voice_rx.squelch_open(now),rx.fault || tone_scan_fault);
            } else if(aprs_listen) {
                struct ReceiveContext {
                    InputStatus &status;
                    aprs::BleExport &ble;
                    aprs::RelayQueue *relay;
                    const aprs::Address &identity;
                    const aprs::Routing &routing;
                    uint32_t now;
                    bool export_allowed,background;
                } receive{input,ble_export,relay_allowed?&relay:nullptr,identity,routing,now,export_allowed,background};
                packet_rx.poll(rx.ready && rx.frequency==config.aprs_rx_hz && !rx.fault,now,
                    [](void *context,const uint8_t *frame,size_t size) {
                        size_t payload=0;
                        auto &received=*static_cast<ReceiveContext *>(context);
                        if(!aprs::ui_payload(frame,size,payload))return;
                        ++received.status.aprs_frames;
                        if(received.background)++received.status.aprs_background_frames;
                        received.status.aprs_stations.record(frame,size,received.now);
                        received.status.aprs_messages.record(frame,size);
                        if(received.relay)received.relay->receive(frame,size,received.identity,
                            received.routing.relay,received.now,aprs::relay_delays[received.routing.delay]);
                        if(received.export_allowed && !received.ble.enqueue(frame,size,received.now))
                            ++received.status.aprs_export_dropped;
                    },&receive);
            } else if(weather_open) {
                auto &wx=app.weather();
                const bool searching=wx.scanning();
                wx.poll(now,rx.frequency==wx.frequency() && rx.sampled,rx.receiving,rx.fault);
                if(searching && wx.alerted())app.activity(now);
            }
#endif
        }
#ifdef RT950_MAIN_TX
        scan_rx_owned=scan_has_target;
        app.aprs_message_counts(input.aprs_messages.count(),input.aprs_messages.count(true));
        input.aprs_receiving=packet_rx.running();
        input.aprs_windows=aprs_background.windows();
        input.aprs_cancelled=aprs_background.cancelled();
        if(export_allowed && uint32_t(now-last_gps_export)>=2000u) {
            last_gps_export=now;
            const aprs::GpsTelemetry gps_status(*nav_get(),config.gps_on,board_gps_bytes()!=0,now);
            ble_export.enqueue_gps(gps_status,now);
        }
#endif
        if(app.page()!=Page::Backup)settings_export.cancel();
        if(app.page()!=Page::Import)settings_import.cancel();
        const bool backup_allowed=app.page()==Page::Backup && !switch_off && !board_ptt_mask() && !transmitting;
        if(app.take_backup_request() && backup_allowed)settings_export.begin(app,now);
        settings_export.poll(now,backup_allowed);
        const bool import_allowed=app.page()==Page::Import && !switch_off && !board_ptt_mask() && !transmitting;
        if(app.take_import_request() && import_allowed && !settings_import.active())settings_import.begin(now);
        settings_import.poll(now,import_allowed);
        app.import_ready(settings_import.phase()==ImportPhase::Checked);
        if(app.take_import_apply() && import_allowed) {
            settings_import.apply(app,now);app.import_ready(false);ui_invalidate();
        }
        input.import_phase=settings_import.phase();input.import_received=settings_import.received();
        input.import_channels=uint16_t(settings_import.channels());
        input.backup_phase=settings_export.phase();input.backup_sent=settings_export.sent();input.backup_total=settings_export.total();
        input.voice=voice_rx.status();
#ifdef RT950_MAIN_TX
        // A packet carrier is not activity on the voice slot shown on the home screen.
        if(aprs_listen || tone_search || spectrum_open || frequency_copy_open)input.voice.receiving=false;
#endif
        input.voice.slot=voice_watch.slot();
#ifdef RT950_MAIN_TX
        if(page==Page::Radio && !raw_ptt && !tx_owns_radio && input.voice.ready && input.voice.receiving)
            app.observe_voice_slot(input.voice.slot,input.voice.frequency);
#endif
        if(page!=Page::Debug)board_radio_led(input.voice.receiving && !input.ptt_mask,transmitting);
        if(gps!=app.config().gps_on) {gps=app.config().gps_on;board_gps_enable(gps);}
        if(!app.screen_awake() && display_awake){board_display_sleep();display_awake=false;}
        if(app.screen_awake() && !display_awake){board_display_begin_wake();ui_invalidate();last_draw=now-200u;display_awake=true;}
#ifdef RT950_MAIN_TX
        using Load=WorkloadTime;
        const bool voice_listening=voice_enabled && !weather_open && !tx_owns_radio && !aprs_listen && !tone_search && !spectrum_open && !frequency_copy_open && input.voice.ready;
        const bool watching=voice_listening && !monitor && !scan_has_target && app.enabled(Option::Watch);
        input.power.workload.observe(now,
            (voice_listening && !watching && !scan_has_target?1u<<Load::SingleRx:0u) |
            (watching?1u<<Load::WatchRx:0u) | (gps?1u<<Load::Gps:0u) |
            (config.ble_on && !switch_off?1u<<Load::Ble:0u) |
            (packet_rx.running()?1u<<Load::AprsRx:0u) | (transmitting?1u<<Load::Tx:0u));
#endif
        if(app.screen_awake() && uint32_t(now-last_draw)>=200u) {
            last_draw=now;ui_draw(app,input,now);
            if(app.page()==Page::AprsMessages && app.aprs_personal())
                input.aprs_messages.read_personal(app.aprs_message_index());
        }
        board_display_brightness(app.screen_awake()?(uint8_t)app.brightness():0);
        // Sleep only the CPU, keep SysTick/UART and GPIO polling active.
        // SysTick wakes us every millisecond; the watchdog still checks main.
        if(app.power_save()) __asm volatile("dsb\n wfi" ::: "memory");
    }
}

