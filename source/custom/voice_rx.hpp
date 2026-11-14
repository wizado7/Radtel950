// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#include "radio_settings.hpp"
#include "voice_sql.hpp"

namespace rt950::voice {
struct RegisterValue { uint8_t address;uint16_t value; };
// Fixed startup writes from OEM 0.29 0x0800A028..0x0800A194.
// Per-unit bits of register 0x40 are read before being updated below.
inline constexpr RegisterValue rx_startup[]={
    {0x00,0x8000},{0x00,0},{0x37,0x9d1f},{0x13,0x03df},{0x12,0x03db},
    {0x11,0x033a},{0x10,0x0318},{0x14,0x0210},{0x49,0x2ab2},{0x7b,0x73dc},
    {0x1c,0x07c0},{0x1d,0xe555},{0x1e,0x4c58},{0x1f,0x865a},{0x3e,0x94c6},
    {0x3f,0x07fe},{0x25,0xc1ba},{0x3a,0x9a7c},{0x19,0x1041},{0x28,0x0b40},
    {0x29,0xaa00},{0x2a,0x6600},{0x2c,0x0022},{0x2f,0x9890},{0x53,0x2028},
    {0x7e,0x303e},{0x46,0x6050},{0x4a,0x5430},{0x48,0xb3bf},{0x49,0x2ab2},
    {0x4a,0x5430},{0x4d,0xa004},{0x4e,0x3815},{0x4f,0x3f3b},{0x77,0xccef},
    {0x7e,0x303e},
};
inline constexpr RegisterValue rx_finish[]={
    {0x7d,0xe912},{0x48,0xb3ff},{0x74,0xdf22},{0x44,0x91c1},{0x45,0x3040},
    {0x75,0xe61c},{0x54,0x91c1},{0x55,0x3040},{0x47,0x6042},
};
// OEM 0.29 0x0801D51C / 0x0801D548 test register 0x0C bit 1.
constexpr bool hardware_sql_open(uint16_t status) {return (status&2u)!=0;}

// RX timing starts after the frequency registers are written.
inline constexpr uint32_t RxRestartMs=1;
inline constexpr uint32_t RxSettleMs=30;
inline constexpr uint32_t RxSampleMs=15;
inline constexpr unsigned RxToneSamples=3;

struct VoiceStatus {
    bool ready=false,fault=false,receiving=false,sampled=false;
    uint8_t slot=0;
    uint16_t rssi=0,status=0;
    uint32_t frequency=0;
};
class BoardReceiver final {
public:
    void enable(bool enabled);
    void tune(uint32_t hz,uint32_t now);
    void tune_profile(uint32_t hz,Tone tone,uint8_t level,uint32_t now,bool narrow,uint8_t sql);

    void stop();
    void set_profile(Tone tone,uint8_t level,uint32_t now,bool narrow=true,uint8_t sql=3);
    void use_sql_calibration(const SqlCalibration &calibration) {sql_calibration_=&calibration;}
    bool squelch_open(uint32_t now);
    enum class WeatherAudio : uint8_t {Off, Muted, Speaker};
    void poll(uint32_t now,bool ptt,bool monitor=false,bool packet_audio=false,WeatherAudio weather=WeatherAudio::Off);
    bool carrier() const {return carrier_;}
    bool has_sample() const {return status_.sampled;}
    const VoiceStatus &status() const {return status_;}
private:
    VoiceStatus status_{};
    uint32_t due_=0,next_sample_=0;
    unsigned init_step_=0;
    uint8_t rx_chip_=0;
    bool active_=false,restart_=false,audio_=false;
    Tone tone_{};
    uint8_t tone_level_=0,tone_matches_=0,tone_programmed_=0;
    bool narrow_=true,carrier_=false,tone_ready_=true;
    const SqlCalibration *sql_calibration_=nullptr;
    uint8_t sql_=3;
    bool sql_open_=false,speaker_=false;
    void audio(bool enabled,bool speaker=false);
};
}
