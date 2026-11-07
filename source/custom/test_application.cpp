// SPDX-License-Identifier: GPL-3.0-or-later
#include "application.hpp"
#include <cassert>
#include <cstring>
#include <cstdio>
using namespace rt950;
static void enter(Application &app, unsigned khz) {
    unsigned div=100000;
    for(unsigned i=0;i<6;++i,div/=10) app.digit(khz/div%10);
    app.key(Key::Ok);
}
int main() {
    Application app(false);
    assert(!app.config().beacon_on);
    assert(app.config().vfo[0].rx_hz==0);
    // Invalid and incomplete entries must never change a tuned frequency.
    enter(app,435125);assert(app.config().vfo[0].rx_hz==435125000);
    enter(app,999999);assert(app.editing());assert(app.config().vfo[0].rx_hz==435125000);
    for(unsigned i=0;i<6;++i) app.key(Key::Back);
    app.digit(1);app.key(Key::Ok);assert(app.config().vfo[0].rx_hz==435125000);
    app.key(Key::Back);
    // A/B/C own independent frequencies; editing cannot leak to another slot.
    app.digit(4);app.key(Key::NextSlot);assert(!app.editing());
    enter(app,145500);app.key(Key::NextSlot);enter(app,433500);
    app.key(Key::NextSlot);assert(app.config().active==0);
    assert(app.config().vfo[0].rx_hz==435125000);
    assert(app.config().vfo[1].rx_hz==145500000);
    assert(app.config().vfo[2].rx_hz==433500000);
    // Memories, browse occupied entries, and VFO restoration.
    assert(app.store(0));assert(!app.store(Application::MemoryCount));
    assert(!app.recall(1));assert(app.memory(64)==nullptr);
    app.set_frequency(435000000);assert(app.store(10));
    app.key(Key::Mode);assert(app.config().vfo[0].channel_mode);
    assert(app.config().vfo[0].rx_hz==435125000);
    app.key(Key::Up);assert(app.config().vfo[0].memory_channel==10);
    app.key(Key::Down);assert(app.config().vfo[0].memory_channel==0);
    app.digit(9);assert(!app.editing());
    app.key(Key::Mode);assert(!app.config().vfo[0].channel_mode);
    assert(app.config().vfo[0].rx_hz==435000000);
    // Tuning steps and band boundaries.
    app.key(Key::Up);assert(app.config().vfo[0].rx_hz==435012500);
    app.set_frequency(470000000);app.key(Key::Up);
    assert(app.config().vfo[0].rx_hz==470000000);
    app.set_frequency(136000000);app.key(Key::Down);
    assert(app.config().vfo[0].rx_hz==136000000);
    assert(!app.set_frequency(200000000));
    // Host TX and beacon cannot be armed without an explicit APRS frequency.
    assert(!app.set_option(Option::Beacon,true));
    assert(app.set_aprs_frequency(144800000));
    assert(!app.set_option(Option::Beacon,true));
    assert(app.set_option(Option::Ble,true));
    app.key(Key::BeaconIdentity);app.key(Key::Up);app.key(Key::Ok);
    assert(app.set_option(Option::Beacon,true));
    app.set_option(Option::Gps,false);assert(!app.config().beacon_on);
    assert(!app.set_option(Option::Beacon,true));
    app.set_option(Option::Ble,false);
    assert(!app.config().ble_export);
    assert(!app.set_option(Option::Export,true));
    app.key(Key::Menu);assert(app.page()==Page::Settings);
    app.key(Key::Ok);assert(app.config().watch_mode==RT_WATCH_SINGLE);
    app.key(Key::Gps);assert(app.page()==Page::Gps);
    app.key(Key::Back);assert(app.page()==Page::Settings);
    app.key(Key::Back);assert(app.page()==Page::Radio);
    // Render every screen during mixed input; sanitizer checks array bounds.
    Display out;
    for(unsigned i=0;i<5000;++i) {
        app.key(Key(i%8));app.digit(i%11);app.render(out);
        assert(std::strlen(out.title)<sizeof(out.title));
        for(auto &line:out.line) assert(std::strlen(line)<sizeof(line));
    }
    std::puts("Custom C++ application: all checks passed");
}
