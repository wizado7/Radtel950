// SPDX-License-Identifier: GPL-3.0-or-later
#include "controls.hpp"
#include "application.hpp"
#include <cassert>
#include <cstdio>
using namespace rt950;
int main() {
    Controls c;
    assert(c.poll(10,0)==255);assert(c.poll(10,29)==255);
    assert(c.poll(10,30)==10);assert(c.poll(10,10000)==255);
    assert(c.poll(255,10001)==255);assert(c.poll(255,10031)==255);
    // Alternating candidates must never accumulate debounce toward a press.
    for(unsigned i=0;i<200;++i)assert(c.poll(i%2?10:11,10040+i*10)==255);
    assert(c.poll(7,13000)==255);assert(c.poll(7,13030)==7);
    assert(c.poll(7,13529)==255);assert(c.poll(7,13530)==7);
    assert(c.poll(7,13649)==255);assert(c.poll(7,13650)==7);
    assert(c.poll(255,13651)==255);assert(c.poll(255,13681)==255);
    assert(c.poll(10,0xfffffff0u)==255);assert(c.poll(10,14)==10);
    PowerSwitch p;
    assert(!p.poll(true,1));assert(!p.poll(true,50));
    assert(!p.poll(false,51));assert(!p.poll(true,52));assert(p.poll(true,102));
    assert(!p.poll(false,103));assert(!p.poll(true,0xfffffff0u));
    assert(p.poll(true,34));
    const int want[12]={1,2,3,-1,4,5,6,0,7,8,9,-1};
    for(unsigned i=0;i<256;++i)assert(key_digit(uint8_t(i))==(i>=10&&i<=21?want[i-10]:-1));
    Application app(false);
    dispatch_key(app,10);assert(app.entered_khz()==1);
    dispatch_key(app,9);assert(!app.editing());
    dispatch_key(app,18);assert(app.entered_khz()==7);
    dispatch_key(app,9);dispatch_key(app,6);assert(app.page()==Page::Settings);
    const bool ble=app.config().ble_on;dispatch_key(app,2);assert(app.config().ble_on==ble);
    dispatch_key(app,9);assert(app.page()==Page::Radio);
    dispatch_key(app,3);assert(app.config().active==1);
    std::puts("Controls: debounce, repeat, release, power timing and wrap: PASS");
}
