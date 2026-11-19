// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "application.hpp"

namespace rt950 {
class GpsIdle {
public:
    bool update(const Application &app,uint32_t now) {
        const auto &config=app.config();
        if(!config.gps_on)return powered_=false;
        if(config.gps_on==RT_GPS_ON)return powered_=true;

        const auto page=app.page();
        const bool navigation=app.screen_awake() &&
            (page==Page::Gps || page==Page::Waypoint || page==Page::Distance);
        const bool live_beacon=config.beacon_on && !app.beacon_saved_point();
        const bool gps_export=config.ble_on && config.ble_export;
        if(navigation || live_beacon || gps_export) {
            last_use_=now;
            return powered_=true;
        }
        // Leave a minute to switch between screens without reacquiring satellites.
        if(uint32_t(now-last_use_)>=60000u)powered_=false;
        return powered_;
    }
private:
    uint32_t last_use_=0;
    bool powered_=false;
};
}
