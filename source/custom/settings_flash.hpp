// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace rt950 {
class Application;
void load_settings(Application &app);
const char *save_settings(const Application &app, bool shutting_down=false);
enum class SaveState : uint8_t { Ready, Pending, Deferred, Failed, Saved };
class SettingsAutosave final {
public:
    SaveState state() const {return state_;}
    void begin(const Application &app,uint32_t now);
    const char *poll(const Application &app,uint32_t now,bool shutting_down,bool ptt);
private:
    uint32_t saved_=0,observed_=0,changed_=0,scanned_=0,last_attempt_=0;
    bool attempted_=false;
    SaveState state_=SaveState::Ready;
};
}
