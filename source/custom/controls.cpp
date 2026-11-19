// SPDX-License-Identifier: GPL-3.0-or-later
#include "controls.hpp"
#include "application.hpp"
namespace rt950 {
uint8_t Controls::poll(uint8_t raw,uint32_t now) {
    if(raw!=candidate_) {candidate_=raw;changed_=now;return 255;}
    if(uint32_t(now-changed_)<30u) return 255;
    if(accepted_!=candidate_) {
        // Defer PF1/PF2, V/M and # until release. A hold must never fire
        // its short action first, nor fire it again after release.
        if((accepted_==0 || accepted_==1 || accepted_==2 || accepted_==3 || accepted_==21) && candidate_==255) {
            const uint8_t released=accepted_;
            accepted_=255;return long_sent_?255:released;
        }
        accepted_=candidate_;repeated_=now;repeating_=false;long_sent_=false;return accepted_==0 || accepted_==1 || accepted_==2 || accepted_==3 || accepted_==21?255:accepted_;
    }
    if((accepted_==0 || accepted_==1 || accepted_==6 || accepted_==9 || accepted_==3 || accepted_==2 || accepted_==3 || accepted_==21 || accepted_==20) && !long_sent_ && uint32_t(now-repeated_)>=700u) {
        long_sent_=true;return uint8_t(accepted_|128u);
    }
    // Only arrows repeat; never repeat digit entry or toggles.
    if(accepted_!=8 && accepted_!=7) return 255;
    if(uint32_t(now-repeated_)<(repeating_?120u:500u)) return 255;
    repeated_=now;repeating_=true;return accepted_;
}
void dispatch_key(Application &app,uint8_t code) {
    if(code==uint8_t(20|128) && app.page()==Page::QuickSettings){app.key(Key::Save);return;}
    if(code==uint8_t(21|128)) {app.key(Key::KeypadLock);return;}
    if(app.page()==Page::VoiceRx) {
        if(code==9 || code==1)app.key(Key::Back);
        return;
    }
    if(app.page()==Page::Debug) {if(code==uint8_t(9|128))app.key(Key::Back);return;}
    if((code&127u)<=1u){app.user_key(code&127u,(code&128u)!=0);return;}
    if(code==uint8_t(3|128)) {app.key(Key::QuickSettings);return;}
    if(code==uint8_t(2|128)) {app.key(Key::Gps);return;}
    if(code==uint8_t(6|128)) {app.key(Key::Menu);return;}
    if(code==uint8_t(9|128)) return;
    const int digit=key_digit(code);
    if(digit>=0){app.digit(unsigned(digit));return;}
    // Physical observations on v0.4: OK was Up (6), V/M was OK (2),
    // Down was Menu (8). ABC (3), Back (9), digits physically reported.
    // Up/1 and Left/7 aliased in weak-pull-up v0.4. OEM PP scan restores
    // the expected separate column codes Up=7/Left=4; verify on hardware.
    switch(code) {
    case 6:app.key(app.page()==Page::Radio && !app.editing()?Key::Menu:Key::Ok);break;
    case 2:app.key(Key::Mode);break;
    case 3:app.key(Key::NextSlot);break;
    case 8:app.key(Key::Down);break;
    case 7:app.key(Key::Up);break;
    case 4:app.key(Key::Left);break;
    case 5:app.key(Key::Right);break;
    case 9:app.key(Key::Back);break;
    case 1:app.key(app.page()==Page::Radio || app.page()==Page::FmProbe?Key::FmRadio:Key::Back);break;
    case 13:app.key(Key::Erase);break;
    case 21:app.key(Key::Ok);break;
    // Navigation columns use OEM push-pull; Up=7 awaits physical check.
    default:break;
    }
}
int key_digit(uint8_t code) {
    // Numeric keypad inferred from the 0.29 matrix layout; physical labels
    // remain a hardware check. Diagnostic codes are confined to GPS status.
    static const int8_t digits[256]={
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,2,3,-1,4,5,
        6,0,7,8,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
    };
    return digits[code];
}
}
