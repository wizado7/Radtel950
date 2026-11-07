// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>

namespace rt950::voice {
// GPIO ownership stays with the board adapter. Protocol from OEM 0.29
// 0x0801E5A0, 0x0801F374, 0x0801F440 and 0x0801F494.
template<class Pins>
class RegisterBus final {
public:
    constexpr explicit RegisterBus(Pins &pins):pins_(pins){}
    bool write(uint8_t chip,uint8_t address,uint16_t value) {
        if(chip>1 || address>0x7f)return false;
        begin(chip);
        send(address);
        send(uint8_t(value>>8));
        send(uint8_t(value));
        end(chip);
        return true;
    }
    bool read(uint8_t chip,uint8_t address,uint16_t &value) {
        if(chip>1 || address>0x7f)return false;
        begin(chip);
        send(uint8_t(address|0x80));
        pins_.clock(false);pins_.delay();
        pins_.input();pins_.delay();
        value=0;
        for(unsigned i=0;i<16;++i) {
            pins_.clock(true);
            value=uint16_t((value<<1)|unsigned(pins_.sample()));
            pins_.delay();pins_.clock(false);pins_.delay();
        }
        end(chip);
        return true;
    }
private:
    Pins &pins_;
    void begin(uint8_t chip) {
        pins_.select(chip,true);pins_.delay();
    }
    void end(uint8_t chip) {
        pins_.delay();pins_.select(chip,false);pins_.delay();
        pins_.clock(false);pins_.drive(false);
    }
    void send(uint8_t byte) {
        for(unsigned i=0;i<8;++i) {
            pins_.clock(false);pins_.drive(bool(byte&0x80));
            pins_.delay();pins_.clock(true);pins_.delay();
            byte=uint8_t(byte<<1);
        }
    }
};
}
