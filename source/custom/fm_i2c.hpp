// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace rt950::fm {
// SDA is driven during transmit and released before sampling the slave.
template<class Lines>
class I2cBus final {
public:
    explicit I2cBus(Lines &lines):lines_(lines){}
    bool write(uint8_t address,const uint8_t *data,size_t size) {
        if(address>127 || !data || !size || size>8)return false;
        if(!start())return false;
        bool ok=send(uint8_t(address<<1));
        for(size_t i=0;i<size && ok;++i)ok=send(data[i]);
        const bool stopped=stop();
        return ok && stopped;
    }
    bool read(uint8_t address,uint8_t *data,size_t size) {
        if(address>127 || !data || !size || size>9)return false;
        if(!start())return false;
        bool ok=send(uint8_t((address<<1)|1));
        for(size_t i=0;i<size && ok;++i)ok=receive(data[i],i+1<size);
        const bool stopped=stop();
        return ok && stopped;
    }
private:
    Lines &lines_;
    bool raise_clock() {
        lines_.clock(true);
        for(unsigned i=0;i<32;++i){lines_.delay();if(lines_.clock_level())return true;}
        return false;
    }
    bool start() {
        lines_.release_data();
        if(!raise_clock() || !lines_.data_level())return false;
        lines_.drive_data(false);lines_.delay();lines_.clock(false);
        return true;
    }
    bool stop() {
        lines_.clock(false);
        lines_.drive_data(false);lines_.delay();
        const bool high=raise_clock();
        lines_.release_data();lines_.delay();
        return high && lines_.data_level();
    }
    bool pulse() {
        lines_.delay();
        if(!raise_clock())return false;
        lines_.clock(false);return true;
    }
    bool read_bit(bool &bit) {
        lines_.delay();
        if(!raise_clock())return false;
        bit=lines_.data_level();
        lines_.clock(false);return true;
    }
    bool send(uint8_t byte) {
        for(unsigned i=0;i<8;++i) {
            lines_.drive_data(byte&0x80u);
            if(!pulse())return false;
            byte=uint8_t(byte<<1);
        }
        lines_.release_data();
        bool nack=true;
        return read_bit(nack) && !nack;
    }
    bool receive(uint8_t &byte,bool acknowledge) {
        lines_.release_data();byte=0;
        for(unsigned i=0;i<8;++i) {
            bool bit=false;
            if(!read_bit(bit))return false;
            byte=uint8_t((byte<<1)|bit);
        }
        if(acknowledge)lines_.drive_data(false);
        else lines_.release_data();
        return pulse();
    }
};
}
