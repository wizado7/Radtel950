// SPDX-License-Identifier: GPL-3.0-or-later
#include "fm_board.hpp"
#include "fm_i2c.hpp"
extern "C" {
#include "drivers/gpio.h"
}

namespace rt950::fm {
namespace {
void output(GPIO_TypeDef *port,uint16_t pin,bool high) {
    if(high)gpio_set_pin(port,pin);
    else gpio_clear_pin(port,pin);
}
void reset_pin(bool high){output(GPIOA,GPIO_PIN_6,high);}
struct Lines {
    void clock(bool high){output(GPIOB,GPIO_PIN_6,high);}
    void drive_data(bool high) {
        output(GPIOB,GPIO_PIN_7,high);
        gpio_config_pin(GPIOB,GPIO_PIN_7,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
    }
    void release_data() {
        gpio_config_pin(GPIOB,GPIO_PIN_7,GPIO_MODE_INPUT,GPIO_CNF_PULL);
        gpio_set_pin(GPIOB,GPIO_PIN_7);
    }
    bool clock_level() const {return gpio_read_pin(GPIOB,GPIO_PIN_6);}
    bool data_level() const {return gpio_read_pin(GPIOB,GPIO_PIN_7);}
    void delay() const {for(volatile unsigned i=0;i<80;++i)__asm volatile("nop");}
};
void restore(GPIO_TypeDef *port,unsigned pin,uint8_t mode,bool high) {
    port->CRL=(port->CRL&~(15u<<(4u*pin)))|(uint32_t(mode)<<(4u*pin));
    output(port,uint16_t(1u<<pin),high);
}
}

void BoardProbe::mute() {
    if(audio_enabled_) {
        gpio_set_pin(GPIOE,GPIO_PIN_1);
        gpio_clear_pin(GPIOB,GPIO_PIN_8);
        gpio_clear_pin(GPIOE,GPIO_PIN_4);
    }
    audio_enabled_=false;audio_step_=0;
}

void BoardProbe::listen(uint16_t frequency,bool seek,bool audio,uint32_t now) {
    if(!active_)return;
    if(!seek) {
        seek_steps_=0;seek_finished_=false;
    }
    if(!audio || probe_.phase()==Probe::Phase::Fault) {
        mute();
        if(seek){seek_finished_=true;seek_result_=0;}
        return;
    }
    if(probe_.phase()!=Probe::Phase::Receiver)return;
    const bool am=probe_.band()!=Band::Fm;
    if(am) {
        if(probe_.seeking()) {
            if(!seek)probe_.cancel_seek(now);
            return;
        }
        uint16_t found=0;
        if(probe_.take_seek_result(found) && seek) {
            seek_finished_=true;seek_result_=found;
            return;
        }
        if(seek && probe_.frequency()==frequency && probe_.seek(now)) {
            mute();
            return;
        }
    } else if(!probe_.frequency())return;
    if(!am && seek && !seek_finished_) {
        Signal signal;
        if(seek_steps_) {
            if(!probe_.signal(now,signal))return;
            if(quality(signal)>=Quality::Fair || seek_steps_>=(MaxFrequency-MinFrequency)/10+1) {
                seek_finished_=true;
                seek_result_=quality(signal)>=Quality::Fair?probe_.frequency():0;
                return;
            }
        }
        const unsigned next=unsigned(probe_.frequency())+10;
        mute();++seek_steps_;probe_.retune(uint16_t(next>MaxFrequency?MinFrequency:next),now);return;
    }
    if(probe_.frequency()!=frequency) {
        mute();audio_error_=false;
        probe_.retune(frequency,now);return;
    }
    if(audio_enabled_ || audio_error_)return;
    Lines lines;I2cBus<Lines> bus(lines);
    if(!audio_step_)audio_deadline_=now+100u;
    uint8_t status=0;
    if(!bus.read(Address,&status,1) || (status&Error) || int32_t(now-audio_deadline_)>=0) {
        audio_error_=true;return;
    }
    if(!(status&Cts))return;
    if(audio_step_<2) {
        // Stock 0.29 sets tuner volume to 63 and clears hard mute.
        const uint8_t command[]={0x12,0,0x40,audio_step_,0,uint8_t(audio_step_?0:63)};
        if(!bus.write(Address,command,sizeof command)){audio_error_=true;return;}
        ++audio_step_;return;
    }
    gpio_set_pin(GPIOE,GPIO_PIN_4);
    gpio_set_pin(GPIOB,GPIO_PIN_8);
    gpio_clear_pin(GPIOE,GPIO_PIN_1);
    audio_enabled_=true;
}

void BoardProbe::poll(bool open,uint32_t now,Band band) {
    if(open && active_ && probe_.band()!=band)poll(false,now);
    if(open && !active_) {
        saved_[0]=uint8_t(GPIOA->CRL>>24&15u);levels_[0]=GPIOA->ODR&GPIO_PIN_6;
        saved_[1]=uint8_t(GPIOB->CRL>>24&15u);levels_[1]=GPIOB->ODR&GPIO_PIN_6;
        saved_[2]=uint8_t(GPIOB->CRL>>28&15u);levels_[2]=GPIOB->ODR&GPIO_PIN_7;
        saved_[3]=uint8_t(GPIOC->CRL>>16&15u);levels_[3]=GPIOC->ODR&GPIO_PIN_4;
        // OEM 0.29 lowers PC4 before pulsing PA6 at 0x080118CC.
        gpio_clear_pin(GPIOC,GPIO_PIN_4);
        gpio_config_pin(GPIOC,GPIO_PIN_4,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
        reset_pin(false);
        gpio_config_pin(GPIOA,GPIO_PIN_6,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
        gpio_set_pin(GPIOB,GPIO_PIN_6|GPIO_PIN_7);
        gpio_config_pin(GPIOB,GPIO_PIN_6,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
        Lines{}.release_data();
        active_=true;audio_error_=false;seek_steps_=0;seek_finished_=false;probe_.begin(now,band);
    }
    if(!open && active_) {
        mute();reset_pin(false);
        restore(GPIOB,7,saved_[2],levels_[2]);
        restore(GPIOB,6,saved_[1],levels_[1]);
        restore(GPIOA,6,saved_[0],levels_[0]);
        restore(GPIOC,4,saved_[3],levels_[3]);
        active_=false;probe_.end();
    }
    if(open) {
        Lines lines;I2cBus<Lines> bus(lines);
        probe_.poll(bus,reset_pin,now);
    }
}
}
