// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_gps_telemetry.hpp"
extern "C" {
#include "../mvp/core.h"
#include "drivers/gpio.h"
}

namespace rt950::aprs {
// TX-only transport for the 0.29 KISS(BLE) route. UART ready is not
// evidence of a BLE connection. Pending data is deliberately bounded.
class BleExport final {
public:
    void enable(bool enabled,uint32_t now) {
        if(enabled==enabled_)return;
        enabled_=enabled;
        if(enabled) {
            CRM->APB2EN|=CRM_APB2EN_USART1EN;
            USART1->CR1=0;USART1->CR2=0;USART1->CR3=0;
            USART1->BRR=6250; // APB2 60 MHz / 9600, OEM KISS data rate
            gpio_config_pin(GPIOA,GPIO_PIN_9,GPIO_MODE_OUT_2MHZ,GPIO_CNF_AF_PP);
            USART1->CR1=USART_CR1_UE|USART_CR1_TE;
            gpio_set_pin(GPIOB,GPIO_PIN_9);
            started_=now;
        } else {
            clear();
            USART1->CR1=0;
            gpio_config_pin(GPIOA,GPIO_PIN_9,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
            gpio_config_pin(GPIOA,GPIO_PIN_10,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
            CRM->APB2EN&=~CRM_APB2EN_USART1EN;
            gpio_clear_pin(GPIOB,GPIO_PIN_9);
        }
    }

    bool enqueue(const uint8_t *frame,size_t size,uint32_t now) {
        size_t payload=0;
        if(!enabled_ || (size_ && (!telemetry_ || sent_)) ||
           size>330 || !ui_payload(frame,size,payload))return false;
        return encode(0,frame,size,now);
    }

    bool enqueue_gps(const GpsTelemetry &gps,uint32_t now) {
        return enabled_ && !size_ && encode(GpsTelemetry::Type,gps.bytes,sizeof gps.bytes,now);
    }

    void poll(bool allowed,uint32_t now) {
        if(!allowed || !enabled_){clear();return;}
        if(!size_)return;
        if(uint32_t(now-progressed_)>=250u){clear();return;}
        if(uint32_t(now-started_)<100u || !(USART1->SR&USART_SR_TXE))return;
        USART1->DR=bytes_[sent_++];
        progressed_=now;
        if(sent_==size_)clear();
    }

private:
    bool encode(uint8_t type,const uint8_t *data,size_t size,uint32_t now) {
        size_=rt_kiss_encode(type,data,size,bytes_,sizeof bytes_);
        telemetry_=type==GpsTelemetry::Type;
        progressed_=now;
        return size_!=0;
    }

    void clear() {sent_=size_=0;}
    uint8_t bytes_[663]{};
    size_t sent_=0,size_=0;
    uint32_t started_=0,progressed_=0;
    bool enabled_=false,telemetry_=false;
};
}
