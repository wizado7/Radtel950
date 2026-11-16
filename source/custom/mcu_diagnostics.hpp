// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "drivers/adc.h"

namespace rt950 {
class McuTemperature final {
public:
    static constexpr int Unavailable=-32768;
    static int estimate(unsigned temperature,unsigned reference) {
        if(!temperature || temperature>=4095 || !reference || reference>=4095)return Unavailable;
        const int millivolts=int((1200u*temperature+reference/2u)/reference);
        // AT32F403A V2.04 p. 78, Table 55: Avg_Slope is -4.26 mV/C.
        const int degrees=25+(millivolts-1280)*100/426;
        return degrees>=-40 && degrees<=105?degrees:Unavailable;
    }
    int reading(uint32_t now) const {
        return uint32_t(now-sampled_)<6000u?degrees_:Unavailable;
    }
    void poll(bool allowed,uint32_t now) {
        if(!allowed) {
            if(state_!=State::Idle)release();
            attempted_=false;
            return;
        }
        const uint32_t elapsed=now-since_;
        if(state_==State::Idle) {
            if(!attempted_ || uint32_t(now-attempted_at_)>=2000u)acquire(now);
            return;
        }
        if(elapsed>=20u) {
            degrees_=Unavailable;
            release();
            return;
        }
        switch(state_) {
        case State::Warmup:
            if(elapsed){ADC1->CR2|=ADC_CR2_RSTCAL;advance(State::ResetCal,now);}
            break;
        case State::ResetCal:
            if(!(ADC1->CR2&ADC_CR2_RSTCAL)){ADC1->CR2|=ADC_CR2_CAL;advance(State::Cal,now);}
            break;
        case State::Cal:
            if(!(ADC1->CR2&ADC_CR2_CAL))convert(16,State::Temperature,now);
            break;
        case State::Temperature:
            if(ADC1->SR&ADC_SR_EOC) {
                temperature_=uint16_t(ADC1->DR);
                convert(17,State::Reference,now);
            }
            break;
        case State::Reference:
            if(ADC1->SR&ADC_SR_EOC) {
                degrees_=estimate(temperature_,ADC1->DR);
                sampled_=now;
                release();
            }
            break;
        case State::Idle:break;
        }
    }
private:
    enum class State : uint8_t {Idle,Warmup,ResetCal,Cal,Temperature,Reference};
    // ADC_CTRL2 internal temperature/reference enable, AT32F403A RM §19.6.3.
    static constexpr uint32_t Internal=1u<<23;
    void advance(State state,uint32_t now){state_=state;since_=now;}
    void acquire(uint32_t now) {
        attempted_=true;attempted_at_=now;
        had_clock_=(CRM->APB2EN&CRM_APB2EN_ADC1EN)!=0;
        CRM->APB2EN|=CRM_APB2EN_ADC1EN;
        if(ADC1->CR2) {
            if(!had_clock_)CRM->APB2EN&=~CRM_APB2EN_ADC1EN;
            return;
        }
        ctrl1_=ADC1->CR1;sample_times_=ADC1->SMPR1;
        sequence_len_=ADC1->SQR1;sequence_=ADC1->SQR3;
        ADC1->CR1=0;ADC1->SQR1=0;
        ADC1->SMPR1=sample_times_|(7u<<18)|(7u<<21); // 239.5 ADC cycles.
        ADC1->CR2=Internal|ADC_CR2_ADON;
        advance(State::Warmup,now);
    }
    void convert(unsigned channel,State next,uint32_t now) {
        ADC1->SR=0;ADC1->SQR3=channel;
        ADC1->CR2=Internal|ADC_CR2_ADON|ADC_CR2_EXTSEL_SWSTART|ADC_CR2_EXTTRIG|ADC_CR2_SWSTART;
        advance(next,now);
    }
    void release() {
        ADC1->CR2=0;ADC1->SR=0;
        ADC1->SQR3=sequence_;ADC1->SQR1=sequence_len_;
        ADC1->SMPR1=sample_times_;ADC1->CR1=ctrl1_;
        if(!had_clock_)CRM->APB2EN&=~CRM_APB2EN_ADC1EN;
        state_=State::Idle;
    }
    State state_=State::Idle;
    bool attempted_=false,had_clock_=false;
    uint32_t since_=0,attempted_at_=0,sampled_=0,ctrl1_=0,sample_times_=0,sequence_len_=0,sequence_=0;
    uint16_t temperature_=0;
    int degrees_=Unavailable;
};
}
