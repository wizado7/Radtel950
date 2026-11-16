// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
extern "C" {
#include "drivers/adc.h"
#include "drivers/gpio.h"
}

namespace rt950::voice {
// ADC1 regular group, PA0. All callers run in the foreground. APRS DMA and
// MCU diagnostics have priority and may cancel this conversion before use.
class MicAdc final {
public:
    enum class Result : uint8_t { Pending, Sample, Unavailable };
    Result poll(bool allowed,uint32_t now,uint16_t &sample) {
        if(!allowed){stop();attempted_=false;return Result::Pending;}
        if(state_==State::Idle) {
            if(attempted_ && uint32_t(now-attempted_at_)<20u)return Result::Pending;
            attempted_=true;attempted_at_=now;
            const uint32_t clock=CRM->APB2EN&CRM_APB2EN_ADC1EN;
            CRM->APB2EN|=CRM_APB2EN_ADC1EN;
            if(ADC1->CR2) {
                if(!clock)CRM->APB2EN&=~CRM_APB2EN_ADC1EN;
                return Result::Unavailable;
            }
            clock_=clock;pin_=GPIOA->CRL&15u;ctrl_=ADC1->CR1;
            sequence_len_=ADC1->SQR1;sequence_=ADC1->SQR3;sample_times_=ADC1->SMPR2;
            gpio_config_pin(GPIOA,GPIO_PIN_0,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
            ADC1->CR1=0;ADC1->SQR1=0;ADC1->SQR3=0;
            ADC1->SMPR2=(sample_times_&~7u)|7u;
            ADC1->CR2=ADC_CR2_ADON;
            advance(State::Warmup,now);
            return Result::Pending;
        }
        const uint32_t elapsed=now-since_;
        if(elapsed>=10u){stop();return Result::Unavailable;}
        switch(state_) {
        case State::Warmup:
            if(elapsed){ADC1->CR2|=ADC_CR2_RSTCAL;advance(State::ResetCal,now);}
            break;
        case State::ResetCal:
            if(!(ADC1->CR2&ADC_CR2_RSTCAL)){ADC1->CR2|=ADC_CR2_CAL;advance(State::Cal,now);}
            break;
        case State::Cal:
            if(!(ADC1->CR2&ADC_CR2_CAL)) {
                ADC1->SR=0;
                ADC1->CR2=ADC_CR2_ADON|ADC_CR2_EXTSEL_SWSTART|ADC_CR2_EXTTRIG|ADC_CR2_SWSTART;
                advance(State::Read,now);
            }
            break;
        case State::Read:
            if(ADC1->SR&ADC_SR_EOC) {
                sample=uint16_t(ADC1->DR);
                stop();return Result::Sample;
            }
            break;
        case State::Idle:break;
        }
        return Result::Pending;
    }
    void stop() {
        if(state_==State::Idle)return;
        ADC1->CR2=0;ADC1->SR=0;
        ADC1->SQR1=sequence_len_;ADC1->SQR3=sequence_;ADC1->SMPR2=sample_times_;ADC1->CR1=ctrl_;
        GPIOA->CRL=(GPIOA->CRL&~15u)|pin_;
        if(!clock_)CRM->APB2EN&=~CRM_APB2EN_ADC1EN;
        state_=State::Idle;
    }
private:
    enum class State : uint8_t {Idle,Warmup,ResetCal,Cal,Read};
    void advance(State state,uint32_t now){state_=state;since_=now;}
    State state_=State::Idle;
    bool attempted_=false;
    uint32_t since_=0,attempted_at_=0,clock_=0,pin_=0,ctrl_=0,sequence_len_=0,sequence_=0,sample_times_=0;
};
}
