// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "aprs_demod.hpp"
extern "C" {
#include "drivers/adc.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
}

namespace rt950::aprs {
class AdcCapture final {
public:
    bool running() const {return state_==State::Running;}

    void poll(bool enabled,uint32_t now,FrameReceiver::Handler handler,void *context) {
        if(!enabled){stop();return;}
        switch(state_) {
        case State::Idle:
            if(!retry_ || uint32_t(now-since_)>=250u)start(now);
            break;
        case State::Warmup:
            if(uint32_t(now-since_)>=1u) {
                ADC1->CR2|=ADC_CR2_RSTCAL;state_=State::ResetCalibration;since_=now;
            }
            break;
        case State::ResetCalibration:
            if(!(ADC1->CR2&ADC_CR2_RSTCAL)) {
                ADC1->CR2|=ADC_CR2_CAL;state_=State::Calibration;since_=now;
            } else if(uint32_t(now-since_)>=20u)fail(now);
            break;
        case State::Calibration:
            if(!(ADC1->CR2&ADC_CR2_CAL))run(now);
            else if(uint32_t(now-since_)>=20u)fail(now);
            break;
        case State::Running:
            consume(now,handler,context);
            break;
        }
    }

    void stop() {
        if(state_!=State::Idle) {
            TIM2->CR1=0;
            ADC1->CR2=0;
            DMA1_CH(1)->CCR=0;
            DMA1->IFCR=DMA_IFCR_ALL(1);
            GPIOA->CRL=(GPIOA->CRL&~(15u<<8))|(uint32_t(pin_mode_)<<8);
            CRM->APB1EN&=~(CRM_APB1EN_TIM2EN&~timer_clock_);
            CRM->APB2EN&=~(CRM_APB2EN_ADC1EN&~adc_clock_);
            CRM->AHBEN&=~(CRM_AHBEN_DMA1EN&~dma_clock_);
            demod_.reset();
        }
        state_=State::Idle;retry_=false;
    }

private:
    enum class State : uint8_t { Idle,Warmup,ResetCalibration,Calibration,Running };
    static constexpr unsigned Half=1024;
    static constexpr uint32_t Events=DMA_ISR_HTIF(1)|DMA_ISR_TCIF(1)|DMA_ISR_TEIF(1);

    void start(uint32_t now) {
        timer_clock_=CRM->APB1EN&CRM_APB1EN_TIM2EN;
        adc_clock_=CRM->APB2EN&CRM_APB2EN_ADC1EN;
        dma_clock_=CRM->AHBEN&CRM_AHBEN_DMA1EN;
        CRM->APB1EN|=CRM_APB1EN_TIM2EN;
        CRM->APB2EN|=CRM_APB2EN_ADC1EN;
        CRM->AHBEN|=CRM_AHBEN_DMA1EN;
        if((TIM2->CR1&1u) || TIM2->DIER || ADC1->CR2 || (DMA1_CH(1)->CCR&15u)) {
            CRM->APB1EN&=~(CRM_APB1EN_TIM2EN&~timer_clock_);
            CRM->APB2EN&=~(CRM_APB2EN_ADC1EN&~adc_clock_);
            CRM->AHBEN&=~(CRM_AHBEN_DMA1EN&~dma_clock_);
            retry_=true;since_=now;return;
        }
        pin_mode_=uint8_t((GPIOA->CRL>>8)&15u);
        gpio_config_pin(GPIOA,GPIO_PIN_2,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
        ADC1->CR1=0;
        ADC1->SQR1=0;ADC1->SQR3=2;
        ADC1->SMPR2=(ADC1->SMPR2&~(7u<<6))|(3u<<6);
        ADC1->CR2=ADC_CR2_ADON;
        state_=State::Warmup;since_=now;
    }

    void run(uint32_t now) {
        // OEM 0.29 0x08005FD8: TIM2 CH2, ADC1 channel 2, DMA1 CH1.
        TIM2->CR1=0;TIM2->CR2=0;TIM2->SMCR=0;
        TIM2->PSC=0;TIM2->ARR=12500;TIM2->CCR2=255;
        TIM2->CCMR1=0x6000;TIM2->CCER=0x10;
        TIM2->EGR=1;TIM2->SR=0;
        auto *dma=DMA1_CH(1);
        DMA1->IFCR=DMA_IFCR_ALL(1);
        dma->CPAR=ADC1_BASE+0x4cu;
        dma->CMAR=reinterpret_cast<uintptr_t>(samples_);
        dma->CNDTR=2*Half;
        dma->CCR=DMA_CCR_CIRC|DMA_CCR_MINC|DMA_CCR_PSIZE_16|
            DMA_CCR_MSIZE_16|DMA_CCR_PL_HIGH|DMA_CCR_EN;
        ADC1->SR=0;
        ADC1->CR2=ADC_CR2_ADON|ADC_CR2_DMA|ADC_CR2_EXTTRIG|0x60000u;
        next_half_=0;since_=now;state_=State::Running;
        TIM2->CR1=1;
    }

    void consume(uint32_t now,FrameReceiver::Handler handler,void *context) {
        const uint32_t flags=DMA1->ISR&Events;
        const uint32_t expected=next_half_?DMA_ISR_TCIF(1):DMA_ISR_HTIF(1);
        if(uint32_t(now-since_)>=225u || (flags && flags!=expected)){fail(now);return;}
        if(!flags)return;
        DMA1->IFCR=expected;
        __asm volatile("dmb" ::: "memory");
        for(unsigned i=0;i<Half;++i)block_[i]=samples_[next_half_*Half+i];
        __asm volatile("dmb" ::: "memory");
        // DMA must not have reached the block while the CPU was copying it.
        if(DMA1->ISR&Events){fail(now);return;}
        next_half_^=1u;since_=now;
        for(uint16_t value:block_)demod_.sample(value,handler,context);
    }

    void fail(uint32_t now) {stop();since_=now;retry_=true;}

    alignas(4) volatile uint16_t samples_[2*Half]{};
    uint16_t block_[Half]{};
    AfskReceiver demod_{};
    uint32_t since_=0,timer_clock_=0,adc_clock_=0,dma_clock_=0;
    unsigned next_half_=0;
    uint8_t pin_mode_=0;
    State state_=State::Idle;
    bool retry_=false;
};
}
