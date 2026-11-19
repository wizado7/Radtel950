// SPDX-License-Identifier: GPL-3.0-or-later
#include "aprs_dac.hpp"
extern "C" {
#include "drivers/dma.h"
#include "drivers/dac_audio.h"
#include "drivers/gpio.h"
#include "custom_clock.h"
}

namespace rt950::aprs {
namespace {
constexpr uint32_t DacMode=DAC_CR_BOFF1|DAC_CR_TEN1|DAC_CR_TSEL1_TIM6;
constexpr uint32_t HalfFlag=DMA_ISR_HTIF(3),FullFlag=DMA_ISR_TCIF(3);
constexpr uint32_t Underrun=1u<<13;
}

void DacBurst::fill(unsigned half) {
    final_[half]=false;
    for(unsigned i=0;i<Half;++i) {
        uint16_t value=AfskWave::Silence;
        if(symbol_<frame_.size()) {
            // The stock DAC tone selection is the complement of our NRZI level.
            value=wave_.sample(!frame_.level(symbol_));
            if(++sample_==AfskWave::SamplesPerSymbol){sample_=0;++symbol_;}
        } else final_[half]=true;
        samples_[half*Half+i]=value;
    }
    __asm volatile("dmb" ::: "memory");
}

bool DacBurst::begin(const FrameTransmitter &frame,uint32_t now) {
    if(state_==State::Running || !frame.size())return false;
    timer_clock_=CRM->APB1EN&CRM_APB1EN_TIM6EN;
    dac_clock_=CRM->APB1EN&CRM_APB1EN_DACEN;
    dma_clock_=CRM->AHBEN&CRM_AHBEN_DMA2EN;
    CRM->APB1EN|=CRM_APB1EN_TIM6EN|CRM_APB1EN_DACEN;
    CRM->AHBEN|=CRM_AHBEN_DMA2EN;
    auto *channel=DMA2_CH(3);
    if((TIM6->CR1&1u) || TIM6->DIER || (channel->CCR&15u) || (DAC->CR&(DAC_CR_EN1|DAC_CR_DMAEN1))) {
        if(!timer_clock_)CRM->APB1EN&=~CRM_APB1EN_TIM6EN;
        if(!dac_clock_)CRM->APB1EN&=~CRM_APB1EN_DACEN;
        if(!dma_clock_)CRM->AHBEN&=~CRM_AHBEN_DMA2EN;
        return false;
    }

    timer_[0]=TIM6->CR1;timer_[1]=TIM6->CR2;timer_[2]=TIM6->PSC;
    timer_[3]=TIM6->ARR;timer_[4]=TIM6->CNT;
    dma_[0]=channel->CCR;dma_[1]=channel->CNDTR;
    dma_[2]=channel->CPAR;dma_[3]=channel->CMAR;
    dac_control_=DAC->CR&0xffffu;dac_sample_=DAC->DHR12R1;
    pa4_mode_=uint8_t((GPIOA->CRL>>16)&15u);
    pc12_mode_=uint8_t((GPIOC->CRH>>16)&15u);
    pc12_high_=(GPIOC->ODR&GPIO_PIN_12)!=0;

    pb8_high_=(GPIOB->ODR&GPIO_PIN_8)!=0;

    frame_=frame;wave_=AfskWave{};symbol_=sample_=next_half_=0;
    fill(0);fill(1);last_event_=now;
    TIM6->CR1=0;TIM6->CR2=0;
    // OEM 0.29 0x08006408 uses PSC=3, ARR=781 and update TRGO.
    TIM6->PSC=custom_clock_low?0u:3u;TIM6->ARR=custom_clock_low?624u:781u;TIM6->EGR=1;TIM6->SR=0;TIM6->CR2=0x20;
    DAC->DHR12R1=AfskWave::Silence;DAC->SR=Underrun;
    gpio_config_pin(GPIOA,GPIO_PIN_4,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
    gpio_set_pin(GPIOC,GPIO_PIN_12);
    gpio_config_pin(GPIOC,GPIO_PIN_12,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
    DMA2->IFCR=DMA_IFCR_ALL(3);
    channel->CPAR=DAC_BASE+8u;channel->CMAR=reinterpret_cast<uintptr_t>(samples_);
    channel->CNDTR=2*Half;
    channel->CCR=DMA_CCR_DIR|DMA_CCR_CIRC|DMA_CCR_MINC|DMA_CCR_PSIZE_16|
        DMA_CCR_MSIZE_16|DMA_CCR_PL_HIGH|DMA_CCR_EN;
    DAC->CR=(DAC->CR&0xffff0000u)|DacMode|DAC_CR_EN1|DAC_CR_DMAEN1;
    // OEM 0.29 asserts PB8 at 0x0800947A before starting the burst.
    gpio_set_pin(GPIOB,GPIO_PIN_8);
    gpio_clear_pin(GPIOC,GPIO_PIN_12);
    state_=State::Running;
    TIM6->CR1=1;
    return true;
}

DacBurst::State DacBurst::poll(uint32_t now) {
    if(state_!=State::Running)return state_;
    const uint32_t flags=DMA2->ISR;
    const uint32_t observed=flags&(HalfFlag|FullFlag);
    const uint32_t expected=next_half_?FullFlag:HalfFlag;
    if((flags&DMA_ISR_TEIF(3)) || (DAC->SR&Underrun) ||
       (observed && observed!=expected) || uint32_t(now-last_event_)>=40u) {
        release(State::Fault);
    } else if(observed) {
        DMA2->IFCR=expected;
        if(final_[next_half_])release(State::Complete);
        else {
            fill(next_half_);next_half_^=1u;last_event_=now;
        }
    }
    return state_;
}

void DacBurst::cancel() {
    if(state_==State::Running)release(State::Idle);
}

void DacBurst::release(State state) {
    gpio_set_pin(GPIOC,GPIO_PIN_12);
    TIM6->CR1=0;
    if(!pb8_high_)gpio_clear_pin(GPIOB,GPIO_PIN_8);
    DAC->CR=(DAC->CR&0xffff0000u)|dac_control_;
    auto *channel=DMA2_CH(3);
    channel->CCR=0;DMA2->IFCR=DMA_IFCR_ALL(3);
    channel->CNDTR=dma_[1];channel->CPAR=dma_[2];channel->CMAR=dma_[3];channel->CCR=dma_[0];
    DAC->DHR12R1=dac_sample_;DAC->SR=Underrun;
    TIM6->CR2=timer_[1];TIM6->PSC=timer_[2];TIM6->ARR=timer_[3];
    TIM6->CNT=timer_[4];TIM6->SR=0;TIM6->CR1=timer_[0];
    GPIOA->CRL=(GPIOA->CRL&~(15u<<16))|(uint32_t(pa4_mode_)<<16);
    if(!pc12_high_)gpio_clear_pin(GPIOC,GPIO_PIN_12);
    GPIOC->CRH=(GPIOC->CRH&~(15u<<16))|(uint32_t(pc12_mode_)<<16);
    if(!timer_clock_)CRM->APB1EN&=~CRM_APB1EN_TIM6EN;
    if(!dac_clock_)CRM->APB1EN&=~CRM_APB1EN_DACEN;
    if(!dma_clock_)CRM->AHBEN&=~CRM_AHBEN_DMA2EN;
    state_=state;
}
}
