// SPDX-License-Identifier: GPL-3.0-or-later
#include "settings_export.hpp"
extern "C" {
#include "drivers/gpio.h"
#include "custom_clock.h"
#include "cortex_m4.h"
}
namespace {
constexpr unsigned RingSize=64;
volatile uint8_t rx_ring[RingSize];
volatile uint32_t rx_head=0,rx_tail=0;
}
extern "C" void UART4_IRQHandler(void) {
    const uint32_t status=UART4->SR;
    if(status&(USART_SR_RXNE|15u)) {
        const uint8_t byte=uint8_t(UART4->DR);
        if(!(status&15u)) {
            const uint32_t next=(rx_head+1u)&(RingSize-1u);
            if(next!=rx_tail){rx_ring[rx_head]=byte;rx_head=next;}
        }
    }
}
namespace rt950 {
void BackupUart::open() {
    if(opened_)return;
    crh_=GPIOC->CRH;odr_=GPIOC->ODR;clock_=CRM->APB1EN&CRM_APB1EN_UART4EN;
    CRM->APB1EN|=CRM_APB1EN_UART4EN;
    UART4->CR1=0;UART4->CR2=0;UART4->CR3=0;
    rx_head=rx_tail=0;
    gpio_set_pin(GPIOC,GPIO_PIN_10|GPIO_PIN_11);
    gpio_config_pin(GPIOC,GPIO_PIN_10,GPIO_MODE_OUT_2MHZ,GPIO_CNF_AF_PP);
    gpio_config_pin(GPIOC,GPIO_PIN_11,GPIO_MODE_INPUT,GPIO_CNF_PULL);
    UART4->BRR=clock_brr(clock_mult(),115200); // OEM 0.29 UART4 init 0x08025B0C.
    (void)UART4->SR;(void)UART4->DR;
    NVIC->ICPR[UART4_IRQn/32]=1u<<(UART4_IRQn%32);NVIC_SetPriority(UART4_IRQn,2);NVIC_EnableIRQ(UART4_IRQn);
    UART4->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
    opened_=true;
}
void BackupUart::close() {
    if(!opened_)return;
    NVIC_DisableIRQ(UART4_IRQn);UART4->CR1=0;NVIC->ICPR[UART4_IRQn/32]=1u<<(UART4_IRQn%32);
    const uint32_t pins=GPIO_PIN_10|GPIO_PIN_11;
    gpio_set_pin(GPIOC,uint16_t(odr_&pins));gpio_clear_pin(GPIOC,uint16_t((~odr_)&pins));
    GPIOC->CRH=(GPIOC->CRH&~0xff00u)|(crh_&0xff00u);
    if(!clock_)CRM->APB1EN&=~CRM_APB1EN_UART4EN;
    rx_head=rx_tail=0;opened_=false;
}
// Keep the FIFO branch explicit, including in the instruction-level test model.
__attribute__((optimize("no-if-conversion","no-if-conversion2")))
bool BackupUart::read(uint8_t &byte) {
    const uint32_t tail=rx_tail;
    if(tail==rx_head)return false;
    byte=rx_ring[tail];rx_tail=(tail+1u)&(RingSize-1u);return true;
}
bool BackupUart::ready() const {return opened_ && (UART4->SR&USART_SR_TXE);}
bool BackupUart::done() const {return opened_ && (UART4->SR&USART_SR_TC);}
void BackupUart::put(uint8_t byte){UART4->DR=byte;}
}
