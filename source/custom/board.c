// SPDX-License-Identifier: GPL-3.0-or-later
#include "board.h"
#include "at32f403a.h"
#include "cortex_m4.h"
#include "drivers/gpio.h"
#include "drivers/lcd.h"

volatile uint32_t custom_heartbeat;
volatile uint8_t custom_watchdog_armed;
#define RING_SIZE 2048u
static volatile uint8_t ring[RING_SIZE];
static volatile uint32_t head,tail,rx_bytes,rx_errors,rx_dropped;
static uint8_t gps_on;
static bool gps_sleeping;
static uint32_t gps_tx_deadline;
static uint8_t gps_tx[20],gps_tx_pos,gps_tx_len;

static void output(GPIO_TypeDef *p,uint16_t pin,uint8_t high) {
    if(high) gpio_set_pin(p,pin); else gpio_clear_pin(p,pin);
    gpio_config_pin(p,pin,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
}
static void input(GPIO_TypeDef *p,uint16_t pin) {
    gpio_set_pin(p,pin);
    gpio_config_pin(p,pin,GPIO_MODE_INPUT,GPIO_CNF_PULL);
}
void board_init(void) {
    if(CRM->APB2EN&CRM_APB2EN_USART1EN) {
        USART1->CR1=0;
        CRM->APB2EN&=~CRM_APB2EN_USART1EN;
    }
    gpio_config_pin(GPIOA,GPIO_PIN_9,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
    gpio_config_pin(GPIOA,GPIO_PIN_10,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
    output(GPIOB,GPIO_PIN_9,0); // OEM Bluetooth module enable
    output(GPIOA,GPIO_PIN_11,1);
    output(GPIOA,GPIO_PIN_6,0); // Hold the broadcast tuner in reset.
    output(GPIOC,GPIO_PIN_13,0);output(GPIOC,GPIO_PIN_14,0); // no RF activity yet
    /* Tested idle routing from GPS v0.4; NEVER activate active-low TX paths. */
    output(GPIOE,GPIO_PIN_7,1);output(GPIOE,GPIO_PIN_12,1);
    output(GPIOE,GPIO_PIN_13,1);output(GPIOE,GPIO_PIN_14,1);
    output(GPIOB,GPIO_PIN_0,1);output(GPIOB,GPIO_PIN_1,1);
    output(GPIOB,GPIO_PIN_8,0);output(GPIOE,GPIO_PIN_4,0);
    output(GPIOE,GPIO_PIN_1,1);output(GPIOA,GPIO_PIN_8,1);
    // Stop channels inherited from the loader. The audio drivers own the
    // clock gates while capture or playback is actually running.
    const uint32_t dma_clocks=CRM_AHBEN_DMA1EN|CRM_AHBEN_DMA2EN;
    CRM->AHBEN|=dma_clocks;
    for(unsigned i=1;i<=7;++i) DMA1_CH(i)->CCR=0;
    for(unsigned i=1;i<=5;++i) DMA2_CH(i)->CCR=0;
    CRM->AHBEN&=~dma_clocks;
    for(unsigned i=0;i<4;++i) {
        const uint16_t pin=(uint16_t)(1u<<i);
        gpio_set_pin(GPIOC,pin);
        gpio_config_pin(GPIOC,pin,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
    }
    for(unsigned i=4;i<8;++i) input(GPIOD,(uint16_t)(1u<<i));
    input(GPIOE,GPIO_PIN_0);input(GPIOE,GPIO_PIN_2);input(GPIOE,GPIO_PIN_3);
    input(GPIOE,GPIO_PIN_5);input(GPIOA,GPIO_PIN_12);
}
/* OEM 0.29 0x08016B90..0x08016BA8 configures GPIOC mask 0x7E7F
 * mode 0x10 + speed 2: push-pull 2 MHz, including PC0..3.
 * Weak pull-ups caused physically distinct keys to alias the all-HIGH pass. */
static void columns(uint16_t low) {
    gpio_set_pin(GPIOC,(uint16_t)(15u & ~low));
    gpio_clear_pin(GPIOC,low);
}
static void settle(void) {for(volatile unsigned n=0;n<310;++n) __asm volatile("nop");}
uint8_t board_scan_key(void) {
    /* Reconstructed from 0.29 scanner 0x08016160 and tables 0x08030916.
     * col 0 has ALL HIGH, the four subsequent columns drive one line LOW.
     * The upstream col*4+row and all-LOW fifth pass are NOT equivalent. */
    
    static const uint16_t clear[5]={0,1,2,4,8};
    static const uint8_t map[20]={10,14,18,13,6,3,9,2,12,16,20,21,11,15,19,17,7,8,4,5};
    if(!(GPIOE->IDR & GPIO_PIN_5)) return 0;
    if(!(GPIOA->IDR & GPIO_PIN_12)) return 1;
    columns(15);settle();
    if((GPIOD->IDR & 240u)==240u) {columns(0);return 255;}
    for(unsigned col=0;col<5;++col) {
        columns(clear[col]);settle();
        const uint32_t rows=GPIOD->IDR & 240u;
        unsigned row=4;
        if(rows==112) row=0;else if(rows==176) row=1;
        else if(rows==208) row=2;else if(rows==224) row=3;
        if(row<4) {columns(0);return map[col*4+row];}
    }
    columns(0);return 255;
}
uint8_t board_power_switch_off(void) {return (GPIOE->IDR & GPIO_PIN_0)!=0;}
uint8_t board_ptt_mask(void) {
    const uint32_t pins=GPIOE->IDR;
    return (uint8_t)((pins&GPIO_PIN_3?0u:1u)|(pins&GPIO_PIN_2?0u:2u)|(pins&GPIO_PIN_5?0u:4u));
}
void board_power_cut(void) {
    board_gps_enable(0);
    /* Safe idle, not LOW: LOW would activate routing pins. */
    gpio_set_pin(GPIOE,GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_1);
    gpio_set_pin(GPIOB,GPIO_PIN_0|GPIO_PIN_1);
    gpio_clear_pin(GPIOB,GPIO_PIN_8);gpio_clear_pin(GPIOE,GPIO_PIN_4);
    gpio_clear_pin(GPIOC,GPIO_PIN_13|GPIO_PIN_14);
    board_display_shutdown();
    /* Match OEM 0.29 0x0800C460..0x0800C476: dark, 100 ms, PA11 LOW.
     * No immediate Sleep In -> rail cut; panel supply topology is unmeasured. */
    extern void delay_ms(uint32_t);
    /* Keep the clock interrupt active during the panel shutdown wait. */
    delay_ms(100);
    __disable_irq();
    /* OEM 0.29 0x0800C476 clears PA11. Release PB9 as well, matching
     * the loss of the latch output when OEM later resets the MCU. */
    gpio_clear_pin(GPIOA,GPIO_PIN_11);
    gpio_clear_pin(GPIOB,GPIO_PIN_9);
    for(;;) {*(volatile uint32_t *)0x40003000u=0xAAAAu;}
}
void USART3_IRQHandler(void) {
    if(!gps_on)return;
    const uint32_t sr=USART3->SR;
    if(sr & ((1u<<5)|15u)) {
        const uint8_t ch=(uint8_t)USART3->DR;
        if(sr & 15u) ++rx_errors;
        if(sr & (1u<<5)) {
            ++rx_bytes;
            const uint32_t next=(head+1u)&(RING_SIZE-1u);
            if(next==tail) ++rx_dropped;else {ring[head]=ch;head=next;}
        }
    }
}
void board_gps_enable(uint8_t on) {
    NVIC_DisableIRQ(USART3_IRQn);
    gps_on=0;gps_sleeping=false;gps_tx_pos=gps_tx_len=0;
    head=tail=0;
    nav_reset();
    if(!on) {
        if(CRM->APB1EN & CRM_APB1EN_USART3EN) {
            USART3->CR1=0;
            CRM->APB1EN&=~CRM_APB1EN_USART3EN;
        }
        // Disable both digital inputs/pull-ups before removing module power.
        gpio_config_pin(GPIOB,GPIO_PIN_10,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
        gpio_config_pin(GPIOB,GPIO_PIN_11,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
        gpio_set_pin(GPIOA,GPIO_PIN_8);
        return;
    }
    CRM->APB1EN|=CRM_APB1EN_USART3EN;
    AFIO->MAPR=(AFIO->MAPR&~((3u<<4)|AFIO_MAPR_SWJ_CFG_MASK))|(2u<<24);
    *(volatile uint32_t *)0x4001002Cu &= ~(15u<<24);
    CRM->APB1RST |= CRM_APB1EN_USART3EN;
    CRM->APB1RST &= ~CRM_APB1EN_USART3EN;
    input(GPIOB,GPIO_PIN_11);
    gpio_config_pin(GPIOB,GPIO_PIN_10,GPIO_MODE_INPUT,GPIO_CNF_FLOATING);
    USART3->CR2=0;USART3->CR3=0;USART3->BRR=6250;
    (void)USART3->SR;(void)USART3->DR;
    USART3->CR1=USART_CR1_RE|USART_CR1_RXNEIE|USART_CR1_UE;
    gps_on=1;
    NVIC_SetPriority(USART3_IRQn,1);NVIC_EnableIRQ(USART3_IRQn);
    gpio_clear_pin(GPIOA,GPIO_PIN_8);
}
void board_gps_poll(uint32_t now) {
    if(!gps_on) return;
    uint32_t mask;
    __asm volatile("mrs %0, primask\ncpsid i":"=r"(mask)::"memory");
    USART3_IRQHandler();
    __asm volatile("msr primask, %0"::"r"(mask):"memory");
    /* Bound foreground work so input and power polling cannot starve. */
    for(unsigned i=0;i<128 && tail!=head;++i) {
        const uint8_t ch=ring[tail];tail=(tail+1u)&(RING_SIZE-1u);
        nav_feed(ch,now);
    }
}
void board_gps_send(const uint8_t *data,unsigned size,uint32_t now) {
    if(!gps_on || gps_tx_len || !size || size>sizeof gps_tx)return;
    for(unsigned i=0;i<size;++i)gps_tx[i]=data[i];
    gps_tx_pos=0;gps_tx_len=(uint8_t)size;gps_tx_deadline=now+500u;
    USART3->CR1|=USART_CR1_TE;
    gpio_config_pin(GPIOB,GPIO_PIN_10,GPIO_MODE_OUT_2MHZ,GPIO_CNF_AF_PP);
}
bool board_gps_transmit(bool inhibit,uint32_t now) {
    if(!gps_on || !gps_tx_len)return false;
    if(inhibit)gps_tx_pos=gps_tx_len;
    const uint32_t status=USART3->SR;
    if((int32_t)(now-gps_tx_deadline)>=0 ||
       (gps_tx_pos==gps_tx_len && status&USART_SR_TC)) {
        gps_tx_pos=gps_tx_len=0;
        return false;
    }
    if(gps_tx_pos<gps_tx_len && status&USART_SR_TXE)USART3->DR=gps_tx[gps_tx_pos++];
    return true;
}
void board_gps_sleeping_set(bool sleeping) {gps_sleeping=sleeping;}
bool board_gps_sleeping(void) {return gps_sleeping;}
uint8_t board_gps_powered(void) {return gps_on;}
uint32_t board_gps_bytes(void) {return rx_bytes;}
uint32_t board_gps_errors(void) {return rx_errors;}
uint32_t board_gps_dropped(void) {return rx_dropped;}

/* AT32F403A reference manual section 1.3.1, read-only F_SIZE. */
uint16_t board_flash_kib(void) {
    return *(const volatile uint16_t *)0x1FFFF7E0u;
}

extern const uint8_t _storage_start[],_storage_end[],_storage_slot_size[];
uint8_t board_storage_probe(void) {
    if(board_flash_kib()!=1024u)return 255;
    const uintptr_t start=(uintptr_t)_storage_start,end=(uintptr_t)_storage_end;
    const uintptr_t size=(uintptr_t)_storage_slot_size;
    uint8_t mask=0;
    for(uintptr_t addr=start;addr<end;addr+=4u) {
        if(*(const volatile uint32_t *)addr!=0xffffffffu)
            mask|=(uint8_t)(addr-start<size?1u:2u);
    }
    return mask;
}
