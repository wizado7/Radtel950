/*
 * system.c - System clock and tick initialization for the RT-950 Pro
 *
 * Target: Artery AT32F403A - 120 MHz from 8 MHz HSE crystal.
 *
 * Clock configuration verified from V0.27 binary:
 *   OEM SystemInit:   fw 0x080216DC (r2 0x080246DC)
 *   OEM clock_config: fw 0x0801FA90 (r2 0x08022A90)
 *
 *   HEXT = 8 MHz crystal (literal pool @ fw 0x0801A054)
 *   PLL  = HEXT x 15 = 120 MHz (CRM_CFG OR 0x80350000)
 *   AHB  = 120 MHz (HCLK, no prescaler)
 *   APB2 = 60 MHz  (divide by 2)
 *   APB1 = 60 MHz  (divide by 2)
 *   Timer clocks = 120 MHz (APB prescaler > 1, so doubled)
 *
 * PLL encoding (Artery-specific, verified from SDK at32f403a_407_crm.h):
 *   CRM_CFG[21:18] = PLLMULT_L = 13
 *   CRM_CFG[30:29] = PLLMULT_H = 0
 *   Full mult index = (0 << 4) | 13 = 13 -> CRM_PLL_MULT_15 -> x15
 *   CRM_CFG[31] = PLLRANGE = 1 (output > 72 MHz)
 *   CRM_CFG[16] = PLLRCS = 1 (HEXT as source)
 *   CRM_CFG[17] = PLLHEXTDIV = 0 (no HEXT division)
 *
 * Flash wait states handled automatically by auto-step (CRM MISC3).
 * The OEM firmware does NOT explicitly write FLASH_ACR during clock init.
 */

#include "at32f403a.h"
#include "debug_uart.h"

#ifdef RT950_CUSTOM
#include "cortex_m4.h"
#include "custom_clock.h"

enum { TimerOverflow=1u, TimerWake=2u, CountsPerMs=2u };
static volatile uint32_t timer_epoch_ms;
volatile uint8_t custom_encoder_interval=2;
volatile uint8_t custom_idle_wake_ms=1;
bool custom_clock_low;

uint32_t get_tick(void)
{
    uint32_t epoch, count, pending;
    do {
        epoch=timer_epoch_ms;
        count=TIM3->CNT;
        pending=TIM3->SR&TimerOverflow;
        if(pending)count=TIM3->CNT;
    } while(epoch!=timer_epoch_ms);
    return epoch+(pending?0x8000u:0u)+(count/CountsPerMs);
}

// Treat rollover as a distinct IRQ action (also when compare is not pending).
__attribute__((noinline)) static void advance_timer_epoch(void)
{
    timer_epoch_ms+=0x8000u;
}

void TIM3_IRQHandler(void)
{
    const uint32_t flags=TIM3->SR;
    TIM3->SR=~flags;
    if(flags&TimerOverflow)advance_timer_epoch();
    if(!(flags&TimerWake))return;

    const uint16_t count=(uint16_t)TIM3->CNT;
    const uint16_t interval=CountsPerMs*custom_idle_wake_ms;
    TIM3->CCR1=(uint16_t)(count+interval);
    if((uint16_t)((uint16_t)TIM3->CNT-count)>=interval)TIM3->EGR=TimerWake;

    extern volatile uint32_t custom_heartbeat;
    extern volatile uint8_t custom_watchdog_armed;
    const uint32_t now=get_tick();
    if(custom_watchdog_armed && (uint32_t)(now-custom_heartbeat)>=500u) {
        extern void board_rf_quiet(void);
        __disable_irq();
        board_rf_quiet();
        // The stalled foreground must not regain control before the reset.
        for(;;)__asm volatile("nop");
    }

    extern void encoder_tick(void);
    static uint32_t encoder_last;
    if((uint32_t)(now-encoder_last)>=custom_encoder_interval) {
        encoder_last=now;
        encoder_tick();
    }
    *(volatile uint32_t *)0x40003000UL=0xAAAAu;
}

static bool clock_ready(volatile uint32_t *reg,uint32_t mask,uint32_t state)
{
    for(unsigned spin=0;spin<16000;++spin)if((*reg&mask)==state)return true;
    return false;
}

static bool select_clock(uint32_t selection)
{
    CRM->CFGR=(CRM->CFGR&~CRM_CFGR_SW_MASK)|selection;
    return clock_ready(&CRM->CFGR,CRM_CFGR_SWS_MASK,selection<<2);
}

static bool start_pll(unsigned mult)
{
    CRM->CR&=~CRM_CR_PLLEN;
    if(!clock_ready(&CRM->CR,CRM_CR_PLLRDY,0))return false;
    const uint32_t range=mult==ClockNormalMult?1u<<31:0;
    CRM->CFGR=(CRM->CFGR&0x1fc0ffffu)|range|CRM_CFGR_PLLSRC|((mult-2u)<<18);
    CRM->CR|=CRM_CR_PLLEN;
    return clock_ready(&CRM->CR,CRM_CR_PLLRDY,CRM_CR_PLLRDY);
}

static void set_bus_dividers(unsigned mult)
{
    const uint32_t mask=CRM_CFGR_PPRE1_MASK|CRM_CFGR_PPRE2_MASK;
    CRM->CFGR=(CRM->CFGR&~mask)|(mult==ClockNormalMult?0x2400u:0);
}

static void uart_retime(unsigned mult)
{
    if(CRM->APB1EN&CRM_APB1EN_USART3EN)USART3->BRR=clock_brr(mult,9600);
    if(CRM->APB2EN&CRM_APB2EN_USART1EN)USART1->BRR=clock_brr(mult,9600);
}

bool board_idle_clock(bool low)
{
    if(low==custom_clock_low)return low;
    const uint32_t gated1=CRM_APB1EN_TIM4EN|CRM_APB1EN_TIM6EN|
                         CRM_APB1EN_SPI2EN|CRM_APB1EN_UART4EN;
    const uint32_t gated2=CRM_APB2EN_ADC1EN;
    if((CRM->APB1EN&gated1) || (CRM->APB2EN&gated2) ||
       (CRM->AHBEN&(CRM_AHBEN_DMA1EN|CRM_AHBEN_DMA2EN)))return custom_clock_low;
    if((CRM->APB1EN&CRM_APB1EN_USART3EN) && (USART3->CR1&USART_CR1_TE) &&
       !(USART3->SR&USART_SR_TC))return custom_clock_low;
    if((CRM->APB2EN&CRM_APB2EN_USART1EN) && !(USART1->SR&USART_SR_TC))
        return custom_clock_low;
    const unsigned mhz=clock_mhz(clock_mult());
    const unsigned target=low?ClockLowMult:ClockNormalMult;
    volatile uint32_t *const dwt=(volatile uint32_t *)0xe0001000u;
    *(volatile uint32_t *)0xe000edfcu|=1u<<24;
    dwt[0]|=1;
    // Start near a timer edge so resetting PSC does not discard a partial tick.
    const uint16_t next=(uint16_t)(TIM3->CNT+1u);
    clock_ready(&TIM3->CNT,0xffffu,next);
    uint32_t primask;
    __asm volatile("mrs %0, primask\ncpsid i":"=r"(primask)::"memory");
    TIM3->CR1=0;
    uint32_t count=TIM3->CNT;
    const uint32_t flags=TIM3->SR;
    const uint16_t compare=(uint16_t)TIM3->CCR1;
    if(flags&TimerOverflow)timer_epoch_ms+=0x8000u;
    const uint32_t start=dwt[1];
    CRM->MISC3|=CRM_MISC3_AUTO_STEP_EN;
    unsigned running=clock_mult();
    uint32_t before=0,crystal=0,after=0;
    if((CRM->CR&CRM_CR_HSERDY) && select_clock(CRM_CFGR_SW_HSE)) {
        const uint32_t on_crystal=dwt[1];
        before=on_crystal-start;
        set_bus_dividers(1);
        uart_retime(1);
        if(start_pll(target)) {
            set_bus_dividers(target);
            if(select_clock(CRM_CFGR_SW_PLL))running=target;
            else running=0;
        } else running=0;
        if(!running) {
            if(!select_clock(CRM_CFGR_SW_HSE) || !start_pll(ClockNormalMult)) {
                for(;;) __asm volatile("nop");
            }
            set_bus_dividers(ClockNormalMult);
            if(!select_clock(CRM_CFGR_SW_PLL)) {
                for(;;) __asm volatile("nop");
            }
            running=ClockNormalMult;
        }
        const uint32_t on_pll=dwt[1];
        crystal=on_pll-on_crystal;
        custom_clock_low=running==ClockLowMult;
        uart_retime(running);
        extern void board_display_retime(void);
        board_display_retime();
        after=dwt[1]-on_pll;
    } else if((CRM->CFGR&CRM_CFGR_SWS_MASK)!=CRM_CFGR_SWS_PLL) {
        for(;;) __asm volatile("nop");
    } else {
        CRM->CFGR=(CRM->CFGR&~CRM_CFGR_SW_MASK)|CRM_CFGR_SW_PLL;
        before=dwt[1]-start;
    }
    CRM->MISC3&=~CRM_MISC3_AUTO_STEP_EN;

    // 120 units per microsecond keeps fractions across both PLL rates and HEXT.
    static uint32_t carry;
    carry+=before*(120u/mhz)+crystal*15u+after*(120u/clock_mhz(running));
    const uint32_t elapsed=carry/60000u;
    carry%=60000u;
    const bool wake=(flags&TimerWake) || (uint16_t)(compare-count)<=elapsed;
    count+=elapsed;
    if(count>=65536u)timer_epoch_ms+=(count>>16)*0x8000u;
    TIM3->PSC=clock_mhz(running)*500u-1u;
    TIM3->EGR=TimerOverflow;
    TIM3->CNT=(uint16_t)count;
    TIM3->SR=0;
    TIM3->CCR1=compare;
    if(wake)TIM3->EGR=TimerWake;
    TIM3->CR1=1;
    __asm volatile("msr primask, %0"::"r"(primask):"memory");
    return custom_clock_low;
}

void board_idle_wait(uint8_t timeout_ms,uint8_t ptt)
{
    __disable_irq();
    custom_idle_wake_ms=timeout_ms;
    const uint16_t start=(uint16_t)TIM3->CNT;
    const uint16_t interval=CountsPerMs*timeout_ms;
    if(!(TIM3->SR&TimerWake) && (uint16_t)((uint16_t)TIM3->CCR1-start)>interval) {
        TIM3->CCR1=(uint16_t)(start+interval);
        if((uint16_t)((uint16_t)TIM3->CNT-start)>=interval)TIM3->EGR=TimerWake;
    }
    extern uint8_t board_ptt_mask(void);
    extern bool board_keypad_pending(void);
    if(board_ptt_mask()==ptt && !board_keypad_pending())__asm volatile("dsb\n wfi":::"memory");
    __enable_irq();
}

static void tick_init(void)
{
    CRM->APB1EN|=CRM_APB1EN_TIM3EN;
    CRM->APB1RST|=CRM_APB1EN_TIM3EN;
    CRM->APB1RST&=~CRM_APB1EN_TIM3EN;
    TIM3->PSC=59999u; /* 120 MHz timer clock / 60000 = 2 kHz. */
    TIM3->ARR=0xffffu;
    TIM3->CCR1=CountsPerMs;
    TIM3->EGR=TimerOverflow;
    TIM3->SR=0;
    TIM3->DIER=TimerOverflow|TimerWake;
    NVIC->ICPR[0]=1u<<TIM3_IRQn;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1=1;
}
#else
static volatile uint32_t systick_ms;

void SysTick_Handler(void)
{
    ++systick_ms;
    *(volatile uint32_t *)0x40003000UL=0xAAAAu;
}

uint32_t get_tick(void)
{
    return systick_ms;
}

static void tick_init(void)
{
    SysTick->LOAD=119999u;
    SysTick->VAL=0;
    SysTick->CTRL=SYSTICK_CTRL_CLKSOURCE|SYSTICK_CTRL_TICKINT|SYSTICK_CTRL_ENABLE;
}
#endif

void delay_ms(uint32_t ms)
{
    const uint32_t start=get_tick();
    while((uint32_t)(get_tick()-start)<ms) {}
}

/* ========================================================================
 *  clock_config - Configure PLL: HEXT 8 MHz x 15 = 120 MHz.
 *
 *  Matches OEM clock_config at fw 0x0801FA90 (r2 0x08022A90).
 *
 *  OEM sequence:
 *    1. Enable HEXT, wait stable (timeout 0x3000)   - fw 0x0801FA90
 *    2. Set APB2 = /2 (bits [13:11] = 100)          - fw 0x0801FAA6
 *    3. Set APB1 = /2 (bits [10:8] = 100)           - fw 0x0801FAB2
 *    4. PLL config: CFG = (CFG & 0x1FC0FFFF) | 0x80350000  - fw 0x0801FABE
 *    5. Enable PLL, wait stable                     - fw 0x0801FACE
 *    6. Auto-step enable (MISC3[5:4] = 11)          - fw 0x0801FADC
 *    7. Switch SCLK to PLL, wait SWS                - fw 0x0801FAE4
 *    8. Auto-step disable                           - fw 0x0801FAF4
 * ======================================================================== */

static void clock_config(void)
{
    /* Step 1: Enable HEXT (8 MHz crystal) - fw 0x0801FA90 */
    CRM->CR |= CRM_CR_HSEEN;

    /* Wait for HEXT stable, timeout 0x3000 - fw 0x0801FA9A */
    uint32_t timeout = 0x3000;
    while (!(CRM->CR & CRM_CR_HSERDY) && --timeout)
        ;

    /* Step 2: APB2 prescaler = /2 - fw 0x0801FAA6 */
    /* OEM: BIC 0x3800 (clear bits[13:11]), ORR 0x2000 (set 100) */
    CRM->CFGR = (CRM->CFGR & ~CRM_CFGR_PPRE2_MASK)
              | (4UL << 11);  /* 100 = /2 -> APB2 = 60 MHz */

    /* Step 3: APB1 prescaler = /2 - fw 0x0801FAB2 */
    /* OEM: BIC 0x700 (clear bits[10:8]), ORR 0x400 (set 100) */
    CRM->CFGR = (CRM->CFGR & ~CRM_CFGR_PPRE1_MASK)
              | (4UL << 8);   /* 100 = /2 -> APB1 = 60 MHz */

    /* Step 4: PLL configuration - fw 0x0801FABE
     * OEM: AND 0x1FC0FFFF (clear PLL bits), ORR 0x80350000
     *
     * 0x80350000 decodes as:
     *   bit[31]     = 1    -> PLLRANGE (output > 72 MHz)
     *   bits[30:29] = 00   -> PLLMULT_H = 0
     *   bits[21:18] = 1101 -> PLLMULT_L = 13
     *   bit[17]     = 0    -> HEXT not divided
     *   bit[16]     = 1    -> PLL source = HEXT
     *   Full mult = (0 << 4) | 13 = 13 -> CRM_PLL_MULT_15 -> x15
     *   SYSCLK = 8 MHz x 15 = 120 MHz */
    CRM->CFGR = (CRM->CFGR & 0x1FC0FFFFUL) | 0x80350000UL;

    /* Step 5: Enable PLL - fw 0x0801FACE */
    CRM->CR |= CRM_CR_PLLEN;
    while (!(CRM->CR & CRM_CR_PLLRDY))
        ;

    /* Step 6: Auto-step enable - fw 0x0801FADC (calls sub @ fw 0x0801A06C)
     * CRM MISC3 (offset 0x54) bits[5:4] = 11
     * Auto-step gradually transitions clock frequency and adjusts flash
     * wait states automatically during the SCLK source switch. */
    CRM->MISC3 |= CRM_MISC3_AUTO_STEP_EN;

    /* Step 7: Switch SCLK to PLL - fw 0x0801FAE4 */
    CRM->CFGR = (CRM->CFGR & ~CRM_CFGR_SW_MASK) | CRM_CFGR_SW_PLL;
    while ((CRM->CFGR & CRM_CFGR_SWS_MASK) != CRM_CFGR_SWS_PLL)
        ;

    /* Step 8: Auto-step disable - fw 0x0801FAF4 (calls sub @ fw 0x0801A084) */
    CRM->MISC3 &= ~CRM_MISC3_AUTO_STEP_EN;
}

/* ========================================================================
 *  SystemInit - Reset clocks to safe state, then configure PLL.
 *
 *  Matches OEM SystemInit at fw 0x080216DC (r2 0x080246DC).
 *
 *  OEM sequence:
 *    1. Enable FPU (CPACR |= 0x00F00000)           - fw 0x080216DE
 *    2. Enable HICK (internal oscillator)           - fw 0x080216EC
 *    3. Reset CRM_CFG (AND 0xE8FF000C)             - fw 0x080216F4
 *    4. Disable HEXT, PLL, CSS                      - fw 0x08021700
 *    5. Clear PLL source/mult (AND 0x1700FFFF)     - fw 0x08021710
 *    6. Clear MISC1 fields (AND 0xFEFEFF00)        - fw 0x08021720
 *    7. Clear clock interrupts (CIR = 0x009F0000)  - fw 0x0802172A
 *    8. Call clock_config()                         - fw 0x08021730
 *    9. Set VTOR = 0x08000000                       - fw 0x08021734
 *
 *  Steps 10-11 are our additions (not part of OEM SystemInit).
 * ======================================================================== */

void SystemInit(void)
{
    /* ---- 1. Enable FPU - fw 0x080216DE ---- */
    /* CPACR (0xE000ED88) |= 0x00F00000: grant CP10/CP11 full access */
    FPU_CPACR |= (0xFUL << 20);

    /* ---- 2. Enable HICK (internal oscillator) - fw 0x080216EC ---- */
    CRM->CR |= CRM_CR_HSIEN;

    /* ---- 3. Reset CRM_CFG - fw 0x080216F4 ---- */
    /* AND 0xE8FF000C: preserves SWS[3:2] and reserved bits,
     * clears prescalers, PLL config, SW bits */
    CRM->CFGR &= 0xE8FF000CUL;

    /* ---- 4. Disable HEXT, PLL, CSS - fw 0x08021700 ---- */
    /* AND 0xFEF6FFFF: clears HSEON(16), CSSON(19), PLLON(24) */
    CRM->CR &= 0xFEF6FFFFUL;
    /* BIC bit 18: clears HSEBYP */
    CRM->CR &= ~CRM_CR_HSEBYP;

    /* ---- 5. Clear PLL source/mult in CFG - fw 0x08021710 ---- */
    /* AND 0x1700FFFF: clears PLLMULT_L[21:18], PLLSRC[16],
     * PLLHEXTDIV[17], PLLMULT_H[30:29], PLLRANGE[31] */
    CRM->CFGR &= 0x1700FFFFUL;

    /* ---- 6. Clear MISC1 fields - fw 0x08021720 ---- */
    /* AND 0xFEFEFF00 */
    CRM->MISC1 &= 0xFEFEFF00UL;

    /* ---- 7. Clear clock interrupts - fw 0x0802172A ---- */
    CRM->CIR = 0x009F0000UL;

    /* ---- 8. Configure PLL: HEXT 8 MHz x 15 = 120 MHz - fw 0x08021730 ---- */
    clock_config();
#ifdef RT950_CUSTOM
    // The crystal supplies the running PLL. Leave the internal oscillator
    // available unless both the clock source and its ready flags agree.
    if ((CRM->CFGR & (CRM_CFGR_SWS_MASK | CRM_CFGR_PLLSRC)) ==
            (CRM_CFGR_SWS_PLL | CRM_CFGR_PLLSRC) &&
        (CRM->CR & (CRM_CR_HSERDY | CRM_CR_PLLRDY)) ==
            (CRM_CR_HSERDY | CRM_CR_PLLRDY))
        CRM->CR &= ~CRM_CR_HSIEN;
#endif

    /* ---- 9. Set VTOR ---- */
    /* OEM SystemInit writes 0x08000000 here (fw 0x08021734), then relocates
     * VTOR to the app base later (fw 0x0801886C).  We set it to our flash
     * origin immediately so that SysTick and other ISRs use our vector
     * table from the start. */
    SCB->VTOR = 0x08003000UL;

    /* ---- 10. NVIC Priority Group 4 (OEM rcc_periph_enable @ 0x08018750) ----
     * OEM sets PRIGROUP=3 → 4 preemption bits, 0 sub-priority bits.
     * Must be configured before any NVIC_SetPriority calls.
     * AIRCR requires VECTKEY (0x05FA) in upper 16 bits for write access. */
    SCB->AIRCR = SCB_AIRCR_VECTKEY | (3UL << 8);

    /* ---- 11. Enable GPIO port clocks (our addition) ---- */
    CRM->APB2EN |= CRM_APB2EN_AFIOEN
                  | CRM_APB2EN_IOPAEN
                  | CRM_APB2EN_IOPBEN
                  | CRM_APB2EN_IOPCEN
                  | CRM_APB2EN_IOPDEN
                  | CRM_APB2EN_IOPEEN;

    /* ---- 11b. Disable JTAG, keep SWD (OEM @ 0x80123D6) ----
     * OEM gpio_modes_init calls AFIO remap with 0x02000000 → SWJ_CFG=010.
     * This frees PA15, PB3, PB4 from JTAG function.
     * IOMUX_REMAP register (0x40010004) bits[26:24] = SWJ_CFG:
     *   000 = Full SWJ (JTAG+SWD)  - PA15,PB3,PB4 locked to JTAG
     *   010 = JTAG disabled, SWD only - PA15,PB3,PB4 freed as GPIO
     * Write is special: SWJ_CFG is write-only, must clear then set. */
    {
        volatile uint32_t *iomux_remap = (volatile uint32_t *)0x40010004UL;
        uint32_t val = *iomux_remap;
        val &= ~(7UL << 24);    /* clear SWJ_CFG[26:24] */
        val |=  (2UL << 24);    /* SWJ_CFG = 010: JTAG off, SWD on */
        *iomux_remap = val;
    }

    /* ---- 12. Start the application time base ---- */
    tick_init();

    /* ---- 13. Debug UART (compile-time optional) ---- */
    dbg_init();
    dbg_puts("\n[DBG] SystemInit OK: HCLK=120MHz VTOR=0x08003000\n");
    dbg_reg("[DBG]   CRM->CR=0x", CRM->CR);
    dbg_reg("[DBG]   CRM->CFGR=0x", CRM->CFGR);
}
