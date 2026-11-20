// SPDX-License-Identifier: GPL-3.0-or-later
#include "voice_calibration.hpp"
#include "voice_battery_limits.hpp"
#include "voice_sql.hpp"
extern "C" {
#include "drivers/gpio.h"
#include "board.h"
#include "drivers/dac_audio.h"
}

namespace rt950::voice {
namespace {
constexpr uint16_t Cs=GPIO_PIN_12;
constexpr uint16_t Pins=Cs|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
constexpr uint32_t PinModes=0xffff0000u;

bool wait_spi(uint32_t mask,bool set) {
    for(unsigned i=0;i<3000;++i)
        if(bool(SPI2->SR&mask)==set)return true;
    return false;
}

struct FactorySpi {
    void select(bool active) {
        if(active)gpio_clear_pin(GPIOB,Cs);
        else gpio_set_pin(GPIOB,Cs);
    }
    bool transfer(uint8_t sent,uint8_t &received) {
        if(!wait_spi(SPI_SR_TXE,true))return false;
        SPI2->DR=sent;
        if(!wait_spi(SPI_SR_RXNE,true))return false;
        received=uint8_t(SPI2->DR);
        return wait_spi(SPI_SR_BSY,false);
    }
};
}

template<class Load>
static bool with_factory_spi(Load load) {
    const uint32_t clock=CRM->APB1EN&CRM_APB1EN_SPI2EN;
    CRM->APB1EN|=CRM_APB1EN_SPI2EN;
    const uint32_t control=SPI2->CR1;
    const uint32_t i2s=SPI2->I2SCFGR;
    if((control&SPI_CR1_SPE) || (i2s&(1u<<10)) || (SPI2->SR&(SPI_SR_RXNE|SPI_SR_BSY))) {
        if(!clock)CRM->APB1EN&=~CRM_APB1EN_SPI2EN;
        return false; // Someone else owns the peripheral.
    }
    const uint32_t control2=SPI2->CR2;
    const uint32_t modes=GPIOB->CRH&PinModes;
    const uint16_t levels=uint16_t(GPIOB->ODR&Pins);
    gpio_set_pin(GPIOB,Cs);
    gpio_config_pin(GPIOB,Cs,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
    gpio_config_pin(GPIOB,GPIO_PIN_13,GPIO_MODE_OUT_2MHZ,GPIO_CNF_AF_PP);
    gpio_config_pin(GPIOB,GPIO_PIN_14,GPIO_MODE_INPUT,GPIO_CNF_FLOATING);
    gpio_config_pin(GPIOB,GPIO_PIN_15,GPIO_MODE_OUT_2MHZ,GPIO_CNF_AF_PP);
    SPI2->CR2=0;
    SPI2->I2SCFGR=i2s&~(1u<<11);
    // Stock 0.29 0x08017168 and 0x0801F636: mode 3, /4, software NSS.
    SPI2->CR1=SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_MSTR|(custom_clock_low?0u:1u<<3)|
              SPI_CR1_SSI|SPI_CR1_SSM|SPI_CR1_SPE;
    FactorySpi spi;
    const bool valid=load([&](uint32_t address,uint8_t *data,size_t length) {
        return read_factory_flash(spi,address,data,length);
    });
    SPI2->CR1=control;
    SPI2->CR2=control2;
    SPI2->I2SCFGR=i2s;
    GPIOB->CRH=(GPIOB->CRH&~PinModes)|modes;
    gpio_set_pin(GPIOB,levels);
    gpio_clear_pin(GPIOB,uint16_t(Pins&~levels));
    if(!clock)CRM->APB1EN&=~CRM_APB1EN_SPI2EN;
    return valid;
}

void initialize_factory_spi() {
    // End the updater's transaction before taking ownership at app startup.
    const uint32_t clock=CRM->APB1EN&CRM_APB1EN_SPI2EN;
    CRM->APB1EN|=CRM_APB1EN_SPI2EN;
    gpio_set_pin(GPIOB,Cs);
    gpio_config_pin(GPIOB,Cs,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
    SPI2->CR2=0;
    SPI2->CR1=0;
    SPI2->I2SCFGR=0;
    CRM->APB1RST|=CRM_APB1EN_SPI2EN;
    (void)CRM->APB1RST;
    CRM->APB1RST&=~CRM_APB1EN_SPI2EN;
    (void)CRM->APB1RST;
    if(!clock)CRM->APB1EN&=~CRM_APB1EN_SPI2EN;
}

bool read_board_uhf_calibration(UhfCalibration &out,CalibrationStatus *status) {
    auto result=CalibrationStatus::SpiUnavailable;
    const bool valid=with_factory_spi([&](auto read){
        result=inspect_uhf_calibration(read,out);
        return result==CalibrationStatus::Ready;
    });
    if(status)*status=result;
    return valid;
}

bool read_board_vhf_calibration(VhfPaCalibration &out) {
    out={};
    return with_factory_spi([&](auto read){return load_vhf_pa_calibration(read,out);});
}

bool read_board_cb_calibration(CbPaCalibration &out) {
    out={};
    return with_factory_spi([&](auto read){return load_cb_pa_calibration(read,out);});
}

bool read_board_battery_limits(BatteryTxLimits &limits) {
    return with_factory_spi([&](auto read) {
        uint8_t raw[7]{};
        return read(0xf200u,raw,sizeof raw) && limits.load(raw);
    });
}

bool read_board_sql_calibration(SqlCalibration &out) {
    SqlCalibration candidate;
    const bool valid=with_factory_spi([&](auto read) {
        return read(0xf0c0u,candidate.open,sizeof candidate.open) &&
            read(0xf0d0u,candidate.close,sizeof candidate.close) &&
            read(0xf0e0u,candidate.uhf,sizeof candidate.uhf) &&
            read(0xf0f0u,candidate.vhf,sizeof candidate.vhf) && candidate.valid();
    });
    if(valid)out=candidate;
    return valid;
}

void prepare_board_pa_bias() {
    // AT32F403A PA5 is DAC2_OUT. OEM enables DAC2 at 0x0800D4B6.
    // Clear the holding register before enabling the DAC output.
    CRM->APB1EN|=CRM_APB1EN_DACEN;
    board_pa_bias_write(0);
    gpio_config_pin(GPIOA,GPIO_PIN_5,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
    DAC->CR|=DAC_CR_EN2;
}

void clear_board_pa_bias() {
    // OEM 0x0801D824: clear DAC2 before other TX teardown.
    if(CRM->APB1EN&CRM_APB1EN_DACEN)board_pa_bias_write(0);
}

void release_board_pa_bias() {
    // Preload the GPIO latch before taking PA5 out of analog mode.
    gpio_clear_pin(GPIOA,GPIO_PIN_5);
    if(CRM->APB1EN&CRM_APB1EN_DACEN) {
        board_pa_bias_write(0);
        DAC->CR&=~DAC_CR_EN2;
    }
    gpio_config_pin(GPIOA,GPIO_PIN_5,GPIO_MODE_OUT_2MHZ,GPIO_CNF_PP);
}
}
