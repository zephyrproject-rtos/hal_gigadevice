/*!
    \file    gd32c2x1_rcu.c
    \brief   RCU driver

    \version 2025-08-08, V1.1.0, firmware for gd32c2x1
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32c2x1_rcu.h"

/* define clock source */
#define SEL_IRC48MDIV      0x00U
#define SEL_HXTAL          0x01U
#define SEL_IRC32K         0x02U
#define SEL_LXTAL          0x03U

/* define startup timeout count */
#define OSC_STARTUP_TIMEOUT         ((uint32_t)0x000FFFFFU)
#define LXTAL_STARTUP_TIMEOUT       ((uint32_t)0x03FFFFFFU)

/*!
    \brief      deinitialize the RCU (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
    \retval     none
*/
void rcu_deinit(void)
{
    /* enable IRC48M */
    RCU_CTL0 |= RCU_CTL0_IRC48MEN;
    while(0U == (RCU_CTL0 & RCU_CTL0_IRC48MSTB)) {
    }
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    /* reset CTL register */
    RCU_CTL0 &= ~(RCU_CTL0_HXTALEN | RCU_CTL0_HXTALBPS );
    /* reset CTL register */
    RCU_CTL0 &= ~(RCU_CTL0_HXTALEN | RCU_CTL0_CKMEN  | RCU_CTL0_HXTALBPS );
    /* reset RCU_CFG0 register */
    RCU_CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC  | RCU_CFG0_APBPSC | \
                   RCU_CFG0_CKOUT0SEL | RCU_CFG0_CKOUT0DIV |RCU_CFG0_CKOUT0SEL | RCU_CFG0_CKOUT0DIV );
    /* reset RCU_CFG1 register */
    RCU_CFG1 &= ~(RCU_CFG1_USART0SEL | RCU_CFG1_I2C0SEL | RCU_CFG1_I2C1SEL | RCU_CFG1_ADCSEL | \
                  RCU_CFG1_ADCPSC | RCU_CFG1_I2SSEL);
    RCU_INT = 0x00000000U;
}

/*!
    \brief      enable the peripherals clock (API_ID(0x0002U))
    \param[in]  periph: RCU peripherals, refer to rcu_periph_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FMC: FMC clock
      \arg        RCU_CRC: CRC clock
      \arg        RCU_DMA: DMA clock
      \arg        RCU_DMAMUX: DMAMUX clock
      \arg        RCU_GPIOx (x=A,B,C,D,F): GPIO ports clock
      \arg        RCU_SYSCFG: SYSCFG clock
      \arg        RCU_CMP: CMP clock
      \arg        RCU_WWDGT: WWDGT clock
      \arg        RCU_ADC: ADC clock
      \arg        RCU_TIMERx (x=0,2,13,15,16): TIMER clock
      \arg        RCU_SPIx (x=0,1): SPI clock
      \arg        RCU_USARTx (x=0,1,2): USART clock
      \arg        RCU_I2Cx (x=0,1): I2C clock
      \arg        RCU_DBGMCU: DBGMCU clock
      \arg        RCU_PMU: PMU clock
      \arg        RCU_RTC: RTC clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_enable(rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock (API_ID(0x0003U))
    \param[in]  periph: RCU peripherals, refer to rcu_periph_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_CRC: CRC clock
      \arg        RCU_DMA: DMA clock
      \arg        RCU_DMAMUX: DMAMUX clock
      \arg        RCU_GPIOx (x=A,B,C,D,F): GPIO ports clock
      \arg        RCU_SYSCFG: SYSCFG clock
      \arg        RCU_CMP: CMP clock
      \arg        RCU_WWDGT: WWDGT clock
      \arg        RCU_ADC: ADC clock
      \arg        RCU_TIMERx (x=0,1,2,5,13,15,16): TIMER clock
      \arg        RCU_SPIx (x=0,1): SPI clock
      \arg        RCU_USARTx (x=0,1,2): USART clock
      \arg        RCU_I2Cx (x=0,1): I2C clock
      \arg        RCU_DBGMCU: DBGMCU clock
      \arg        RCU_PMU: PMU clock
      \arg        RCU_RTC: RTC clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_disable(rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      enable the peripherals clock when sleep mode (API_ID(0x0004U))
    \param[in]  periph: RCU peripherals, refer to rcu_periph_sleep_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_SRAM_SLP: SRAM clock
      \arg        RCU_FMC_SLP: FMC clock
      \arg        RCU_CRC_SLP: CRC clock
      \arg        RCU_DMA_SLP: DMA clock 
      \arg        RCU_DMAMUX_SLP: DMAMUX clock
      \arg        RCU_GPIOx_SLP (x = A,B,C,D,F): GPIO ports clock
      \arg        RCU_SYSCFG_SLP: SYSCFG clock
      \arg        RCU_CMP_SLP: CMP clock
      \arg        RCU_WWDGT_SLP: WWDGT clock
      \arg        RCU_ADC_SLP: ADC clock
      \arg        RCU_TIMERx_SLP (x=0,2,13,15,16): TIMER clock
      \arg        RCU_SPIx_SLP (x = 0,1): SPI clock
      \arg        RCU_USARTx_SLP (x = 0,1,2): USART clock
      \arg        RCU_I2Cx_SLP (x = 0,1): I2C clock
      \arg        RCU_PMU_SLP: PMU clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock when sleep mode (API_ID(0x0005U))
    \param[in]  periph: RCU peripherals, refer to rcu_periph_sleep_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_SRAM_SLP: SRAM clock
      \arg        RCU_FMC_SLP: FMC clock
      \arg        RCU_CRC_SLP: CRC clock
      \arg        RCU_DMA_SLP: DMA clock 
      \arg        RCU_DMAMUX_SLP: DMAMUX clock
      \arg        RCU_GPIOx_SLP (x = A,B,C,D,F): GPIO ports clock
      \arg        RCU_SYSCFG_SLP: SYSCFG clock
      \arg        RCU_CMP_SLP: CMP clock
      \arg        RCU_WWDGT_SLP: WWDGT clock
      \arg        RCU_ADC_SLP: ADC clock
      \arg        RCU_TIMERx_SLP (x=0,2,13,15,16): TIMER clock
      \arg        RCU_SPIx_SLP (x = 0,1): SPI clock
      \arg        RCU_USARTx_SLP (x = 0,1,2): USART clock
      \arg        RCU_I2Cx_SLP (x = 0,1): I2C clock
      \arg        RCU_PMU_SLP: PMU clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}
/*!
    \brief      reset the peripherals (API_ID(0x0006U))
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_CRCRST: reset CRC
      \arg        RCU_DMARST: reset DMA
      \arg        RCU_DMAMUXRST: reset DMAMUX
      \arg        RCU_GPIOxRST (x = A,B,C,D,F): reset GPIO ports
      \arg        RCU_SYSCFGRST: reset SYSCFG
      \arg        RCU_CMPRST: reset CMP
      \arg        RCU_WWDGTRST: reset WWDGT
      \arg        RCU_ADCRST: reset ADC
      \arg        RCU_TIMERxRST (x=0,2,13,15,16): reset TIMER
      \arg        RCU_SPIxRST (x = 0,1): reset SPI
      \arg        RCU_USARTxRST (x = 0,1,2): reset USART
      \arg        RCU_I2CxRST (x = 0,1): reset I2C
      \arg        RCU_PMURST: reset PMU
    \param[out] none
    \retval     none
*/
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) |= BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      disable reset the peripheral (API_ID(0x0007U))
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_CRCRST: reset CRC
      \arg        RCU_DMARST: reset DMA
      \arg        RCU_DMAMUXRST: reset DMAMUX
      \arg        RCU_GPIOxRST (x = A,B,C,D,F): reset GPIO ports
      \arg        RCU_SYSCFGRST: reset SYSCFG
      \arg        RCU_CMPRST: reset CMP
      \arg        RCU_WWDGTRST: reset WWDGT
      \arg        RCU_ADCRST: reset ADC
      \arg        RCU_TIMERxRST (x=0,2,13,15,16): reset TIMER
      \arg        RCU_SPIxRST (x = 0,1): reset SPI
      \arg        RCU_USARTxRST (x = 0,1,2): reset USART
      \arg        RCU_I2CxRST (x = 0,1): reset I2C
      \arg        RCU_PMURST: reset PMU
    \param[out] none
    \retval     none
*/
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) &= ~BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      reset the BKP (API_ID(0x0008U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bkp_reset_enable(void)
{
    RCU_CTL1 |= RCU_CTL1_BKPRST;
}

/*!
    \brief      disable the BKP reset (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bkp_reset_disable(void)
{
    RCU_CTL1 &= ~RCU_CTL1_BKPRST;
}

/*!
    \brief      configure the system clock source (API_ID(0x000CU))
    \param[in]  ck_sys: system clock source select
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKSYSSRC_IRC48MDIV_SYS: select IRC48MDIV_SYS as the CK_SYS source
      \arg        RCU_CKSYSSRC_HXTAL: select CK_HXTAL as the CK_SYS source
      \arg        RCU_CKSYSSRC_IRC32K: select IRC32K as the CK_SYS source
      \arg        RCU_CKSYSSRC_LXTAL: select LXTLA as the CK_SYS source
    \param[out] none
    \retval     none
*/
void rcu_system_clock_source_config(uint32_t ck_sys)
{
    uint32_t cksys_source;
    cksys_source = RCU_CFG0;
    /* reset the SCS bits and set according to ck_sys */
    cksys_source &= ~RCU_CFG0_SCS;
    RCU_CFG0 = ((ck_sys & RCU_CFG0_SCS) | cksys_source);
}

/*!
    \brief      get the system clock source (API_ID(0x000DU))
    \param[in]  none
    \param[out] none
    \retval     which clock is selected as CK_SYS source
      \arg        RCU_CKSYSSRC_IRC48MDIV_SYS: select IRC48MDIV_SYS as the CK_SYS source
      \arg        RCU_CKSYSSRC_HXTAL: select CK_HXTAL as the CK_SYS source
      \arg        RCU_CKSYSSRC_IRC32K: select IRC32K as the CK_SYS source
      \arg        RCU_CKSYSSRC_LXTAL: select LXTLA as the CK_SYS source
*/
uint32_t rcu_system_clock_source_get(void)
{
    return (RCU_CFG0 & RCU_CFG0_SCSS);
}

/*!
    \brief      configure the AHB clock prescaler selection (API_ID(0x000EU))
    \param[in]  ck_ahb: AHB clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_AHB_CKSYS_DIVx, x=1, 2, 4, 8, 16, 64, 128, 256, 512: select CK_SYS/x as CK_AHB
    \param[out] none
    \retval     none
*/
void rcu_ahb_clock_config(uint32_t ck_ahb)
{
    uint32_t ahbpsc;
    ahbpsc = RCU_CFG0;
    /* reset the AHBPSC bits and set according to ck_ahb */
    ahbpsc &= ~RCU_CFG0_AHBPSC;
    RCU_CFG0 = ((ck_ahb & RCU_CFG0_AHBPSC) | ahbpsc);
}


/*!
    \brief      configure the APB clock prescaler selection (API_ID(0x000FU))
    \param[in]  ck_apb: APB clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB_CKAHB_DIV1: select CK_AHB as CK_APB
      \arg        RCU_APB_CKAHB_DIV2: select CK_AHB/2 as CK_APB
      \arg        RCU_APB_CKAHB_DIV4: select CK_AHB/4 as CK_APB
      \arg        RCU_APB_CKAHB_DIV8: select CK_AHB/8 as CK_APB
      \arg        RCU_APB_CKAHB_DIV16: select CK_AHB/16 as CK_APB
    \param[out] none
    \retval     none
*/
void rcu_apb_clock_config(uint32_t ck_apb)
{
    uint32_t apbpsc;
    apbpsc = RCU_CFG0;
    /* reset the APBPSC and set according to ck_apb */
    apbpsc &= ~RCU_CFG0_APBPSC;
    RCU_CFG0 = ((ck_apb & RCU_CFG0_APBPSC) | apbpsc);
}

/*!
    \brief      configure the ADC clock source and prescaler selection (API_ID(0x0010U))
    \param[in]  adc_clock_source: ADC clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_ADCSRC_CKSYS: ADC clock source select CK_SYS
      \arg        RCU_ADCSRC_IRC48M_PER: ADC clock source select CK_IRC48MDIV_PER
    \param[in]  ck_adc: ADC clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_ADCCK_DIV1: ADC clock prescaler select not divided
      \arg        RCU_ADCCK_DIV2: ADC clock prescaler select divided by 2
      \arg        RCU_ADCCK_DIV4: ADC clock prescaler select divided by 4
      \arg        RCU_ADCCK_DIV6: ADC clock prescaler select divided by 6
      \arg        RCU_ADCCK_DIV8: ADC clock prescaler select divided by 8
      \arg        RCU_ADCCK_DIV10: ADC clock prescaler select divided by 10
      \arg        RCU_ADCCK_DIV12: ADC clock prescaler select divided by 12
      \arg        RCU_ADCCK_DIV16: ADC clock prescaler select divided by 16
      \arg        RCU_ADCCK_DIV32: ADC clock prescaler select divided by 32
      \arg        RCU_ADCCK_DIV64: ADC clock prescaler select divided by 64
      \arg        RCU_ADCCK_DIV128: ADC clock prescaler select divided by 128
      \arg        RCU_ADCCK_DIV256: ADC clock prescaler select divided by 256
    \param[out] none
    \retval     none
*/
void rcu_adc_clock_config(uint32_t adc_clock_source, uint32_t ck_adc)
{
    uint32_t cfg1_val;

    /* Read the current value of RCU_CFG1 */
    cfg1_val = RCU_CFG1;

    /* Reset the ADCPSC and ADCSEL bits */
    cfg1_val &= ~RCU_CFG1_ADCPSC;
    cfg1_val &= ~RCU_CFG1_ADCSEL;

    /* Set the ADC clock source according to adc_clock_source */
    cfg1_val |= (adc_clock_source & RCU_CFG1_ADCSEL);

    /* Set the ADC clock prescaler according to ck_adc */
    cfg1_val |= (ck_adc & RCU_CFG1_ADCPSC);

    /* Write back the modified value to RCU_CFG1 */
    RCU_CFG1 = cfg1_val;
}

/*!
    \brief      configure the CK_OUT0 clock source and divider (API_ID(0x0012U))
    \param[in]  ckout_src: CK_OUT0 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKOUT0SRC_NONE:   no clock selected
      \arg        RCU_CKOUT0SRC_CKSYS:  CK_OUT0 clock source select CKSYS
      \arg        RCU_CKOUT0SRC_IRC48M: CK_OUT0 clock source select IRC48M
      \arg        RCU_CKOUT0SRC_HXTAL:  CK_OUT0 clock source select HXTAL
      \arg        RCU_CKOUT0SRC_IRC32K: CK_OUT0 clock source select IRC32K
      \arg        RCU_CKOUT0SRC_LXTAL:  CK_OUT0 clock source select LXTAL
    \param[in]  ckout_div: CK_OUT0 divider
      \arg        RCU_CKOUT0_DIVx(x=1,2,4,8,16,32,64,128): CK_OUT is divided by x
    \param[out] none
    \retval     none
*/
void rcu_ckout0_config(uint32_t ckout0_src, uint32_t ckout0_div)
{
    uint32_t ckout;
    ckout = RCU_CFG0;
    /* reset the CKOUT0SEL and CKOUT0DIV bits and set according to ckout_src and ckout_div */
    ckout &= ~(RCU_CFG0_CKOUT0SEL | RCU_CFG0_CKOUT0DIV );
    RCU_CFG0 = (ckout | (ckout0_src & RCU_CFG0_CKOUT0SEL) | (ckout0_div & RCU_CFG0_CKOUT0DIV));
}

/*!
    \brief      configure the CK_OUT1 clock source and divider (API_ID(0x0013U))
    \param[in]  ckout_src: CK_OUT1 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKOUT1SRC_NONE:   no clock selected
      \arg        RCU_CKOUT1SRC_CKSYS:  CK_OUT1 clock source select CKSYS
      \arg        RCU_CKOUT1SRC_IRC48M: CK_OUT1 clock source select IRC48M
      \arg        RCU_CKOUT1SRC_HXTAL:  CK_OUT1 clock source select HXTAL
      \arg        RCU_CKOUT1SRC_IRC32K: CK_OUT1 clock source select IRC32K
      \arg        RCU_CKOUT1SRC_LXTAL:  CK_OUT1 clock source select LXTAL
    \param[in]  ckout_div: CK_OUT divider
      \arg        RCU_CKOUT1_DIVx(x=1,2,4,8,16,32,64,128): CK_OUT is divided by x
    \param[out] none
    \retval     none
*/
void rcu_ckout1_config(uint32_t ckout1_src, uint32_t ckout1_div)
{
    uint32_t ckout;
    ckout = RCU_CFG0;
    /* reset the CKOUT1SEL and CKOUT1DIV bits and set according to ckout_src and ckout_div */
    ckout &= ~(RCU_CFG0_CKOUT1SEL | RCU_CFG0_CKOUT1DIV );
    RCU_CFG0 = (ckout | (ckout1_src & RCU_CFG0_CKOUT1SEL) | (ckout1_div | RCU_CFG0_CKOUT1DIV));
}

/*!
    \brief      enable the low speed clock output (API_ID(0x0014U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lsckout_enable(void)
{
    RCU_CTL1 |= RCU_CTL1_LSCKOUTEN;
}

/*!
    \brief      disable the low speed clock output (API_ID(0x0015U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lsckout_disable(void)
{
    RCU_CTL1 &= ~RCU_CTL1_LSCKOUTEN;
}

/*!
    \brief      configure the LSCKOUT clock source (API_ID(0x0016U))
    \param[in]  lsc_ckout_src: LSCKOUT clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_LSCKOUTSRC_IRC32K: IRC32K clock selected
      \arg        RCU_LSCKOUTSRC_LXTAL: LXTAL selected
    \param[out] none
    \retval     none
*/
void rcu_lsckout_config(uint32_t lsckout_src)
{
    uint32_t reg;

    reg = RCU_CTL1;
    /* reset the LSCKOUTSEL */
    reg &= ~(RCU_CTL1_LSCKOUTSEL);
    RCU_CTL1 = (reg | (lsckout_src & (RCU_CTL1_LSCKOUTSEL)));
}

/*!
    \brief      configure the USART clock source selection (API_ID(0x0017U))
    \param[in]  usart_idx: IDX_USART0
    \param[in]  ck_usart: USART clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USART0SRC_CKAPB: CK_USART select CK_APB
      \arg        RCU_USART0SRC_CKSYS: CK_USART select CK_SYS
      \arg        RCU_USART0SRC_IRC48MDIV_PER: CK_USART select CK_IRC48MDIV_PER
      \arg        RCU_USART0SRC_LXTAL: CK_USART select LXTAL
    \param[out] none
    \retval     none
*/
void rcu_usart_clock_config(usart_idx_enum usart_idx, uint32_t ck_usart)
{
    switch(usart_idx) {
    case IDX_USART0:
        {
            uint32_t reg_val = RCU_CFG1;
            /* reset the USART0SEL bits and set according to ck_usart */
            reg_val &= ~RCU_CFG1_USART0SEL;
            reg_val |= (ck_usart & RCU_CFG1_USART0SEL);
            RCU_CFG1 = reg_val;
        }
        break;
    default:
        break;
    }
}

/*!
    \brief      configure the I2Cx(x=0,1) clock source selection (API_ID(0x0018U))
    \param[in]  i2c_idx: IDX_I2Cx(x=0,1)
    \param[in]  ck_i2c: I2C clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_I2CSRC_CKAPB: CK_I2C select CK_APB
      \arg        RCU_I2CSRC_CKSYS: CK_I2C select CK_SYS
      \arg        RCU_I2CSRC_IRC48MDIV_PER: CK_I2C select CK_IRC48MDIV_PER
    \param[out] none
    \retval     none
*/
void rcu_i2c_clock_config(i2c_idx_enum i2c_idx, uint32_t ck_i2c)
{
    switch(i2c_idx) {
    case IDX_I2C0:
        {
        uint32_t reg_val = RCU_CFG1;
        /* reset the I2C0SEL bits and set according to ck_i2c */
        reg_val &= ~RCU_CFG1_I2C0SEL;
        reg_val |= (ck_i2c & RCU_CFG1_I2C0SEL);
        RCU_CFG1 = reg_val;
        }
        break;
    case IDX_I2C1:
        {
        uint32_t reg_val = RCU_CFG1;
        /* reset the I2C1SEL bits and set according to ck_i2c */
        reg_val &= ~RCU_CFG1_I2C1SEL;
        reg_val |= ((uint32_t)ck_i2c & RCU_CFG1_I2C0SEL) << 2U;
        RCU_CFG1 = reg_val;
        }
        break;
    default:
        break;
    }
}

/*!
    \brief      configure the I2S clock source selection (API_ID(0x0019U))
    \param[in]  ck_i2s: I2S clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_I2SSRC_CKSYS: CK_I2S select CK_SYS
      \arg        RCU_I2SSRC_IRC48MDIV_PER: CK_I2S select IRC48MDIV_PER
      \arg        RCU_I2SSRC_CKIN: CK_I2S select I2S_CKIN
    \param[out] none
    \retval     none
*/
void rcu_i2s_clock_config(uint32_t ck_i2s)
{
    uint32_t reg_val = RCU_CFG1;
    /* reset the I2SSEL bits and set according to ck_i2s */
    reg_val &= ~RCU_CFG1_I2SSEL;
    reg_val |= (ck_i2s & RCU_CFG1_I2SSEL);
    RCU_CFG1 = reg_val;
}

/*!
    \brief      configure the IRC48MDIV_SYS clock selection (API_ID(0x001AU))
    \param[in]  ck_irc48mdiv_sys: IRC48MDIV clock selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_IRC48MDIV_SYS_1: CK_IRC48MDIV_SYS select CK_IRC48M
      \arg        RCU_IRC48MDIV_SYS_2: CK_IRC48MDIV_SYS select CK_IRC48M divided by 2
      \arg        RCU_IRC48MDIV_SYS_4: CK_IRC48MDIV_SYS select CK_IRC48M divided by 4
      \arg        RCU_IRC48MDIV_SYS_8: CK_IRC48MDIV_SYS select CK_IRC48M divided by 8
      \arg        RCU_IRC48MDIV_SYS_16: CK_IRC48MDIV_SYS select CK_IRC48M divided by 16
      \arg        RCU_IRC48MDIV_SYS_32: CK_IRC48MDIV_SYS select CK_IRC48M divided by 32
      \arg        RCU_IRC48MDIV_SYS_64: CK_IRC48MDIV_SYS select CK_IRC48M divided by 64
      \arg        RCU_IRC48MDIV_SYS_128: CK_IRC48MDIV_SYS select CK_IRC48M divided by 128

    \param[out] none
    \retval     none
*/
void rcu_irc48mdiv_sys_clock_config(uint32_t ck_irc48mdiv_sys)
{
    uint32_t reg_val = RCU_CTL0;
    /* reset the IRC48MDIV_SYS bits and set according to ck_irc48mdiv_sys */
    reg_val &= ~RCU_CTL0_IRC48MDIV_SYS;
    reg_val |= (ck_irc48mdiv_sys & RCU_CTL0_IRC48MDIV_SYS);
    RCU_CTL0 = reg_val;
}


/*!
    \brief      configure the IRC48MDIV clock selection (API_ID(0x001BU))
    \param[in]  ck_irc48mdiv_per: IRC48MDIV clock selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_IRC48MDIV_PER_1: CK_IRC48MDIV_PER select CK_IRC48M
      \arg        RCU_IRC48MDIV_PER_2: CK_IRC48MDIV_PER select CK_IRC48M divided by 2
      \arg        RCU_IRC48MDIV_PER_3: CK_IRC48MDIV_PER select CK_IRC48M divided by 3
      \arg        RCU_IRC48MDIV_PER_4: CK_IRC48MDIV_PER select CK_IRC48M divided by 4
      \arg        RCU_IRC48MDIV_PER_5: CK_IRC48MDIV_PER select CK_IRC48M divided by 5
      \arg        RCU_IRC48MDIV_PER_6: CK_IRC48MDIV_PER select CK_IRC48M divided by 6
      \arg        RCU_IRC48MDIV_PER_7: CK_IRC48MDIV_PER select CK_IRC48M divided by 7
      \arg        RCU_IRC48MDIV_PER_8: CK_IRC48MDIV_PER select CK_IRC48M divided by 8

    \param[out] none
    \retval     none
*/
void rcu_irc48mdiv_per_clock_config(uint32_t ck_irc48mdiv_per)
{
    uint32_t reg_val = RCU_CTL0;
    /* reset the IRC48MDIV_PER bits and set according to ck_irc48mdiv_per */
    reg_val &= ~RCU_CTL0_IRC48MDIV_PER;
    reg_val |= (ck_irc48mdiv_per & RCU_CTL0_IRC48MDIV_PER);
    RCU_CTL0 = reg_val;
}

/*!
    \brief      configure the RTC clock source selection (API_ID(0x001CU))
    \param[in]  rtc_clock_source: RTC clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_RTCSRC_NONE: no clock selected
      \arg        RCU_RTCSRC_LXTAL: CK_LXTAL selected as RTC source clock
      \arg        RCU_RTCSRC_IRC32K: CK_IRC32K selected as RTC source clock
      \arg        RCU_RTCSRC_HXTAL_DIV32: CK_HXTAL/32 selected as RTC source clock
    \param[out] none
    \retval     none
*/
void rcu_rtc_clock_config(uint32_t rtc_clock_source)
{
    /* reset the RTCSRC bits and set according to rtc_clock_source */
    uint32_t reg_val = RCU_CTL1;
    reg_val &= ~RCU_CTL1_RTCSRC;
    reg_val |= (rtc_clock_source & RCU_CTL1_RTCSRC);
    RCU_CTL1 = reg_val;
}

/*!
    \brief      configure the LXTAL drive capability (API_ID(0x001DU))
    \param[in]  lxtal_dricap: drive capability of LXTAL
                only one parameter can be selected which is shown as below:
      \arg        RCU_LXTAL_LOWDRI: lower driving capability
      \arg        RCU_LXTAL_HIGHDRI: higher driving capability
    \param[out] none
    \retval     none
*/
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap)
{
    /* reset the LXTALDRI bits and set according to lxtal_dricap */
    uint32_t reg_val = RCU_CTL1;
    reg_val &= ~RCU_CTL1_LXTALDRI;
    reg_val |= (lxtal_dricap & RCU_CTL1_LXTALDRI);
    RCU_CTL1 = reg_val;
}

/*!
    \brief      wait until oscillator stabilization flags is SET (API_ID(0x001EU))
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC32K: IRC32K
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
    \note       This function includes timeout exit scenarios.
                Modify according to the user's actual usage scenarios.
*/
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci)
{
    uint32_t stb_cnt = 0U;
    ErrStatus reval = ERROR;
    FlagStatus osci_stat = RESET;
    switch(osci) {
    case RCU_HXTAL:
        /* wait until HXTAL is stabilization and osci_stat is not more than timeout */
        while((RESET == osci_stat) && (HXTAL_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_HXTALSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_HXTALSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait LXTAL stable */
    case RCU_LXTAL:
        while((RESET == osci_stat) && (LXTAL_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_LXTALSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_LXTALSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait IRC48M stable */
    case RCU_IRC48M:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_IRC48MSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_IRC48MSTB)) {
            reval = SUCCESS;
        }
        break;

    /* wait IRC32K stable */
    case RCU_IRC32K:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_IRC32KSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_IRC32KSTB)) {
            reval = SUCCESS;
        }
        break;

    default:
        break;
    }
    /* return value */
    return reval;
}

/*!
    \brief      turn on the oscillator (API_ID(0x001FU))
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC32K: IRC32K
    \param[out] none
    \retval     none
*/
void rcu_osci_on(rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) |= BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      turn off the oscillator (API_ID(0x0022U))
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC32K: IRC32K
    \param[out] none
    \retval     none
*/
void rcu_osci_off(rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) &= ~BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it (API_ID(0x0023U))
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
    \param[out] none
    \retval     none
*/
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci)
{
    uint32_t reg;
    switch(osci) {
    case RCU_HXTAL:
        /* HXTALEN must be reset before enable the oscillator bypass mode */
        reg = RCU_CTL0;
        RCU_CTL0 &= ~RCU_CTL0_HXTALEN;
        RCU_CTL0 = (reg | RCU_CTL0_HXTALBPS);
        break;
    case RCU_LXTAL:
        /* LXTALEN must be reset before enable the oscillator bypass mode */
        reg = RCU_CTL1;
        RCU_CTL1 &= ~RCU_CTL1_LXTALEN;
        RCU_CTL1 = (reg | RCU_CTL1_LXTALBPS);
        break;
    default:
        break;
    }
}

/*!
    \brief      disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it (API_ID(0x0024U))
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
    \param[out] none
    \retval     none
*/
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci)
{
    uint32_t reg;
    switch(osci) {
    case RCU_HXTAL:
        /* HXTALEN must be reset before disable the oscillator bypass mode */
        reg = RCU_CTL0;
        RCU_CTL0 &= ~RCU_CTL0_HXTALEN;
        RCU_CTL0 = (reg & (~RCU_CTL0_HXTALBPS));
        break;
    case RCU_LXTAL:
        /* LXTALEN must be reset before disable the oscillator bypass mode */
        reg = RCU_CTL1;
        RCU_CTL1 &= ~RCU_CTL1_LXTALEN;
        RCU_CTL1 = (reg & (~RCU_CTL1_LXTALBPS));
        break;
    default:
        break;
    }
}

/*!
    \brief      set the IRC48M adjust value (API_ID(0x0025U))
    \param[in]  irc48m_adjval: IRC48M adjust value, must be between 0 and 0x3F
    \param[out] none
    \retval     none
*/
void rcu_irc48m_adjust_value_set(uint8_t irc48m_adjval)
{
    uint32_t adjust;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_IRC48M_ADJ(irc48m_adjval)) {
        fw_debug_report_err(RCU_MODULE_ID, API_ID(0x0025U), ERR_PARAM_INVALID);
    }else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        adjust = RCU_CTL0;
        /* reset the IRC48MADJ bits and set according to irc48m_adjval */
        adjust &= ~RCU_CTL0_IRC48MADJ;
        RCU_CTL0 = (adjust | (((uint32_t)irc48m_adjval) << 2U));
    }
}

/*!
    \brief      enable LXTAL stabilization reset (API_ID(0x0026U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lxtal_stab_reset_enable(void)
{
    RCU_CTL1 |= RCU_CTL1_LXTALSTBRST;
}

/*!
    \brief      disable LXTAL stabilization reset (API_ID(0x0027U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lxtal_stab_reset_disable(void)
{
    RCU_CTL1 &= ~RCU_CTL1_LXTALSTBRST;
}

/*!
    \brief      enable the HXTAL clock monitor (API_ID(0x0028U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_clock_monitor_enable(void)
{
    RCU_CTL0 |= RCU_CTL0_CKMEN;
}

/*!
    \brief      disable the HXTAL clock monitor (API_ID(0x0029U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_clock_monitor_disable(void)
{
    RCU_CTL0 &= ~RCU_CTL0_CKMEN;
}

/*!
    \brief      enable the LXTAL clock monitor (API_ID(0x002AU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lxtal_clock_monitor_enable(void)
{
    RCU_CTL1 |= RCU_CTL1_LCKMEN;
}

/*!
    \brief      disable the LXTAL clock monitor (API_ID(0x002BU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lxtal_clock_monitor_disable(void)
{
    RCU_CTL1 &= ~RCU_CTL1_LCKMEN;
}


/*!
    \brief      get the system clock, bus and peripheral clock frequency (API_ID(0x002CU))
    \param[in]  clock: the clock frequency which to get
                only one parameter can be selected which is shown as below:
      \arg        CK_SYS: system clock frequency
      \arg        CK_AHB: AHB clock frequency
      \arg        CK_APB: APB clock frequency
      \arg        CK_ADC: ADC clock frequency
      \arg        CK_USART0: USART0 clock frequency
    \param[out] none
    \retval     clock frequency of system, AHB, APB, ADC or USRAT0
*/
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock)
{
    uint32_t sws, ck_freq = 0U;
    uint32_t cksys_freq, ahb_freq, apb_freq, irc48m_per_freq;
    uint32_t adc_freq, usart_freq;
    uint32_t idx, clk_exp;
    /* exponent of AHB, APB, ADC clock divider */
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t apb_exp[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    const uint8_t IRC48M_exp[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    const uint8_t irc48m_per_exp[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint16_t adc_exp[12] = {1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256};
    
    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch(sws) {
    /* IRC48M is selected as CK_SYS */
    case SEL_IRC48MDIV:
        idx = GET_BITS(RCU_CTL0, 29, 31);
        clk_exp = IRC48M_exp[idx];
        cksys_freq = IRC48M_VALUE >> clk_exp ;
        break;
    /* HXTAL is selected as CK_SYS */
    case SEL_HXTAL:
        cksys_freq = HXTAL_VALUE;
        break;
    /* IRC32k is selected as CK_SYS */
    case SEL_IRC32K:
        cksys_freq = IRC32K_VALUE;
        break;
    /* LXTAL is selected as CK_SYS */
    default:
        cksys_freq = LXTAL_VALUE;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    ahb_freq = cksys_freq >> clk_exp;
    
    /* calculate APB clock frequency */
    idx = GET_BITS(RCU_CFG0, 11, 13);
    clk_exp = apb_exp[idx];
    apb_freq = ahb_freq >> clk_exp;
    
    idx =  GET_BITS(RCU_CTL0, 25, 27);
    clk_exp = irc48m_per_exp[idx];
    irc48m_per_freq = IRC48M_VALUE / clk_exp;
    /* return the clocks frequency */
    switch(clock) {
    case CK_SYS:
        ck_freq = cksys_freq;
        break;
    case CK_AHB:
        ck_freq = ahb_freq;
        break;
    case CK_APB:
        ck_freq = apb_freq;
        break;
    case CK_ADC:
        /* calculate ADC clock frequency */
        if(RCU_ADCSRC_CKSYS == (RCU_CFG1 & RCU_CFG1_ADCSEL)) {
            adc_freq = cksys_freq;
        } else if(RCU_ADCSRC_IRC48M_PER == (RCU_CFG1 & RCU_CFG1_ADCSEL)){
            adc_freq = irc48m_per_freq;
        } else {
            adc_freq = HXTAL_VALUE;
        }
        idx = GET_BITS(RCU_CFG1, 9, 12);
        if(idx < sizeof(adc_exp) / sizeof(adc_exp[0])){
            clk_exp = adc_exp[idx];
            ck_freq = adc_freq / clk_exp;
        } else {
            ck_freq = 0U;
        }
        break;
    case CK_USART0:
        /* calculate USART0 clock frequency */
        if(RCU_USART0SRC_CKAPB == (RCU_CFG1 & RCU_CFG1_USART0SEL)) {
            usart_freq = apb_freq;
        } else if(RCU_USART0SRC_CKSYS == (RCU_CFG1 & RCU_CFG1_USART0SEL)) {
            usart_freq = cksys_freq;
        } else if(RCU_USART0SRC_LXTAL == (RCU_CFG1 & RCU_CFG1_USART0SEL)) {
            usart_freq = LXTAL_VALUE;
        } else if(RCU_USART0SRC_IRC48MDIV_PER == (RCU_CFG1 & RCU_CFG1_USART0SEL)) {
            /* calculate IRC48MDIV clock frequency */
            idx = GET_BITS(RCU_CTL0, 25, 27);
            usart_freq = IRC48M_VALUE /(idx + 1U);
        } else if(RCU_USART0SRC_CKSYS == (RCU_CFG1 & RCU_CFG1_USART0SEL)) {
            usart_freq = cksys_freq;
        }else {
            usart_freq = 0U;
        }
        ck_freq = usart_freq;
        break;
    default:
        break;
    }
    return ck_freq;
}

/*!
    \brief      get the clock stabilization and periphral reset flags (API_ID(0x002DU))
    \param[in]  flag: the clock stabilization and periphral reset flags, refer to rcu_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FLAG_IRC32KSTB: IRC32K stabilization flag
      \arg        RCU_FLAG_LXTALSTB: LXTAL stabilization flag
      \arg        RCU_FLAG_IRC48MSTB: IRC48M stabilization flag
      \arg        RCU_FLAG_HXTALSTB: HXTAL stabilization flag
      \arg        RCU_FLAG_OBLRST: option byte loader reset flag
      \arg        RCU_FLAG_EPRST: external pin reset flag
      \arg        RCU_FLAG_PORRST: power reset flag
      \arg        RCU_FLAG_SWRST: software reset flag
      \arg        RCU_FLAG_FWDGTRST: FWDGT reset flag
      \arg        RCU_FLAG_WWDGTRST: WWDGT reset flag
      \arg        RCU_FLAG_LPRST: low-power reset flag
      \arg        RCU_FLAG_LCKCMD: LXTAL clock failure detection flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rcu_flag_get(rcu_flag_enum flag)
{
    FlagStatus reval;
    if(0U != (RCU_REG_VAL(flag) & BIT(RCU_BIT_POS(flag)))) {
          reval = SET;
      } else {
          reval = RESET;
      }
      return reval;
}

/*!
    \brief      clear the reset flag (API_ID(0x002EU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_all_reset_flag_clear(void)
{
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
}

/*!
    \brief      enable the stabilization interrupt (API_ID(0x002FU))
    \param[in]  stab_int: clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC32KSTB: IRC32K stabilization interrupt enable
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt enable
      \arg        RCU_INT_IRC48MSTB: IRC48M stabilization interrupt enable
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt enable
    \param[out] none
    \retval     none
*/
void rcu_interrupt_enable(rcu_int_enum stab_int)
{
    RCU_REG_VAL(stab_int) |= BIT(RCU_BIT_POS(stab_int));
}

/*!
    \brief      disable the stabilization interrupt (API_ID(0x0030U))
    \param[in]  stab_int: clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC32KSTB: IRC32K stabilization interrupt disable
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt disable
      \arg        RCU_INT_IRC48MSTB: IRC48M stabilization interrupt disable
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt disable
    \param[out] none
    \retval     none
*/
void rcu_interrupt_disable(rcu_int_enum stab_int)
{
    RCU_REG_VAL(stab_int) &= ~BIT(RCU_BIT_POS(stab_int));
}

/*!
    \brief      get the clock stabilization interrupt and ckm flags (API_ID(0x0031U))
    \param[in]  int_flag: interrupt and ckm flags, refer to rcu_int_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC32KSTB: IRC32K stabilization interrupt flag
      \arg        RCU_INT_FLAG_LXTALSTB: LXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_IRC48MSTB: IRC48M stabilization interrupt flag
      \arg        RCU_INT_FLAG_HXTALSTB: HXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_LXTALCKM: LXTAL clock stuck interrupt flag
      \arg        RCU_INT_FLAG_CKM: HXTAL clock stuck interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag)
{
    FlagStatus reval; 
    if(0U != (RCU_REG_VAL(int_flag) & BIT(RCU_BIT_POS(int_flag)))) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear the interrupt flags (API_ID(0x0032U))
    \param[in]  int_flag_clear: clock stabilization and stuck interrupt flags clear, refer to rcu_int_flag_clear_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC32KSTB_CLR: IRC32K stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_LXTALSTB_CLR: LXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_IRC48MSTB_CLR: IRC48M stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_HXTALSTB_CLR: HXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_LXTALCKM_CLR: LXTAL clock stuck interrupt flag clear
      \arg        RCU_INT_FLAG_CKM_CLR: clock stuck interrupt flag clear
    \param[out] none
    \retval     none
*/
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag_clear)
{
    RCU_REG_VAL(int_flag_clear) |= BIT(RCU_BIT_POS(int_flag_clear));
}
