/*!
    \file    gd32a50x_rcu.c
    \brief   RCU driver

    \version 2022-01-30, V1.0.0, firmware for GD32A50x
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

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

#include "gd32a50x_rcu.h"

/* define startup timeout count */
#define OSC_STARTUP_TIMEOUT         ((uint32_t)0x000FFFFFU)
#define LXTAL_STARTUP_TIMEOUT       ((uint32_t)0x03FFFFFFU)

/* RCU IRC8M adjust value mask and offset*/
#define RCU_IRC8M_ADJUST_MASK       ((uint8_t)0x1FU)
#define RCU_IRC8M_ADJUST_OFFSET     ((uint32_t)3U)

/*!
    \brief      deinitialize the RCU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_deinit(void)
{
    /* enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;
    if(ERROR == rcu_osci_stab_wait(RCU_IRC8M)) {
        while(1) {
        }
    }

    RCU_CFG0 &= ~RCU_CFG0_SCS;
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_IRC8M) {
    }
    /* reset CTL register */
    RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN | RCU_CTL_HXTALSCAL | RCU_CTL_LCKMEN | RCU_CTL_PLLMEN);
    RCU_CTL &= ~RCU_CTL_HXTALBPS;
    /* reset CFG0 register */
    RCU_CFG0 = 0x00020000U;
    /* reset INT and CFG1 and CFG2 register */
    RCU_INT = 0x00FF0000U;
    RCU_CFG1 = 0x00000000U;
    RCU_CFG2 = 0x00000000U;
}

/*!
    \brief      enable the peripherals clock
    \param[in]  periph: RCU peripherals, refer to rcu_periph_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_DMAx (x=0,1): DMA clock
      \arg        RCU_DMAMUX: DMAMUX clock
      \arg        RCU_CRC: CRC clock
      \arg        RCU_GPIOx (x=A,B,C,D,E,F): GPIO ports clock
      \arg        RCU_SYSCFG: SYSCFG clock
      \arg        RCU_CMP: CMP clock
      \arg        RCU_ADCx (x=0,1): ADCx clock
      \arg        RCU_TIMERx (x=0,1,5,6,7,19,20): TIMER clock
      \arg        RCU_SPIx (x=0,1): SPIx clock
      \arg        RCU_USARTx (x=0,1,2): USARTx clock
      \arg        RCU_MFCOM: MFCOM clock
      \arg        RCU_TRIGSEL: TRIGSEL clock
      \arg        RCU_CANx (x=0,1): CANx clock
      \arg        RCU_I2Cx (x=0,1): I2Cx clock
      \arg        RCU_WWDGT: WWDGT clock
      \arg        RCU_BKP: BKP clock
      \arg        RCU_PMU: PMU clock
      \arg        RCU_DAC: DAC clock
      \arg        RCU_RTC: RTC clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_enable(rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock
    \param[in]  periph: RCU peripherals, refer to rcu_periph_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_DMAx (x=0,1): DMA clock
      \arg        RCU_DMAMUX: DMAMUX clock
      \arg        RCU_CRC: CRC clock
      \arg        RCU_GPIOx (x=A,B,C,D,E,F): GPIO ports clock
      \arg        RCU_SYSCFG: SYSCFG clock
      \arg        RCU_CMP: CMP clock
      \arg        RCU_ADCx (x=0,1): ADCx clock
      \arg        RCU_TIMERx (x=0,1,5,6,7,19,20): TIMER clock
      \arg        RCU_SPIx (x=0,1): SPIx clock
      \arg        RCU_USARTx (x=0,1,2): USARTx clock
      \arg        RCU_MFCOM: MFCOM clock
      \arg        RCU_TRIGSEL: TRIGSEL clock
      \arg        RCU_CANx (x=0,1): CANx clock
      \arg        RCU_I2Cx (x=0,1): I2Cx clock
      \arg        RCU_WWDGT: WWDGT clock
      \arg        RCU_BKP: BKP clock
      \arg        RCU_PMU: PMU clock
      \arg        RCU_DAC: DAC clock
      \arg        RCU_RTC: RTC clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_disable(rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      reset the peripherals
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_DMAxRST (x=0,1): DMA reset enable
      \arg        RCU_DMAMUXRST: DMAMUX reset enable
      \arg        RCU_MFCOMRST: MFCOM reset enable
      \arg        RCU_CRCRST: CRC reset enable
      \arg        RCU_GPIOxRST (x=A,B,C,D,E,F): GPIO ports reset enable
      \arg        RCU_SYSCFGRST: SYSCFG reset enable
      \arg        RCU_CMPRST: CMP reset enable
      \arg        RCU_ADCxRST (x=0,1): ADCx reset enable
      \arg        RCU_TIMERxRST (x=0,1,5,6,7,19,20): TIMER reset enable
      \arg        RCU_SPIxRST (x=0,1): SPIx reset enable
      \arg        RCU_USARTxRST (x=0,1,2): USARTx reset enable
      \arg        RCU_CANxRST (x=0,1): CANx reset enable
      \arg        RCU_I2CxRST(x=0,1): I2Cx reset enable
      \arg        RCU_WWDGTRST: WWDGT reset enable
      \arg        RCU_PMURST: PMU reset enable
      \arg        RCU_DACRST: DAC reset enable
    \param[out] none
    \retval     none
*/
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) |= BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      disable reset the peripheral
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_DMAxRST (x=0,1): DMA reset enable
      \arg        RCU_DMAMUXRST: DMAMUX reset enable
      \arg        RCU_MFCOMRST: MFCOM reset enable
      \arg        RCU_CRCRST: CRC reset enable
      \arg        RCU_GPIOxRST (x=A,B,C,D,E,F): GPIO ports reset enable
      \arg        RCU_SYSCFGRST: SYSCFG reset enable
      \arg        RCU_CMPRST: CMP reset enable
      \arg        RCU_ADCxRST (x=0,1): ADCx reset enable
      \arg        RCU_TIMERxRST (x=0,1,5,6,7,19,20): TIMER reset enable
      \arg        RCU_SPIxRST (x=0,1): SPIx reset enable
      \arg        RCU_USARTxRST (x=0,1,2): USARTx reset enable
      \arg        RCU_CANxRST (x=0,1): CANx reset enable
      \arg        RCU_I2CxRST(x=0,1): I2Cx reset enable
      \arg        RCU_WWDGTRST: WWDGT reset enable
      \arg        RCU_PMURST: PMU reset enable
      \arg        RCU_DACRST: DAC reset enable
    \param[out] none
    \retval     none
*/
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) &= ~BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      enable the peripherals clock when in sleep mode
    \param[in]  periph: RCU peripherals, refer to rcu_periph_sleep_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FMC_SLP: FMC clock
      \arg        RCU_SRAM_SLP: SRAM clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock when in sleep mode
    \param[in]  periph: RCU peripherals, refer to rcu_periph_sleep_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FMC_SLP: FMC clock
      \arg        RCU_SRAM_SLP: SRAM clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      reset the BKP domain control register
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bkp_reset_enable(void)
{
    RCU_BDCTL |= RCU_BDCTL_BKPRST;
}

/*!
    \brief      disable the BKP domain control register reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bkp_reset_disable(void)
{
    RCU_BDCTL &= ~RCU_BDCTL_BKPRST;
}

/*!
    \brief      configure the system clock source
    \param[in]  ck_sys: system clock source select
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKSYSSRC_IRC8M: select CK_IRC8M as the CK_SYS source
      \arg        RCU_CKSYSSRC_HXTAL: select CK_HXTAL as the CK_SYS source
      \arg        RCU_CKSYSSRC_PLL: select CK_PLL as the CK_SYS source
    \param[out] none
    \retval     none
*/
void rcu_system_clock_source_config(uint32_t ck_sys)
{
    uint32_t reg;
    reg = RCU_CFG0;
    /* reset the SCS bits and set according to ck_sys */
    reg &= ~RCU_CFG0_SCS;
    RCU_CFG0 = (reg | ck_sys);
}

/*!
    \brief      get the system clock source
    \param[in]  none
    \param[out] none
    \retval     which clock is selected as CK_SYS source
      \arg        RCU_SCSS_IRC8M: CK_IRC8M is selected as the CK_SYS source
      \arg        RCU_SCSS_HXTAL: CK_HXTAL is selected as the CK_SYS source
      \arg        RCU_SCSS_PLL: CK_PLL is selected as the CK_SYS source
*/
uint32_t rcu_system_clock_source_get(void)
{
    return (RCU_CFG0 & RCU_CFG0_SCSS);
}

/*!
    \brief      configure the AHB clock prescaler selection
    \param[in]  ck_ahb: AHB clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_AHB_CKSYS_DIVx, x=1, 2, 4, 8, 16, 64, 128, 256, 512
    \param[out] none
    \retval     none
*/
void rcu_ahb_clock_config(uint32_t ck_ahb)
{
    uint32_t reg;
    reg = RCU_CFG0;
    /* reset the AHBPSC bits and set according to ck_ahb */
    reg &= ~RCU_CFG0_AHBPSC;
    RCU_CFG0 = (reg | ck_ahb);
}

/*!
    \brief      configure the APB1 clock prescaler selection
    \param[in]  ck_apb1: APB1 clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB1_CKAHB_DIV1: select CK_AHB as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV2: select CK_AHB/2 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV4: select CK_AHB/4 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV8: select CK_AHB/8 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV16: select CK_AHB/16 as CK_APB1
    \param[out] none
    \retval     none
*/
void rcu_apb1_clock_config(uint32_t ck_apb1)
{
    uint32_t reg;
    reg = RCU_CFG0;
    /* reset the APB1PSC and set according to ck_apb1 */
    reg &= ~RCU_CFG0_APB1PSC;
    RCU_CFG0 = (reg | ck_apb1);
}

/*!
    \brief      configure the APB2 clock prescaler selection
    \param[in]  ck_apb2: APB2 clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB2_CKAHB_DIV1: select CK_AHB as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV2: select CK_AHB/2 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV4: select CK_AHB/4 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV8: select CK_AHB/8 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV16: select CK_AHB/16 as CK_APB2
    \param[out] none
    \retval     none
*/
void rcu_apb2_clock_config(uint32_t ck_apb2)
{
    uint32_t reg;
    reg = RCU_CFG0;
    /* reset the APB2PSC and set according to ck_apb2 */
    reg &= ~RCU_CFG0_APB2PSC;
    RCU_CFG0 = (reg | ck_apb2);
}

/*!
    \brief      configure the CK_OUT clock source
    \param[in]  ckout0_src: CK_OUT clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKOUTSRC_NONE: no clock selected
      \arg        RCU_CKOUTSRC_IRC40K£ºIRC40K selected
      \arg        RCU_CKOUTSRC_LXTAL£º LXTAL selected
      \arg        RCU_CKOUTSRC_CKSYS: system clock selected
      \arg        RCU_CKOUTSRC_IRC8M: high speed 8M internal oscillator clock selected
      \arg        RCU_CKOUTSRC_HXTAL: HXTAL selected
      \arg        RCU_CKOUTSRC_CKPLL_DIV1: CK_PLL selected
      \arg        RCU_CKOUTSRC_CKPLL_DIV2: CK_PLL/2 selected
    \param[in]  ckout_div: CK_OUT divider
      \arg        RCU_CKOUT_DIVx(x=1,2,4,8,16,32,64,128): CK_OUT is divided by x
    \param[out] none
    \retval     none
*/
void rcu_ckout_config(uint32_t ckout_src, uint32_t ckout_div)
{
    uint32_t ckout = 0U;
    ckout = RCU_CFG0;
    /* reset the CKOUTSEL, CKOUTDIV and PLLDV bits and set according to ckout_src and ckout_div */
    ckout &= ~(RCU_CFG0_CKOUTSEL | RCU_CFG0_CKOUTDIV | RCU_CFG0_PLLDV);
    RCU_CFG0 = (ckout | ckout_src | ckout_div);
}

/*!
    \brief      configure the PLL clock source selection and PLL multiply factor
    \param[in]  pll_src: PLL clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLSRC_IRC8M_DIV2: select CK_IRC8M/2 as PLL source clock
      \arg        RCU_PLLSRC_HXTAL: select HXTAL as PLL source clock
    \param[in]  pll_mul: PLL multiply factor
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLL_MULx(x=2..32): PLL source clock * x
    \param[out] none
    \retval     none
*/
void rcu_pll_config(uint32_t pll_src, uint32_t pll_mul)
{
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4);
    RCU_CFG0 |= (pll_src | pll_mul);
}

/*!
    \brief      enable double PLL clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_double_pll_enable(void)
{
    RCU_CFG0 &= ~RCU_CFG0_DPLL;
}

/*!
    \brief      disable double PLL clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_double_pll_disable(void)
{
    RCU_CFG0 |= RCU_CFG0_DPLL;
}

/*!
    \brief      enable RCU system reset
    \param[in]  reset_source: reset source
                one or more parameters can be selected which is shown as below:
      \arg        RCU_SYSRST_LOCKUP: CPU lock-up reset
      \arg        RCU_SYSRST_LVD: low voltage detection reset
      \arg        RCU_SYSRST_ECC: ECC 2 bits error reset
      \arg        RCU_SYSRST_LOH: lost of HXTAL reset
      \arg        RCU_SYSRST_LOP: lost of PLL reset
    \param[out] none
    \retval     none
*/
void rcu_system_reset_enable(uint32_t reset_source)
{
    RCU_RSTSCK |= reset_source;
}

/*!
    \brief      disable RCU system reset
    \param[in]  reset_source: reset source
                one or more parameters can be selected which is shown as below:
      \arg        RCU_SYSRST_LOCKUP: CPU lock-up reset
      \arg        RCU_SYSRST_LVD: low voltage detection reset
      \arg        RCU_SYSRST_ECC: ECC 2 bits error reset
      \arg        RCU_SYSRST_LOH: lost of HXTAL reset
      \arg        RCU_SYSRST_LOP: lost of PLL reset
    \param[out] none
    \retval     none
*/
void rcu_system_reset_disable(uint32_t reset_source)
{
    RCU_RSTSCK &= ~reset_source;
}

/*!
    \brief      configure the ADC prescaler factor
    \param[in]  adc_psc: ADC prescaler factor
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKADC_CKAHB_DIVx (x=2,3,...,32): ADC prescaler select CK_AHB/(x)
    \param[out] none
    \retval     none
*/
void rcu_adc_clock_config(uint32_t adc_psc)
{
    uint32_t reg;
    /* reset the ADCPSC bits */
    reg = RCU_CFG2;
    reg &= ~RCU_CFG2_ADCPSC;
    /* set the ADC prescaler factor */
    reg |= adc_psc & RCU_CFG2_ADCPSC;
    /* set the register */
    RCU_CFG2 = reg;
}

/*!
    \brief      configure the RTC clock source selection
    \param[in]  rtc_clock_source: RTC clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_RTCSRC_NONE: no clock selected
      \arg        RCU_RTCSRC_LXTAL: CK_LXTAL selected as RTC source clock
      \arg        RCU_RTCSRC_IRC40K: CK_IRC40K selected as RTC source clock
      \arg        RCU_RTCSRC_HXTAL_DIV_128: CK_HXTAL/128 selected as RTC source clock
    \param[out] none
    \retval     none
*/
void rcu_rtc_clock_config(uint32_t rtc_clock_source)
{
    uint32_t reg;
    reg = RCU_BDCTL;
    /* reset the RTCSRC bits and set according to rtc_clock_source */
    reg &= ~RCU_BDCTL_RTCSRC;
    RCU_BDCTL = (reg | rtc_clock_source);
}

/*!
    \brief      configure the USART clock source selection
    \param[in]  usart_periph: USARTx(x=0,1,2)
    \param[in]  usart_clock_source: USART clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USARTSRC_HXTAL: HXTAL clock selected as USART source clock
      \arg        RCU_USARTSRC_CKSYS: system clock selected as USART source clock
      \arg        RCU_USARTSRC_LXTAL: LXTAL clock selected as USART source clock
      \arg        RCU_USARTSRC_IRC8M: IRC8M clock selected as USART source clock
    \param[out] none
    \retval     none
*/
void rcu_usart_clock_config(uint32_t usart_periph, uint32_t usart_clock_source)
{
    uint32_t reg;
    reg = RCU_CFG2;
    switch(usart_periph) {
    case USART0:
        /* reset the USART0SEL bit and set according to usart_clock_source */
        reg &= ~RCU_CFG2_USART0SEL;
        RCU_CFG2 = (reg | usart_clock_source);
        break;
    case USART1:
        /* reset the USART1SEL bit and set according to usart_clock_source */
        reg &= ~RCU_CFG2_USART1SEL;
        RCU_CFG2 = (reg | (uint32_t)(usart_clock_source << 4U));
        break;
    case USART2:
        /* reset the USART2SEL bit and set according to usart_clock_source */
        reg &= ~RCU_CFG2_USART2SEL;
        RCU_CFG2 = (reg | (uint32_t)(usart_clock_source << 6U));
        break;
    default:
        break;
    }
}

/*!
    \brief      configure the CAN clock source selection
    \param[in]  can_periph: CANx(x=0,1)
    \param[in]  can_clock_source: CAN clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CANSRC_HXTAL: HXTAL clock selected as CAN source clock
      \arg        RCU_CANSRC_PCLK2: PCLK2 clock selected as CAN source clock
      \arg        RCU_CANSRC_PCLK2_DIV_2: PCLK2/2 clock selected as CAN source clock
      \arg        RCU_CANSRC_IRC8M: IRC8M clock selected as CAN source clock
    \param[out] none
    \retval     none
*/
void rcu_can_clock_config(uint32_t can_periph, uint32_t can_clock_source)
{
    uint32_t reg;
    reg = RCU_CFG2;
    switch(can_periph) {
    case CAN0:
        /* reset the CAN0SEL bits and set according to can_clock_source */
        reg &= ~RCU_CFG2_CAN0SEL;
        RCU_CFG2 = (reg | can_clock_source);
        break;
    case CAN1:
        /* reset the CAN1SEL bits and set according to can_clock_source */
        reg &= ~RCU_CFG2_CAN1SEL;
        RCU_CFG2 = (reg | (uint32_t)(can_clock_source << 2U));
        break;
    default:
        break;
    }
}

/*!
    \brief      configure the LXTAL drive capability
    \param[in]  lxtal_dricap: drive capability of LXTAL
                only one parameter can be selected which is shown as below:
      \arg        RCU_LXTAL_LOWDRI: lower driving capability
      \arg        RCU_LXTAL_MED_LOWDRI: medium low driving capability
      \arg        RCU_LXTAL_MED_HIGHDRI: medium high driving capability
      \arg        RCU_LXTAL_HIGHDRI: higher driving capability
    \param[out] none
    \retval     none
*/
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap)
{
    uint32_t reg;
    reg = RCU_BDCTL;
    /* reset the LXTALDRI bits and set according to lxtal_dricap */
    reg &= ~RCU_BDCTL_LXTALDRI;
    RCU_BDCTL = (reg | lxtal_dricap);
}

/*!
    \brief      wait for oscillator stabilization flags is SET or oscillator startup is timeout
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
      \arg        RCU_IRC8M: internal 8M RC oscillators(IRC8M)
      \arg        RCU_IRC40K: internal 40K RC oscillator(IRC40K)
      \arg        RCU_PLL_CK: phase locked loop(PLL)
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci)
{
    uint32_t stb_cnt = 0U;
    ErrStatus reval = ERROR;
    FlagStatus osci_stat = RESET;

    switch(osci) {
    /* wait HXTAL stable */
    case RCU_HXTAL:
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
    /* wait IRC8M stable */
    case RCU_IRC8M:
        while((RESET == osci_stat) && (IRC8M_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_IRC8MSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_IRC8MSTB)) {
            reval = SUCCESS;
        }
        break;
    /* wait IRC40K stable */
    case RCU_IRC40K:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_IRC40KSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_IRC40KSTB)) {
            reval = SUCCESS;
        }
        break;
    /* wait PLL stable */
    case RCU_PLL_CK:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)) {
            osci_stat = rcu_flag_get(RCU_FLAG_PLLSTB);
            stb_cnt++;
        }
        /* check whether flag is set or not */
        if(RESET != rcu_flag_get(RCU_FLAG_PLLSTB)) {
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
    \brief      turn on the oscillator
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
      \arg        RCU_IRC8M: internal 8M RC oscillators(IRC8M)
      \arg        RCU_IRC40K: internal 40K RC oscillator(IRC40K)
      \arg        RCU_PLL_CK: phase locked loop(PLL)
    \param[out] none
    \retval     none
*/
void rcu_osci_on(rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) |= BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      turn off the oscillator
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
      \arg        RCU_IRC8M: internal 8M RC oscillators(IRC8M)
      \arg        RCU_IRC40K: internal 40K RC oscillator(IRC40K)
      \arg        RCU_PLL_CK: phase locked loop(PLL)
    \param[out] none
    \retval     none
*/
void rcu_osci_off(rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) &= ~BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
    \param[out] none
    \retval     none
*/
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci)
{
    uint32_t reg;

    switch(osci) {
    /* enable HXTAL to bypass mode */
    case RCU_HXTAL:
        reg = RCU_CTL;
        RCU_CTL &= ~RCU_CTL_HXTALEN;
        RCU_CTL = (reg | RCU_CTL_HXTALBPS);
        break;
    /* enable LXTAL to bypass mode */
    case RCU_LXTAL:
        reg = RCU_BDCTL;
        RCU_BDCTL &= ~RCU_BDCTL_LXTALEN;
        RCU_BDCTL = (reg | RCU_BDCTL_LXTALBPS);
        break;
    default:
        break;
    }
}

/*!
    \brief      disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
    \param[out] none
    \retval     none
*/
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci)
{
    uint32_t reg;

    switch(osci) {
    /* disable HXTAL to bypass mode */
    case RCU_HXTAL:
        reg = RCU_CTL;
        RCU_CTL &= ~RCU_CTL_HXTALEN;
        RCU_CTL = (reg & ~RCU_CTL_HXTALBPS);
        break;
    /* disable LXTAL to bypass mode */
    case RCU_LXTAL:
        reg = RCU_BDCTL;
        RCU_BDCTL &= ~RCU_BDCTL_LXTALEN;
        RCU_BDCTL = (reg & ~RCU_BDCTL_LXTALBPS);
        break;
    default:
        break;
    }
}

/*!
    \brief      HXTAL frequency scale select
    \param[in]  hxtal_scal: HXTAL frequency scale
                only one parameter can be selected which is shown as below:
      \arg        HXTAL_SCALE_2M_TO_8M: HXTAL scale is 2-8MHz
      \arg        HXTAL_SCALE_8M_TO_40M: HXTAL scale is 8-40MHz
    \param[out] none
    \retval     none
*/
void rcu_hxtal_frequency_scale_select(uint32_t hxtal_scal)
{
    if(HXTAL_SCALE_2M_TO_8M == hxtal_scal) {
        RCU_CTL &= ~RCU_CTL_HXTALSCAL;
    } else {
        RCU_CTL |= RCU_CTL_HXTALSCAL;
    }
}

/*!
    \brief      configure the HXTAL divider used as input of PLL
    \param[in]  hxtal_prediv: HXTAL previous PLL
                only one parameter can be selected which is shown as below:
      \arg        RCU_PREDV_DIVx(x=1..16): HXTAL divided x used as input of PLL
    \param[out] none
    \retval     none
*/
void rcu_hxtal_prediv_config(uint32_t hxtal_prediv)
{
    uint32_t prediv = 0U;
    prediv = RCU_CFG1;
    /* reset the PREDV bits and set according to hxtal_prediv */
    prediv &= ~RCU_CFG1_PREDV;
    RCU_CFG1 = (prediv | hxtal_prediv);
}

/*!
    \brief      set the IRC8M adjust value
    \param[in]  irc8m_adjval: IRC8M adjust value, must be between 0 and 0x1F
      \arg        0x00 - 0x1F
    \param[out] none
    \retval     none
*/
void rcu_irc8m_adjust_value_set(uint32_t irc8m_adjval)
{
    uint32_t reg;
    reg = RCU_CTL;
    /* reset the IRC8MADJ bits and set according to irc8m_adjval */
    reg &= ~RCU_CTL_IRC8MADJ;
    RCU_CTL = (reg | ((irc8m_adjval & RCU_IRC8M_ADJUST_MASK) << RCU_IRC8M_ADJUST_OFFSET));
}

/*!
    \brief      enable the HXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_clock_monitor_enable(void)
{
    RCU_CTL |= RCU_CTL_CKMEN;
}

/*!
    \brief      disable the HXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_clock_monitor_disable(void)
{
    RCU_CTL &= ~RCU_CTL_CKMEN;
}

/*!
    \brief      enable the LXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lxtal_clock_monitor_enable(void)
{
    RCU_CTL |= RCU_CTL_LCKMEN;
}

/*!
    \brief      disable the LXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_lxtal_clock_monitor_disable(void)
{
    RCU_CTL &= ~RCU_CTL_LCKMEN;
}

/*!
    \brief      enable the PLL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_pll_clock_monitor_enable(void)
{
    RCU_CTL |= RCU_CTL_PLLMEN;
}

/*!
    \brief      disable the PLL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_pll_clock_monitor_disable(void)
{
    RCU_CTL &= ~RCU_CTL_PLLMEN;
}

/*!
    \brief      unlock the voltage key
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_voltage_key_unlock(void)
{
    /* reset the KEY bits and set 0x1A2B3C4D */
    RCU_VKEY = ~RCU_VKEY_KEY;
    RCU_VKEY = RCU_VKEY_UNLOCK;
}

/*!
    \brief      deep-sleep mode voltage select
    \param[in]  dsvol: deep sleep mode voltage
                only one parameter can be selected which is shown as below:
      \arg        RCU_DEEPSLEEP_V_0_8: the core voltage is 0.8V
      \arg        RCU_DEEPSLEEP_V_0_9: the core voltage is 0.9V
      \arg        RCU_DEEPSLEEP_V_1_0: the core voltage is 1.0V
      \arg        RCU_DEEPSLEEP_V_1_1: the core voltage is 1.1V
    \param[out] none
    \retval     none
*/
void rcu_deepsleep_voltage_set(uint32_t dsvol)
{
    /* reset the DSLPVS bits and set according to dsvol */
    RCU_DSV &= ~RCU_DSV_DSLPVS;
    RCU_DSV |= dsvol;
}

/*!
    \brief      get the system clock, bus and peripheral clock frequency
    \param[in]  clock: the clock frequency which to get
                only one parameter can be selected which is shown as below:
      \arg        CK_SYS: system clock frequency
      \arg        CK_AHB: AHB clock frequency
      \arg        CK_APB1: APB1 clock frequency
      \arg        CK_APB2: APB2 clock frequency
      \arg        CK_USART0: USART0 clock frequency
      \arg        CK_USART1: USART1 clock frequency
      \arg        CK_USART2: USART2 clock frequency
    \param[out] none
    \retval     clock frequency of system, AHB, APB1, APB2, USART
*/
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock)
{
    uint32_t sws, ck_freq = 0U;
    uint32_t cksys_freq, ahb_freq, apb1_freq, apb2_freq;
    uint32_t usart_freq = 0U;
    uint32_t pllsel, pllmf, ck_src, idx, clk_exp;
    uint32_t predv0;

    /* exponent of AHB, APB1 and APB2 clock divider */
    uint8_t ahb_exp[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
    uint8_t apb1_exp[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
    uint8_t apb2_exp[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

    sws = RCU_CFG0 & RCU_CFG0_SCSS;
    switch(sws) {
    /* IRC8M is selected as CK_SYS */
    case RCU_SCSS_IRC8M:
        cksys_freq = IRC8M_VALUE;
        break;
    /* HXTAL is selected as CK_SYS */
    case RCU_SCSS_HXTAL:
        cksys_freq = HXTAL_VALUE;
        break;
    /* PLL is selected as CK_SYS */
    case RCU_SCSS_PLL:
        /* PLL clock source selection, HXTAL or IRC8M/2 */
        pllsel = (RCU_CFG0 & RCU_CFG0_PLLSEL);
        if(RCU_PLLSRC_HXTAL == pllsel) {
            /* PLL clock source is HXTAL */
            ck_src = HXTAL_VALUE;
            predv0 = (RCU_CFG1 & RCU_CFG1_PREDV) + 1U;
            ck_src /= predv0;
        } else {
            /* PLL clock source is IRC8M/2 */
            ck_src = IRC8M_VALUE / 2U;
        }
        /* PLL multiplication factor */
        pllmf = GET_BITS(RCU_CFG0, 18, 21);
        pllmf += ((RCU_CFG0 & RCU_CFG0_PLLMF_4) ? 15U : 0U);
        pllmf += ((RCU_CFG0_PLLMF == (RCU_CFG0 & RCU_CFG0_PLLMF)) ? 1U : 2U);
        cksys_freq = ck_src * pllmf;
        break;
    /* IRC8M is selected as CK_SYS */
    default:
        cksys_freq = IRC8M_VALUE;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    ahb_freq = cksys_freq >> clk_exp;
    /* calculate APB1 clock frequency */
    idx = GET_BITS(RCU_CFG0, 8, 10);
    clk_exp = apb1_exp[idx];
    apb1_freq = ahb_freq >> clk_exp;
    /* calculate APB2 clock frequency */
    idx = GET_BITS(RCU_CFG0, 11, 13);
    clk_exp = apb2_exp[idx];
    apb2_freq = ahb_freq >> clk_exp;
    /* return the clocks frequency */
    switch(clock) {
    case CK_SYS:
        ck_freq = cksys_freq;
        break;
    case CK_AHB:
        ck_freq = ahb_freq;
        break;
    case CK_APB1:
        ck_freq = apb1_freq;
        break;
    case CK_APB2:
        ck_freq = apb2_freq;
        break;
    case CK_USART0:
        /* calculate USART0 clock frequency */
        if(RCU_USARTSRC_HXTAL == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = HXTAL_VALUE;
        } else if(RCU_USARTSRC_CKSYS == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = cksys_freq;
        } else if(RCU_USARTSRC_LXTAL == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = LXTAL_VALUE;
        } else if(RCU_USARTSRC_IRC8M == (RCU_CFG2 & RCU_CFG2_USART0SEL)) {
            usart_freq = IRC8M_VALUE;
        } else {
        }
        ck_freq = usart_freq;
        break;
    case CK_USART1:
        /* calculate USART1 clock frequency */
        if(RCU_USARTSRC_HXTAL == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART1SEL) >> 4U)) {
            usart_freq = HXTAL_VALUE;
        } else if(RCU_USARTSRC_CKSYS == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART1SEL) >> 4U)) {
            usart_freq = cksys_freq;
        } else if(RCU_USARTSRC_LXTAL == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART1SEL) >> 4U)) {
            usart_freq = LXTAL_VALUE;
        } else if(RCU_USARTSRC_IRC8M == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART1SEL) >> 4U)) {
            usart_freq = IRC8M_VALUE;
        } else {
        }
        ck_freq = usart_freq;
        break;
    case CK_USART2:
        /* calculate USART2 clock frequency */
        if(RCU_USARTSRC_HXTAL == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART2SEL) >> 6U)) {
            usart_freq = HXTAL_VALUE;
        } else if(RCU_USARTSRC_CKSYS == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART2SEL) >> 6U)) {
            usart_freq = cksys_freq;
        } else if(RCU_USARTSRC_LXTAL == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART2SEL) >> 6U)) {
            usart_freq = LXTAL_VALUE;
        } else if(RCU_USARTSRC_IRC8M == (uint32_t)((uint32_t)(RCU_CFG2 & RCU_CFG2_USART2SEL) >> 6U)) {
            usart_freq = IRC8M_VALUE;
        } else {
        }
        ck_freq = usart_freq;
        break;
    default:
        break;
    }
    return ck_freq;
}

/*!
    \brief      get the clock stabilization and peripheral reset flags
    \param[in]  flag: the clock stabilization and peripheral reset flags, refer to rcu_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FLAG_IRC8MSTB: IRC8M stabilization flag
      \arg        RCU_FLAG_HXTALSTB: HXTAL stabilization flag
      \arg        RCU_FLAG_PLLSTB: PLL stabilization flag
      \arg        RCU_FLAG_LXTALSTB: LXTAL stabilization flag
      \arg        RCU_FLAG_IRC40KSTB: IRC40K stabilization flag
      \arg        RCU_FLAG_BORRST: BOR reset flag
      \arg        RCU_FLAG_LOCKUPRST: CPU LOCK UP error reset flag
      \arg        RCU_FLAG_LVDRST: low voltage detect error reset flag
      \arg        RCU_FLAG_ECCRST: 2 bits ECC error reset flag
      \arg        RCU_FLAG_LOHRST: lost of HXTAL error reset flag
      \arg        RCU_FLAG_LOPRST: lost of PLL error reset flag
      \arg        RCU_FLAG_V11RST: 1.1V domain Power reset flag
      \arg        RCU_FLAG_OBLRST: option byte loader reset flag
      \arg        RCU_FLAG_EPRST: external PIN reset flags
      \arg        RCU_FLAG_PORRST: power reset flag
      \arg        RCU_FLAG_SWRST: software reset flag
      \arg        RCU_FLAG_FWDGTRST: FWDGT reset flag
      \arg        RCU_FLAG_WWDGTRST: WWDGT reset flag
      \arg        RCU_FLAG_LPRST: low-power reset flag
    \param[out] none
    \retval     none
*/
FlagStatus rcu_flag_get(rcu_flag_enum flag)
{
    /* get the rcu flag */
    if(0U != (RCU_REG_VAL(flag) & BIT(RCU_BIT_POS(flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear all the reset flag
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_all_reset_flag_clear(void)
{
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
}

/*!
    \brief      get the clock stabilization interrupt and ckm flags
    \param[in]  int_flag: interrupt and ckm flags, refer to rcu_int_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC40KSTB: IRC40K stabilization interrupt flag
      \arg        RCU_INT_FLAG_LXTALSTB: LXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_IRC8MSTB: IRC8M stabilization interrupt flag
      \arg        RCU_INT_FLAG_HXTALSTB: HXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_PLLSTB: PLL stabilization interrupt flag
      \arg        RCU_INT_FLAG_LCKM: LXTAL clock monitor interrupt flag
      \arg        RCU_INT_FLAG_PLLM: PLL clock monitor interrupt flag
      \arg        RCU_INT_FLAG_CKM: HXTAL clock stuck interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag)
{
    /* get the rcu interrupt flag */
    if(0U != (RCU_REG_VAL(int_flag) & BIT(RCU_BIT_POS(int_flag)))) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear the interrupt flags
    \param[in]  int_flag: clock stabilization and stuck interrupt flags clear, refer to rcu_int_flag_clear_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC40KSTB_CLR: IRC40K stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_LXTALSTB_CLR: LXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_IRC8MSTB_CLR: IRC8M stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_HXTALSTB_CLR: HXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_PLLSTB_CLR: PLL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_LCKM_CLR: LXTAL clock monitor interrupt flag clear
      \arg        RCU_INT_FLAG_PLLM_CLR: PLL clock monitor interrupt flag clear
      \arg        RCU_INT_FLAG_CKM_CLR: HXTAL clock monitor interrupt flag clear
    \param[out] none
    \retval     none
*/
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag)
{
    RCU_REG_VAL(int_flag) |= BIT(RCU_BIT_POS(int_flag));
}

/*!
    \brief      enable the stabilization interrupt
    \param[in]  interrupt clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC40KSTB: IRC40K stabilization interrupt
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt
      \arg        RCU_INT_IRC8MSTB: IRC8M stabilization interrupt
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt
      \arg        RCU_INT_PLLSTB: PLL stabilization interrupt
      \arg        RCU_INT_LCKM: LXTAL clock monitor interrupt
      \arg        RCU_INT_PLLM: PLL clock monitor interrupt
    \param[out] none
    \retval     none
*/
void rcu_interrupt_enable(rcu_int_enum interrupt)
{
    RCU_REG_VAL(interrupt) |= BIT(RCU_BIT_POS(interrupt));
}

/*!
    \brief      disable the stabilization interrupt
    \param[in]  interrupt clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC40KSTB: IRC40K stabilization interrupt
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt
      \arg        RCU_INT_IRC8MSTB: IRC8M stabilization interrupt
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt
      \arg        RCU_INT_PLLSTB: PLL stabilization interrupt
      \arg        RCU_INT_LCKM: LXTAL clock monitor interrupt
      \arg        RCU_INT_PLLM: PLL clock monitor interrupt
    \param[out] none
    \retval     none
*/
void rcu_interrupt_disable(rcu_int_enum interrupt)
{
    RCU_REG_VAL(interrupt) &= ~BIT(RCU_BIT_POS(interrupt));
}
