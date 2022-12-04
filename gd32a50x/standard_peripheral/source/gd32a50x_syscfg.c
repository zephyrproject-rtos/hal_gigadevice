/*!
    \file    gd32a50x_syscfg.c
    \brief   SYSCFG driver

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

#include "gd32a50x_syscfg.h"

/* SYSCFG parameter mask */
#define         SYSCFG_TIMER0_ETI_SEL_MASK            (uint32_t)0x3FFFFFFFU      /*!< mask value of SYSCFG_TIMER0_ETI_SEL bits */
#define         SYSCFG_TIMER7_ETI_SEL_MASK            (uint32_t)0xCFFFFFFFU      /*!< mask value of SYSCFG_TIMER7_ETI_SEL bits */
#define         SYSCFG_TIMER19_ETI_SEL_MASK           (uint32_t)0xFCFFFFFFU      /*!< mask value of SYSCFG_TIMER19_ETI_SEL bits */
#define         SYSCFG_TIMER20_ETI_SEL_MASK           (uint32_t)0xFF3FFFFFU      /*!< mask value of SYSCFG_TIMER20_ETI_SEL bits */
#define         SYSCFG_CFG0_BOOTMODE_MASK             (uint8_t)0x03U             /*!< mask value of SYSCFG_CFG0_BOOT_MODE bits */

/* SYSCFG parameter offset */
#define         SYSCFG_TIMER0_ETI_SEL_OFFSET          (uint32_t)30U             /*!< offset value of SYSCFG_TIMER0_ETI_SEL bits */
#define         SYSCFG_TIMER7_ETI_SEL_OFFSET          (uint32_t)28U             /*!< offset value of SYSCFG_TIMER7_ETI_SEL bits */
#define         SYSCFG_TIMER19_ETI_SEL_OFFSET         (uint32_t)24U             /*!< offset value of SYSCFG_TIMER19_ETI_SEL bits */
#define         SYSCFG_TIMER20_ETI_SEL_OFFSET         (uint32_t)22U             /*!< offset value of SYSCFG_TIMER20_ETI_SEL bits */
#define         SYSCFG_CFG3_SRAMECCEADDR_OFFSET       (uint8_t)18U              /*!< offset value of SYSCFG_CFG3_SRAMECCEADDR bits */
#define         SYSCFG_CFG3_SRAMECCSERRBIT_OFFSET     (uint8_t)12U              /*!< offset value of SYSCFG_CFG3_SRAMECCSERRBITS bits */

/*!
    \brief      reset the SYSCFG registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void syscfg_deinit(void)
{
    rcu_periph_reset_enable(RCU_SYSCFGRST);
    rcu_periph_reset_disable(RCU_SYSCFGRST);
}

/*!
    \brief      configure the GPIO pin as EXTI Line
    \param[in]  exti_port: specify the gpio port used in EXTI
                only one parameter can be selected which is shown as below:
      \arg        EXTI_SOURCE_GPIOx(x = A,B,C,D,E,F): EXTI gpio port
    \param[in]  exti_pin: specify the EXTI line
                only one parameter can be selected which is shown as below:
      \arg        EXTI_SOURCE_PINx(for GPIOA\GPIOB\GPIOC\GPIOD\GPIOE, x = 0..15, for GPIOF, x = 0..7): EXTI GPIO pin
    \param[out] none
    \retval     none
*/
void syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin)
{
    uint32_t clear_exti_mask = ~((uint32_t)EXTI_SS_MASK << (EXTI_SS_MSTEP(exti_pin)));
    uint32_t config_exti_mask = ((uint32_t)exti_port) << (EXTI_SS_MSTEP(exti_pin));

    switch(exti_pin / EXTI_SS_JSTEP) {
    case EXTISS0:
        /* clear EXTI source line(0..3) */
        SYSCFG_EXTISS0 &= clear_exti_mask;
        /* configure EXTI soure line(0..3) */
        SYSCFG_EXTISS0 |= config_exti_mask;
        break;
    case EXTISS1:
        /* clear EXTI soure line(4..7) */
        SYSCFG_EXTISS1 &= clear_exti_mask;
        /* configure EXTI soure line(4..7) */
        SYSCFG_EXTISS1 |= config_exti_mask;
        break;
    case EXTISS2:
        /* clear EXTI soure line(8..11) */
        SYSCFG_EXTISS2 &= clear_exti_mask;
        /* configure EXTI soure line(8..11) */
        SYSCFG_EXTISS2 |= config_exti_mask;
        break;
    case EXTISS3:
        /* clear EXTI soure line(12..15) */
        SYSCFG_EXTISS3 &= clear_exti_mask;
        /* configure EXTI soure line(12..15) */
        SYSCFG_EXTISS3 |= config_exti_mask;
        break;
    default:
        break;
    }
}

/*!
    \brief      enable remap pin function
    \param[in]  remap_pin: specify pin
                only one parameter can be selected which are shown as below:
      \arg        SYSCFG_PA9_PA12_REMAP: PA9/PA12 pins are mapping on PA10/PA11 pins
      \arg        SYSCFG_BOOT0_REMAP_PF0: PF0 pin is mapping on the BOOT0 pin
    \param[out] none
    \retval     none
*/
void syscfg_pin_remap_enable(uint32_t remap_pin)
{
    SYSCFG_CFG0 |= remap_pin;
}

/*!
    \brief      disable remap pin function
    \param[in]  remap_pin: specify pin
                only one parameter can be selected which are shown as below:
      \arg        SYSCFG_PA9_PA12_REMAP: PA9/PA12 pins are mapping on PA10/PA11 pins
      \arg        SYSCFG_BOOT0_REMAP_PF0: PF0 pin is mapping on the BOOT0 pin
    \param[out] none
    \retval     none
*/
void syscfg_pin_remap_disable(uint32_t remap_pin)
{
    SYSCFG_CFG0 &= ~remap_pin;
}

/*!
    \brief      configure ADC channel GPIO pin remap function
    \param[in]  adcx_iny_remap: specify ADC channel
                only one parameter can be returned which is shown as below:
      \arg        ADC1_IN14_REMAP: ADC1 channel 14 GPIO pin remap
      \arg        ADC1_IN15_REMAP: ADC1 channel 15 GPIO pin remap
      \arg        ADC0_IN8_REMAP: ADC0 channel 8 GPIO pin remap
      \arg        ADC0_IN9_REMAP: ADC0 channel 9 GPIO pin remap
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void syscfg_adc_ch_remap_config(syscfg_adcx_chy_enum adcx_iny_remap, ControlStatus newvalue)
{
    switch(adcx_iny_remap) {
    case ADC1_IN14_REMAP:
        /* configure ADC1 channel 14 pin remap function */
        if(ENABLE == newvalue) {
            SYSCFG_CFG1 |= SYSCFG_CFG1_ADC1CH14RMP;
        } else {
            SYSCFG_CFG1 &= (~SYSCFG_CFG1_ADC1CH14RMP);
        }
        break;
    case ADC1_IN15_REMAP:
        /* configure ADC1 channel 15 pin remap function */
        if(ENABLE == newvalue) {
            SYSCFG_CFG1 |= SYSCFG_CFG1_ADC1CH15RMP;
        } else {
            SYSCFG_CFG1 &= (~SYSCFG_CFG1_ADC1CH15RMP);
        }
        break;
    case ADC0_IN8_REMAP:
        /* configure ADC0 channel 8 pin remap function */
        if(ENABLE == newvalue) {
            SYSCFG_CFG1 |= SYSCFG_CFG1_ADC0CH8RMP;
        } else {
            SYSCFG_CFG1 &= (~SYSCFG_CFG1_ADC0CH8RMP);
        }
        break;
    case ADC0_IN9_REMAP:
        /* configure ADC0 channel 9 pin remap function */
        if(ENABLE == newvalue) {
            SYSCFG_CFG1 |= SYSCFG_CFG1_ADC0CH9RMP;
        } else {
            SYSCFG_CFG1 &= (~SYSCFG_CFG1_ADC0CH9RMP);
        }
        break;
    default:
        break;
    }
}

/*!
    \brief      select TIMER external trigger source
    \param[in]  timer_num: specify TIMER
                only one parameter can be returned which is shown as below:
      \arg        TIMER0SEL: select TIMER0
      \arg        TIMER7SEL: select TIMER7
      \arg        TIMER19SEL: select TIMER19
      \arg        TIMER20SEL: select TIMER20
    \param[in]  etr_num: specify external trigger source
                only one parameter can be returned which is shown as below:
      \arg        TIMER_ETI_TRGx (x = 0,1,2,3): TIMER external trigger source x
      \arg        TIMER_ETI_TRG_NONE: do not seclet TIMER external trigger source
    \param[out] none
    \retval     none
*/
void syscfg_timer_eti_sel(syscfg_timersel_enum timer_num, uint32_t eti_num)
{
    switch(timer_num) {
    case TIMER0SEL:
        /* select TIMER0 external trigger source */
        SYSCFG_TIMERINSEL &= SYSCFG_TIMER0_ETI_SEL_MASK;
        SYSCFG_TIMERINSEL |= (eti_num << SYSCFG_TIMER0_ETI_SEL_OFFSET);
        break;
    case TIMER7SEL:
        /* select TIMER7 external trigger source */
        SYSCFG_TIMERINSEL &= SYSCFG_TIMER7_ETI_SEL_MASK;
        SYSCFG_TIMERINSEL |= (eti_num << SYSCFG_TIMER7_ETI_SEL_OFFSET);
        break;
    case TIMER19SEL:
        /* select TIMER19 external trigger source */
        SYSCFG_TIMERINSEL &= SYSCFG_TIMER19_ETI_SEL_MASK;
        SYSCFG_TIMERINSEL |= (eti_num << SYSCFG_TIMER19_ETI_SEL_OFFSET);
        break;
    case TIMER20SEL:
        /* select TIMER20 external trigger source */
        SYSCFG_TIMERINSEL &= SYSCFG_TIMER20_ETI_SEL_MASK;
        SYSCFG_TIMERINSEL |= (eti_num << SYSCFG_TIMER20_ETI_SEL_OFFSET);
        break;
    default:
        break;
    }
}

/*!
    \brief      select TRIGSEL as TIMER break input source
    \param[in]  bkin_source: specify TIMER break input source
                only one parameter can be selected which are shown as below:
      \arg        TIMERx_BKINy_TRIG (x=0,7,19,20 y=0,1,2,3):TIMERx break input y source select from TRIGSEL
    \param[out] none
    \retval     none
*/
void syscfg_timer_bkin_select_trigsel(uint32_t bkin_source)
{
    SYSCFG_TIMERINSEL |= (uint32_t)bkin_source;
}

/*!
    \brief      select GPIO as TIMER break input source
    \param[in]  bkin_source: specify TIMER break input source
                only one parameter can be selected which are shown as below:
      \arg        TIMERx_BKINy_TRIG (x=0,7,19,,20 y=0,1,2,3): TIMERx break input y source select from TRIGSEL
    \param[out] none
    \retval     none
*/
void syscfg_timer_bkin_select_gpio(uint32_t bkin_source)
{
    SYSCFG_TIMERINSEL &= ~(uint32_t)bkin_source;
}

/*!
    \brief      select TIMER7 channel0 complementary input source
    \param[in]  timer7_ch0n_in: specify TIMER7 channel0 complementary input source
                only one parameter can be returned which is shown as below:
      \arg        TIMER7CH0N_TIMER7CH0_TIMER0CH0_IN : select exclusive or of TIMER7_CH0_IN, TIMER7_CH0N_IN, and TIMER0_CH0_IN
      \arg        TIMER7_CH0N_IN : select TIMER7_CH0N_IN
    \param[out] none
    \retval     none
*/
void syscfg_timer7_ch0n_select(uint32_t timer7_ch0n_in)
{
    if(TIMER7CH0N_TIMER7CH0_TIMER0CH0_IN == timer7_ch0n_in) {
        /* select exclusive or of TIMER7_CH0_IN, TIMER7_CH0N_IN, and TIMER0_CH0_IN */
        SYSCFG_TIMERINSEL |= SYSCFG_TIMERINSEL_TIMER7_CH0N_SEL;
    } else {
        /* select TIMER7_CH0N_IN */
        SYSCFG_TIMERINSEL &= TIMER7_CH0N_IN;
    }
}

/*!
    \brief      configure TIMER0/7/19/20 break input to the selected parameter connection
    \param[in]  syscfg_lock: specify the parameter to be connected
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_LOCK_LOCKUP: Cortex-M33 lockup output connected to the break input
      \arg        SYSCFG_LOCK_SRAM_ECC_ERROR: SRAM ECC check error connected to the break input
      \arg        SYSCFG_LOCK_LVD: LVD interrupt connected to the break input
    \param[out] none
    \retval     none
*/
void syscfg_lock_config(uint32_t syscfg_lock)
{
    SYSCFG_CFG2 |= syscfg_lock;
}

/*!
    \brief      get SYSCFG flags
    \param[in]  syscfg_flag: specify the flag in SYSCFG_STAT to check
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_FLAG_SRAMECCMERR: SRAM multi-bits non-correction ECC error flag
      \arg        SYSCFG_FLAG_SRAMECCSERR: SRAM single bit correction ECC error flag
      \arg        SYSCFG_FLAG_FLASHECCERR: FLASH ECC NMI error flag
      \arg        SYSCFG_FLAG_CKMNMIERR: HXTAL clock monitor NMI error flag
      \arg        SYSCFG_FLAG_NMIPINERR: NMI pin error flag
    \param[out] none
    \retval     the syscfg_flag state returned (SET or RESET)
  */
FlagStatus syscfg_flag_get(uint32_t syscfg_flag)
{
    if(SYSCFG_STAT & syscfg_flag) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear SYSCFG flags
    \param[in]  syscfg_flag: specify the flag in SYSCFG_STAT to check
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_FLAG_SRAMECCMERR: SRAM multi-bits non-correction ECC error flag
      \arg        SYSCFG_FLAG_SRAMECCSERR: SRAM single bit correction ECC error flag
      \arg        SYSCFG_FLAG_FLASHECCERR: FLASH ECC NMI error flag
      \arg        SYSCFG_FLAG_CKMNMIERR: HXTAL clock monitor NMI error flag
    \param[out] none
    \retval     none
*/
void syscfg_flag_clear(uint32_t syscfg_flag)
{
    SYSCFG_STAT |= (uint32_t) syscfg_flag;
}

/*!
    \brief      enable SYSCFG interrupts
    \param[in]  interrupt: specify the interrupt in SYSCFG_CFG3
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_INT_SRAMECCME: SRAM multi-bits non-correction ECC error interrupt
      \arg        SYSCFG_INT_SRAMECCSE: SRAM single bit correction ECC error interrupt
      \arg        SYSCFG_INT_FLASHECCE: FLASH ECC NMI error interrupt
      \arg        SYSCFG_INT_CKMNMI: HXTAL clock monitor NMI error interrupt
      \arg        SYSCFG_INT_NMIPIN: NMI pin error interrupt
    \param[out] none
    \retval     none
*/
void syscfg_interrupt_enable(uint32_t interrupt)
{
    SYSCFG_CFG3 |= (uint32_t)interrupt;
}

/*!
    \brief      disable SYSCFG interrupts
    \param[in]  interrupt: specify the interrupt in SYSCFG_CFG3
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_INT_SRAMECCME: SRAM multi-bits non-correction ECC error interrupt
      \arg        SYSCFG_INT_SRAMECCSE: SRAM single bit correction ECC error interrupt
      \arg        SYSCFG_INT_FLASHECCE: FLASH ECC NMI error interrupt
      \arg        SYSCFG_INT_CKMNMI: HXTAL clock monitor NMI error interrupt
      \arg        SYSCFG_INT_NMIPIN: NMI pin error interrupt
    \param[out] none
    \retval     none
*/
void syscfg_interrupt_disable(uint32_t interrupt)
{
    SYSCFG_CFG3 &= (uint32_t)(~interrupt);
}

/*!
    \brief      get SYSCFG interrupt flag status
    \param[in]  interrupt: specify the interrupt flag status
                only one parameter can be selected which is shown as below:
      \arg        SYSCFG_INT_FLAG_SRAMECCMERR: SRAM multi-bits non-correction ECC error interrupt flag
      \arg        SYSCFG_INT_FLAG_SRAMECCSERR: SRAM single bit correction ECC error interrupt flag
      \arg        SYSCFG_INT_FLAG_FLASHECCERR: FLASH ECC NMI error interrupt flag
      \arg        SYSCFG_INT_FLAG_CKMNMIERR: HXTAL clock monitor NMI error interrupt flag
      \arg        SYSCFG_INT_FLAG_NMIPINERR: NMI pin error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus syscfg_interrupt_flag_get(uint32_t interrupt)
{
    uint32_t reg1 = SYSCFG_STAT;
    uint32_t reg2 = SYSCFG_CFG3;

    switch(interrupt) {
    /* SRAM multi-bits non-correction ECC error interrupt */
    case SYSCFG_INT_FLAG_SRAMECCMERR:
        reg1 = reg1 & SYSCFG_STAT_SRAMECCMEIF;
        reg2 = reg2 & SYSCFG_CFG3_SRAMECCMEIE;
        break;
    /* SRAM single bit correction ECC error interrupt */
    case SYSCFG_INT_FLAG_SRAMECCSERR:
        reg1 = reg1 & SYSCFG_STAT_SRAMECCSEIF;
        reg2 = reg2 & SYSCFG_CFG3_SRAMECCSEIE;
        break;
    /* FLASH ECC NMI error interrupt */
    case SYSCFG_INT_FLAG_FLASHECCERR:
        reg1 = reg1 & SYSCFG_STAT_FLASHECCIF;
        reg2 = reg2 & SYSCFG_CFG3_FLASHECCIE;
        break;
    /* HXTAL clock monitor NMI error interrupt */
    case SYSCFG_INT_FLAG_CKMNMIERR:
        reg1 = reg1 & SYSCFG_STAT_CKMNMIIF;
        reg2 = reg2 & SYSCFG_CFG3_CKMNMIIE;
        break;
    /* NMI pin error interrupt */
    case SYSCFG_INT_FLAG_NMIPINERR:
        reg1 = reg1 & SYSCFG_STAT_NMIPINIF;
        reg2 = reg2 & SYSCFG_CFG3_NMIPINIE;
        break;
    default :
        break;
    }
    /*get SYSCFG interrupt flag status */
    if(reg1 && reg2) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      get the current boot mode
    \param[in]  none
    \param[out] none
    \retval     the boot mode
      \arg        SYSCFG_BOOTMODE_FLASH: boot from the main flash
      \arg        SYSCFG_BOOTMODE_SYSTEM: boot from the system flash memory
      \arg        SYSCFG_BOOTMODE_SRAM: boot from the embedded SRAM
*/
uint8_t syscfg_bootmode_get(void)
{
    return (uint8_t)(SYSCFG_CFG0 & SYSCFG_CFG0_BOOTMODE_MASK);
}

/*!
    \brief      get the address where SRAM ECC error occur on
    \param[in]  none
    \param[out] none
    \retval     uint16_t: the address where SRAM ECC error occur on
*/
uint16_t syscfg_sram_ecc_address_get(void)
{
    return (uint16_t)(SYSCFG_CFG3 >> SYSCFG_CFG3_SRAMECCEADDR_OFFSET);
}

/*!
    \brief      get the bit which has SRAM ECC signle error
    \param[in]  none
    \param[out] none
    \retval     uint8_t: which bit has SRAM ECC signle error
*/
uint8_t syscfg_sram_ecc_bit_get(void)
{
    return (uint8_t)((SYSCFG_CFG3 & SYSCFG_CFG3_SRAMECCSERRBITS) >> SYSCFG_CFG3_SRAMECCSERRBIT_OFFSET);
}
