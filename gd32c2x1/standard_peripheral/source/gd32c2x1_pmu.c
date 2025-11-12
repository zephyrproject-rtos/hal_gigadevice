/*!
    \file    gd32c2x1_pmu.c
    \brief   PMU driver

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

#include "gd32c2x1_pmu.h"

/* PMU register bit offset */
#define PAR_TWK_EFLASH_OFFSET              ((uint32_t)21U)  /*!< bit offset of TWK_EFLASH in PMU_PAR */
#define PAR_TSW_IRC48MCNT_OFFSET           ((uint32_t)16U)  /*!< bit offset of TSW_IRC48MCNT in PMU_PAR */

/* bit mask */
#define PMU_LPMOD_MASK      ((uint32_t)0x3U)
#define PMU_DSMODVS_MASK    ((uint32_t)0x3000U)
#define PMU_WAKEUP_MASK     ((uint32_t)0x2F00U)

/*!
    \brief      reset PMU registers (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_deinit(void)
{
    /* reset PMU */
    rcu_periph_reset_enable(RCU_PMURST);
    rcu_periph_reset_disable(RCU_PMURST);
}

/*!
    \brief      select deepsleep mode voltage (API_ID(0x0002U))
    \param[in]  dsv_n:
                only one parameter can be selected which is shown as below:
      \arg        PMU_DSV_0: 0.9V (only valid when NPLDO is off)
      \arg        PMU_DSV_1: 1.0V
      \arg        PMU_DSV_2: 1.1V
      \arg        PMU_DSV_3: 1.2V
    \param[out] none
    \retval     none
*/
void pmu_deepsleep_voltage_select(uint32_t dsv_n)
{
    PMU_CTL0 &= ~PMU_CTL0_DSMODVS;
    PMU_CTL0 |= (dsv_n & PMU_DSMODVS_MASK);
}

/*!
    \brief      enable low power LDO (API_ID(0x0003U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_low_power_ldo_enable(void)
{
    PMU_CTL0 |= PMU_CTL0_LPLDOEN;
}

/*!
    \brief      disable low power LDO (API_ID(0x0004U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_low_power_ldo_disable(void)
{
    PMU_CTL0 &= ~PMU_CTL0_LPLDOEN;
}

/*!
    \brief      PMU work in Sleep mode (API_ID(0x0005U))
    \param[in]  sleepmodecmd: sleep mode command
                only one parameter can be selected which is shown as below:
      \arg        WFI_CMD: use WFI command
      \arg        WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void pmu_to_sleepmode(uint8_t sleepmodecmd)
{
    /* clear sleepDeep bit of Cortex-M23 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);

    /* select WFI or WFE command to enter sleep mode */
    if(WFI_CMD == sleepmodecmd) {
        __WFI();
    } else {
        __SEV();
        __WFE();
        __WFE();
    }
}

/*!
    \brief      PMU work in Deep-sleep mode (API_ID(0x0006U))
    \param[in]  deepsleepmodecmd: deepsleep mode command
                only one parameter can be selected which is shown as below:
      \arg        WFI_CMD: use WFI command
      \arg        WFE_CMD: use WFE command
    \param[in]  deepsleepmode: deepsleep mode
                only one parameter can be selected which is shown as below:
      \arg        PMU_DEEPSLEEP: Deep-sleep mode
      \arg        PMU_DEEPSLEEP1: Deep-sleep mode 1
    \param[out] none
    \retval     none
*/
void pmu_to_deepsleepmode(uint8_t deepsleepmodecmd, uint8_t deepsleepmode)
{
    /* configure lowpower mode */
    PMU_CTL0 &= ~((uint32_t)(PMU_CTL0_LPMOD));
    PMU_CTL0 |= (deepsleepmode & PMU_LPMOD_MASK);

    /* set sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* select WFI or WFE command to enter Deep-sleep mode */
    if(WFI_CMD == deepsleepmodecmd) {
        __WFI();
    } else {
        __SEV();
        __WFE();
        __WFE();
    }
    /* reset sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    /* clear lowpower mode */
    PMU_CTL0 &= ~((uint32_t)(PMU_CTL0_LPMOD));
}

/*!
    \brief      pmu work in standby mode (API_ID(0x0007U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_to_standbymode(void)
{
    /* select the low-power mode to enter */
    PMU_CTL0 |= PMU_STANDBY;

    /* reset wakeup flag and standby flag */
    PMU_CTL0 |= (PMU_CTL0_WURST | PMU_CTL0_STBRST);

    /* set sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    REG32(0xE000E010U) &= 0x00010004U;
    REG32(0xE000E180U)  = 0xFFFFFFF3U;
    REG32(0xE000E184U)  = 0xFFFFFDFFU;
    REG32(0xE000E188U)  = 0xFFFFFFFFU;

    /* select WFI command to enter standby mode */
    __WFI();
}

/*!
    \brief      enable PMU wakeup pin (API_ID(0x0008U))
    \param[in]  wakeup_pin: wakeup pin
                one or more parameters can be selected which are shown as below:
      \arg        PMU_WAKEUP_PIN0: WKUP Pin 0 (PA0)
      \arg        PMU_WAKEUP_PIN1: WKUP Pin 1 (PC13/PA4)
      \arg        PMU_WAKEUP_PIN2: WKUP Pin 2 (PB6)
      \arg        PMU_WAKEUP_PIN3: WKUP Pin 3 (PA2)
      \arg        PMU_WAKEUP_PIN5: WKUP Pin 5 (PB5)
    \param[out] none
    \retval     none
*/
void pmu_wakeup_pin_enable(uint32_t wakeup_pin)
{
    PMU_CS |= (wakeup_pin & PMU_WAKEUP_MASK);
}

/*!
    \brief      disable PMU wakeup pin (API_ID(0x0009U))
    \param[in]  wakeup_pin: wakeup pin
                one or more parameters can be selected which are shown as below:
      \arg        PMU_WAKEUP_PIN0: WKUP Pin 0 (PA0)
      \arg        PMU_WAKEUP_PIN1: WKUP Pin 1 (PC13/PA4)
      \arg        PMU_WAKEUP_PIN2: WKUP Pin 2 (PB6)
      \arg        PMU_WAKEUP_PIN3: WKUP Pin 3 (PA2)
      \arg        PMU_WAKEUP_PIN5: WKUP Pin 5 (PB5)
    \param[out] none
    \retval     none
*/
void pmu_wakeup_pin_disable(uint32_t wakeup_pin)
{
    PMU_CS &= ~(wakeup_pin & PMU_WAKEUP_MASK);
}

/*!
    \brief      enable write access to the backup registers in RTC (API_ID(0x000AU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_backup_write_enable(void)
{
    PMU_CTL0 |= PMU_CTL0_BKPWEN;
}

/*!
    \brief      disable write access to the backup registers in RTC (API_ID(0x000BU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_backup_write_disable(void)
{
    PMU_CTL0 &= ~PMU_CTL0_BKPWEN;
}

/*!
    \brief      configure power state of eflash domain when enter run/run1 (API_ID(0x000CU))
    \param[in]  state: power state of eflash domain
                only one parameter can be selected which is shown as below:
      \arg        ENABLE: eflash domain power on
      \arg        DISABLE: eflash domain power off
    \param[out] none
    \retval     none
*/
void pmu_eflash_run_power_config(ControlStatus state)
{
    if(ENABLE == state) {
        PMU_CTL1 &= ~PMU_CTL1_EFPSLEEP;
    } else {
        PMU_CTL1 |= PMU_CTL1_EFPSLEEP;
    }
}

/*!
    \brief      configure power state of eflash domain when enter deepsleep/deepsleep1 (API_ID(0x000DU))
    \param[in]  state: power state of eflash domain when enter deepsleep/deepsleep1
                only one parameter can be selected which is shown as below:
      \arg        ENABLE: eflash domain power on
      \arg        DISABLE: eflash domain power off
    \param[out] none
    \retval     none
*/
void pmu_eflash_deepsleep_power_config(ControlStatus state)
{
    if(ENABLE == state) {
        PMU_CTL1 &= ~PMU_CTL1_EFDSPSLEEP;
    } else {
        PMU_CTL1 |= PMU_CTL1_EFDSPSLEEP;
    }
}

/*!
    \brief      configure eflash wakeup time, using IRC48M counter (API_ID(0x000EU))
    \param[in]  wakeup_time: 0~0xFF
    \param[out] none
    \retval     none
*/
void pmu_eflash_wakeup_time_config(uint32_t wakeup_time)
{
    PMU_PAR &= ~PMU_PAR_TWK_EFLASH;
    PMU_PAR |= ((wakeup_time & 0x000000FFU) << PAR_TWK_EFLASH_OFFSET);
}

/*!
    \brief      configure IRC48M counter before enter Deepsleep/Deepsleep1 mode (API_ID(0x000FU))
    \param[in]  wait_time: 0~0x1F
    \param[out] none
    \retval     none
*/
void pmu_deepsleep_wait_time_config(uint32_t wait_time)
{
    PMU_PAR &= ~PMU_PAR_TSW_IRC48MCNT;
    PMU_PAR |= ((wait_time & 0x0000001FU) << PAR_TSW_IRC48MCNT_OFFSET);
}

/*!
    \brief      get flag state (API_ID(0x0010U))
    \param[in]  flag: PMU flags
                only one parameter can be selected which is shown as below:
      \arg        PMU_FLAG_WAKEUP: wakeup flag
      \arg        PMU_FLAG_STANDBY: standby flag
      \arg        PMU_FLAG_LDOVSRF: LDO voltage select ready flag
      \arg        PMU_FLAG_NPRDY: normal-power LDO ready flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus pmu_flag_get(uint32_t flag)
{
    FlagStatus ret;
    if(0U != (PMU_CS & flag)) {
        ret = SET;
    }else{
        ret = RESET;
    }
    return ret;
}

/*!
    \brief      clear flag bit (API_ID(0x0011U))
    \param[in]  flag: PMU flags
                only one parameter can be selected which is shown as below:
      \arg        PMU_FLAG_WAKEUP: wakeup flag
      \arg        PMU_FLAG_STANDBY: standby flag
    \param[out] none
    \retval     none
*/
void pmu_flag_clear(uint32_t flag)
{
    switch(flag) {
    case PMU_FLAG_WAKEUP:
        /* clear wakeup flag */
        PMU_CTL0 |= PMU_CTL0_WURST;
        break;
    case PMU_FLAG_STANDBY:
        /* clear standby flag */
        PMU_CTL0 |= PMU_CTL0_STBRST;
        break;
    default :
        break;
    }
}
