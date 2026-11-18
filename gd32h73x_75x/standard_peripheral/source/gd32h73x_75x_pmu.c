/*!
    \file    gd32h73x_75x_pmu.c
    \brief   PMU driver

    \version 2026-07-30, V1.7.0, firmware for GD32H73x_75x
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

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

#include "gd32h73x_75x_pmu.h"

/* PMU register bit offset */
#define PAR_TSW_IRCCNT_OFFSET               ((uint32_t)0x00000010U)               /*!< bit offset of TSW_HSICNT in PMU_PAR */
#define PMU_WUPEN_MASK                      ((uint32_t)0x00002B00U)               /*!< mask of wakeup pin enable bits in PMU_CS */

/*!
    \brief      reset PMU register (API_ID(0x0001U))
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
    \brief      select low voltage detector threshold (API_ID(0x0002U))
    \param[in]  lvdt_n: voltage threshold of low voltage detector
                only one parameter can be selected which is shown as below:
      \arg        PMU_LVDT_0: voltage threshold is 1.9V
      \arg        PMU_LVDT_1: voltage threshold is 2.1V
      \arg        PMU_LVDT_2: voltage threshold is 2.2V
      \arg        PMU_LVDT_3: voltage threshold is 2.4V
      \arg        PMU_LVDT_4: voltage threshold is 2.5V
      \arg        PMU_LVDT_5: voltage threshold is 2.7V
      \arg        PMU_LVDT_6: voltage threshold is 2.9V
      \arg        PMU_LVDT_7: input analog voltage on PB7 (compared with 1.2V)
    \param[out] none
    \retval     none
*/
void pmu_lvd_select(uint32_t lvdt_n)
{
    uint32_t temp;
    temp = PMU_CTL0;
    /* clear LVDT bits */
    temp &= ~PMU_CTL0_LVDT;
    /* set LVDT bits according to lvdt_n */
    temp |= (lvdt_n & PMU_CTL0_LVDT);
    PMU_CTL0 = temp;
}

/*!
    \brief      enable PMU lvd (API_ID(0x0003U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_lvd_enable(void)
{
    PMU_CTL0 |= PMU_CTL0_LVDEN;
}

/*!
    \brief      disable PMU lvd (API_ID(0x0004U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_lvd_disable(void)
{
    PMU_CTL0 &= ~PMU_CTL0_LVDEN;
}

/*!
    \brief      select analog voltage detector threshold (API_ID(0x0005U))
    \param[in]  vavdt_n: voltage threshold of analog voltage detector
                only one parameter can be selected which is shown as below:
      \arg        PMU_VAVDVC_0: voltage threshold of analog voltage detector is 1.7V
      \arg        PMU_VAVDVC_1: voltage threshold of analog voltage detector is 2.1V
      \arg        PMU_VAVDVC_2: voltage threshold of analog voltage detector is 2.5V
      \arg        PMU_VAVDVC_3: voltage threshold of analog voltage detector is 2.8V
    \param[out] none
    \retval     none
*/
void pmu_vavd_select(uint32_t vavdt_n)
{
    uint32_t temp;
    temp = PMU_CTL0;
    /* clear VAVDVC bits */
    temp &= ~PMU_CTL0_VAVDVC;
    /* set VAVDVC bits according to vavdt_n */
    temp |= (vavdt_n & PMU_CTL0_VAVDVC);
    PMU_CTL0 = temp;
}

/*!
    \brief      enable PMU analog voltage detector (API_ID(0x0006U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_vavd_enable(void)
{
    PMU_CTL0 |= PMU_CTL0_VAVDEN;
}

/*!
    \brief      disable PMU analog voltage detector (API_ID(0x0007U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_vavd_disable(void)
{
    PMU_CTL0 &= ~PMU_CTL0_VAVDEN;
}

/*!
    \brief      enable PMU Vcore core voltage detector (API_ID(0x0008U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_vovd_enable(void)
{
    PMU_CTL0 |= PMU_CTL0_VOVDEN;
}

/*!
    \brief      disable PMU Vcore core voltage detector (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_vovd_disable(void)
{
    PMU_CTL0 &= ~PMU_CTL0_VOVDEN;
}

/*!
    \brief      control the Vcore core voltage level (API_ID(0x000AU))
    \param[in]  ldo_n: LDO output voltage select
                only one parameter can be selected which is shown as below:
      \arg        PMU_LDOVS_0: LDO output voltage 0.8V mode
      \arg        PMU_LDOVS_1: LDO output voltage 0.85V mode
      \arg        PMU_LDOVS_2: LDO output voltage 0.9V mode
      \arg        PMU_LDOVS_3: LDO output voltage 0.95V mode
      \arg        PMU_LDOVS_4: LDO output voltage 0.975V mode
      \arg        PMU_LDOVS_5: LDO output voltage 1V mode
    \param[out] none
    \retval     none
*/
void pmu_ldo_output_select(uint32_t ldo_n)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_LDO(ldo_n)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x000AU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t temp;
        temp = PMU_CTL3;
        temp &= ~PMU_CTL3_LDOVS;
        temp |= ldo_n;
        PMU_CTL3 = temp;
    }
}

/*!
    \brief      Deep-sleep mode Vcore core voltage select (API_ID(0x000BU))
    \param[in]  sldo: SLDOVS output voltage select
                only one parameter can be selected which is shown as below:
      \arg        PMU_SLDOVS_0: SLDOVS scale 0.6V
      \arg        PMU_SLDOVS_1: SLDOVS scale 0.7V
      \arg        PMU_SLDOVS_2: SLDOVS scale 0.8V
      \arg        PMU_SLDOVS_3: SLDOVS scale 0.9V
    \param[out] none
    \retval     none
*/
void pmu_sldo_output_select(uint32_t sldo_n)
{
    uint32_t temp;
    temp = PMU_CTL0;
    temp &= ~PMU_CTL0_SLDOVS;
    temp |= (sldo_n & PMU_CTL0_SLDOVS);
    PMU_CTL0 = temp;
}

/*!
    \brief      PMU VBAT battery charging resistor selection (API_ID(0x000CU))
    \param[in]  resistor: VBAT battery charging resistor select
                only one parameter can be selected which is shown as below:
      \arg         PMU_VCRSEL_5K: 5kOhms resistor is selected for charing VBAT battery
      \arg         PMU_VCRSEL_1P5K: 1.5kOhms resistor is selected for charing VBAT battery
    \param[out] none
    \retval     none
*/
void pmu_vbat_charging_select(uint32_t resistor)
{
    PMU_CTL2 &= ~PMU_CTL2_VCRSEL;
    PMU_CTL2 |= (resistor & PMU_CTL2_VCRSEL);
}

/*!
    \brief      enable VBAT battery charging (API_ID(0x000DU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_vbat_charging_enable(void)
{
    PMU_CTL2 |= PMU_CTL2_VCEN;
}

/*!
    \brief      disable VBAT battery charging (API_ID(0x000EU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_vbat_charging_disable(void)
{
    PMU_CTL2 &= ~PMU_CTL2_VCEN;
}

/*!
    \brief      enable VBAT and temperature monitoring (API_ID(0x000FU))
    \param[in]  none
    \param[out] none
    \retval     none
    \note       PMU_CTL0_BKPWEN bit should be set before calling this function.
*/
void pmu_vbat_temp_moniter_enable(void)
{
    PMU_CTL1 |= PMU_CTL1_VBTMEN;
}

/*!
    \brief      disable VBAT and temperature monitoring (API_ID(0x0010U))
    \param[in]  none
    \param[out] none
    \retval     PMU_CTL0_BKPWEN bit should be set before calling this function.
*/
void pmu_vbat_temp_moniter_disable(void)
{
    PMU_CTL1 &= ~PMU_CTL1_VBTMEN;
}

/*!
    \brief      enable USB regulator (API_ID(0x0011U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_usb_regulator_enable(void)
{
    PMU_CTL2 |= PMU_CTL2_USBSEN;
}

/*!
    \brief      disable USB regulator (API_ID(0x0012U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_usb_regulator_disable(void)
{
    PMU_CTL2 &= ~PMU_CTL2_USBSEN;
}

/*!
    \brief      enable VDD33USB voltage level detector (API_ID(0x0013U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_usb_voltage_detector_enable(void)
{
    PMU_CTL2 |= PMU_CTL2_VUSB33DEN;
}

/*!
    \brief      disable VDD33USB voltage level detector (API_ID(0x0014U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_usb_voltage_detector_disable(void)
{
    PMU_CTL2 &= ~PMU_CTL2_VUSB33DEN;
}

/*!
    \brief      power supply configurations (API_ID(0x0015U))
    \param[in]  smpsmode: power supply mode select
                only one parameter can be selected which is shown as below:
      \arg        PMU_LDO_SUPPLY: Vcore domains are suppplied from the LDO
      \arg        PMU_DIRECT_SMPS_SUPPLY: Vcore domains are suppplied from the SMPS only
      \arg        PMU_BYPASS: the SMPS disabled and the LDO Bypass. The Vcore domains are supplied from an external source
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
    \note       This function includes timeout exit scenarios.
                Modify according to the user's actual usage scenarios.
*/
ErrStatus pmu_smps_ldo_supply_config(uint32_t smpsmode)
{
    uint32_t timeout = PMU_LDO_CFG_TIMEOUT_COUNT;
    ErrStatus ret = ERROR;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_SMPS_LDO_SUPPLY(smpsmode)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x0015U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t temp;
        temp = PMU_CTL2;
        temp &= ~(PMU_CTL2_DVSEN | PMU_CTL2_LDOEN | PMU_CTL2_BYPASS);
        temp |= smpsmode;
        PMU_CTL2 = temp;

        while((0U == (PMU_CTL3 & PMU_CTL3_VOVRF)) && (timeout > 0U)) {
            timeout--;
        }
        if(0U == (PMU_CTL3 & PMU_CTL3_VOVRF)) {
            ret = ERROR;
        } else {
            ret = SUCCESS;
        }
    }
    return ret;
}

/*!
    \brief      enter sleep mode (API_ID(0x0016U))
    \param[in]  sleepmodecmd: sleep mode command
                only one parameter can be selected which is shown as below:
      \arg          WFI_CMD: use WFI command
      \arg          WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void pmu_to_sleepmode(uint8_t sleepmodecmd)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_CMD(sleepmodecmd)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x0016U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* clear sleepdeep bit of Cortex-M7 system control register */
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
}

/*!
    \brief      enter deepsleep mode (API_ID(0x0017U))
    \param[in]  deepsleepmodecmd: deepsleep mode command
                only one parameter can be selected which is shown as below:
      \arg          WFI_CMD: use WFI command
      \arg          WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void pmu_to_deepsleepmode(uint8_t deepsleepmodecmd)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_CMD(deepsleepmodecmd)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x0017U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* clear standby mode */
        PMU_CTL0 &= ~((uint32_t)(PMU_CTL0_STBMOD));

        /* set sleepdeep bit of Cortex-M7 system control register */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* select WFI or WFE command to enter deepsleep mode */
        if(WFI_CMD == deepsleepmodecmd) {
            __WFI();
        } else {
            __SEV();
            __WFE();
            __WFE();
        }
        /* reset sleepdeep bit of Cortex-M7 system control register */
        SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    }
}

/*!
    \brief      enter standby mode (API_ID(0x0018U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_to_standbymode(void)
{
    /* set stbmod bit */
    PMU_CTL0 |= PMU_CTL0_STBMOD;

    /* reset wakeup flag */
    PMU_CTL0 |= PMU_CTL0_WURST;

    /* set sleepdeep bit of Cortex-M7 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    REG32( 0xE000E010U ) &= 0x00010004U;
    REG32( 0xE000E180U )  = 0xFFFFFFF3U;
    REG32( 0xE000E184U )  = 0xFFFFFDFFU;
    REG32( 0xE000E188U )  = 0xFFFFFFFFU;
    REG32( 0xE000E18CU )  = 0xFFFFFFFFU;
    REG32( 0xE000E190U )  = 0xFFFFFFFFU;
    REG32( 0xE000E194U )  = 0xFFFFFFFFU;
    REG32( 0xE000E198U )  = 0xFFFFFFFFU;

    /* enter standby mode */
    __WFI();
}

/*!
    \brief      enable PMU wakeup pin (API_ID(0x0019U))
    \param[in]  wakeup_pin: wakeup pin select
                one or more parameters can be selected which is shown as below:
      \arg        PMU_WAKEUP_PIN0: WKUP Pin 0
      \arg        PMU_WAKEUP_PIN1: WKUP Pin 1
      \arg        PMU_WAKEUP_PIN3: WKUP Pin 3
      \arg        PMU_WAKEUP_PIN5: WKUP Pin 5
    \param[out] none
    \retval     none
*/
void pmu_wakeup_pin_enable(uint32_t wakeup_pin)
{
    PMU_CS |= (wakeup_pin & PMU_WUPEN_MASK);
}

/*!
    \brief      disable PMU wakeup pin (API_ID(0x001AU))
    \param[in]  wakeup_pin: wakeup pin select
                one or more parameters can be selected which is shown as below:
      \arg        PMU_WAKEUP_PIN0: WKUP Pin 0
      \arg        PMU_WAKEUP_PIN1: WKUP Pin 1
      \arg        PMU_WAKEUP_PIN3: WKUP Pin 3
      \arg        PMU_WAKEUP_PIN5: WKUP Pin 5
    \param[out] none
    \retval     none
*/
void pmu_wakeup_pin_disable(uint32_t wakeup_pin)
{
    PMU_CS &= ~(wakeup_pin & PMU_WUPEN_MASK);
}

/*!
    \brief      enable backup domain write (API_ID(0x001BU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_backup_write_enable(void)
{
    PMU_CTL0 |= PMU_CTL0_BKPWEN;
}

/*!
    \brief      disable backup domain write (API_ID(0x001CU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_backup_write_disable(void)
{
    PMU_CTL0 &= ~PMU_CTL0_BKPWEN;
}

/*!
    \brief      enable backup voltage stabilizer (API_ID(0x001DU))
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
    \note       This function includes timeout exit scenarios.
                Modify according to the user's actual usage scenarios.
*/
ErrStatus pmu_backup_voltage_stabilizer_enable(void)
{
    uint32_t timeout = PMU_BKUP_TIMEOUT_COUNT;
    ErrStatus ret = ERROR;

    PMU_CTL1 |= PMU_CTL1_BKPVSEN;
    while((RESET == (PMU_CTL1 & PMU_CTL1_BKPVSRF)) && (timeout > 0U)) {
        timeout--;
    }
    if(RESET != (PMU_CTL1 & PMU_CTL1_BKPVSRF)) {
        ret = SUCCESS;
    }
    return ret;
}

/*!
    \brief      disable backup voltage stabilizer (API_ID(0x001EU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pmu_backup_voltage_stabilizer_disable(void)
{
    PMU_CTL1 &= ~PMU_CTL1_BKPVSEN;
}

/*!
    \brief      configure IRC counter before enter Deep-sleep mode (API_ID(0x001FU))
    \param[in]  wait_time: 0x0~0x1F, IRC counter before enter Deep-sleep mode
    \param[out] none
    \retval     none
*/
void pmu_enter_deepsleep_wait_time_config(uint32_t wait_time)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_ENTER_DEEPSLEEP_WAIT_TIME(wait_time)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x001FU), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t temp;
        temp = PMU_PAR;
        temp &= ~PMU_PAR_TSW_IRCCNT;
        temp |= ((uint32_t)wait_time << PAR_TSW_IRCCNT_OFFSET);
        PMU_PAR = temp;
    }
}

/*!
    \brief      configure IRC counter before exit Deep-sleep mode (API_ID(0x0020U))
    \param[in]  wait_time: 0x0~0xFFF, IRC counter before exit Deep-sleep mode
    \param[out] none
    \retval     none
*/
void pmu_exit_deepsleep_wait_time_config(uint32_t wait_time)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_EXIT_DEEPSLEEP_WAIT_TIME(wait_time)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x0020U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t temp;
        temp = PMU_PAR;
        temp &= ~PMU_PAR_CNT;
        temp |= (uint32_t)(wait_time);
        PMU_PAR = temp;
    }
}

/*!
    \brief      get flag state (API_ID(0x0021U))
    \param[in]  flag: flag bit to get
                only one parameter can be selected which is shown as below:
      \arg          PMU_FLAG_WAKEUP: wakeup flag
      \arg          PMU_FLAG_STANDBY: standby flag
      \arg          PMU_FLAG_LVDF: low voltage detector status flag
      \arg          PMU_FLAG_VAVDF: VDDA analog voltage detector voltage output on VDDA flag
      \arg          PMU_FLAG_VOVDF: peripheral voltage on VDDA detector flag
      \arg          PMU_FLAG_VBATLF: VBAT level monitoring versus low threshold
      \arg          PMU_FLAG_VBATHF: VBAT level monitoring versus high threshold
      \arg          PMU_FLAG_TEMPLF: temperature level monitoring versus low threshold
      \arg          PMU_FLAG_TEMPHF: temperature level monitoring versus high threshold
      \arg          PMU_FLAG_DVSRF: step-down voltage stabilizer ready flag bit
      \arg          PMU_FLAG_USB33RF: USB supply ready flag bit
      \arg          PMU_FLAG_PWRRF: power Ready flag bit.
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus pmu_flag_get(uint32_t flag)
{
    FlagStatus ret = RESET;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_FLAG(flag)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x0021U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(PMU_REG_VAL(flag) & BIT(PMU_BIT_POS(flag))) {
            ret = SET;
        }
    }
    return  ret;
}

/*!
    \brief      clear flag bit (API_ID(0x0022U))
    \param[in]  flag_reset: flag bit to be cleared
                only one parameter can be selected which is shown as below:
      \arg        PMU_FLAG_WAKEUP: wakeup flag
      \arg        PMU_FLAG_STANDBY: standby flag
    \param[out] none
    \retval     none
*/
void pmu_flag_clear(uint32_t flag_reset)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_PMU_FLAG_RESET(flag_reset)) {
        fw_debug_report_err(PMU_MODULE_ID, API_ID(0x0022U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(PMU_FLAG_WAKEUP == flag_reset) {
            PMU_CTL0 |= PMU_CTL0_WURST;
        } else {
            if(PMU_FLAG_STANDBY == flag_reset) {
                PMU_CTL0 |= PMU_CTL0_STBRST;
            }
        }
    }
}
