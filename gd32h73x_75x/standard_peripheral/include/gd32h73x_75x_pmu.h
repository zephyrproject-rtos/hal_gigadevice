/*!
    \file    gd32h73x_75x_pmu.h
    \brief   definitions for the PMU

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

#ifndef GD32H73X_75X_PMU_H
#define GD32H73X_75X_PMU_H

#include "gd32h73x_75x.h"

/* PMU definitions */
#define PMU                             PMU_BASE                            /*!< PMU base address */

/* registers definitions */
#define PMU_CTL0                        REG32((PMU) + 0x00000000U)          /*!< PMU control register 0 */
#define PMU_CS                          REG32((PMU) + 0x00000004U)          /*!< PMU control and status register */
#define PMU_CTL1                        REG32((PMU) + 0x00000008U)          /*!< PMU control register 1 */
#define PMU_CTL2                        REG32((PMU) + 0x00000010U)          /*!< PMU control register 2 */
#define PMU_CTL3                        REG32((PMU) + 0x00000014U)          /*!< PMU control register 3 */
#define PMU_PAR                         REG32((PMU) + 0x00000018U)          /*!< PMU parameter register */

/* bits definitions */
/* PMU_CTL0 */
#define PMU_CTL0_STBMOD                 BIT(1)                              /*!< standby mode */
#define PMU_CTL0_WURST                  BIT(2)                              /*!< wakeup flag reset */
#define PMU_CTL0_STBRST                 BIT(3)                              /*!< standby flag reset */
#define PMU_CTL0_LVDEN                  BIT(4)                              /*!< low voltage detector enable */
#define PMU_CTL0_LVDT                   BITS(5,7)                           /*!< low voltage detector threshold */
#define PMU_CTL0_BKPWEN                 BIT(8)                              /*!< backup domain write enable */
#define PMU_CTL0_SLDOVS                 BITS(14,15)                         /*!< deep sleep mode mode LDO output voltage select */
#define PMU_CTL0_VAVDEN                 BIT(16)                             /*!< VDDA analog voltage detector enable */
#define PMU_CTL0_VAVDVC                 BITS(17,18)                         /*!< VDDA analog voltage detector level configure bits */
#define PMU_CTL0_VOVDEN                 BIT(19)                             /*!< peripheral voltage on Vcore detector enable bit */

/* PMU_CS */
#define PMU_CS_WUF                      BIT(0)                              /*!< wakeup flag */
#define PMU_CS_STBF                     BIT(1)                              /*!< standby flag */
#define PMU_CS_LVDF                     BIT(2)                              /*!< low voltage detector status flag */
#define PMU_CS_WUPEN0                   BIT(8)                              /*!< wakeup pin0(PA0) enable */
#define PMU_CS_WUPEN1                   BIT(9)                              /*!< wakeup pin1(PA2) enable */
#define PMU_CS_WUPEN3                   BIT(11)                             /*!< wakeup pin3(PC13) enable */
#define PMU_CS_WUPEN5                   BIT(13)                             /*!< wakeup pin5(PC1) enable */
#define PMU_CS_VAVDF                    BIT(16)                             /*!< VDDA analog voltage detector voltage output on VDDA flag */
#define PMU_CS_VOVDF                    BIT(20)                             /*!< peripheral voltage on Vcore detector flag bit */

/* PMU_CTL1 */
#define PMU_CTL1_BKPVSEN                BIT(0)                              /*!< backup voltage stabilizer enable */
#define PMU_CTL1_VBTMEN                 BIT(4)                              /*!< VBAT and temperature monitoring enable */
#define PMU_CTL1_BKPVSRF                BIT(16)                             /*!< backup regulator ready */
#define PMU_CTL1_VBATLF                 BIT(20)                             /*!< VBAT level monitoring versus low threshold */
#define PMU_CTL1_VBATHF                 BIT(21)                             /*!< VBAT level monitoring versus high threshold */
#define PMU_CTL1_TEMPLF                 BIT(22)                             /*!< temperature level monitoring versus low threshold */
#define PMU_CTL1_TEMPHF                 BIT(23)                             /*!< temperature level monitoring versus high threshold */

/* PMU_CTL2 */
#define PMU_CTL2_BYPASS                 BIT(0)                              /*!< power management unit bypass control bit */
#define PMU_CTL2_LDOEN                  BIT(1)                              /*!< Low drop-out voltage stabilizer enable bit */
#define PMU_CTL2_DVSEN                  BIT(2)                              /*!< step-down voltage stabilizer enable bit */
#define PMU_CTL2_VCEN                   BIT(8)                              /*!< VBAT battery charging enable */
#define PMU_CTL2_VCRSEL                 BIT(9)                              /*!< VBAT battery charging resistor selection */
#define PMU_CTL2_DVSRF                  BIT(16)                             /*!< step-down voltage stabilizer ready flag bit */
#define PMU_CTL2_VUSB33DEN              BIT(24)                             /*!< VDD33USB voltage level detector enable bit */
#define PMU_CTL2_USBSEN                 BIT(25)                             /*!< USB voltage stabilizer enable */
#define PMU_CTL2_USB33RF                BIT(26)                             /*!< USB supply ready flag */

/* PMU_CTL3 */
#define PMU_CTL3_LDOVS                  BITS(12,14)                         /*!< voltage scaling selection according to performance */
#define PMU_CTL3_VOVRF                  BIT(16)                             /*!< Vcore voltage ready bit */

/* PMU_PAR */
#define PMU_PAR_CNT                     BITS(0,11)                          /*!< exit deep-sleep mode wait time count configure bits */
#define PMU_PAR_TSW_IRCCNT              BITS(16,20)                         /*!< when enter deep-sleep, switch to IRC wait clock */

/* constants definitions */
/* PMU low voltage detector threshold definitions */
#define CTL0_LVDT(regval)               (BITS(5,7) & ((uint32_t)(regval) << 5U))
#define PMU_LVDT_0                      CTL0_LVDT(0)                        /*!< voltage threshold is 1.9V */
#define PMU_LVDT_1                      CTL0_LVDT(1)                        /*!< voltage threshold is 2.1V */
#define PMU_LVDT_2                      CTL0_LVDT(2)                        /*!< voltage threshold is 2.2V */
#define PMU_LVDT_3                      CTL0_LVDT(3)                        /*!< voltage threshold is 2.4V */
#define PMU_LVDT_4                      CTL0_LVDT(4)                        /*!< voltage threshold is 2.5V */
#define PMU_LVDT_5                      CTL0_LVDT(5)                        /*!< voltage threshold is 2.7V */
#define PMU_LVDT_6                      CTL0_LVDT(6)                        /*!< voltage threshold is 2.9V */
#define PMU_LVDT_7                      CTL0_LVDT(7)                        /*!< input analog voltage on PB7 (compared with 1.2V) */

/* PMU deep-sleep mode voltage scaling selection */
#define CTL0_SLDOVS(regval)             (BITS(14,15) & ((uint32_t)(regval) << 14U))
#define PMU_SLDOVS_0                    CTL0_SLDOVS(0)                      /*!< SLDOVS scale 0.6V */
#define PMU_SLDOVS_1                    CTL0_SLDOVS(1)                      /*!< SLDOVS scale 0.7V */
#define PMU_SLDOVS_2                    CTL0_SLDOVS(2)                      /*!< SLDOVS scale 0.8V (default) */
#define PMU_SLDOVS_3                    CTL0_SLDOVS(3)                      /*!< SLDOVS scale 0.9V */

/* PMU analog voltage detector threshold definitions */
#define CTL0_VAVDVC(regval)             (BITS(17,18) & ((uint32_t)(regval) << 17U))
#define PMU_VAVDVC_0                    CTL0_VAVDVC(0)                      /*!< voltage threshold is 1.7V */
#define PMU_VAVDVC_1                    CTL0_VAVDVC(1)                      /*!< voltage threshold is 2.1V */
#define PMU_VAVDVC_2                    CTL0_VAVDVC(2)                      /*!< voltage threshold is 2.5V */
#define PMU_VAVDVC_3                    CTL0_VAVDVC(3)                      /*!< voltage threshold is 2.8V */

/* PMU VBAT battery charging resistor selection */
#define CTL2_VCRSEL(regval)             (BIT(9) & ((uint32_t)(regval) << 9U))
#define PMU_VCRSEL_5K                   CTL2_VCRSEL(0)                      /*!< 5 kOhms resistor is selected for charing VBAT battery */
#define PMU_VCRSEL_1P5K                 CTL2_VCRSEL(1)                      /*!< 1.5 kOhms resistor is selected for charing VBAT battery */

/* PMU LDO output voltage select definitions */
#define CTL3_LDOVS(regval)              (BITS(12,14) & ((uint32_t)(regval) << 12U))
#define PMU_LDOVS_0                     CTL3_LDOVS(0)                       /*!< LDO output voltage 0.8V mode */
#define PMU_LDOVS_1                     CTL3_LDOVS(1)                       /*!< LDO output voltage 0.85V mode */
#define PMU_LDOVS_2                     CTL3_LDOVS(2)                       /*!< LDO output voltage 0.9V mode */
#define PMU_LDOVS_3                     CTL3_LDOVS(3)                       /*!< LDO output voltage 0.95V mode */
#define PMU_LDOVS_4                     CTL3_LDOVS(4)                       /*!< LDO output voltage 0.975V mode */
#define PMU_LDOVS_5                     CTL3_LDOVS(5)                       /*!< LDO output voltage 1V mode */

/* PMU flag definitions */
#define PMU_REGIDX_BIT(regidx, bitpos)  (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define PMU_REG_VAL(periph)             (REG32(PMU + ((uint32_t)(periph) >> 6U)))
#define PMU_BIT_POS(val)                ((uint32_t)(val) & 0x1FU)

#define PMU_CTL0_OFFSET                 (0x00000000U)                       /*!< PMU control register 0 offset */
#define PMU_CS_OFFSET                   (0x00000004U)                       /*!< PMU control and status register offset */
#define PMU_CTL1_OFFSET                 (0x00000008U)                       /*!< PMU control register 1 offset */
#define PMU_CTL2_OFFSET                 (0x00000010U)                       /*!< PMU control register 2 offset */
#define PMU_CTL3_OFFSET                 (0x00000014U)                       /*!< PMU control register 3 offset */
#define PMU_PAR_OFFSET                  (0x00000018U)                       /*!< PMU parameter register offset */

#define PMU_FLAG_WAKEUP                 PMU_REGIDX_BIT(PMU_CS_OFFSET, 0)    /*!< wakeup flag */
#define PMU_FLAG_STANDBY                PMU_REGIDX_BIT(PMU_CS_OFFSET, 1)    /*!< standby flag */
#define PMU_FLAG_LVDF                   PMU_REGIDX_BIT(PMU_CS_OFFSET, 2)    /*!< low voltage detector status flag */
#define PMU_FLAG_VAVDF                  PMU_REGIDX_BIT(PMU_CS_OFFSET, 16)   /*!< VDDA analog voltage detector voltage output on VDDA flag */
#define PMU_FLAG_VOVDF                  PMU_REGIDX_BIT(PMU_CS_OFFSET, 20)   /*!< peripheral voltage on VDDA detector flag */
#define PMU_FLAG_BKPVSRF                PMU_REGIDX_BIT(PMU_CTL1_OFFSET, 16) /*!< backup voltage stabilizer ready flag */
#define PMU_FLAG_VBATLF                 PMU_REGIDX_BIT(PMU_CTL1_OFFSET, 20) /*!< VBAT level monitoring versus low threshold */
#define PMU_FLAG_VBATHF                 PMU_REGIDX_BIT(PMU_CTL1_OFFSET, 21) /*!< VBAT level monitoring versus high threshold */
#define PMU_FLAG_TEMPLF                 PMU_REGIDX_BIT(PMU_CTL1_OFFSET, 22) /*!< temperature level monitoring versus low threshold */
#define PMU_FLAG_TEMPHF                 PMU_REGIDX_BIT(PMU_CTL1_OFFSET, 23) /*!< temperature level monitoring versus high threshold */
#define PMU_FLAG_DVSRF                  PMU_REGIDX_BIT(PMU_CTL2_OFFSET, 16) /*!< step-down voltage stabilizer ready flag bit */
#define PMU_FLAG_USB33RF                PMU_REGIDX_BIT(PMU_CTL2_OFFSET, 26) /*!< USB supply ready flag bit */
#define PMU_FLAG_PWRRF                  PMU_REGIDX_BIT(PMU_CTL3_OFFSET, 16) /*!< Power Ready flag bit */

/* PMU wakeup pin definitions */
#define PMU_WAKEUP_PIN0                 PMU_CS_WUPEN0                       /*!< wakeup pin 0 */
#define PMU_WAKEUP_PIN1                 PMU_CS_WUPEN1                       /*!< wakeup pin 1 */
#define PMU_WAKEUP_PIN3                 PMU_CS_WUPEN3                       /*!< wakeup pin 3 */
#define PMU_WAKEUP_PIN5                 PMU_CS_WUPEN5                       /*!< wakeup pin 5 */

/* PMU SMPS LDO supply mode definitions */
#define PMU_LDO_SUPPLY                  PMU_CTL2_LDOEN                      /*!< Vcore domains are suppplied from the LDO */
#define PMU_DIRECT_SMPS_SUPPLY          PMU_CTL2_DVSEN                      /*!< Vcore domains are suppplied from the SMPS only */
#define PMU_BYPASS                      PMU_CTL2_BYPASS                     /*!< The SMPS disabled and the LDO Bypass. The Vcore domains are supplied from an external source */

/* PMU command constants definitions */
#define WFI_CMD                         ((uint8_t)0x00U)                    /*!< use WFI command */
#define WFE_CMD                         ((uint8_t)0x01U)                    /*!< use WFE command */

#define PMU_LDO_CFG_TIMEOUT_COUNT       ((uint32_t)0x000FFFFFU)            /*!< count to judge of PMU ldo configuration timeout */
#define PMU_BKUP_TIMEOUT_COUNT          ((uint32_t)0x000FFFFFU)            /*!< count to judge of PMU backup voltage stabilizer configuration timeout */

/* parameter check definitions */
#ifdef FW_DEBUG_ERR_REPORT

/* check PMU Vcore core voltage level */
#define NOT_PMU_LDO(ldo_n)                               (((ldo_n) != PMU_LDOVS_0) && ((ldo_n) != PMU_LDOVS_1) && \
                                                         ((ldo_n) != PMU_LDOVS_2) && ((ldo_n) != PMU_LDOVS_3) && \
                                                         ((ldo_n) != PMU_LDOVS_4) && ((ldo_n) != PMU_LDOVS_5))

/* check PMU power supply */
#define NOT_PMU_SMPS_LDO_SUPPLY(smpsmode)                (((smpsmode) != PMU_LDO_SUPPLY) && ((smpsmode) != PMU_DIRECT_SMPS_SUPPLY) && \
                                                         ((smpsmode) != PMU_BYPASS))

/* check PMU low power mode */
#define NOT_PMU_CMD(cmd)                                 (((cmd) != WFI_CMD) && ((cmd) != WFE_CMD))

/* check PMU IRC counter before enter Deep-sleep mode */
#define NOT_PMU_ENTER_DEEPSLEEP_WAIT_TIME(wait_time)     ((wait_time) > 0x1FU)

/* check PMU IRC counter before exit Deep-sleep mode */
#define NOT_PMU_EXIT_DEEPSLEEP_WAIT_TIME(wait_time)      ((wait_time) > 0xFFFU)

/* check PMU flag state */
#define NOT_PMU_FLAG(flag)                               (((flag) != PMU_FLAG_WAKEUP) && ((flag) != PMU_FLAG_STANDBY) && \
                                                         ((flag) != PMU_FLAG_LVDF) && ((flag) != PMU_FLAG_VAVDF) && \
                                                         ((flag) != PMU_FLAG_VOVDF) && ((flag) != PMU_FLAG_VBATLF) && \
                                                         ((flag) != PMU_FLAG_VBATHF) && ((flag) != PMU_FLAG_TEMPLF) && \
                                                         ((flag) != PMU_FLAG_TEMPHF) && ((flag) != PMU_FLAG_DVSRF) && \
                                                         ((flag) != PMU_FLAG_USB33RF) && ((flag) != PMU_FLAG_PWRRF))

/* check PMU reset flag state */
#define NOT_PMU_FLAG_RESET(flag_reset)                   (((flag_reset) != PMU_FLAG_WAKEUP) && ((flag_reset) != PMU_FLAG_STANDBY))

#endif /* FW_DEBUG_ERR_REPORT */

/* function declarations */
/* reset PMU registers */
void pmu_deinit(void);

/* select low voltage detector threshold */
void pmu_lvd_select(uint32_t lvdt_n);
/* enable PMU lvd */
void pmu_lvd_enable(void);
/* disable PMU lvd */
void pmu_lvd_disable(void);
/* select analog voltage detector threshold */
void pmu_vavd_select(uint32_t vavdt_n);
/* enable PMU analog voltage detector */
void pmu_vavd_enable(void);
/* disable PMU analog voltage detector */
void pmu_vavd_disable(void);
/* enable PMU core voltage detector */
void pmu_vovd_enable(void);
/* disable PMU Vcore core voltage detector */
void pmu_vovd_disable(void);
/* control the Vcore core voltage level */
void pmu_ldo_output_select(uint32_t ldo_n);
/* Deep-sleep mode Vcore core voltage select */
void pmu_sldo_output_select(uint32_t sldo_n);

/* PMU VBAT battery charging resistor selection */
void pmu_vbat_charging_select(uint32_t resistor);
/* enable VBAT battery charging */
void pmu_vbat_charging_enable(void);
/* disable VBAT battery charging */
void pmu_vbat_charging_disable(void);
/* enable VBAT and temperature monitoring */
void pmu_vbat_temp_moniter_enable(void);
/* disable VBAT and temperature monitoring */
void pmu_vbat_temp_moniter_disable(void);

/* USB regulator */
/* enable USB regulator */
void pmu_usb_regulator_enable(void);
/* disable USB regulator */
void pmu_usb_regulator_disable(void);
/* enable VDD33USB voltage level detector */
void pmu_usb_voltage_detector_enable(void);
/* disable VDD33USB voltage level detector */
void pmu_usb_voltage_detector_disable(void);

/* power supply configurations */
ErrStatus pmu_smps_ldo_supply_config(uint32_t smpsmode);

/* set PMU mode */
/* enter sleep mode */
void pmu_to_sleepmode(uint8_t sleepmodecmd);
/* enter deepsleep mode */
void pmu_to_deepsleepmode(uint8_t deepsleepmodecmd);
/* enter standby mode */
void pmu_to_standbymode(void);
/* enable PMU wakeup pin */
void pmu_wakeup_pin_enable(uint32_t wakeup_pin);
/* disable PMU wakeup pin */
void pmu_wakeup_pin_disable(uint32_t wakeup_pin);

/* backup related functions */
/* enable backup domain write */
void pmu_backup_write_enable(void);
/* disable backup domain write */
void pmu_backup_write_disable(void);
/* enable backup voltage stabilizer */
ErrStatus pmu_backup_voltage_stabilizer_enable(void);
/* disable backup voltage stabilizer */
void pmu_backup_voltage_stabilizer_disable(void);

/* configure IRC counter before enter Deep-sleep mode */
void pmu_enter_deepsleep_wait_time_config(uint32_t wait_time);
/* configure IRC counter before exit Deep-sleep mode */
void pmu_exit_deepsleep_wait_time_config(uint32_t wait_time);

/* flag functions */
/* get flag state */
FlagStatus pmu_flag_get(uint32_t flag);
/* clear flag bit */
void pmu_flag_clear(uint32_t flag_reset);

#endif /* GD32H73X_75X_PMU_H */
