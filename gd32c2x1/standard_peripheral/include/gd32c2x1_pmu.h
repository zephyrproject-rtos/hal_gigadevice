/*!
    \file    gd32c2x1_pmu.h
    \brief   definitions for the PMU

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

#ifndef GD32C2X1_PMU_H
#define GD32C2X1_PMU_H

#include "gd32c2x1.h"

/* PMU definitions */
#define PMU                           PMU_BASE                            /*!< PMU base address */

/* registers definitions */
#define PMU_CTL0                      REG32(PMU + 0x00000000U)            /*!< PMU control register 0 */
#define PMU_CS                        REG32(PMU + 0x00000004U)            /*!< PMU control and status register */
#define PMU_CTL1                      REG32(PMU + 0x00000008U)            /*!< PMU control register 1 */
#define PMU_STAT                      REG32(PMU + 0x0000000CU)            /*!< PMU status register */
#define PMU_PAR                       REG32(PMU + 0x00000010U)            /*!< PMU parameter register */

/* bits definitions */
/* PMU_CTL0 */
#define PMU_CTL0_LPMOD                BITS(0,1)                             /*!< select the low-power mode to enter */
#define PMU_CTL0_WURST                BIT(2)                                /*!< wakeup flag reset */
#define PMU_CTL0_STBRST               BIT(3)                                /*!< standby flag reset */
#define PMU_CTL0_BKPWEN               BIT(8)                                /*!< backup domain write enable */
#define PMU_CTL0_DSMODVS              BITS(12,13)                           /*!< Deep-sleep mode voltage selection */
#define PMU_CTL0_LPLDOEN              BIT(18)                               /*!< Low power LDO enable */

/* PMU_CS */
#define PMU_CS_WUF                    BIT(0)                                /*!< wakeup flag */
#define PMU_CS_STBF                   BIT(1)                                /*!< standby flag */
#define PMU_CS_WUPEN0                 BIT(8)                                /*!< wakeup pin0 enable */
#define PMU_CS_WUPEN1                 BIT(9)                                /*!< wakeup pin1 enable */
#define PMU_CS_WUPEN2                 BIT(10)                               /*!< wakeup pin2 enable */
#define PMU_CS_WUPEN3                 BIT(11)                               /*!< wakeup pin3 enable */
#define PMU_CS_WUPEN5                 BIT(13)                               /*!< wakeup pin5 enable */
#define PMU_CS_LDOVSRF                BIT(14)                               /*!< LDO voltage select ready flag */
#define PMU_CS_NPRDY                  BIT(16)                               /*!< NPLDO ready flag */

/* PMU_CTL1 */
#define PMU_CTL1_EFPSLEEP             BIT(4)                                /*!< eflash domain power-off when enter run/run1 */
#define PMU_CTL1_EFDSPSLEEP           BIT(5)                                /*!< eflash domain power-off when enter deepsleep/deepsleep1 */

/* PMU_STAT */
#define PMU_STAT_EFLASHPS_SLEEP       BIT(4)                                /*!< EFLASH domain is power-off */
#define PMU_STAT_EFLASHPS_ACTIVE      BIT(5)                                /*!< EFLASH domain is in active state */

/* PMU_PAR */
#define PMU_PAR_TSW_IRC48MCNT         BITS(16,20)                           /*!< wait the IRC48M COUNTER and then set Deep-sleep signal */
#define PMU_PAR_TWK_EFLASH            BITS(21,28)                           /*!< EFLASH wake up from Deep-sleep/Deep-sleep1 state counter */

/* constants definitions */
/* select the low-power mode to enter */
#define CTL0_LPMOD(regval)            (BITS(0,1)&((uint32_t)(regval)<<0))
#define PMU_DEEPSLEEP                 CTL0_LPMOD(0)                         /*!< Deep-sleep mode */
#define PMU_DEEPSLEEP1                CTL0_LPMOD(1)                         /*!< Deep-sleep mode 1 */
#define PMU_STANDBY                   CTL0_LPMOD(3)                         /*!< standby mode */

/* select the deep-sleep voltage */
#define CTL0_DSMODVS(regval)          (BITS(12,13)&((uint32_t)(regval)<<12))
#define PMU_DSV_0                     CTL0_DSMODVS(0)                       /*!< 0.9V */
#define PMU_DSV_1                     CTL0_DSMODVS(1)                       /*!< 1.0V */
#define PMU_DSV_2                     CTL0_DSMODVS(2)                       /*!< 1.1V */
#define PMU_DSV_3                     CTL0_DSMODVS(3)                       /*!< 1.2V */

/* PMU wakeup pin definitions */
#define PMU_WAKEUP_PIN0               PMU_CS_WUPEN0                         /*!< WKUP Pin 0 (PA0) */
#define PMU_WAKEUP_PIN1               PMU_CS_WUPEN1                         /*!< WKUP Pin 1 (PC13/PA4) */
#define PMU_WAKEUP_PIN2               PMU_CS_WUPEN2                         /*!< WKUP Pin 2 (PB6) */
#define PMU_WAKEUP_PIN3               PMU_CS_WUPEN3                         /*!< WKUP Pin 3 (PA2) */
#define PMU_WAKEUP_PIN5               PMU_CS_WUPEN5                         /*!< WKUP Pin 5 (PB5) */

/* PMU flag definitions */
#define PMU_FLAG_WAKEUP               PMU_CS_WUF                            /*!< wakeup flag status */
#define PMU_FLAG_STANDBY              PMU_CS_STBF                           /*!< standby flag status */
#define PMU_FLAG_LDOVSRF              PMU_CS_LDOVSRF                        /*!< LDO voltage select ready flag */
#define PMU_FLAG_NPRDY                PMU_CS_NPRDY                          /*!< normal-power LDO ready flag */

/* PMU command constants definitions */
#define WFI_CMD                       ((uint8_t)0x00U)                      /*!< use WFI command */
#define WFE_CMD                       ((uint8_t)0x01U)                      /*!< use WFE command */

/* function declarations */
/* reset PMU registers */
void pmu_deinit(void);
/* select deepsleep mode voltage */
void pmu_deepsleep_voltage_select(uint32_t dsv_n);
/* enable low power LDO */
void pmu_low_power_ldo_enable(void);
/* disable low power LDO */
void pmu_low_power_ldo_disable(void);
/* PMU work in Sleep mode */
void pmu_to_sleepmode(uint8_t sleepmodecmd);
/* PMU work in Deep-sleep mode */
void pmu_to_deepsleepmode(uint8_t deepsleepmodecmd, uint8_t deepsleepmode);
/* PMU work in standby mode */
void pmu_to_standbymode(void);
/* enable PMU wakeup pin */
void pmu_wakeup_pin_enable(uint32_t wakeup_pin);
/* disable PMU wakeup pin */
void pmu_wakeup_pin_disable(uint32_t wakeup_pin);

/* backup related functions */
/* enable write access to the backup registers in RTC */
void pmu_backup_write_enable(void);
/* disable write access to the backup registers in RTC */
void pmu_backup_write_disable(void);

/* configure power state of eflash domain in run/run1 mode */
void pmu_eflash_run_power_config(ControlStatus state);
/* configure power state of eflash domain when enter deepsleep/deepsleep1 */
void pmu_eflash_deepsleep_power_config(ControlStatus state);

/* configure eflash wakeup time, using IRC48M counter */
void pmu_eflash_wakeup_time_config(uint32_t wakeup_time);
/* configure IRC48M counter before enter Deepsleep/Deepsleep1 mode */
void pmu_deepsleep_wait_time_config(uint32_t wait_time);

/* flag functions */
/* get flag state */
FlagStatus pmu_flag_get(uint32_t flag);
/* clear flag bit */
void pmu_flag_clear(uint32_t flag);

#endif /* GD32C2X1_PMU_H */
