/*!
    \file    gd32c2x1_fmc.h
    \brief   definitions for the FMC

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

#ifndef GD32C2X1_FMC_H
#define GD32C2X1_FMC_H

#include "gd32c2x1.h"

/* FMC and option bytes definition */
#define FMC                                    FMC_BASE                                         /*!< FMC base address */

/* registers definitions */
#define FMC_WS                                 REG32(FMC + 0x00000000U)                         /*!< FMC wait state register */
#define FMC_KEY                                REG32(FMC + 0x00000008U)                         /*!< FMC unlock key register */
#define FMC_OBKEY                              REG32(FMC + 0x0000000CU)                         /*!< FMC option bytes unlock key register */
#define FMC_STAT                               REG32(FMC + 0x00000010U)                         /*!< FMC status register */
#define FMC_CTL                                REG32(FMC + 0x00000014U)                         /*!< FMC control register */
#define FMC_OBCTL                              REG32(FMC + 0x00000020U)                         /*!< FMC option byte register */
#define FMC_DCRP_SADDR0                        REG32(FMC + 0x00000024U)                         /*!< FMC DCRP area 0 start address register */
#define FMC_DCRP_EADDR0                        REG32(FMC + 0x00000028U)                         /*!< FMC DCRP area 0 end address register */
#define FMC_WP0                                REG32(FMC + 0x0000002CU)                         /*!< FMC erase/program protection area 0 register */
#define FMC_WP1                                REG32(FMC + 0x00000030U)                         /*!< FMC erase/program protection area 1 register */
#define FMC_DCRP_SADDR1                        REG32(FMC + 0x00000034U)                         /*!< FMC DCRP area 1 start address register */
#define FMC_DCRP_EADDR1                        REG32(FMC + 0x00000038U)                         /*!< FMC DCRP area 1 end address register */
#define FMC_SCR                                REG32(FMC + 0x00000080U)                         /*!< FMC securable area register */
#define FMC_PID                                REG32(FMC + 0x00000110U)                         /*!< FMC product ID register */

/* bits definitions */
/* FMC_WS */
#define FMC_WS_WSCNT                           BITS(0,2)                                        /*!< wait state counter */
#define FMC_WS_PFEN                            BIT(8)                                           /*!< pre-fetch enable */
#define FMC_WS_ICEN                            BIT(9)                                           /*!< IBUS cache enable */
#define FMC_WS_ICRST                           BIT(11)                                          /*!< IBUS cache reset */
#define FMC_WS_MFPE                            BIT(16)                                          /*!< main flash programmed or empty flag */
#define FMC_WS_DBGEN                           BIT(18)                                          /*!< enable/disable the debugger by software */

/* FMC_KEY */
#define FMC_KEY_KEY                            BITS(0,31)                                       /*!< FMC_CTL unlock key */

/* FMC_OBKEY */
#define FMC_OBKEY_OBKEY                        BITS(0,31)                                       /*!< option bytes unlock key */

/* FMC_STAT */
#define FMC_STAT_ENDF                          BIT(0)                                           /*!< end of operation flag */
#define FMC_STAT_OPRERR                        BIT(1)                                           /*!< operation error flag bit */
#define FMC_STAT_PGERR                         BIT(3)                                           /*!< program error flag */
#define FMC_STAT_WPERR                         BIT(4)                                           /*!< erase/program protection error flag bit */
#define FMC_STAT_PGAERR                        BIT(5)                                           /*!< program alignment error flag */
#define FMC_STAT_PGMERR                        BIT(6)                                           /*!< program size not match error flag bit */
#define FMC_STAT_PGSERR                        BIT(7)                                           /*!< program sequence error flag bit */
#define FMC_STAT_FSTPERR                       BIT(9)                                           /*!< fast programming error flag bit */
#define FMC_STAT_RPERR                         BIT(14)                                          /*!< read protection error flag bit */
#define FMC_STAT_OBERR                         BIT(15)                                          /*!< option byte read error bit */
#define FMC_STAT_BUSY                          BIT(16)                                          /*!< flash busy flag */

/* FMC_CTL */
#define FMC_CTL_PG                             BIT(0)                                           /*!< main flash page program command bit */
#define FMC_CTL_PER                            BIT(1)                                           /*!< main flash page erase command bit */
#define FMC_CTL_MER                            BIT(2)                                           /*!< main flash mass erase command bit */
#define FMC_CTL_PN                             BITS(3,8)                                        /*!< page number to erase */
#define FMC_CTL_START                          BIT(16)                                          /*!< send erase command to FMC bit */
#define FMC_CTL_OBSTART                        BIT(17)                                          /*!< option byte change command */
#define FMC_CTL_FSTPG                          BIT(18)                                          /*!< send option byte change command to FMC bit */
#define FMC_CTL_ENDIE                          BIT(24)                                          /*!< end of operation interrupt enable bit */
#define FMC_CTL_ERRIE                          BIT(25)                                          /*!< operation error interrupt enable bit */
#define FMC_CTL_RPERRIE                        BIT(26)                                          /*!< read protection error interrupt enable bit */
#define FMC_CTL_OBRLD                          BIT(27)                                          /*!< option byte reload bit */
#define FMC_CTL_SCR                            BIT(28)                                          /*!< security protection enable bit */
#define FMC_CTL_OBLK                           BIT(30)                                          /*!< FMC_OBCTL lock bit */
#define FMC_CTL_LK                             BIT(31)                                          /*!< FMC_CTL lock bit */

/* FMC_OBCTL */
#define FMC_OBCTL_SPC                          BITS(0,7)                                        /*!< security protection value */
#define FMC_OBCTL_BORST_EN                     BIT(8)                                           /*!< brown out reset enable */
#define FMC_OBCTL_BORR_TH                      BITS(9,10)                                       /*!< BOR threshold at rising VDD supply */
#define FMC_OBCTL_BORF_TH                      BITS(11,12)                                      /*!< BOR threshold at falling VDD supply */
#define FMC_OBCTL_NRST_STDBY                   BIT(14)                                          /*!< option byte standby reset value bit */
#define FMC_OBCTL_HXTAL_REMAP                  BIT(21)                                          /*!< HXTAL remapping bit */
#define FMC_OBCTL_SRAM_ECC_EN                  BIT(22)                                          /*!< SRAM ECC disable bit */
#define FMC_OBCTL_NRST_DPSLP                   BIT(13)                                          /*!< option byte deepsleep reset value bit */
#define FMC_OBCTL_NFWDG_HW                     BIT(16)                                          /*!< free watchdog configuration bit */
#define FMC_OBCTL_NWWDG_HW                     BIT(19)                                          /*!< window watchdog configuration bit */
#define FMC_OBCTL_SWBT0                        BIT(24)                                          /*!< software BOOT0 bit */
#define FMC_OBCTL_NBOOT1                       BIT(25)                                          /*!< NBOOT1 option bit */
#define FMC_OBCTL_NBOOT0                       BIT(26)                                          /*!< NBOOT0 option bit */
#define FMC_OBCTL_NRST_MDSEL                   BITS(27,28)                                      /*!< NRST pin mode bit */

/* FMC_DCRP_SADDR0 */
#define FMC_DCRP_SADDR0_DCRP0_SADDR            BITS(0,6)                                        /*!< start offset of DCRP area 0 */

/* FMC_DCRP_EADDR0 */ 
#define FMC_DCRP_EADDR0_DCRP0_EADDR            BITS(0,6)                                        /*!< end offset DCRP area0 */
#define FMC_DCRP_EADDR0_DCRP_EREN              BIT(31)                                          /*!< DCRP area erase enable configuration bit */

/* FMC_WP0 */
#define FMC_WP0_WP0_SADDR                      BITS(0,5)                                        /*!< start offset of write protection area 0 */
#define FMC_WP0_WP0_EADDR                      BITS(16,21)                                      /*!< end offset of write protection area 0 */

/* FMC_WP1 */
#define FMC_WP1_WP1_SADDR                      BITS(0,5)                                        /*!< start offset of write protection area 1 */
#define FMC_WP1_WP1_EADDR                      BITS(16,21)                                      /*!< end offset of write protection area 1 */

/* FMC_DCRP_SADDR1 */ 
#define FMC_DCRP_SADDR1_DCRP1_SADDR            BITS(0,6)                                        /*!< start offset of DCRP area1 */
                                                                                               
/* FMC_DCRP_EADDR1 */                                                                       
#define FMC_DCRP_EADDR1_DCRP1_EADDR            BITS(0,6)                                        /*!< end offset DCRP area 1 */

/* FMC_SCR */
#define FMC_SCR_SCR_PAGE_CNT                   BITS(0,6)                                        /*!< configure the number of pages of securable area */
#define FMC_SCR_BOOTLK                         BIT(16)                                          /*!< this bit is set to force boot from user flash area */

/* FMC_PID */
#define FMC_PID_PID                            BITS(0,31)                                       /*!< product ID bits */

/* constants definitions */
/* FMC_CTL unlock key */
#define FMC_UNLOCK_KEY0                        ((uint32_t)0x45670123U)                          /*!< FMC_CTL unlock key 0 */
#define FMC_UNLOCK_KEY1                        ((uint32_t)0xCDEF89ABU)                          /*!< FMC_CTL unlock key 1 */

/* FMC_CTL_OBLK unlock key */
#define OB_UNLOCK_KEY0                         ((uint32_t)0x08192A3B)                           /*!< FMC_CTL_OBLK unlock key 0 */
#define OB_UNLOCK_KEY1                         ((uint32_t)0x4C5D6E7F)                           /*!< FMC_CTL_OBLK unlock key 1 */

/* FMC wait state added */
#define WS_WSCNT(regval)                       (BITS(0,2) & ((uint32_t)(regval)))
#define FMC_WAIT_STATE_0                       WS_WSCNT(0)                                      /*!< 0 wait state added */
#define FMC_WAIT_STATE_1                       WS_WSCNT(1)                                      /*!< 1 wait state added */

/* FMC first location empty check status */
#define WS_MFPE(regval)                        (BIT(16) & ((uint32_t)(regval)))
#define FMC_WS_MFPE_PROGRAMMED                 WS_MFPE(0)                                       /*!< the first location of main flash is programmed */
#define FMC_WS_MFPE_EMPTY                      WS_MFPE(1)                                       /*!< the first location of main flash is empty */

/* option byte security protection configuration */
#define FMC_NSPC                               ((uint8_t)0xA5U)                                 /*!< no protection */
#define FMC_LSPC                               ((uint8_t)0xBBU)                                 /*!< protection level low */
#define FMC_HSPC                               ((uint8_t)0xCCU)                                 /*!< protection level high */

/* option byte BOR threshold value at rising VDD supply */
#define OBCTL_BORR_TH(regval)                  (BITS(9,10) & ((uint32_t)(regval) << 9U))
#define OB_BORR_TH_VALUE0                      OBCTL_BORR_TH(0)                                 /*!< BOR rising level 1 */
#define OB_BORR_TH_VALUE1                      OBCTL_BORR_TH(1)                                 /*!< BOR rising level 2 */
#define OB_BORR_TH_VALUE2                      OBCTL_BORR_TH(2)                                 /*!< BOR rising level 3 */
#define OB_BORR_TH_VALUE3                      OBCTL_BORR_TH(3)                                 /*!< BOR rising level 4 */

/* option byte BOR threshold value at falling VDD supply */
#define OBCTL_BORF_TH(regval)                  (BITS(11,12) & ((uint32_t)(regval) << 11U))
#define OB_BORF_TH_VALUE0                      OBCTL_BORF_TH(0)                                 /*!< BOR falling level 1 */
#define OB_BORF_TH_VALUE1                      OBCTL_BORF_TH(1)                                 /*!< BOR falling level 2 */
#define OB_BORF_TH_VALUE2                      OBCTL_BORF_TH(2)                                 /*!< BOR falling level 3 */
#define OB_BORF_TH_VALUE3                      OBCTL_BORF_TH(3)                                 /*!< BOR falling level 4 */

/* option byte BOR reset configuration */
#define OBCTL_BORST(regval)                   (BIT(8) & ((uint32_t)(regval) << 8U))
#define OB_BORST_DISABLE                      (OBCTL_BORST(0))                                  /*!< Brown out reset disable, power-on reset defined by POR/PDR levels */
#define OB_BORST_ENABLE                       (OBCTL_BORST(1))                                  /*!< Brown out reset enable, values of BORR_TH and BORF_TH taken into account */

/* option byte reset or not entering deep sleep mode */
#define OBCTL_NRST_DPSLP(regval)               (BIT(13) & ((uint32_t)(regval) << 13U))
#define OB_DEEPSLEEP_RST                       OBCTL_NRST_DPSLP(0)                              /*!< generate a reset instead of entering deepsleep mode */
#define OB_DEEPSLEEP_NRST                      OBCTL_NRST_DPSLP(1)                              /*!< no reset when entering deepsleep mode */

/* option byte reset or not entering standby mode */
#define OBCTL_NRST_STDBY(regval)               (BIT(14) & ((uint32_t)(regval) << 14U))
#define OB_STDBY_RST                           OBCTL_NRST_STDBY(0)                              /*!< generate a reset instead of entering standby mode */
#define OB_STDBY_NRST                          OBCTL_NRST_STDBY(1)                              /*!< no reset when entering deepsleep mode */

/* option byte software/hardware free watchdog timer */
#define OBCTL_NFWDG_HW(regval)                 (BIT(16) & ((uint32_t)(regval) << 16U))
#define OB_FWDGT_HW                            OBCTL_NFWDG_HW(0)                                /*!< hardware free watchdog */
#define OB_FWDGT_SW                            OBCTL_NFWDG_HW(1)                                /*!< software free watchdog */

/* option byte software/hardware window watchdog timer */
#define OBCTL_NWWDG_HW(regval)                 (BIT(19) & ((uint32_t)(regval) << 19U))
#define OB_WWDGT_HW                            OBCTL_NWWDG_HW(0)                                /*!< hardware window watchdog */
#define OB_WWDGT_SW                            OBCTL_NWWDG_HW(1)                                /*!< software window watchdog */

/* option byte software boot0 */
#define OBCTL_SWBT0(regval)                    (BIT(24) & ((uint32_t)(regval) << 24U))
#define OB_SWBOOT0_FROM_OB_BOOT0               OBCTL_SWBT0(1)                                   /*!< BOOT0 taken from the option bit nBOOT0 */
#define OB_SWBOOT0_FROM_PIN                    OBCTL_SWBT0(0)                                   /*!< BOOT0 taken from PB8/BOOT0 pin */

/* option byte boot1 configuration */
#define OBCTL_NBOOT1(regval)                   (BIT(25) & ((uint32_t)(regval) << 25U))
#define OB_NBOOT1_VALUE_0                      OBCTL_NBOOT1(0)                                  /*!< option byte NBOOT1 is value 0 */
#define OB_NBOOT1_VALUE_1                      OBCTL_NBOOT1(1)                                  /*!< option byte NBOOT1 is value 1 */

/* option byte boot0 configuration */
#define OBCTL_NBOOT0(regval)                   (BIT(26) & ((uint32_t)(regval) << 26U))
#define OB_NBOOT0_VALUE_0                      OBCTL_NBOOT0(0)                                  /*!< option byte NBOOT0 is value 0 */
#define OB_NBOOT0_VALUE_1                      OBCTL_NBOOT0(1)                                  /*!< option byte NBOOT0 is value 1 */

/* option byte reset pin mode */
#define OBCTL_NRST_MDSEL(regval)               (BITS(27,28) & ((uint32_t)(regval) << 27U))
#define OB_NRST_PIN_INPUT_MODE                 OBCTL_NRST_MDSEL(1)                              /*!< a low level on the NRST pin can reset system, internal reset can not drive NRST pin */
#define OB_NRST_PIN_NORMAL_GPIO                OBCTL_NRST_MDSEL(2)                              /*!< NRST pin function as normal GPIO */
#define OB_NRST_PIN_INPUT_OUTPUT_MODE          OBCTL_NRST_MDSEL(3)                              /*!< NRST pin configure as input/output mode */

/* option byte DCRP erase enable configuration */
#define OB_DCRP_EADDR_DCRP_EREN(regval)        (BIT(31) & ((uint32_t)(regval) << 31U))
#define OB_DCRP_AREA_ERASE_DISABLE             OB_DCRP_EADDR_DCRP_EREN(0)                       /*!< DCRP is not erased when a SPC value is decreased from value 1 to value 0 */
#define OB_DCRP_AREA_ERASE_ENABLE              OB_DCRP_EADDR_DCRP_EREN(1)                       /*!< DCRP is erased when a SPC value is decreased from value 1 to value 0 */

/* option byte boot lock */
#define OB_SCR_BOOTLK(regval)                  (BIT(16) & ((uint32_t)(regval) << 16U))
#define OB_BOOT_UNLOCK                         OB_SCR_BOOTLK(0)                                 /*!< unlock boot */
#define OB_BOOT_LOCK_FROM_MAIN_FLASH           OB_SCR_BOOTLK(1)                                 /*!< boot from main flash */

/* option byte HXTAL remapping */
#define OB_HXTAL_REMAP(regval)                 (BIT(21) & ((uint32_t)(regval) << 21U))
#define OB_HXTAL_REMAP_ENABLE                  OB_HXTAL_REMAP(0)                                /*!< HXTAL remapping enable */
#define OB_HXTAL_REMAP_DISABLE                 OB_HXTAL_REMAP(1)                                /*!< HXTAL remapping disable */

/* option byte SRAM ECC disable */
#define OB_SRAM_ECC_EN(regval)                 (BIT(22) & ((uint32_t)(regval) << 22U))
#define OB_SRAM_ECC_ENABLE                      OB_SRAM_ECC_EN(0)                               /*!< SRAM ECC check enable */
#define OB_SRAM_ECC_DISABLE                     OB_SRAM_ECC_EN(1)                               /*!< SRAM ECC check disable */

/* FMC interrupt enable */
#define FMC_INT_END                            FMC_CTL_ENDIE                                    /*!< FMC end of operation interrupt enable */
#define FMC_INT_ERR                            FMC_CTL_ERRIE                                    /*!< FMC error interrupt enable */
#define FMC_INT_RPERR                          FMC_CTL_RPERRIE                                  /*!< read protection error interrupt enable */

/* FMC flags */
#define FMC_FLAG_BUSY                          FMC_STAT_BUSY                                    /*!< FMC busy flag */
#define FMC_FLAG_ENDF                          FMC_STAT_ENDF                                    /*!< FMC end of operation flag */
#define FMC_FLAG_OBERR                         FMC_STAT_OBERR                                   /*!< option byte read error */
#define FMC_FLAG_RPERR                         FMC_STAT_RPERR                                   /*!< read protection error */
#define FMC_FLAG_FSTPERR                       FMC_STAT_FSTPERR                                 /*!< fast programming error */
#define FMC_FLAG_PGSERR                        FMC_STAT_PGSERR                                  /*!< program sequence error */
#define FMC_FLAG_PGMERR                        FMC_STAT_PGMERR                                  /*!< program size error*/
#define FMC_FLAG_PGAERR                        FMC_STAT_PGAERR                                  /*!< program alignment error */
#define FMC_FLAG_WPERR                         FMC_STAT_WPERR                                   /*!< erase/program protection error */
#define FMC_FLAG_PGERR                         FMC_STAT_PGERR                                   /*!< program error */
#define FMC_FLAG_OPRERR                        FMC_STAT_OPRERR                                  /*!< operation error */

/* FMC interrupt flags */
#define FMC_INT_FLAG_RPERR                     FMC_STAT_RPERR                                   /*!< read protection error interrupt flag */
#define FMC_INT_FLAG_OPRERR                    FMC_STAT_OPRERR                                  /*!< operation error interrupt flag */
#define FMC_INT_FLAG_END                       FMC_STAT_ENDF                                    /*!< end of operation interrupt flag */

#define OBCTL_SPC_OFFSET                       (0U)                                             /*!< bit offset of SPC offset in FMC_OBCTL register */
#define OBCTL_USER_OFFSET                      (8U)                                             /*!< bit offset of USER offset in FMC_OBCTL register */
#define CTL_PN_OFFSET                          (3U)                                             /*!< bit offset of PNSEL offset in FMC_CTL register */
#define SCR_PAGE_CNT_OFFSET                    (0U)                                             /*!< bit offset of SCR_PAGE_CNT in FMC_SCR */
#define DCRP_SADDR_OFFSET                      (0U)                                             /*!< bit offset of DCRP0_SADDR/DCRP1_SADDR in FMC_DCRP_SADDR0/FMC_DCRP_SADDR1 */
#define DCRP_EADDR_OFFSET                      (0U)                                             /*!< bit offset of DCRP0_EADDR/DCRP1_EADDR in FMC_DCRP_EADDR0/FMC_DCRP_EADDR1 */
#define DCRP_EREN_OFFSET                       (31U)                                            /*!< bit offset of DCRP_EREN in FMC_DCRP_EADDR */
#define WP_SADDR_OFFSET                        (0U)                                             /*!< bit offset of WP0_SADDR/WP1_SADDR in FMC_WP0/FMC_WP1 */
#define WP_EADDR_OFFSET                        (16U)                                            /*!< bit offset of WP0_EADDR/WP1_EADDR in FMC_WP0/FMC_WP1 */

#define MAIN_FLASH_BASE_ADDRESS                ((uint32_t)0x08000000U)                          /*!< main flash base address */
#define MAIN_FLASH_SIZE                        ((uint32_t)0x00010000U)                          /*!< main flash size */
#define MAIN_FLASH_PAGE_SIZE                   ((uint32_t)0x00000400U)                          /*!< main flash sub page size */ 
#define MAIN_FLASH_PAGE_TOTAL_NUM              ((uint32_t)0x00000040U)                          /*!< main flash page total number */
#define FMC_TIMEOUT_COUNT                      ((uint32_t)0xFFFFFFFFU)                          /*!< count to judge FMC timeout */
#define DOUBLEWORD_CNT_IN_ROW                  ((uint8_t)8U)
#define DCRP_AREA_SUBPAGE_SIZE                 ((uint32_t)0x00000200U)                          /*!< DCRP area subpage size */
#define DCRP_AREA_SUBPAGE_MAX_INDEX            ((MAIN_FLASH_SIZE / DCRP_AREA_SUBPAGE_SIZE) - 1U)/*!< DCRP area subpage max index */
#define WP_AREA_SUBPAGE_SIZE                   ((uint32_t)0x00000400U)                          /*!< write protection area subpage size */

#define INVLD_RETURN_VALUE                     ((uint32_t)0x00000000U)                          /*!< the return value is invalid */
#define VLD_RETURN_VALUE                       ((uint32_t)0x00000001U)                          /*!< the return value is valid */

/* DCRP area definition */
#define DCRP_AREA_0                            ((uint32_t)0x00000000U)                          /*!< DCRP area 0 */
#define DCRP_AREA_1                            ((uint32_t)0x00000001U)                          /*!< DCRP area 1 */
/* write protection area definition */
#define WP_AREA_0                              ((uint32_t)0x00000000U)                          /*!< write protection area 0 */
#define WP_AREA_1                              ((uint32_t)0x00000001U)                          /*!< write protection area 1 */

/* fmc state */
typedef enum {
    FMC_READY = 0,                                                                              /*!< the operation has been completed */
    FMC_BUSY,                                                                                   /*!< the operation is in progress */
    FMC_OBERR,                                                                                  /*!< option byte read error */
    FMC_RPERR,                                                                                  /*!< read protection error */
    FMC_FSTPERR,                                                                                /*!< fast programming error */
    FMC_PGSERR,                                                                                 /*!< program sequence error */
    FMC_PGMERR,                                                                                 /*!< program size error*/
    FMC_PGAERR,                                                                                 /*!< program alignment error */
    FMC_WPERR,                                                                                  /*!< erase/program protection error */
    FMC_PGERR,                                                                                  /*!< program error */
    FMC_OPRERR,                                                                                 /*!< operation error */
    FMC_TOERR,                                                                                  /*!< timeout error */
    FMC_OB_HSPC,                                                                                /*!< high security protection */
    FMC_UNDEFINEDERR                                                                            /*!< undefined error for function input parameter checking */
} fmc_state_enum;

/* user data extract infomation */
typedef enum {
    OBCTL_USER_DATA_BORST_EN              =          FMC_OBCTL_BORST_EN | 8U,                    /*!< brown out reset enable mask and it's start bit position */
    OBCTL_USER_DATA_BORR_TH               =          FMC_OBCTL_BORR_TH | 9U,                     /*!< BOR threshold at rising VDD supply mask and it's start bit position */
    OBCTL_USER_DATA_BORF_TH               =          FMC_OBCTL_BORF_TH | 11U,                    /*!< BOR threshold at falling VDD supply mask and it's start bit position */
    OBCTL_USER_DATA_NRST_STDBY            =          FMC_OBCTL_NRST_STDBY | 14U,                 /*!< option byte standby reset value bit mask and it's start bit position */
    OBCTL_USER_DATA_HXTAL_REMAP           =          FMC_OBCTL_HXTAL_REMAP | 21U,                /*!< HXTAL remapping bit mask and it's start bit position */
    OBCTL_USER_DATA_SRAM_ECC_EN           =          FMC_OBCTL_SRAM_ECC_EN | 22U,                /*!< SRAM ECC enable bit mask and it's start bit position */ 
    OBCTL_USER_DATA_NRST_DPSLP            =          FMC_OBCTL_NRST_DPSLP | 13U,                 /*!< option byte deepsleep reset value bit mask and it's start bit position */
    OBCTL_USER_DATA_NFWDG_HW              =          FMC_OBCTL_NFWDG_HW | 16U,                   /*!< free watchdog configuration bit mask and it's start bit position */
    OBCTL_USER_DATA_NWWDG_HW              =          FMC_OBCTL_NWWDG_HW | 19U,                   /*!< window watchdog configuration bit mask and it's start bit position */
    OBCTL_USER_DATA_SWBT0                 =          FMC_OBCTL_SWBT0 | 24U,                      /*!< software BOOT0 bit mask and it's start bit position */
    OBCTL_USER_DATA_NBOOT1                =          FMC_OBCTL_NBOOT1 | 25U,                     /*!< NBOOT1 option bit mask and it's start bit position */
    OBCTL_USER_DATA_NBOOT0                =          FMC_OBCTL_NBOOT0 | 26U,                     /*!< NBOOT0 option bit mask and it's start bit position */
    OBCTL_USER_DATA_NRST_MDSEL            =          FMC_OBCTL_NRST_MDSEL | 27U                  /*!< NRST pin mode bit mask and it's start bit position */
} ob_user_data_extract_info_enum;

/* parameter check definitions */
#ifdef FW_DEBUG_ERR_REPORT
/* check FMC empty check status */
#define NOT_FMC_EMPTY_CHECK_STATUS(empty_check_status)       ((empty_check_status) & (~FMC_WS_MFPE))

/* check FMC wait state configuration */
#define NOT_FMC_WAIT_STATE(wscnt)                            ((wscnt) > FMC_WAIT_STATE_1)

/* check FMC option byte USER mask */
#define NOT_FMC_OB_USER_MASK(ob_user_mask)                   ((ob_user_mask) & ~(FMC_OBCTL_BORST_EN    | FMC_OBCTL_BORR_TH  | FMC_OBCTL_BORF_TH  | FMC_OBCTL_NRST_DPSLP  | \
                                                                                 FMC_OBCTL_NRST_STDBY  | FMC_OBCTL_NFWDG_HW | FMC_OBCTL_NWWDG_HW | FMC_OBCTL_HXTAL_REMAP | \
                                                                                 FMC_OBCTL_SRAM_ECC_EN | FMC_OBCTL_SWBT0    | FMC_OBCTL_NBOOT1   | FMC_OBCTL_NBOOT0      | \
                                                                                 FMC_OBCTL_NRST_MDSEL ))

/* check FMC DCRP area configuration */
#define NOT_FMC_DCRP_AREA_VALID_CFG(dcrp_area,dcrp_eren,dcrp_start,dcrp_end)               ((((dcrp_area) != DCRP_AREA_0) && ((dcrp_area) != DCRP_AREA_1)) || \
                                                                                            (0U != ((dcrp_eren) & (~FMC_DCRP_EADDR0_DCRP_EREN))) || \
                                                                                            ((dcrp_start) > DCRP_AREA_SUBPAGE_MAX_INDEX) || \
                                                                                            ((dcrp_end) > DCRP_AREA_SUBPAGE_MAX_INDEX)   || \
                                                                                            ((dcrp_end) < (dcrp_start))) 

/* check FMC WP area configuration */
#define NOT_FMC_WP_AREA_VALID_CFG(wp_area,wp_start,wp_end)   ((((wp_area) != WP_AREA_0) && ((wp_area) != WP_AREA_1)) || \
                                                               ((wp_start) > (MAIN_FLASH_PAGE_TOTAL_NUM - 1U))       || \
                                                               ((wp_end) > (MAIN_FLASH_PAGE_TOTAL_NUM - 1U))         || \
                                                               ((wp_end) < (wp_start)))

/* check FMC data flash page number */
#define NOT_FMC_VALID_DATA_FLASH_PAGE(page_num)              ((page_num) > (DATA_FLASH_PAGE_TOTAL_NUM - 1U))
#endif 

/* function declarations */
/* FMC main flash operation functions */
/* unlock the main FMC operation */
void fmc_unlock(void);
/* lock the main FMC operation */
void fmc_lock(void);
/* get the main flash  empty check status */
FlagStatus fmc_main_flash_empty_stat_get(void);
/* modify the main flash empty check status */
void fmc_main_flash_empty_stat_modify(uint32_t empty_check_status);
/* set the wait state */
void fmc_wscnt_set(uint32_t wscnt);
/* enable pre-fetch */
void fmc_prefetch_enable(void);
/* disable pre-fetch */
void fmc_prefetch_disable(void);
/* enable IBUS cache */
void fmc_icache_enable(void);
/* disable IBUS cache */
void fmc_icache_disable(void);
/* reset IBUS cache  */
void fmc_icache_reset(void);
/* erase page */
fmc_state_enum fmc_page_erase(uint32_t page_number);
/* erase whole chip */
fmc_state_enum fmc_mass_erase(void);
/* program a doubleword at the given address in main flash */
fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data);
/* fast program a row at the corresponding address */
fmc_state_enum fmc_fast_program(uint32_t address, uint32_t data_buf);
/* enable debugger */
void fmc_debugger_enable(void);
/* disable debugger */
void fmc_debugger_disable(void);
/* enable secure area protection */
void fmc_scr_area_enable(void);

/* FMC option bytes operation functions */
/* unlock the option bytes operation */
void ob_unlock(void);
/* lock the option bytes operation */
void ob_lock(void);
/* reload the option bytes operation */
void ob_reload(void);
/* program the option bytes USER */
fmc_state_enum ob_user_write(uint32_t ob_user, uint32_t ob_user_mask);
/* configure security protection level*/
fmc_state_enum ob_security_protection_level_config(uint8_t ob_spc);
/* configure the option bytes DCRP area */
fmc_state_enum ob_dcrp_area_config(uint32_t dcrp_area, uint32_t dcrp_eren, uint32_t dcrp_start,
                                   uint32_t dcrp_end);
/* configure the option bytes write protection area */
fmc_state_enum ob_write_protection_area_config(uint32_t wp_area, uint32_t wp_start,
                                               uint32_t wp_end);
/* configure the option bytes secure area */
fmc_state_enum ob_scr_area_config(uint32_t secure_size);
/* configure the option bytes boot lock */
fmc_state_enum ob_boot_lock_config(uint32_t boot_config);
/* get the value of option bytes USER */
uint32_t ob_user_get(ob_user_data_extract_info_enum user_data_extract_info, uint8_t * ob_user_data);
/* get the value of option bytes security protection level in FMC_OBCTL register */
uint8_t ob_security_protection_level_get(void);
/* get configuration of DCRP area*/
uint32_t ob_dcrp_area_get(uint32_t dcrp_area, uint32_t *dcrp_erase_option,
                          uint32_t *dcrp_start, uint32_t *dcrp_end);
/* get address of write protection area */
uint32_t ob_write_protection_area_get(uint32_t wp_area, uint32_t *wp_start,
                                      uint32_t *wp_end);
/* get size of secure area */
uint32_t ob_scr_area_get(uint32_t *scr_area_byte_cnt);
/* get boot lock configuration */
uint32_t ob_boot_lock_get(void);

/* FMC interrupts and flags management functions */
/* get FMC flag status */
FlagStatus fmc_flag_get(uint32_t flag);
/* clear the FMC flag */
void fmc_flag_clear(uint32_t flag);
/* enable FMC interrupt */
void fmc_interrupt_enable(uint32_t interrupt);
/* disable FMC interrupt */
void fmc_interrupt_disable(uint32_t interrupt);
/* get FMC interrupt flag */
FlagStatus fmc_interrupt_flag_get(uint32_t flag);
/* clear FMC interrupt flag */
void fmc_interrupt_flag_clear(uint32_t flag);
/* get the FMC state*/
fmc_state_enum fmc_state_get(void);
/* check whether FMC is ready or not */
fmc_state_enum fmc_ready_wait(uint32_t timeout);

#endif /* GD32C2X1_FMC_H */
