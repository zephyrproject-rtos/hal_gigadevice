/*!
    \file    gd32a50x_fmc.h
    \brief   definitions for the FMC

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

#ifndef GD32A50X_FMC_H
#define GD32A50X_FMC_H

#include "gd32a50x.h"

/* FMC and option bytes definition */
#define FMC                          FMC_BASE                                           /*!< FMC register base address */
#define OB                           OB_BASE                                            /*!< option bytes 0 base address */

/* registers definitions */
#define FMC_WS                       REG32((FMC) + 0x00000000U)                         /*!< FMC wait state register */
#define FMC_ECCCS                    REG32((FMC) + 0x00000004U)                         /*!< FMC ECC control and status register */
#define FMC_KEY0                     REG32((FMC) + 0x00000008U)                         /*!< FMC unlock key register 0 */
#define FMC_STAT0                    REG32((FMC) + 0x0000000CU)                         /*!< FMC status register 0 */
#define FMC_CTL0                     REG32((FMC) + 0x00000010U)                         /*!< FMC control register 0 */
#define FMC_ADDR0                    REG32((FMC) + 0x00000014U)                         /*!< FMC address register 0 */
#define FMC_OBKEY                    REG32((FMC) + 0x00000044U)                         /*!< FMC option byte unlock key register */
#define FMC_KEY1                     REG32((FMC) + 0x00000048U)                         /*!< FMC unlock key register 1 */
#define FMC_STAT1                    REG32((FMC) + 0x0000004CU)                         /*!< FMC status register 1 */
#define FMC_CTL1                     REG32((FMC) + 0x00000050U)                         /*!< FMC control register 1 */
#define FMC_ADDR1                    REG32((FMC) + 0x00000054U)                         /*!< FMC address register 1 */
#define FMC_EPCNT                    REG32((FMC) + 0x00000058U)                         /*!< FMC EEPROM counter register */
#define FMC_OBSTAT                   REG32((FMC) + 0x0000005CU)                         /*!< FMC option byte status register */
#define FMC_WP0                      REG32((FMC) + 0x00000060U)                         /*!< FMC erase/program protection register 0 */
#define FMC_WP1                      REG32((FMC) + 0x00000064U)                         /*!< FMC erase/program protection register 1 */
#define FMC_OB1CS                    REG32((FMC) + 0x00000068U)                         /*!< FMC option byte 1 control and status register */
#define FMC_PID                      REG32((FMC) + 0x00000100U)                         /*!< FMC product ID register */

#define OP_BYTE(x)                   REG32(OB + ((uint32_t)((uint32_t)0x04U * (x))))    /*!< option bytes 0 value */
#define OB_SPC                       REG16((OB) + 0x00000000U)                          /*!< option bytes 0 security protection value*/
#define OB_USER                      REG16((OB) + 0x00000002U)                          /*!< option bytes 0 user value*/
#define OB_SPC_USER                  REG32((OB) + 0x00000000U)                          /*!< option bytes 0 security protection value and user value */
#define OB_DATA                      REG32((OB) + 0x00000004U)                          /*!< option bytes 0 data value */

/* bits definitions */
/* FMC_WS */
#define FMC_WS_WSCNT                 BITS(0,2)                                          /*!< wait state counter */
#define FMC_WS_PFEN                  BIT(4)                                             /*!< pre-fetch enable */
#define FMC_WS_IDCEN                 BIT(9)                                             /*!< cache enable */
#define FMC_WS_IDRST                 BIT(11)                                            /*!< cache reset */
#define FMC_WS_SLEEP_SLP             BIT(14)                                            /*!< flash goto sleep mode or power-down mode when MCU enters deepsleep mode */
#define FMC_WS_ERAMRDY               BIT(16)                                            /*!< EEPROM SRAM ready flag */
#define FMC_WS_BRAMRDY               BIT(17)                                            /*!< basic SRAM ready flag */
#define FMC_WS_PRAMRDY               BIT(18)                                            /*!< fast PG SRAM ready flag */

/* FMC_ECCCS */
#define FMC_ECCCS_ECCADDR            BITS(0,14)                                         /*!< the offset address of double word where an ECC error is detected */
#define FMC_ECCCS_OB0_ECC            BIT(19)                                            /*!< option bytes 0 one bit error flag */
#define FMC_ECCCS_BK1_ECC            BIT(20)                                            /*!< bank1 one bit error flag */
#define FMC_ECCCS_SYS_ECC            BIT(21)                                            /*!< system memory one bit error flag */
#define FMC_ECCCS_DF_ECC             BIT(22)                                            /*!< data flash one bit error flag */
#define FMC_ECCCS_OTP_ECC            BIT(23)                                            /*!< OTP one bit error flag */
#define FMC_ECCCS_ECCCORIE           BIT(24)                                            /*!< one bit error correct interrupt enable */
#define FMC_ECCCS_ECCDETIE           BIT(25)                                            /*!< two bits error detect interrupt enable */
#define FMC_ECCCS_OB1ECCDET          BIT(26)                                            /*!< option bytes 1 two bits error detect flag */
#define FMC_ECCCS_OB0ECCDET          BIT(27)                                            /*!< option bytes 0 two bits error detect flag */
#define FMC_ECCCS_EPECCDET           BIT(29)                                            /*!< EEPROM two bits error detect flag */
#define FMC_ECCCS_ECCCOR             BIT(30)                                            /*!< one bit error detected and correct flag */
#define FMC_ECCCS_ECCDET             BIT(31)                                            /*!< two bits error detect flag */

/* FMC_KEY0 */
#define FMC_KEY0_KEY                 BITS(0,31)                                         /*!< flash bank0 unlock key bits */

/* FMC_STAT0 */
#define FMC_STAT0_BUSY               BIT(0)                                             /*!< flash bank0 busy flag */
#define FMC_STAT0_PGSERR             BIT(1)                                             /*!< flash bank0 program sequence error flag */
#define FMC_STAT0_PGERR              BIT(2)                                             /*!< flash bank0 program error flag */
#define FMC_STAT0_PGAERR             BIT(3)                                             /*!< flash bank0 program alignment error flag */
#define FMC_STAT0_WPERR              BIT(4)                                             /*!< flash bank0 erase/program protection error flag */
#define FMC_STAT0_ENDF               BIT(5)                                             /*!< flash bank0 end of operation flag */
#define FMC_STAT0_CBCMDERR           BIT(6)                                             /*!< flash bank0 checked area by the check blank command is all 0xFF or not */
#define FMC_STAT0_RSTERR             BIT(15)                                            /*!< flash bank0 BOR/POR or system reset during erase/program flag */

/* FMC_CTL0 */
#define FMC_CTL0_PG                  BIT(0)                                             /*!< flash bank0 program command bit */
#define FMC_CTL0_PER                 BIT(1)                                             /*!< flash bank0 page erase bit */
#define FMC_CTL0_MER                 BIT(2)                                             /*!< flash bank0 mass erase bit */
#define FMC_CTL0_START               BIT(6)                                             /*!< send erase command to flash bank0 bit */
#define FMC_CTL0_LK                  BIT(7)                                             /*!< flash bank0 lock bit */
#define FMC_CTL0_FSTPG               BIT(8)                                             /*!< flash bank0 fast program command bit */
#define FMC_CTL0_ERRIE               BIT(10)                                            /*!< flash bank0 error interrupt enable bit */
#define FMC_CTL0_ENDIE               BIT(12)                                            /*!< flash bank0 end of operation interrupt enable bit */
#define FMC_CTL0_CBCMD               BIT(16)                                            /*!< send check blank command to flash bank0 bit */
#define FMC_CTL0_CBCMDLEN            BITS(29,31)                                        /*!< check blank command read length to flash bank0 */

/* FMC_ADDR0 */
#define FMC_ADDR0_ADDR               BITS(0,31)                                         /*!< flash bank0 command address bits */

/* FMC_OBKEY */
#define FMC_OBKEY_OBKEY              BITS(0,31)                                         /*!< option bytes unlock key bits */

/* FMC_KEY1 */
#define FMC_KEY1_KEY                 BITS(0,31)                                         /*!< flash bank1 unlock key bits */

/* FMC_STAT1 */
#define FMC_STAT1_BUSY               BIT(0)                                             /*!< flash bank1 busy flag */
#define FMC_STAT1_PGSERR             BIT(1)                                             /*!< flash bank1 program sequence error flag */
#define FMC_STAT1_PGERR              BIT(2)                                             /*!< flash bank1 program error flag */
#define FMC_STAT1_PGAERR             BIT(3)                                             /*!< flash bank1 program alignment error flag */
#define FMC_STAT1_WPERR              BIT(4)                                             /*!< flash bank1 erase/program protection error flag */
#define FMC_STAT1_ENDF               BIT(5)                                             /*!< flash bank1 end of operation flag */
#define FMC_STAT1_CBCMDERR           BIT(6)                                             /*!< flash bank1 checked area by the check blank command is all 0xFF or not */
#define FMC_STAT1_RSTERR             BIT(15)                                            /*!< flash bank1 BOR/POR or system reset during erase/program flag */

/* FMC_CTL1 */
#define FMC_CTL1_PG                  BIT(0)                                             /*!< flash bank1 program command bit */
#define FMC_CTL1_PER                 BIT(1)                                             /*!< flash bank1 page erase bit */
#define FMC_CTL1_MER                 BIT(2)                                             /*!< flash bank1 mass erase bit */
#define FMC_CTL1_MERDF               BIT(3)                                             /*!< data flash mass erase bit */
#define FMC_CTL1_OB0PG               BIT(4)                                             /*!< option bytes 0 program command bit */
#define FMC_CTL1_OB0ER               BIT(5)                                             /*!< option bytes 0 erase command bit */
#define FMC_CTL1_START               BIT(6)                                             /*!< send erase command to FMC bit */
#define FMC_CTL1_LK                  BIT(7)                                             /*!< flash bank1 lock bit */
#define FMC_CTL1_FSTPG               BIT(8)                                             /*!< fast program command bit */
#define FMC_CTL1_OBWEN               BIT(9)                                             /*!< option bytes erase/program enable bit */
#define FMC_CTL1_ERRIE               BIT(10)                                            /*!< flash bank1 error interrupt enable bit */
#define FMC_CTL1_ENDIE               BIT(12)                                            /*!< flash bank1 end of operation interrupt enable bit */
#define FMC_CTL1_OBRLD               BIT(13)                                            /*!< option bytes reload bit */
#define FMC_CTL1_CBCMD               BIT(16)                                            /*!< send check blank command to flash bank1 bit */
#define FMC_CTL1_SRAMCMD             BITS(24,25)                                        /*!< send shared SRAM command */
#define FMC_CTL1_CBCMDLEN            BITS(29,31)                                        /*!< check blank command read length to flash bank1 */

/* FMC_ADDR1 */
#define FMC_ADDR1_ADDR               BITS(0,31)                                         /*!< flash bank1 command address bits */

/* FMC_EPCNT */
#define FMC_EPCNT_EPCNT              BITS(0,31)                                         /*!< EEPROM erase counter */

/* FMC_OBSTAT */
#define FMC_OBSTAT_OBERR             BIT(0)                                             /*!< option bytes read error bit */
#define FMC_OBSTAT_PLEVEL            BITS(1,2)                                          /*!< protection level bits */
#define FMC_OBSTAT_USER              BITS(8,15)                                         /*!< option bytes user bits */
#define FMC_OBSTAT_DATA              BITS(16,31)                                        /*!< option bytes data bits */

/* FMC_WP0 */
#define FMC_WP0_BK0WP                BITS(0,31)                                         /*!< store OB_BK0WP[31:0] of option bytes 0 block after system reset */

/* FMC_WP1 */
#define FMC_WP1_BK1WP                BITS(0,7)                                          /*!< store OB_BK1WP[7:0] of option bytes 0 block after system reset */
#define FMC_WP1_DFWP                 BITS(8,15)                                         /*!< store OB_DFWP[7:0] of option bytes 0 block after system reset */
#define FMC_WP1_EPWP                 BITS(16,23)                                        /*!< store OB_EPWP[7:0] of option bytes 0 block after system reset */

/* FMC_OB1CS */
#define FMC_OB1CS_OB1ERR             BIT(0)                                             /*!< option bytes 1 read error bit */
#define FMC_OB1CS_OB1START           BIT(1)                                             /*!< send option bytes 1 change command to FMC */
#define FMC_OB1CS_OB1LK              BIT(2)                                             /*!< option bytes 1 lock bit */
#define FMC_OB1CS_EFALC              BITS(4,7)                                          /*!< load EFALC of option bytes 1 after reset */
#define FMC_OB1CS_EPSIZE             BITS(8,11)                                         /*!< load EPSIZE of option bytes 1 after reset */
#define FMC_OB1CS_EPLOAD             BIT(15)                                            /*!< load EPLOAD of option bytes 1 after reset */
#define FMC_OB1CS_LKVAL              BITS(16,31)                                        /*!< load LKVAL of option bytes 1 after reset */

/* FMC_PID */
#define FMC_PID_PID                  BITS(0,31)                                         /*!< product ID bits */

/* constants definitions */
/* fmc state */
typedef enum {
    FMC_READY,                                                                          /*!< the operation has been completed */
    FMC_BUSY,                                                                           /*!< the operation is in progress */
    FMC_PGSERR,                                                                         /*!< program sequence error */
    FMC_PGERR,                                                                          /*!< program error */
    FMC_PGAERR,                                                                         /*!< program alignment error */
    FMC_WPERR,                                                                          /*!< erase/program protection error */
    FMC_TOERR,                                                                          /*!< timeout error */
    FMC_CBCMDERR,                                                                       /*!< the checked area not blank error */
    FMC_RSTERR,                                                                         /*!< BOR/POR or system reset during flash erase/program error */
    FMC_OB_HSPC,                                                                        /*!< FMC is under high security protection */
    FMC_OB1_LK                                                                          /*!< option bytes 1 is locked */
} fmc_state_enum;

/* shared SRAM mode */
typedef enum {
    NO_SRAM_MODE = 0,                                                                   /*!< SRAM mode is not configured */
    FASTPG_SRAM_MODE = 1,                                                               /*!< fast program SRAM mode */
    BASIC_SRAM_MODE = 2,                                                                /*!< basic SRAM mode */
    EEPROM_SRAM_MODE = 3                                                                /*!< EEPROM SRAM mode */
} fmc_sram_mode_enum;

/* FMC area */
typedef enum {
    BANK0_AREA = 0,                                                                     /*!< main flash bank0 area */
    BANK1_AREA = 1,                                                                     /*!< main flash bank1 area */
    DATA_FLASH_AREA = 2,                                                                /*!< data flash area */
    EEPROM_AREA = 3                                                                     /*!< EEPROM area */
} fmc_area_enum;

/* define the FMC bit position and its register index offset */
#define FMC_REGIDX_BIT(regidx, bitpos)  (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define FMC_REG_VAL(offset)             (REG32(FMC + (((uint32_t)(offset) & 0x0000FFFFU) >> 6)))
#define FMC_BIT_POS(val)                ((uint32_t)(val) & 0x0000001FU)
#define FMC_REGIDX_BIT2(regidx, bitpos, regidx2, bitpos2)   (((uint32_t)(regidx2) << 22) | (uint32_t)((bitpos2) << 16)\
                                                            | (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos)))
#define FMC_REG_VAL2(offset)            (REG32(FMC + ((uint32_t)(offset) >> 22)))
#define FMC_BIT_POS2(val)               (((uint32_t)(val) & 0x001F0000U) >> 16)

/* register offset */
#define FMC_STAT0_REG_OFFSET            ((uint32_t)0x0000000CU)                         /*!< STAT0 register offset */
#define FMC_STAT1_REG_OFFSET            ((uint32_t)0x0000004CU)                         /*!< STAT1 register offset */
#define FMC_ECCCS_REG_OFFSET            ((uint32_t)0x00000004U)                         /*!< ECCCS register offset */
#define FMC_OB1CS_REG_OFFSET            ((uint32_t)0x00000068U)                         /*!< OB1CS register offset */
#define FMC_CTL0_REG_OFFSET             ((uint32_t)0x00000010U)                         /*!< CTL0 register offset */
#define FMC_CTL1_REG_OFFSET             ((uint32_t)0x00000050U)                         /*!< CTL1 register offset */
#define FMC_OBSTAT_REG_OFFSET           ((uint32_t)0x0000005CU)                         /*!< OBSTAT register offset */

/* FMC flags */
typedef enum {
    /* flags in STAT0 register */
    FMC_BANK0_FLAG_BUSY = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 0U),                     /*!< flash bank0 busy flag */
    FMC_BANK0_FLAG_PGSERR = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 1U),                   /*!< flash bank0 program sequence error flag */
    FMC_BANK0_FLAG_PGERR = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 2U),                    /*!< flash bank0 program error flag */
    FMC_BANK0_FLAG_PGAERR = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 3U),                   /*!< flash bank0 program alignment error flag */
    FMC_BANK0_FLAG_WPERR = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 4U),                    /*!< flash bank0 erase/program protection error flag */
    FMC_BANK0_FLAG_END = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 5U),                      /*!< flash bank0 end of operation flag */
    FMC_BANK0_FLAG_CBCMDERR = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 6U),                 /*!< flash bank0 checked area by the check blank command is all 0xFF or not flag */
    FMC_BANK0_FLAG_RSTERR = FMC_REGIDX_BIT(FMC_STAT0_REG_OFFSET, 15U),                  /*!< flash bank0 BOR/POR or system reset during erase/program flag */
    /* flags in STAT1 register */
    FMC_BANK1_FLAG_BUSY = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 0U),                     /*!< flash bank1 busy flag */
    FMC_BANK1_FLAG_PGSERR = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 1U),                   /*!< flash bank1 program sequence error flag */
    FMC_BANK1_FLAG_PGERR = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 2U),                    /*!< flash bank1 program error flag */
    FMC_BANK1_FLAG_PGAERR = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 3U),                   /*!< flash bank1 program alignment error flag */
    FMC_BANK1_FLAG_WPERR = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 4U),                    /*!< flash bank1 erase/program protection error flag */
    FMC_BANK1_FLAG_END = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 5U),                      /*!< flash bank1 end of operation flag */
    FMC_BANK1_FLAG_CBCMDERR = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 6U),                 /*!< flash bank1 checked area by the check blank command is all 0xFF or not flag */
    FMC_BANK1_FLAG_RSTERR = FMC_REGIDX_BIT(FMC_STAT1_REG_OFFSET, 15U),                  /*!< flash bank1 BOR/POR or system reset during erase/program flag */
    /* flags in ECCCS register */
    FMC_FLAG_BK0ECC = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 28U),                        /*!< an ECC bit error is detected in bank 0 flag */
    FMC_FLAG_OB0ECC = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 19U),                        /*!< an ECC bit error is detected in option byte 0 flag */
    FMC_FLAG_BK1ECC = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 20U),                        /*!< an ECC bit error is detected in bank 1 flag */
    FMC_FLAG_SYSECC = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 21U),                        /*!< an ECC bit error is detected in system memory flag */
    FMC_FLAG_DFECC = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 22U),                         /*!< an ECC bit error is detected in data flash flag */
    FMC_FLAG_OTPECC = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 23U),                        /*!< an ECC bit error is detected in OTP flag */
    FMC_FLAG_OB1ECCDET = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 26U),                     /*!< option bytes 1 two bit error detect flag */
    FMC_FLAG_OB0ECCDET = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 27U),                     /*!< option bytes 0 two bit error detect flag */
    FMC_FLAG_EPECCDET = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 29U),                      /*!< EEPROM two bits error detect flag */
    FMC_FLAG_ECCCOR = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 30U),                        /*!< one bit error detected and correct flag */
    FMC_FLAG_ECCDET = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 31U),                        /*!< OTP/data flash/system memory/bank1 two bit error detect flag */
    /* flags in OBSTAT register */
    FMC_FLAG_OBERR = FMC_REGIDX_BIT(FMC_OBSTAT_REG_OFFSET, 0U),                         /*!< option bytes 0 error flag */
    /* flags in OB1CS register */
    FMC_FLAG_OB1ERR = FMC_REGIDX_BIT(FMC_OB1CS_REG_OFFSET, 0U)                          /*!< option bytes 1 read error flag */
} fmc_flag_enum;

/* FMC interrupt flags */
typedef enum {
    /* interrupt flags in STAT0 register */
    FMC_BANK0_INT_FLAG_PGSERR = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 1U),        /*!< flash bank0 program sequence error interrupt flag */
    FMC_BANK0_INT_FLAG_PGERR = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 2U),         /*!< flash bank0 program error interrupt flag */
    FMC_BANK0_INT_FLAG_PGAERR = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 3U),        /*!< flash bank0 program alignment error interrupt flag */
    FMC_BANK0_INT_FLAG_WPERR = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 4U),         /*!< flash bank0 erase/program protection error interrupt flag */
    FMC_BANK0_INT_FLAG_END = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 12U, FMC_STAT0_REG_OFFSET, 5U),           /*!< flash bank0 end of operation interrupt flag */
    FMC_BANK0_INT_FLAG_CBCMDERR = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 6U),      /*!< flash bank0 checked area by the check blank command is all 0xFF or not interrupt flag */
    FMC_BANK0_INT_FLAG_RSTERR = FMC_REGIDX_BIT2(FMC_CTL0_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 15U),       /*!< flash bank0 BOR/POR or system reset during erase/program interrupt flag */
    /* interrupt flags in STAT1 register */
    FMC_BANK1_INT_FLAG_PGSERR = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 1U),        /*!< flash bank1 program sequence error interrupt flag */
    FMC_BANK1_INT_FLAG_PGERR = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 2U),         /*!< flash bank1 program error interrupt flag */
    FMC_BANK1_INT_FLAG_PGAERR = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 3U),        /*!< flash bank1 program alignment error interrupt flag */
    FMC_BANK1_INT_FLAG_WPERR = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 4U),         /*!< flash bank1 erase/program protection error interrupt flag */
    FMC_BANK1_INT_FLAG_END = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 12U, FMC_STAT0_REG_OFFSET, 5U),           /*!< flash bank1 end of operation interrupt flag */
    FMC_BANK1_INT_FLAG_CBCMDERR = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 6U),      /*!< flash bank1 checked area by the check blank command is all 0xFF or not interrupt flag */
    FMC_BANK1_INT_FLAG_RSTERR = FMC_REGIDX_BIT2(FMC_CTL1_REG_OFFSET, 10U, FMC_STAT0_REG_OFFSET, 15U),       /*!< flash bank1 BOR/POR or system reset during erase/program interrupt flag */
    /* interrupt flags in ECCCS register */
    FMC_INT_FLAG_OB1ECCDET = FMC_REGIDX_BIT2(FMC_ECCCS_REG_OFFSET, 25U, FMC_ECCCS_REG_OFFSET, 26U),         /*!< option bytes 1 two bits error detect interrupt flag */
    FMC_INT_FLAG_OB0ECCDET = FMC_REGIDX_BIT2(FMC_ECCCS_REG_OFFSET, 25U, FMC_ECCCS_REG_OFFSET, 27U),         /*!< option bytes 0 two bits error detect interrupt flag */
    FMC_INT_FLAG_EPECCDET = FMC_REGIDX_BIT2(FMC_ECCCS_REG_OFFSET, 25U, FMC_ECCCS_REG_OFFSET, 29U),          /*!< EEPROM two bits error detect interrupt flag */
    FMC_INT_FLAG_ECCCOR = FMC_REGIDX_BIT2(FMC_ECCCS_REG_OFFSET, 24U, FMC_ECCCS_REG_OFFSET, 30U),            /*!< one bit error detected and correct interrupt flag */
    FMC_INT_FLAG_ECCDET = FMC_REGIDX_BIT2(FMC_ECCCS_REG_OFFSET, 25U, FMC_ECCCS_REG_OFFSET, 31U)             /*!< two bits error detect interrupt flag */
} fmc_interrupt_flag_enum;

/* FMC interrupt */
typedef enum {
    /* interrupt in CTL0 register */
    FMC_BANK0_INT_ERR = FMC_REGIDX_BIT(FMC_CTL0_REG_OFFSET, 10U),                       /*!< FMC bank0 error interrupt */
    FMC_BANK0_INT_END = FMC_REGIDX_BIT(FMC_CTL0_REG_OFFSET, 12U),                       /*!< FMC bank0 end of operation interrupt */
    /* interrupt in CTL1 register */
    FMC_BANK1_INT_ERR = FMC_REGIDX_BIT(FMC_CTL1_REG_OFFSET, 10U),                       /*!< FMC bank1 error interrupt */
    FMC_BANK1_INT_END = FMC_REGIDX_BIT(FMC_CTL1_REG_OFFSET, 12U),                       /*!< FMC bank1 end of operation interrupt */
    /* interrupt in ECCCS register */
    FMC_INT_ECCCOR = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 24U),                         /*!< FMC one bit error correct interrupt */
    FMC_INT_ECCDET = FMC_REGIDX_BIT(FMC_ECCCS_REG_OFFSET, 25U)                          /*!< FMC two bits error interrupt */
} fmc_interrupt_enum;

/* unlock key */
#define UNLOCK_KEY0                  ((uint32_t)0x45670123U)                            /*!< unlock key 0 */
#define UNLOCK_KEY1                  ((uint32_t)0xCDEF89ABU)                            /*!< unlock key 1 */

/* wait state counter value */
#define WS_WSCNT(regval)             (BITS(0,2) & ((uint32_t)(regval) << 0))
#define WS_WSCNT_0                   WS_WSCNT(0)                                        /*!< 0 wait state added */
#define WS_WSCNT_1                   WS_WSCNT(1)                                        /*!< 1 wait state added */
#define WS_WSCNT_2                   WS_WSCNT(2)                                        /*!< 2 wait state added */
#define WS_WSCNT_3                   WS_WSCNT(3)                                        /*!< 3 wait state added */

/* shared SRAM command */
#define CTL1_SRAMCMD(regval)         (BITS(24,25) & ((uint32_t)(regval) << 24))
#define FASTPG_SRAM_CMD              CTL1_SRAMCMD(1)                                    /*!< set fast PG RAM mode */
#define BASIC_SRAM_CMD               CTL1_SRAMCMD(2)                                    /*!< set basic RAM mode */
#define EEPROM_SRAM_CMD              CTL1_SRAMCMD(3)                                    /*!< set EEPROM RAM mode */

/* option bytes security protection level in FMC_OBSTAT register */
#define OB_OBSTAT_PLEVEL_NO          ((uint8_t)0x00U)                                   /*!< no security protection */
#define OB_OBSTAT_PLEVEL_LOW         ((uint8_t)0x01U)                                   /*!< low security protection */
#define OB_OBSTAT_PLEVEL_HIGH        ((uint8_t)0x03U)                                   /*!< high security protection */

/* option bytes read protection configuration */
#define FMC_NSPC                     ((uint16_t)0x5AA5U)                                /*!< no security protection */
#define FMC_LSPC                     ((uint16_t)0x44BBU)                                /*!< low security protection, any value except 0xA5 or 0xCC */
#define FMC_HSPC                     ((uint16_t)0x33CCU)                                /*!< high security protection */

/* option bytes software/hardware free watchdog timer */
#define OB_FWDGT_HW                  ((uint16_t)0x0100U)                                /*!< hardware free watchdog timer */
#define OB_FWDGT_SW                  ((uint16_t)0x0001U)                                /*!< software free watchdog timer */

/* option bytes reset or not entering deep sleep mode */
#define OB_DEEPSLEEP_RST             ((uint16_t)0x0200U)                                /*!< generate a reset instead of entering deepsleep mode */
#define OB_DEEPSLEEP_NRST            ((uint16_t)0x0002U)                                /*!< no reset when entering deepsleep mode */

/* option bytes reset or not entering standby mode */
#define OB_STDBY_RST                 ((uint16_t)0x0400U)                                /*!< generate a reset instead of entering standby mode */
#define OB_STDBY_NRST                ((uint16_t)0x0004U)                                /*!< no reset when entering deepsleep mode */

/* option bytes boot from bank0 or bank1 when configured boot from main flash */
#define OB_BOOT_FROM_BANK1           ((uint16_t)0x0800U)                                /*!< boot from bank1 or bank0 if bank1 is void, when configured boot from main memory */
#define OB_BOOT_FROM_BANK0           ((uint16_t)0x0008U)                                /*!< boot from bank0, when configured boot from main memory */

/* option bytes OTA configuration */
#define OB_BOOT_OTA_ENABLE           ((uint16_t)0x1000U)                                /*!< when configured boot from main memory, if the BB is 0, all data will be copied from bank1 to bank0 and then boot from bank0 */
#define OB_BOOT_OTA_DISABLE          ((uint16_t)0x0010U)                                /*!< no effect */

/* option bytes brownout configuration */
#define OB_BOR_DISABLE               ((uint16_t)0x0080U)                                /*!< disable brown out */
#define OB_BOR_ENABLE                ((uint16_t)0x8000U)                                /*!< enable brown out, brownout threshold 2.6V */

/* option bytes 1 lock value in FMC_OB1CS register */
#define OB1CS_OB1_LKVAL(regval)      (BITS(16,31) & ((uint32_t)(regval) << 16))
#define OB1CS_OB1_LK                 OB1CS_OB1_LKVAL((uint16_t)0x33CCU)                 /*!< option byte1 cannot be modified */
#define OB1CS_OB1_NOT_LK             OB1CS_OB1_LKVAL((uint16_t)0x00FFU)                 /*!< option byte1 is not locked */

/* option bytes 1 shared RAM init mode in FMC_OB1CS register */
#define OB1CS_EPLOAD(regval)         (BIT(15) & ((uint32_t)(regval) << 15))
#define OB1CS_EPLOAD_NOT_LOAD_EPDATA OB1CS_EPLOAD((uint16_t)0x00U)                      /*!< shared SRAM is not loaded with valid EEPROM data during FMC reset */
#define OB1CS_EPLOAD_LOAD_EPDATA     OB1CS_EPLOAD((uint16_t)0x01U)                      /*!< shared SRAM is loaded with valid EEPROM data during FMC reset */

/* option bytes 1 EEPROM size in FMC_OB1CS register */
#define OB1CS_EPSIZE(regval)         (BITS(8, 11) & ((uint32_t)(regval) << 8))
#define OB1CS_EPSIZE_NONE            OB1CS_EPSIZE((uint8_t)0x0FU)                      /*!< no EEPROM */
#define OB1CS_EPSIZE_4K              OB1CS_EPSIZE((uint8_t)0x08U)                      /*!< 4KB EEPROM, 384K flash or 256K flash */
#define OB1CS_EPSIZE_2K              OB1CS_EPSIZE((uint8_t)0x04U)                      /*!< 2KB EEPROM, 128K flash */
#define OB1CS_EPSIZE_1K              OB1CS_EPSIZE((uint8_t)0x02U)                      /*!< 1KB EEPROM, 64K flash */

/* option bytes 1 extend flash block allocation in FMC_OB1CS register */
/* 384K flash or 256K flash */
#define OB1CS_EFALC(regval)          (BITS(4, 7) & ((uint32_t)(regval) << 4))
#define OB1CS_DF_64K_EF_0K           OB1CS_EFALC((uint8_t)0x00U)                        /*!< data flash size is 64KB, EEPROM backup size is 0KB */
#define OB1CS_DF_48K_EF_16K          OB1CS_EFALC((uint8_t)0x03U)                        /*!< data flash size is 48KB, EEPROM backup size is 16KB */
#define OB1CS_DF_32K_EF_32K          OB1CS_EFALC((uint8_t)0x04U)                        /*!< data flash size is 32KB, EEPROM backup size is 32KB */
#define OB1CS_DF_16K_EF_48K          OB1CS_EFALC((uint8_t)0x05U)                        /*!< data flash size is 16KB, EEPROM backup size is 48KB */
#define OB1CS_DF_0K_EF_64K           OB1CS_EFALC((uint8_t)0x08U)                        /*!< data flash size is 0KB, EEPROM backup size is 64KB */
#define OB1CS_DF_EF_INVALID          OB1CS_EFALC((uint8_t)0x0FU)                        /*!< data flash and EEPROM backup are invalid */
/* 128K flash */
#define OB1CS_DF_32K_EF_0K           OB1CS_EFALC((uint8_t)0x01U)                        /*!< data flash size is 64KB, EEPROM backup size is 0KB */
#define OB1CS_DF_8K_EF_24K           OB1CS_EFALC((uint8_t)0x06U)                        /*!< data flash size is 48KB, EEPROM backup size is 16KB */
#define OB1CS_DF_0K_EF_32K           OB1CS_EFALC((uint8_t)0x09U)                        /*!< data flash size is 32KB, EEPROM backup size is 32KB */
#define OB1CS_DF_16K_EF_16K          OB1CS_EFALC((uint8_t)0x0BU)                        /*!< data flash size is 16KB, EEPROM backup size is 48KB */
#define OB1CS_DF_24K_EF_8K           OB1CS_EFALC((uint8_t)0x0CU)                        /*!< data flash size is 0KB, EEPROM backup size is 64KB */
/* 64K flash */
#define OB1CS_DF_16K_EF_0K           OB1CS_EFALC((uint8_t)0x02U)                        /*!< data flash size is 64KB, EEPROM backup size is 0KB */
#define OB1CS_DF_4K_EF_12K           OB1CS_EFALC((uint8_t)0x07U)                        /*!< data flash size is 48KB, EEPROM backup size is 16KB */
#define OB1CS_DF_0K_EF_16K           OB1CS_EFALC((uint8_t)0x0AU)                        /*!< data flash size is 32KB, EEPROM backup size is 32KB */
#define OB1CS_DF_8K_EF_8K            OB1CS_EFALC((uint8_t)0x0DU)                        /*!< data flash size is 16KB, EEPROM backup size is 48KB */
#define OB1CS_DF_12K_EF_4K           OB1CS_EFALC((uint8_t)0x0EU)                        /*!< data flash size is 0KB, EEPROM backup size is 64KB */

#define BANK0_BASE_ADDRESS           ((uint32_t)0x08000000U)                            /*!< FMC bank0 base address */
#define BANK0_SIZE                   ((uint32_t)0x00040000U)                            /*!< FMC bank0 size */
#define BANK1_BASE_ADDRESS           ((uint32_t)(BANK0_BASE_ADDRESS + BANK0_SIZE))      /*!< FMC bank1 base address */
#define OB_WORD_CNT                  ((uint8_t)0x06U)                                   /*!< word count of option bytes */
#define OB_DOUBLEWORD_CNT            ((uint8_t)0x03U)                                   /*!< double-word count of option bytes */
#define FMC_TIMEOUT_COUNT            ((uint32_t)0x00FF0000U)                            /*!< count to judge of FMC timeout */
#define DOUBLEWORD_CNT_IN_ROW        ((uint8_t)0x20U)                                   /*!< double-word count in one row data */
#define CBCMDLEN_OF_ONE_ROW          ((uint8_t)0x05U)                                   /*!< CBCMD read length of one row data */

/* function declarations */
/* FMC programming functions */
/* unlock the main flash operation */
void fmc_unlock(void);
/* unlock the main flash bank0 operation */
void fmc_bank0_unlock(void);
/* unlock the main flash bank1 operation */
void fmc_bank1_unlock(void);
/* lock the main flash operation */
void fmc_lock(void);
/* lock the main flash bank0 operation */
void fmc_bank0_lock(void);
/* lock the main flash bank1 operation */
void fmc_bank1_lock(void);

/* set the wait state counter value */
void fmc_wscnt_set(uint8_t wscnt);
/* enable pre-fetch */
void fmc_prefetch_enable(void);
/* disable pre-fetch */
void fmc_prefetch_disable(void);
/* enable cache */
void fmc_cache_enable(void);
/* disable cache */
void fmc_cache_disable(void);
/* enable cache reset if cache is disabled */
void fmc_cache_reset_enable(void);
/* disable cache reset */
void fmc_cache_reset_disable(void);
/* flash goto power-down mode when MCU enters deepsleep mode */
void fmc_powerdown_mode_set(void);
/* flash goto sleep mode when MCU enters deepsleep mode */
void fmc_sleep_mode_set(void);
/* configure shared SRAM mode */
void fmc_sram_mode_config(fmc_sram_mode_enum sram_mode);
/* get shared SRAM mode */
fmc_sram_mode_enum fmc_sram_mode_get(void);

/* check whether flash page is blank or not by check blank command */
fmc_state_enum fmc_blank_check(uint32_t address, uint8_t length);
/* erase main flash page */
fmc_state_enum fmc_page_erase(uint32_t page_address);
/* erase flash bank0 */
fmc_state_enum fmc_bank0_mass_erase(void);
/* erase flash bank1 */
fmc_state_enum fmc_bank1_mass_erase(void);
/* erase the data flash */
fmc_state_enum fmc_dflash_mass_erase(void);
/* erase whole chip */
fmc_state_enum fmc_mass_erase(void);

/* program a double word at the corresponding address in main flash */
fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data);
/* FMC fast program one row data (32 double-word) starting at the corresponding address */
fmc_state_enum fmc_fast_program(uint32_t address, uint64_t data[]);
/* program a double word at the corresponding address in OTP */
fmc_state_enum otp_doubleword_program(uint32_t address, uint64_t data);
/* program a word at the corresponding address in EEPROM */
fmc_state_enum eeprom_word_program(uint32_t address, uint32_t data);
/* read a word at the corresponding address in EEPROM */
uint32_t eeprom_word_read(uint32_t address);

/* FMC option bytes 0 functions */
/* unlock the option bytes 0 operation */
void ob_unlock(void);
/* lock the option bytes 0 operation */
void ob_lock(void);
/* force to reload the option bytes 0 */
void ob_reset(void);
/* erase the option bytes 0 */
fmc_state_enum ob_erase(void);
/* enable option bytes 0 write protection */
fmc_state_enum ob_write_protection_enable(fmc_area_enum wp_area, uint32_t ob_wp);
/* configure security protection */
fmc_state_enum ob_security_protection_config(uint16_t ob_spc);
/* program the FMC user option bytes 0 */
fmc_state_enum ob_user_write(uint16_t ob_user);
/* program the FMC data option bytes 0 */
fmc_state_enum ob_data_program(uint16_t ob_data);
/* get the value of FMC option bytes OB_USER in FMC_OBSTAT register */
uint8_t ob_user_get(void);
/* get the value of FMC option bytes OB_DATA in FMC_OBSTAT register */
uint16_t ob_data_get(void);
/* get the value of FMC option bytes BK0WP in FMC_WP0 register */
uint32_t ob_write_protection_get(void);
/* get the value of FMC option bytes BK1WP in FMC_WP1 register */
uint8_t ob_bk1_write_protection_get(void);
/* get the value of FMC option bytes DFWP in FMC_WP1 register */
uint8_t ob_df_write_protection_get(void);
/* get the value of FMC option bytes EPWP in FMC_WP1 register */
uint8_t ob_ep_write_protection_get(void);
/* get the value of FMC option bytes 0 security protection level (PLEVEL) in FMC_OBSTAT register */
uint8_t ob_plevel_get(void);

/* FMC option bytes 1 functions */
/* configure lock value in option bytes 1 */
fmc_state_enum ob1_lock_config(uint32_t lk_value);
/* configure the EPLOAD value of option bytes 1 loaded after the system reset */
fmc_state_enum ob1_epload_config(uint32_t epload);
/* configure option bytes 1 EEPROM parameters */
fmc_state_enum ob1_eeprom_parameter_config(uint32_t efalc, uint32_t epsize);
/* get data flash size in byte unit */
uint32_t dflash_size_get(void);
/* get EEPROM backup size in byte unit */
uint32_t eeprom_backup_size_get(void);
/* get EEPROM size in byte unit */
uint32_t eeprom_size_get(void);

/* interrupt & flag functions */
/* get FMC flag status */
FlagStatus fmc_flag_get(fmc_flag_enum flag);
/* clear FMC flag status */
void fmc_flag_clear(fmc_flag_enum flag);
/* enable FMC interrupt */
void fmc_interrupt_enable(fmc_interrupt_enum interrupt);
/* disable FMC interrupt */
void fmc_interrupt_disable(fmc_interrupt_enum interrupt);
/* get FMC interrupt flag status */
FlagStatus fmc_interrupt_flag_get(fmc_interrupt_flag_enum int_flag);
/* clear FMC interrupt flag status */
void fmc_interrupt_flag_clear(fmc_interrupt_flag_enum int_flag);

#endif /* GD32A50X_FMC_H */
