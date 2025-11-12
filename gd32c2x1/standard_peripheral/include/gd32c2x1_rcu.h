/*!
    \file    gd32c2x1_rcu.h
    \brief   definitions for the RCU

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

#ifndef GD32C2X1_RCU_H
#define GD32C2X1_RCU_H

#include "gd32c2x1.h"

/* RCU definitions */
#define RCU                         RCU_BASE

/* registers definitions */
#define RCU_CTL0                    REG32(RCU + 0x00000000U)        /*!< control 0 register */
#define RCU_CFG0                    REG32(RCU + 0x00000008U)        /*!< configuration register 0 */
#define RCU_INT                     REG32(RCU + 0x0000000CU)        /*!< interrupt register */
#define RCU_AHB1RST                 REG32(RCU + 0x00000010U)        /*!< AHB1 reset register */
#define RCU_AHB2RST                 REG32(RCU + 0x00000014U)        /*!< AHB1 reset register */
#define RCU_APBRST                  REG32(RCU + 0x00000024U)        /*!< APB reset register */
#define RCU_AHB1EN                  REG32(RCU + 0x00000030U)        /*!< AHB1 enable register */
#define RCU_AHB2EN                  REG32(RCU + 0x00000034U)        /*!< AHB2 enable register */
#define RCU_APBEN                   REG32(RCU + 0x00000044U)        /*!< APB enable register */
#define RCU_AHB1SPDPEN              REG32(RCU + 0x00000050U)        /*!< AHB1 sleep and deep sleep mode enable register */
#define RCU_AHB2SPDPEN              REG32(RCU + 0x00000054U)        /*!< AHB2 sleep and deep sleep mode enable register */
#define RCU_APBSPDPEN               REG32(RCU + 0x00000064U)        /*!< APB sleep and deep sleep mode enable register */
#define RCU_CTL1                    REG32(RCU + 0x00000070U)        /*!< control 1 register */
#define RCU_RSTSCK                  REG32(RCU + 0x00000074U)        /*!< reset source /clock register */
#define RCU_CFG1                    REG32(RCU + 0x0000008CU)        /*!< configuration register 1 */

/* bits definitions */
/* RCU_CTL0 */
#define RCU_CTL0_IRC48MEN            BIT(0)                          /*!< internal high speed oscillator enable */
#define RCU_CTL0_IRC48MSTB           BIT(1)                          /*!< IRC48M high speed internal oscillator stabilization flag */
#define RCU_CTL0_IRC48MADJ           BITS(3,7)                       /*!< high speed internal oscillator clock trim adjust value */
#define RCU_CTL0_IRC48MCALIB         BITS(8,15)                      /*!< high speed internal oscillator calibration value register */
#define RCU_CTL0_HXTALEN             BIT(16)                         /*!< external high speed oscillator enable */
#define RCU_CTL0_HXTALSTB            BIT(17)                         /*!< external crystal oscillator clock stabilization flag */
#define RCU_CTL0_HXTALBPS            BIT(18)                         /*!< external crystal oscillator clock bypass mode enable */
#define RCU_CTL0_CKMEN               BIT(19)                         /*!< HXTAL clock monitor enable */
#define RCU_CTL0_IRC48MDIV_PER       BITS(25,27)                     /*!< IRC48M clock divider factor for peripheral clock */
#define RCU_CTL0_IRC48M_PEREN        BIT(28)                         /*!< IRC48M always-enable for peripheral */
#define RCU_CTL0_IRC48MDIV_SYS       BITS(29,31)                     /*!< IRC48M clock divider factor for system clock */

/* RCU_CFG0 */
#define RCU_CFG0_SCS                BITS(0,1)                       /*!< system clock switch */
#define RCU_CFG0_SCSS               BITS(2,3)                       /*!< system clock switch status */
#define RCU_CFG0_AHBPSC             BITS(4,7)                       /*!< AHB prescaler selection */
#define RCU_CFG0_APBPSC             BITS(11,13)                     /*!< APB prescaler selection */
#define RCU_CFG0_CKOUT1SEL          BITS(16,18)                     /*!< CK_OUT1 clock source selection */
#define RCU_CFG0_CKOUT1DIV          BITS(20,22)                     /*!< CK_OUT1 divider which the CK_OUT frequency can be reduced */
#define RCU_CFG0_CKOUT0SEL          BITS(24,26)                     /*!< CK_OUT0 clock source selection */
#define RCU_CFG0_CKOUT0DIV          BITS(28,30)                     /*!< CK_OUT0 divider which the CK_OUT frequency can be reduced */

/* RCU_INT */
#define RCU_INT_IRC32KSTBIF         BIT(0)                          /*!< IRC32K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF          BIT(1)                          /*!< LXTAL stabilization interrupt flag */
#define RCU_INT_IRC48MSTBIF         BIT(2)                          /*!< IRC48MM stabilization interrupt flag */
#define RCU_INT_LCKMIF              BIT(6)                          /*!< LXTAL clock stuck interrupt flag */
#define RCU_INT_IRC32KSTBIE         BIT(8)                          /*!< IRC32K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE          BIT(9)                          /*!< LXTAL stabilization interrupt enable */
#define RCU_INT_IRC48MSTBIE         BIT(10)                         /*!< IRC48M stabilization interrupt enable */
#define RCU_INT_IRC32KSTBIC         BIT(16)                         /*!< IRC32K stabilization interrupt clear */
#define RCU_INT_LXTALSTBIC          BIT(17)                         /*!< LXTAL stabilization interrupt clear */
#define RCU_INT_IRC48MSTBIC         BIT(18)                         /*!< IRC48M stabilization interrupt clear */
#define RCU_INT_LCKMIC              BIT(22)                         /*!< LXTAL clock stuck interrupt clear */
#define RCU_INT_HXTALSTBIF          BIT(3)                          /*!< HXTAL stabilization interrupt flag */
#define RCU_INT_CKMIF               BIT(7)                          /*!< HXTAL clock stuck interrupt flag */
#define RCU_INT_HXTALSTBIE          BIT(11)                         /*!< HXTAL stabilization interrupt enable */
#define RCU_INT_HXTALSTBIC          BIT(19)                         /*!< HXTAL stabilization interrupt clear */
#define RCU_INT_CKMIC               BIT(23)                         /*!< HXTAL clock stuck interrupt clear */

/* RCU_AHB1RST */
#define RCU_AHB1RST_CRCRST          BIT(12)                         /*!< CRC */
#define RCU_AHB1RST_DMARST          BIT(21)                         /*!< DMA reset */
#define RCU_AHB1RST_DMAMUXRST       BIT(23)                         /*!< DMAMUX reset */

/* RCU_AHB2RST */
#define RCU_AHB2RST_PARST           BIT(17)                         /*!< PA reset */
#define RCU_AHB2RST_PBRST           BIT(18)                         /*!< PB reset */
#define RCU_AHB2RST_PCRST           BIT(19)                         /*!< PC reset */
#define RCU_AHB2RST_PDRST           BIT(20)                         /*!< PD reset */
#define RCU_AHB2RST_PFRST           BIT(22)                         /*!< PF reset */

/* RCU_APBRST */
#define RCU_APBRST_SYSCFGRST        BIT(0)                          /*!< system configuration reset */
#define RCU_APBRST_CMPRST           BIT(1)                          /*!< comparator reset */
#define RCU_APBRST_WWDGTRST         BIT(8)                          /*!< comparator reset */
#define RCU_APBRST_ADCRST           BIT(9)                          /*!< ADC reset */
#define RCU_APBRST_TIMER0RST        BIT(10)                         /*!< TIMER0 reset */
#define RCU_APBRST_TIMER2RST        BIT(11)                         /*!< TIMER2 reset */
#define RCU_APBRST_SPI0RST          BIT(12)                         /*!< SPI0 reset */
#define RCU_APBRST_SPI1RST          BIT(13)                         /*!< SPI1 reset */
#define RCU_APBRST_USART0RST        BIT(14)                         /*!< USART0 reset */
#define RCU_APBRST_USART1RST        BIT(15)                         /*!< USART1 reset */
#define RCU_APBRST_TIMER13RST       BIT(16)                         /*!< TIMER13 reset */
#define RCU_APBRST_TIMER15RST       BIT(17)                         /*!< TIMER15 reset */
#define RCU_APBRST_TIMER16RST       BIT(18)                         /*!< TIMER16 reset */
#define RCU_APBRST_USART2RST        BIT(19)                         /*!< USART2 reset */
#define RCU_APBRST_I2C0RST          BIT(21)                         /*!< I2C0 reset */
#define RCU_APBRST_I2C1RST          BIT(22)                         /*!< I2C1 reset */
#define RCU_APBRST_PMURST           BIT(28)                         /*!< PMU reset */

/* RCU_AHB1EN */
#define RCU_AHB1EN_FMCEN            BIT(4)                          /*!< FMC enable */
#define RCU_AHB1EN_CRCEN            BIT(12)                         /*!< CRC enable*/
#define RCU_AHB1EN_DMAEN            BIT(21)                         /*!< DMA enable */
#define RCU_AHB1EN_DMAMUXEN         BIT(23)                         /*!< DMAMUX enable */

/* RCU_AHB2EN */
#define RCU_AHB2EN_PAEN             BIT(17)                         /*!< PA enable */
#define RCU_AHB2EN_PBEN             BIT(18)                         /*!< PB enable */
#define RCU_AHB2EN_PCEN             BIT(19)                         /*!< PC enable */
#define RCU_AHB2EN_PDEN             BIT(20)                         /*!< PD enable */
#define RCU_AHB2EN_PFEN             BIT(22)                         /*!< PF enable */

/* RCU_APBEN */
#define RCU_APBEN_SYSCFGEN          BIT(0)                          /*!< system configuration enable */
#define RCU_APBEN_CMPEN             BIT(1)                          /*!< comparator enable */
#define RCU_APBEN_WWDGTEN           BIT(8)                          /*!< comparator enable */
#define RCU_APBEN_ADCEN             BIT(9)                          /*!< ADC enable */
#define RCU_APBEN_TIMER0EN          BIT(10)                         /*!< TIMER0 enable */
#define RCU_APBEN_TIMER2EN          BIT(11)                         /*!< TIMER2 enable */
#define RCU_APBEN_SPI0EN            BIT(12)                         /*!< SPI0 enable */
#define RCU_APBEN_SPI1EN            BIT(13)                         /*!< SPI0 enable */
#define RCU_APBEN_USART0EN          BIT(14)                         /*!< USART0 enable */
#define RCU_APBEN_USART1EN          BIT(15)                         /*!< USART1 enable */
#define RCU_APBEN_TIMER13EN         BIT(16)                         /*!< TIMER13 enable */
#define RCU_APBEN_TIMER15EN         BIT(17)                         /*!< TIMER15 enable */
#define RCU_APBEN_TIMER16EN         BIT(18)                         /*!< TIMER16 enable */
#define RCU_APBEN_USART2EN          BIT(19)                         /*!< USART2 enable */
#define RCU_APBEN_I2C0EN            BIT(21)                         /*!< I2C0 enable */
#define RCU_APBEN_I2C1EN            BIT(22)                         /*!< I2C1 enable */
#define RCU_APBEN_DBGEN             BIT(27)                         /*!< DBG enable */
#define RCU_APBEN_PMUEN             BIT(28)                         /*!< PMU enable */

/* RCU_AHB1SPDPEN */
#define RCU_AHB1SPDPEN_SRAMEN       BIT(2)                          /*!< SRAM enable */
#define RCU_AHB1SPDPEN_FMCEN        BIT(4)                          /*!< FMC enable */
#define RCU_AHB1SPDPEN_CRCEN        BIT(12)                         /*!< CRC enable*/
#define RCU_AHB1SPDPEN_DMAEN        BIT(21)                         /*!< DMA enable */
#define RCU_AHB1SPDPEN_DMAMUXEN     BIT(23)                         /*!< DMAMUX enable */

/* RCU_AHB2SPDPEN */
#define RCU_AHB2SPDPEN_PAEN         BIT(17)                         /*!< PA enable */
#define RCU_AHB2SPDPEN_PBEN         BIT(18)                         /*!< PB enable */
#define RCU_AHB2SPDPEN_PCEN         BIT(19)                         /*!< PC enable */
#define RCU_AHB2SPDPEN_PDEN         BIT(20)                         /*!< PD enable */
#define RCU_AHB2SPDPEN_PFEN         BIT(22)                         /*!< PF enable */

/* RCU_APBSPDPEN */
#define RCU_APBSPDPEN_SYSCFGEN       BIT(0)                         /*!< system configuration enable */
#define RCU_APBSPDPEN_CMPEN          BIT(1)                         /*!< comparator enable */
#define RCU_APBSPDPEN_WWDGTEN        BIT(8)                         /*!< comparator enable */
#define RCU_APBSPDPEN_ADCEN          BIT(9)                         /*!< ADC enable */
#define RCU_APBSPDPEN_TIMER0EN       BIT(10)                        /*!< TIMER0 enable */
#define RCU_APBSPDPEN_TIMER2EN       BIT(11)                        /*!< TIMER2 enable */
#define RCU_APBSPDPEN_SPI0EN         BIT(12)                        /*!< SPI0 enable */
#define RCU_APBSPDPEN_SPI1EN         BIT(13)                        /*!< SPI1 enable */
#define RCU_APBSPDPEN_USART0EN       BIT(14)                        /*!< USART0 enable */
#define RCU_APBSPDPEN_USART1EN       BIT(15)                        /*!< USART1 enable */
#define RCU_APBSPDPEN_TIMER13EN      BIT(16)                        /*!< TIMER13 enable */
#define RCU_APBSPDPEN_TIMER15EN      BIT(17)                        /*!< TIMER15 enable */
#define RCU_APBSPDPEN_TIMER16EN      BIT(18)                        /*!< TIMER16 enable */
#define RCU_APBSPDPEN_USART2EN       BIT(19)                        /*!< USART2 enable */
#define RCU_APBSPDPEN_I2C0EN         BIT(21)                        /*!< I2C0 enable */
#define RCU_APBSPDPEN_I2C1EN         BIT(22)                        /*!< I2C1 enable */
#define RCU_APBSPDPEN_PMUEN          BIT(28)                        /*!< PMU enable */

/* RCU_CTL1 */
#define RCU_CTL1_LXTALEN             BIT(0)                         /*!< LXTAL enable */
#define RCU_CTL1_LXTALSTB            BIT(1)                         /*!< external low-speed oscillator stabilization */
#define RCU_CTL1_LXTALBPS            BIT(2)                         /*!< LXTAL bypass mode enable */
#define RCU_CTL1_LXTALDRI            BIT(3)                         /*!< LXTAL drive capability */
#define RCU_CTL1_LCKMEN              BIT(5)                         /*!< LXTAL clock monitor enable */
#define RCU_CTL1_LCKMD               BIT(6)                         /*!< LXTAL clock failure detection */
#define RCU_CTL1_LXTALSTBRST         BIT(7)                         /*!< external low-speed oscillator stabilization reset*/
#define RCU_CTL1_RTCSRC              BITS(8,9)                      /*!< RTC clock entry selection */
#define RCU_CTL1_RTCEN               BIT(15)                        /*!< RTC clock enable */
#define RCU_CTL1_BKPRST              BIT(16)                        /*!< backup domain reset */
#define RCU_CTL1_LSCKOUTEN           BIT(24)                        /*!< Low speed clock output enable */
#define RCU_CTL1_LSCKOUTSEL          BIT(25)                        /*!< Low speed clock output selection */

/* RCU_RSTSCK */
#define RCU_RSTSCK_IRC32KEN         BIT(0)                          /*!< IRC32K enable */
#define RCU_RSTSCK_IRC32KSTB        BIT(1)                          /*!< IRC32K stabilization */
#define RCU_RSTSCK_OBLRSTF          BIT(23)                         /*!< option byte loader reset flag */
#define RCU_RSTSCK_RSTFC            BIT(24)                         /*!< reset flag clear */
#define RCU_RSTSCK_EPRSTF           BIT(26)                         /*!< external pin reset flag */
#define RCU_RSTSCK_PORRSTF          BIT(27)                         /*!< power reset flag */
#define RCU_RSTSCK_SWRSTF           BIT(28)                         /*!< software reset flag */
#define RCU_RSTSCK_FWDGTRSTF        BIT(29)                         /*!< free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF        BIT(30)                         /*!< window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF           BIT(31)                         /*!< low-power reset flag */

/* RCU_CFG1 */
#define RCU_CFG1_USART0SEL          BITS(0,1)                        /*!< CK_USART0 clock source selection */
#define RCU_CFG1_I2C0SEL            BITS(2,3)                        /*!< CK_I2C0 clock source selection */
#define RCU_CFG1_I2C1SEL            BITS(4,5)                        /*!< CK_I2C1 clock source selection */
#define RCU_CFG1_ADCSEL             BITS(7,8)                        /*!< CK_ADC clock source selection */
#define RCU_CFG1_ADCPSC             BITS(9,12)                       /*!< ADC clock prescaler selection */
#define RCU_CFG1_I2SSEL             BITS(14,15)                      /*!< I2S clock source selection */

/* constants definitions */
/* define the peripheral clock enable bit position and its register index offset */
#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx)<<6) | (uint32_t)(bitpos))
#define RCU_REG_VAL(periph)                 (REG32(RCU + ((uint32_t)(periph)>>6)))
#define RCU_BIT_POS(val)                    ((uint32_t)(val) & 0x1FU)

/* register index */
/* peripherals enable */
#define AHB1EN_REG_OFFSET               0x30U                                    /*!< AHB1 enable register offset */
#define AHB2EN_REG_OFFSET               0x34U                                    /*!< AHB2 enable register offset */
#define APBEN_REG_OFFSET                0x44U                                    /*!< APB enable register offset */

/* peripherals reset */
#define AHB1RST_REG_OFFSET              0x10U                                    /*!< AHB1 reset register offset */
#define AHB2RST_REG_OFFSET              0x14U                                    /*!< AHB2 reset register offset */
#define APBRST_REG_OFFSET               0x24U                                    /*!< APB reset register offset */

/* peripherals sleep mode and deepsleep mode enable */
#define AHB1SPDPEN_REG_OFFSET           0x50U                                    /*!< AHB1 reset register offset */
#define AHB2SPDPEN_REG_OFFSET           0x54U                                    /*!< AHB2 reset register offset */
#define APBSPDPEN_REG_OFFSET            0x64U                                    /*!< APB reset register offset */

/* reset source and clock */
#define RSTSCK_REG_OFFSET               0x74U                                    /*!< reset source/clock register offset */

/* clock control */
#define CTL0_REG_OFFSET                 0x00U                                    /*!< control register 0 offset */  
#define CTL1_REG_OFFSET                 0x70U                                    /*!< control register 1 offset */

/* clock stabilization and stuck interrupt */
#define INT_REG_OFFSET                  0x0CU                                    /*!< clock interrupt register offset */

/* configuration register */
#define CFG0_REG_OFFSET                 0x08U                                    /*!< clock configuration register 0 offset */
#define CFG1_REG_OFFSET                 0x8CU                                    /*!< clock configuration register 1 offset */

/* peripheral clock enable */
typedef enum {
    /* AHB1 peripherals */
    RCU_FMC     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 4U),                         /*!< FMC clock */
    RCU_CRC     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 12U),                        /*!< CRC clock */
    RCU_DMA     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 21U),                        /*!< DMA clock */
    RCU_DMAMUX  = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 23U),                        /*!< DMAMUX clock */
    
    /* AHB2 peripherals */
    RCU_GPIOA   = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 17U),                        /*!< GPIOA clock */
    RCU_GPIOB   = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 18U),                        /*!< GPIOB clock */
    RCU_GPIOC   = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 19U),                        /*!< GPIOC clock */
    RCU_GPIOD   = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 20U),                        /*!< GPIOD clock */
    RCU_GPIOF   = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 22U),                        /*!< GPIOF clock */

    /* APB peripherals */
    RCU_SYSCFG  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 0U),                          /*!< SYSCFG clock */
    RCU_CMP     = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 1U),                          /*!< CMP clock */
    RCU_WWDGT   = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 8U),                          /*!< WWDGT clock */
    RCU_ADC     = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 9U),                          /*!< ADC clock */
    RCU_TIMER0  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 10U),                         /*!< TIMER0 clock */
    RCU_TIMER2  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 11U),                         /*!< TIMER2 clock */
    RCU_SPI0    = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 12U),                         /*!< SPI0 clock */
    RCU_SPI1    = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 13U),                         /*!< SPI1 clock */
    RCU_USART0  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 14U),                         /*!< USART0 clock */
    RCU_USART1  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 15U),                         /*!< USART1 clock */
    RCU_TIMER13 = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 16U),                         /*!< TIMER13 clock */
    RCU_TIMER15 = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 17U),                         /*!< TIMER15 clock */
    RCU_TIMER16 = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 18U),                         /*!< TIMER16 clock */
    RCU_USART2  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 19U),                         /*!< USART2 clock */
    RCU_I2C0    = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 21U),                         /*!< I2C0 clock */
    RCU_I2C1    = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 22U),                         /*!< I2C1 clock */
    RCU_DBGMCU  = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 27U),                         /*!< DBGMCU clock */
    RCU_PMU     = RCU_REGIDX_BIT(APBEN_REG_OFFSET, 28U),                         /*!< PMU clock */

    /* control register 1(RCU_CTL1) */
    RCU_RTC     = RCU_REGIDX_BIT(CTL1_REG_OFFSET, 15U)                           /*!< RTC clock */
} rcu_periph_enum;

/* peripheral clock enable when sleep mode*/
typedef enum {
    /* AHB1 peripherals */
    RCU_SRAM_SLP    = RCU_REGIDX_BIT(AHB1SPDPEN_REG_OFFSET, 2U),                 /*!< SRAM clock */
    RCU_FMC_SLP     = RCU_REGIDX_BIT(AHB1SPDPEN_REG_OFFSET, 4U),                 /*!< FMC clock */
    RCU_CRC_SLP     = RCU_REGIDX_BIT(AHB1SPDPEN_REG_OFFSET, 12U),                /*!< CRC clock */
    RCU_DMA_SLP     = RCU_REGIDX_BIT(AHB1SPDPEN_REG_OFFSET, 21U),                /*!< DMA clock */
    RCU_DMAMUX_SLP  = RCU_REGIDX_BIT(AHB1SPDPEN_REG_OFFSET, 23U),                /*!< DMAMUX clock */
    
    /* AHB2 peripherals */
    RCU_GPIOA_SLP   = RCU_REGIDX_BIT(AHB2SPDPEN_REG_OFFSET, 17U),                /*!< GPIOA clock */
    RCU_GPIOB_SLP   = RCU_REGIDX_BIT(AHB2SPDPEN_REG_OFFSET, 18U),                /*!< GPIOB clock */
    RCU_GPIOC_SLP   = RCU_REGIDX_BIT(AHB2SPDPEN_REG_OFFSET, 19U),                /*!< GPIOC clock */
    RCU_GPIOD_SLP   = RCU_REGIDX_BIT(AHB2SPDPEN_REG_OFFSET, 20U),                /*!< GPIOD clock */
    RCU_GPIOF_SLP   = RCU_REGIDX_BIT(AHB2SPDPEN_REG_OFFSET, 22U),                /*!< GPIOF clock */

    /* APB peripherals */
    RCU_SYSCFG_SLP  = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 0U),                  /*!< SYSCFG clock */
    RCU_CMP_SLP     = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 1U),                  /*!< CMP clock */
    RCU_WWDGT_SLP   = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 8U),                  /*!< WWDGT clock */
    RCU_ADC_SLP     = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 9U),                  /*!< ADC clock */
    RCU_TIMER0_SLP  = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 10U),                 /*!< TIMER0 clock */
    RCU_TIMER2_SLP  = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 11U),                 /*!< TIMER2 clock */
    RCU_SPI0_SLP    = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 12U),                 /*!< SPI0 clock */
    RCU_SPI1_SLP    = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 13U),                 /*!< SPI1 clock */
    RCU_USART0_SLP  = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 14U),                 /*!< USART0 clock */
    RCU_USART1_SLP  = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 15U),                 /*!< USART1 clock */
    RCU_TIMER13_SLP = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 16U),                 /*!< TIMER13 clock */
    RCU_TIMER15_SLP = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 17U),                 /*!< TIMER15 clock */
    RCU_TIMER16_SLP = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 18U),                 /*!< TIMER16 clock */
    RCU_USART2_SLP  = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 19U),                 /*!< USART2 clock */
    RCU_I2C0_SLP    = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 21U),                 /*!< I2C0 clock */
    RCU_I2C1_SLP    = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 22U),                 /*!< I2C1 clock */
    RCU_PMU_SLP     = RCU_REGIDX_BIT(APBSPDPEN_REG_OFFSET, 28U),                 /*!< PMU clock */
} rcu_periph_sleep_enum;

/* peripherals reset */
typedef enum {
    /* AHB1 peripherals */
    RCU_CRCRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 12U),                    /*!< CRC clock */
    RCU_DMARST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 21U),                    /*!< DMA clock */
    RCU_DMAMUXRST  = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 23U),                    /*!< DMA clock */
                                        
    /* AHB2 peripherals */              
    RCU_GPIOARST   = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 17U),                    /*!< GPIOA clock */
    RCU_GPIOBRST   = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 18U),                    /*!< GPIOB clock */
    RCU_GPIOCRST   = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 19U),                    /*!< GPIOC clock */
    RCU_GPIODRST   = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 20U),                    /*!< GPIOD clock */
    RCU_GPIOFRST   = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 22U),                    /*!< GPIOF clock */
                                      
    /* APB peripherals */             
    RCU_SYSCFGRST  = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 0U),                      /*!< SYSCFG clock */
    RCU_CMPRST     = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 1U),                      /*!< CMP clock */
    RCU_WWDGTRST   = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 8U),                      /*!< WWDGT clock */
    RCU_ADCRST     = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 9U),                      /*!< ADC clock */
    RCU_TIMER0RST  = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 10U),                     /*!< TIMER0 clock */
    RCU_TIMER2RST  = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 11U),                     /*!< TIMER2 clock */
    RCU_SPI0RST    = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 12U),                     /*!< SPI0 clock */
    RCU_SPI1RST    = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 13U),                     /*!< SPI1 clock */
    RCU_USART0RST  = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 14U),                     /*!< USART0 clock */
    RCU_USART1RST  = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 15U),                     /*!< USART1 clock */
    RCU_TIMER13RST = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 16U),                     /*!< TIMER13 clock */
    RCU_TIMER15RST = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 17U),                     /*!< TIMER15 clock */
    RCU_TIMER16RST = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 18U),                     /*!< TIMER16 clock */
    RCU_USART2RST  = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 19U),                     /*!< USART2 clock */
    RCU_I2C0RST    = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 21U),                     /*!< I2C0 clock */
    RCU_I2C1RST    = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 22U),                     /*!< I2C1 clock */
    RCU_PMURST     = RCU_REGIDX_BIT(APBRST_REG_OFFSET, 28U),                     /*!< PMU clock */
} rcu_periph_reset_enum;

/* clock stabilization, peripheral reset and clock dection flags */
typedef enum {
    /* clock stabilization flags */
    RCU_FLAG_IRC32KSTB    = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 1U),               /*!< IRC32K stabilization flag */
    RCU_FLAG_LXTALSTB     = RCU_REGIDX_BIT(CTL1_REG_OFFSET, 1U),                 /*!< LXTAL stabilization flag */
    RCU_FLAG_IRC48MSTB    = RCU_REGIDX_BIT(CTL0_REG_OFFSET, 1U),                 /*!< IRC48M stabilization flag */
    RCU_FLAG_HXTALSTB     = RCU_REGIDX_BIT(CTL0_REG_OFFSET, 17U),                /*!< HXTAL stabilization flag */

    /* reset source flags */
    RCU_FLAG_OBLRST       = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 23U),              /*!< option byte loader reset flag */
    RCU_FLAG_EPRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 26U),              /*!< External PIN reset flag */
    RCU_FLAG_PORRST       = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 27U),              /*!< power reset flag */
    RCU_FLAG_SWRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 28U),              /*!< SW reset flag */
    RCU_FLAG_FWDGTRST     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 29U),              /*!< FWDGT reset flag */
    RCU_FLAG_WWDGTRST     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 30U),              /*!< WWDGT reset flag */
    RCU_FLAG_LPRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 31U),              /*!< low-power reset flag */
    /* clock failure flag */
    RCU_FLAG_LCKCMD       = RCU_REGIDX_BIT(CTL1_REG_OFFSET, 6U)                  /*!< LXTAL clock failure detection flag */
} rcu_flag_enum;

/* clock stabilization and ckm interrupt flags */
typedef enum {
    RCU_INT_FLAG_IRC32KSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 0U),                 /*!< IRC32K stabilization interrupt flag */
    RCU_INT_FLAG_LXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 1U),                 /*!< LXTAL stabilization interrupt flag */
    RCU_INT_FLAG_IRC48MSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 2U),                 /*!< IRC48M stabilization interrupt flag */
    RCU_INT_FLAG_LXTALCKM  = RCU_REGIDX_BIT(INT_REG_OFFSET, 6U),                 /*!< LXTAL clock stuck interrupt flag */
    RCU_INT_FLAG_HXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 3U),                 /*!< HXTAL stabilization interrupt flag */
    RCU_INT_FLAG_CKM       = RCU_REGIDX_BIT(INT_REG_OFFSET, 7U),                 /*!< CKM interrupt flag */
} rcu_int_flag_enum;

/* clock stabilization and stuck interrupt flags clear */
typedef enum {
    RCU_INT_FLAG_IRC32KSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 16U),            /*!< IRC32K stabilization interrupt flags clear */
    RCU_INT_FLAG_LXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 17U),            /*!< LXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_IRC48MSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 18U),            /*!< IRC48M stabilization interrupt flags clear */
    RCU_INT_FLAG_LXTALCKM_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 22U),            /*!< LXTAL clock stuck interrupt flag clear */
    RCU_INT_FLAG_HXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 19U),            /*!< HXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_CKM_CLR       = RCU_REGIDX_BIT(INT_REG_OFFSET, 23U),            /*!< CKM interrupt flags clear */
} rcu_int_flag_clear_enum;

/* clock stabilization interrupt enable or disable */
typedef enum {
    RCU_INT_IRC32KSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 8U),                /*!< IRC32K stabilization interrupt */
    RCU_INT_LXTALSTB        = RCU_REGIDX_BIT(INT_REG_OFFSET, 9U),                /*!< LXTAL stabilization interrupt */
    RCU_INT_IRC48MSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 10U),               /*!< IRC48M stabilization interrupt */
    RCU_INT_HXTALSTB        = RCU_REGIDX_BIT(INT_REG_OFFSET, 11U),               /*!< HXTAL stabilization interrupt */
} rcu_int_enum;

/* oscillator types */
typedef enum {
    RCU_HXTAL   = RCU_REGIDX_BIT(CTL0_REG_OFFSET, 16U),                          /*!< HXTAL */
    RCU_LXTAL   = RCU_REGIDX_BIT(CTL1_REG_OFFSET, 0U),                           /*!< LXTAL */
    RCU_IRC48M  = RCU_REGIDX_BIT(CTL0_REG_OFFSET, 0U),                           /*!< IRC48M */
    RCU_IRC32K  = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 0U),                         /*!< IRC32K */
} rcu_osci_type_enum;

/* rcu clock frequency */
typedef enum {
    CK_SYS      = 0U,                                                            /*!< system clock */
    CK_AHB,                                                                      /*!< AHB clock */
    CK_APB,                                                                      /*!< APB clock */
    CK_ADC,                                                                      /*!< ADC clock */
    CK_USART0,                                                                   /*!< USART0 clock */
    CK_I2C0,                                                                     /*!< I2C0 clock */
    CK_I2C1,                                                                     /*!< I2C1 clock */
    CK_I2C2,                                                                     /*!< I2C2 clock */
    CK_USART1,                                                                   /*!< USART1 clock */
} rcu_clock_freq_enum;

typedef enum {
    IDX_USART0 = 0U,                                                             /*!< index of USART0 */
    IDX_USART1                                                                   /*!< index of USART1 */
} usart_idx_enum;

typedef enum {
    IDX_I2C0 = 0U,                                                               /*!< index of I2C0 */
    IDX_I2C1,                                                                    /*!< index of I2C1 */
} i2c_idx_enum;

/* IRC48MDIV_SYS clock source selection */
#define CTL_IRC48MDIV_SYS_SEL(regval)    (BITS(29,31) & ((uint32_t)(regval) << 29U))
#define RCU_IRC48MDIV_SYS_1              CTL_IRC48MDIV_SYS_SEL(0)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M */
#define RCU_IRC48MDIV_SYS_2              CTL_IRC48MDIV_SYS_SEL(1)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 2 */
#define RCU_IRC48MDIV_SYS_4              CTL_IRC48MDIV_SYS_SEL(2)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 4 */
#define RCU_IRC48MDIV_SYS_8              CTL_IRC48MDIV_SYS_SEL(3)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 8 */
#define RCU_IRC48MDIV_SYS_16             CTL_IRC48MDIV_SYS_SEL(4)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 16 */
#define RCU_IRC48MDIV_SYS_32             CTL_IRC48MDIV_SYS_SEL(5)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 32 */
#define RCU_IRC48MDIV_SYS_64             CTL_IRC48MDIV_SYS_SEL(6)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 64 */
#define RCU_IRC48MDIV_SYS_128            CTL_IRC48MDIV_SYS_SEL(7)                /*!< CK_IRC48MDIV_SYS select CK_IRC48M divided by 128 */

/* IRC48MDIV_PER clock source selection */
#define CTL_IRC48MDIV_PER_SEL(regval)    (BITS(25,27) & ((uint32_t)(regval) << 25U))
#define RCU_IRC48MDIV_PER_1              CTL_IRC48MDIV_PER_SEL(0)                /*!< CK_IRC48MDIV_PER select CK_IRC48M */
#define RCU_IRC48MDIV_PER_2              CTL_IRC48MDIV_PER_SEL(1)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 2 */
#define RCU_IRC48MDIV_PER_3              CTL_IRC48MDIV_PER_SEL(2)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 3 */
#define RCU_IRC48MDIV_PER_4              CTL_IRC48MDIV_PER_SEL(3)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 4 */
#define RCU_IRC48MDIV_PER_5              CTL_IRC48MDIV_PER_SEL(4)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 5 */
#define RCU_IRC48MDIV_PER_6              CTL_IRC48MDIV_PER_SEL(5)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 6 */
#define RCU_IRC48MDIV_PER_7              CTL_IRC48MDIV_PER_SEL(6)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 7 */
#define RCU_IRC48MDIV_PER_8              CTL_IRC48MDIV_PER_SEL(7)                /*!< CK_IRC48MDIV_PER select CK_IRC48M divided by 8 */

/* system clock source select */
#define CFG0_SCS(regval)            (BITS(0,1) & ((uint32_t)(regval) << 0U))
#define RCU_CKSYSSRC_IRC48MDIV_SYS   CFG0_SCS(0)                                 /*!< system clock source select CK_IRC48MDIV_SYS */
#define RCU_CKSYSSRC_HXTAL           CFG0_SCS(1)                                 /*!< system clock source select HXTAL */
#define RCU_CKSYSSRC_IRC32K          CFG0_SCS(2)                                 /*!< system clock source select IRC32K */
#define RCU_CKSYSSRC_LXTAL           CFG0_SCS(3)                                 /*!< system clock source select LXTAL */

/* system clock source select status */
#define CFG0_SCSS(regval)           (BITS(2,3) & ((uint32_t)(regval) << 2U))
#define RCU_SCSS_IRC48MDIV           CFG0_SCSS(0)                                /*!< system clock source select IRC48M */
#define RCU_SCSS_HXTAL               CFG0_SCSS(1)                                /*!< system clock source select HXTAL */
#define RCU_SCSS_IRC32K              CFG0_SCSS(2)                                /*!< system clock source select IRC32K */
#define RCU_SCSS_LXTAL               CFG0_SCSS(3)                                /*!< system clock source select LXTAL */

/* AHB prescaler selection */
#define CFG0_AHBPSC(regval)         (BITS(4,7) & ((uint32_t)(regval) << 4U))
#define RCU_AHB_CKSYS_DIV1          CFG0_AHBPSC(0)                               /*!< AHB prescaler select CK_SYS */
#define RCU_AHB_CKSYS_DIV2          CFG0_AHBPSC(8)                               /*!< AHB prescaler select CK_SYS/2 */
#define RCU_AHB_CKSYS_DIV4          CFG0_AHBPSC(9)                               /*!< AHB prescaler select CK_SYS/4 */
#define RCU_AHB_CKSYS_DIV8          CFG0_AHBPSC(10)                              /*!< AHB prescaler select CK_SYS/8 */
#define RCU_AHB_CKSYS_DIV16         CFG0_AHBPSC(11)                              /*!< AHB prescaler select CK_SYS/16 */
#define RCU_AHB_CKSYS_DIV64         CFG0_AHBPSC(12)                              /*!< AHB prescaler select CK_SYS/64 */
#define RCU_AHB_CKSYS_DIV128        CFG0_AHBPSC(13)                              /*!< AHB prescaler select CK_SYS/128 */
#define RCU_AHB_CKSYS_DIV256        CFG0_AHBPSC(14)                              /*!< AHB prescaler select CK_SYS/256 */
#define RCU_AHB_CKSYS_DIV512        CFG0_AHBPSC(15)                              /*!< AHB prescaler select CK_SYS/512 */

/* APB prescaler selection */
#define CFG0_APBPSC(regval)        (BITS(11,13) & ((uint32_t)(regval) << 11U))
#define RCU_APB_CKAHB_DIV1         CFG0_APBPSC(0)                                /*!< APB prescaler select CK_AHB */
#define RCU_APB_CKAHB_DIV2         CFG0_APBPSC(4)                                /*!< APB prescaler select CK_AHB/2 */
#define RCU_APB_CKAHB_DIV4         CFG0_APBPSC(5)                                /*!< APB prescaler select CK_AHB/4 */
#define RCU_APB_CKAHB_DIV8         CFG0_APBPSC(6)                                /*!< APB prescaler select CK_AHB/8 */
#define RCU_APB_CKAHB_DIV16        CFG0_APBPSC(7)                                /*!< APB prescaler select CK_AHB/16 */

/* CK_OUT0 clock source selection */
#define CFG0_CKOUT0SEL(regval)       (BITS(24,26) & ((uint32_t)(regval) << 24U))
#define RCU_CKOUT0SRC_NONE           CFG0_CKOUT0SEL(0)                           /*!< no clock selected */
#define RCU_CKOUT0SRC_CKSYS          CFG0_CKOUT0SEL(1)                           /*!< CK_OUT0 clock source select CKSYS */
#define RCU_CKOUT0SRC_IRC48M         CFG0_CKOUT0SEL(3)                           /*!< CK_OUT0 clock source select IRC48M */
#define RCU_CKOUT0SRC_HXTAL          CFG0_CKOUT0SEL(4)                           /*!< CK_OUT0 clock source select HXTAL */
#define RCU_CKOUT0SRC_IRC32K         CFG0_CKOUT0SEL(6)                           /*!< CK_OUT0 clock source select IRC32K */
#define RCU_CKOUT0SRC_LXTAL          CFG0_CKOUT0SEL(7)                           /*!< CK_OUT0 clock source select LXTAL */

/* CK_OUT0 divider */
#define CFG0_CKOUT0DIV(regval)       (BITS(28,30) & ((uint32_t)(regval) << 28U))
#define RCU_CKOUT0_DIV1              CFG0_CKOUT0DIV(0)                           /*!< CK_OUT0 is divided by 1 */
#define RCU_CKOUT0_DIV2              CFG0_CKOUT0DIV(1)                           /*!< CK_OUT0 is divided by 2 */
#define RCU_CKOUT0_DIV4              CFG0_CKOUT0DIV(2)                           /*!< CK_OUT0 is divided by 4 */
#define RCU_CKOUT0_DIV8              CFG0_CKOUT0DIV(3)                           /*!< CK_OUT0 is divided by 8 */
#define RCU_CKOUT0_DIV16             CFG0_CKOUT0DIV(4)                           /*!< CK_OUT0 is divided by 16 */
#define RCU_CKOUT0_DIV32             CFG0_CKOUT0DIV(5)                           /*!< CK_OUT0 is divided by 32 */
#define RCU_CKOUT0_DIV64             CFG0_CKOUT0DIV(6)                           /*!< CK_OUT0 is divided by 64 */
#define RCU_CKOUT0_DIV128            CFG0_CKOUT0DIV(7)                           /*!< CK_OUT0 is divided by 128 */

/* CK_OUT1 clock source selection */
#define CFG0_CKOUT1SEL(regval)       (BITS(16,18) & ((uint32_t)(regval) << 16U))
#define RCU_CKOUT1SRC_NONE           CFG0_CKOUT1SEL(0)                           /*!< no clock selected */
#define RCU_CKOUT1SRC_CKSYS          CFG0_CKOUT1SEL(1)                           /*!< CK_OUT1 clock source select CKSYS  */
#define RCU_CKOUT1SRC_IRC48M         CFG0_CKOUT1SEL(3)                           /*!< CK_OUT1 clock source select IRC48M */
#define RCU_CKOUT1SRC_HXTAL          CFG0_CKOUT1SEL(4)                           /*!< CK_OUT1 clock source select HXTAL */
#define RCU_CKOUT1SRC_IRC32K         CFG0_CKOUT1SEL(6)                           /*!< CK_OUT1 clock source select IRC32K */
#define RCU_CKOUT1SRC_LXTAL          CFG0_CKOUT1SEL(7)                           /*!< CK_OUT1 clock source select LXTAL */

/* CK_OUT1 divider */
#define CFG0_CKOUT1DIV(regval)       (BITS(20,22) & ((uint32_t)(regval) << 20U))
#define RCU_CKOUT1_DIV1              CFG0_CKOUT1DIV(0)                           /*!< CK_OUT1 is divided by 1 */
#define RCU_CKOUT1_DIV2              CFG0_CKOUT1DIV(1)                           /*!< CK_OUT1 is divided by 2 */
#define RCU_CKOUT1_DIV4              CFG0_CKOUT1DIV(2)                           /*!< CK_OUT1 is divided by 4 */
#define RCU_CKOUT1_DIV8              CFG0_CKOUT1DIV(3)                           /*!< CK_OUT1 is divided by 8 */
#define RCU_CKOUT1_DIV16             CFG0_CKOUT1DIV(4)                           /*!< CK_OUT1 is divided by 16 */
#define RCU_CKOUT1_DIV32             CFG0_CKOUT1DIV(5)                           /*!< CK_OUT1 is divided by 32 */
#define RCU_CKOUT1_DIV64             CFG0_CKOUT1DIV(6)                           /*!< CK_OUT1 is divided by 64 */
#define RCU_CKOUT1_DIV128            CFG0_CKOUT1DIV(7)                           /*!< CK_OUT1 is divided by 128 */

/* low speed clock output source selection */
#define RCU_LSCKOUTSRC_IRC32K              (uint32_t)(0X00000000U)               /*!< IRC32K clock selected */
#define RCU_LSCKOUTSRC_LXTAL               RCU_CTL1_LSCKOUTSEL                   /*!< LXTAL clock selected */

/* LXTAL drive capability */
#define RCU_LXTAL_LOWDRI            (uint32_t)(0X00000000U)                      /*!< lower driving capability */
#define RCU_LXTAL_HIGHDRI           RCU_CTL1_LXTALDRI                            /*!< higher driving capability */

/* RTC clock entry selection */
#define CTL1_RTCSRC(regval)        (BITS(8,9) & ((uint32_t)(regval) << 8U))
#define RCU_RTCSRC_NONE             CTL1_RTCSRC(0)                              /*!< no clock selected */
#define RCU_RTCSRC_LXTAL            CTL1_RTCSRC(1)                              /*!< LXTAL selected as RTC source clock */
#define RCU_RTCSRC_IRC32K           CTL1_RTCSRC(2)                              /*!< IRC32K selected as RTC source clock */
#define RCU_RTCSRC_HXTAL_DIV32      CTL1_RTCSRC(3)                              /*!< HXTAL/32 selected as RTC source clock */

/* USART0 clock source selection */
#define CFG1_USART0SEL(regval)      (BITS(0,1) & ((uint32_t)(regval) << 0U))
#define RCU_USART0SRC_CKAPB         CFG1_USART0SEL(0)                           /*!< CK_USART0 select CK_APB */
#define RCU_USART0SRC_CKSYS         CFG1_USART0SEL(1)                           /*!< CK_USART0 select CK_SYS */
#define RCU_USART0SRC_IRC48MDIV_PER CFG1_USART0SEL(2)                           /*!< CK_USART0 select CK_IRC48MDIV_PER */
#define RCU_USART0SRC_LXTAL         CFG1_USART0SEL(3)                           /*!< CK_USART0 select LXTAL */

/* I2Cx(x=0,1) clock source selection */
#define CFG1_I2C0SEL(regval)        (BITS(2,3) & ((uint32_t)(regval) << 2U))
#define RCU_I2CSRC_CKAPB             CFG1_I2C0SEL(0)                            /*!< CK_I2C select CK_APB */
#define RCU_I2CSRC_CKSYS             CFG1_I2C0SEL(1)                            /*!< CK_I2C select CK_SYS */
#define RCU_I2CSRC_IRC48MDIV_PER     CFG1_I2C0SEL(2)                            /*!< CK_I2C select CK_IRC48MDIV_PER */

/* I2S clock source selection */
#define CFG1_I2SSEL(regval)        (BITS(14,15) & ((uint32_t)(regval) << 14U))
#define RCU_I2SSRC_CKSYS             CFG1_I2SSEL(0)                             /*!< CK_I2S select CK_SYS */
#define RCU_I2SSRC_IRC48MDIV_PER     CFG1_I2SSEL(2)                             /*!< CK_I2S select CK_IRC48MDIV_PER */
#define RCU_I2SSRC_CKIN              CFG1_I2SSEL(3)                             /*!< CK_I2S select I2S_CKIN */

/* ADC clock source selection */
#define RCU_ADCSRC_CKSYS                (uint32_t)0x00000000U                    /*!< ADC clock source select CK_SYS */
#define RCU_ADCSRC_IRC48M_PER           (uint32_t)0x00000100U                    /*!< ADC clock source select CK_IRC48MDIV_PER */
#define RCU_ADCSRC_HXTAL                 RCU_CFG1_ADCSEL                         /*!< ADC clock source select CK_HXTAL */

/* ADC clock prescaler selection */
#define CFG1_ADCPSC(regval)         (BITS(9,12) & ((uint32_t)(regval) << 9U))
#define RCU_ADCCK_DIV1               CFG1_ADCPSC(0)                               /*!< ADC clock prescaler select not divided */
#define RCU_ADCCK_DIV2               CFG1_ADCPSC(1)                               /*!< ADC clock prescaler select divided by 2 */
#define RCU_ADCCK_DIV4               CFG1_ADCPSC(2)                               /*!< ADC clock prescaler select divided by 4*/
#define RCU_ADCCK_DIV6               CFG1_ADCPSC(3)                               /*!< ADC clock prescaler select divided by 6*/
#define RCU_ADCCK_DIV8               CFG1_ADCPSC(4)                               /*!< ADC clock prescaler select divided by 8*/
#define RCU_ADCCK_DIV10              CFG1_ADCPSC(5)                               /*!< ADC clock prescaler select divided by 10 */
#define RCU_ADCCK_DIV12              CFG1_ADCPSC(6)                               /*!< ADC clock prescaler select divided by 12 */
#define RCU_ADCCK_DIV16              CFG1_ADCPSC(7)                               /*!< ADC clock prescaler select divided by 16 */
#define RCU_ADCCK_DIV32              CFG1_ADCPSC(8)                               /*!< ADC clock prescaler select divided by 32 */
#define RCU_ADCCK_DIV64              CFG1_ADCPSC(9)                               /*!< ADC clock prescaler select divided by 64 */
#define RCU_ADCCK_DIV128             CFG1_ADCPSC(10)                              /*!< ADC clock prescaler select divided by 128 */
#define RCU_ADCCK_DIV256             CFG1_ADCPSC(11)                              /*!< ADC clock prescaler select divided by 256 */

#ifdef FW_DEBUG_ERR_REPORT
/* check maximum irc48m adjust value */
#define IRC48M_ADJ_HIGH_VALUE                     ((uint32_t)0x0000003FU)
#define NOT_IRC48M_ADJ(irc48m_adjval)             ((IRC48M_ADJ_HIGH_VALUE < (irc48m_adjval))) 
#endif /* FW_DEBUG_ERR_REPORT */

/* function declarations */
/* initialization, peripheral clock and reset configuration functions */
/* deinitialize the RCU */
void rcu_deinit(void);
/* enable the peripherals clock */
void rcu_periph_clock_enable(rcu_periph_enum periph);
/* disable the peripherals clock */
void rcu_periph_clock_disable(rcu_periph_enum periph);
/* enable the peripherals clock when sleep mode */
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph);
/* disable the peripherals clock when sleep mode */
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph);
/* reset the peripherals */
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset);
/* disable reset the peripheral */
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset);
/* reset the BKP */
void rcu_bkp_reset_enable(void);
/* disable the BKP reset */
void rcu_bkp_reset_disable(void);

/* system clock, AHB, APB, ADC and clock out configuration functions */
/* configure the system clock source */
void rcu_system_clock_source_config(uint32_t ck_sys);
/* get the system clock source */
uint32_t rcu_system_clock_source_get(void);
/* configure the AHB prescaler selection */
void rcu_ahb_clock_config(uint32_t ck_ahb);
/* configure the APB prescaler selection */
void rcu_apb_clock_config(uint32_t ck_apb);
/* configure the ADC clock source and prescaler selection */
void rcu_adc_clock_config(uint32_t adc_clock_source, uint32_t ck_adc);
/* configure the CK_OUT0 clock source and divider */
void rcu_ckout0_config(uint32_t ckout0_src, uint32_t ckout0_div);
/* configure the CK_OUT1 clock source and divider */
void rcu_ckout1_config(uint32_t ckout1_src, uint32_t ckout1_div);
/* enable the low speed clock output */
void rcu_lsckout_enable(void);
/* disable the low speed clock output */
void rcu_lsckout_disable(void);
/* configure the LSCKOUT clock source */
void rcu_lsckout_config(uint32_t lsckout_src);

/* configure the USART clock source selection */
void rcu_usart_clock_config(usart_idx_enum usart_idx, uint32_t ck_usart);
/* configure the I2Cx(x=0,1) clock source selection */
void rcu_i2c_clock_config(i2c_idx_enum i2c_idx, uint32_t ck_i2c);
/* configure the I2S clock source selection */
void rcu_i2s_clock_config(uint32_t ck_i2s);
/* configure the IRC48MDIV_SYS clock selection */
void rcu_irc48mdiv_sys_clock_config(uint32_t ck_irc48mdiv_sys);
/* configure the IRC48MDIV_PER clock selection */
void rcu_irc48mdiv_per_clock_config(uint32_t ck_irc48mdiv_per) ;
/* configure the RTC clock source selection */
void rcu_rtc_clock_config(uint32_t rtc_clock_source);
/* configure the LXTAL drive capability */
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap);

/* oscillator configuration functions */
/* wait until oscillator stabilization flags is SET */
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci);
/* turn on the oscillator */
void rcu_osci_on(rcu_osci_type_enum osci);
/* turn off the oscillator */
void rcu_osci_off(rcu_osci_type_enum osci);
/* enable the oscillator bypass mode */
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci);
/* disable the oscillator bypass mode */
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci);
/* set the IRC48M adjust value */
void rcu_irc48m_adjust_value_set(uint8_t irc48m_adjval);
/* LXTAL stabilization reset */
void rcu_lxtal_stab_reset_enable(void); 
/* disable LXTAL stabilization reset */
void rcu_lxtal_stab_reset_disable(void);

/* clock monitor configure functions */
/* enable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_enable(void);
/* disable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_disable(void);
/* enable the LXTAL clock monitor */
void rcu_lxtal_clock_monitor_enable(void);
/* disable the LXTAL clock monitor */
void rcu_lxtal_clock_monitor_disable(void);

/* clock frequency get functions */
/* get the system clock, bus and peripheral clock frequency */
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock);

/* flag and interrupt functions */
/* get the clock stabilization and periphral reset flags */
FlagStatus rcu_flag_get(rcu_flag_enum flag);
/* clear the reset flag */
void rcu_all_reset_flag_clear(void);
/* enable the stabilization interrupt */
void rcu_interrupt_enable(rcu_int_enum stab_int);
/* disable the stabilization interrupt */
void rcu_interrupt_disable(rcu_int_enum stab_int);
/* get the clock stabilization interrupt and ckm flags */
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag);
/* clear the interrupt flags */
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag_clear);
#endif /* GD32C2X1_RCU_H */
