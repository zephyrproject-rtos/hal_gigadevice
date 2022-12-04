/*!
    \file    gd32a50x_rcu.h
    \brief   definitions for the RCU

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

#ifndef GD32A50X_RCU_H
#define GD32A50X_RCU_H

#include "gd32a50x.h"

/* RCU definitions */
#define RCU                             RCU_BASE                              /*!< RCU base address */

/* registers definitions */
#define RCU_CTL                         REG32(RCU + 0x00000000U)              /*!< control register */
#define RCU_CFG0                        REG32(RCU + 0x00000004U)              /*!< clock configuration register 0 */
#define RCU_INT                         REG32(RCU + 0x00000008U)              /*!< clock interrupt register */
#define RCU_APB2RST                     REG32(RCU + 0x0000000CU)              /*!< APB2 reset register */
#define RCU_APB1RST                     REG32(RCU + 0x00000010U)              /*!< APB1 reset register */
#define RCU_AHBEN                       REG32(RCU + 0x00000014U)              /*!< AHB1 enable register */
#define RCU_APB2EN                      REG32(RCU + 0x00000018U)              /*!< APB2 enable register */
#define RCU_APB1EN                      REG32(RCU + 0x0000001CU)              /*!< APB1 enable register */
#define RCU_BDCTL                       REG32(RCU + 0x00000020U)              /*!< backup domain control register */
#define RCU_RSTSCK                      REG32(RCU + 0x00000024U)              /*!< reset source / clock register */
#define RCU_AHBRST                      REG32(RCU + 0x00000028U)              /*!< AHB reset register */
#define RCU_CFG1                        REG32(RCU + 0x0000002CU)              /*!< clock configuration register 1 */
#define RCU_CFG2                        REG32(RCU + 0x00000030U)              /*!< clock configuration register 2 */
#define RCU_VKEY                        REG32(RCU + 0x00000100U)              /*!< voltage key register */
#define RCU_DSV                         REG32(RCU + 0x00000134U)              /*!< deep-sleep mode voltage register */

/* bits definitions */
/* RCU_CTL */
#define RCU_CTL_IRC8MEN                 BIT(0)                                /*!< internal high speed oscillator enable */
#define RCU_CTL_IRC8MSTB                BIT(1)                                /*!< IRC8M high speed internal oscillator stabilization flag */
#define RCU_CTL_IRC8MADJ                BITS(3,7)                             /*!< internal 8M RC oscillator clock trim adjust value */
#define RCU_CTL_IRC8MCALIB              BITS(8,15)                            /*!< internal 8M RC oscillator calibration value */
#define RCU_CTL_HXTALEN                 BIT(16)                               /*!< external high speed oscillator enable */
#define RCU_CTL_HXTALSTB                BIT(17)                               /*!< external crystal oscillator clock stabilization flag */
#define RCU_CTL_HXTALBPS                BIT(18)                               /*!< external crystal oscillator clock bypass mode enable */
#define RCU_CTL_CKMEN                   BIT(19)                               /*!< HXTAL clock monitor enable */
#define RCU_CTL_PLLMEN                  BIT(20)                               /*!< PLL clock monitor enable */
#define RCU_CTL_LCKMEN                  BIT(21)                               /*!< LXTAL clock monitor enable */
#define RCU_CTL_HXTALSCAL               BIT(22)                               /*!< HXTAL frequency scale select */
#define RCU_CTL_PLLEN                   BIT(24)                               /*!< PLL enable */
#define RCU_CTL_PLLSTB                  BIT(25)                               /*!< PLL clock stabilization flag */

/* RCU_CFG0 */
#define RCU_CFG0_SCS                    BITS(0,1)                             /*!< system clock switch */
#define RCU_CFG0_SCSS                   BITS(2,3)                             /*!< system clock switch status */
#define RCU_CFG0_AHBPSC                 BITS(4,7)                             /*!< AHB prescaler selection */
#define RCU_CFG0_APB1PSC                BITS(8,10)                            /*!< APB1 prescaler selection */
#define RCU_CFG0_APB2PSC                BITS(11,13)                           /*!< APB2 prescaler selection */
#define RCU_CFG0_PLLSEL                 BIT(16)                               /*!< PLL clock source selection */
#define RCU_CFG0_DPLL                   BIT(17)                               /*!< double PLL clock */
#define RCU_CFG0_PLLMF                  BITS(18,21)                           /*!< PLL clock multiplication factor */
#define RCU_CFG0_CKOUTSEL               BITS(24,26)                           /*!< CKOUT clock source selection */
#define RCU_CFG0_PLLMF_4                BIT(27)                               /*!< bit 4 of PLLMF */
#define RCU_CFG0_CKOUTDIV               BITS(28,30)                           /*!< CK_OUT divider which the CK_OUT frequency can be reduced */
#define RCU_CFG0_PLLDV                  BIT(31)                               /*!< CK_PLL divide by 1 or 2 for CK_OUT */

/* RCU_INT */
#define RCU_INT_IRC40KSTBIF             BIT(0)                                /*!< IRC40K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF              BIT(1)                                /*!< LXTAL stabilization interrupt flag */
#define RCU_INT_IRC8MSTBIF              BIT(2)                                /*!< IRC8M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF              BIT(3)                                /*!< HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF                BIT(4)                                /*!< PLL stabilization interrupt flag */
#define RCU_INT_LCKMIF                  BIT(5)                                /*!< LXTAL clock monitor interrupt flag */
#define RCU_INT_PLLMIF                  BIT(6)                                /*!< PLL clock monitor interrupt flag */
#define RCU_INT_CKMIF                   BIT(7)                                /*!< HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC40KSTBIE             BIT(8)                                /*!< IRC40K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE              BIT(9)                                /*!< LXTAL stabilization interrupt enable */
#define RCU_INT_IRC8MSTBIE              BIT(10)                               /*!< IRC8M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE              BIT(11)                               /*!< HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE                BIT(12)                               /*!< PLL stabilization interrupt enable */
#define RCU_INT_LCKMIE                  BIT(13)                               /*!< LXTAL clock monitor interrupt enable */
#define RCU_INT_PLLMIE                  BIT(14)                               /*!< PLL clock monitor interrupt enable */
#define RCU_INT_IRC40KSTBIC             BIT(16)                               /*!< IRC40K stabilization interrupt clear */
#define RCU_INT_LXTALSTBIC              BIT(17)                               /*!< LXTAL stabilization interrupt clear */
#define RCU_INT_IRC8MSTBIC              BIT(18)                               /*!< IRC8M stabilization interrupt clear */
#define RCU_INT_HXTALSTBIC              BIT(19)                               /*!< HXTAL stabilization interrupt clear */
#define RCU_INT_PLLSTBIC                BIT(20)                               /*!< PLL stabilization interrupt clear */
#define RCU_INT_LCKMIC                  BIT(21)                               /*!< LXTAL clock monitor interrupt clear */
#define RCU_INT_PLLMIC                  BIT(22)                               /*!< PLL clock monitor interrupt clear */
#define RCU_INT_CKMIC                   BIT(23)                               /*!< HXTAL clock stuck interrupt clear */

/* RCU_APB2RST */
#define RCU_APB2RST_CFGRST              BIT(0)                                /*!< system configuration reset */
#define RCU_APB2RST_CMPRST              BIT(1)                                /*!< comparator reset */
#define RCU_APB2RST_ADC0RST             BIT(9)                                /*!< ADC0 reset */
#define RCU_APB2RST_ADC1RST             BIT(10)                               /*!< ADC1 reset */
#define RCU_APB2RST_TIMER0RST           BIT(11)                               /*!< TIMER0 reset */
#define RCU_APB2RST_SPI0RST             BIT(12)                               /*!< SPI0 reset */
#define RCU_APB2RST_TIMER7RST           BIT(13)                               /*!< TIMER7 reset */
#define RCU_APB2RST_USART0RST           BIT(14)                               /*!< USART0 reset */
#define RCU_APB2RST_TIMER19RST          BIT(20)                               /*!< TIMER19 reset */
#define RCU_APB2RST_TIMER20RST          BIT(21)                               /*!< TIMER20 reset */
#define RCU_APB2RST_CAN0RST             BIT(30)                               /*!< CAN0 reset */
#define RCU_APB2RST_CAN1RST             BIT(31)                               /*!< CAN1 reset */

/* RCU_APB1RST */
#define RCU_APB1RST_TIMER1RST           BIT(0)                                /*!< TIMER1 reset */
#define RCU_APB1RST_TIMER5RST           BIT(4)                                /*!< TIMER5 reset */
#define RCU_APB1RST_TIMER6RST           BIT(5)                                /*!< TIMER6 reset */
#define RCU_APB1RST_WWDGTRST            BIT(11)                               /*!< WWDGT reset */
#define RCU_APB1RST_SPI1RST             BIT(14)                               /*!< SPI1 reset */
#define RCU_APB1RST_USART1RST           BIT(17)                               /*!< USART1 reset */
#define RCU_APB1RST_USART2RST           BIT(18)                               /*!< USART2 reset */
#define RCU_APB1RST_I2C0RST             BIT(21)                               /*!< I2C0 reset */
#define RCU_APB1RST_I2C1RST             BIT(22)                               /*!< I2C1 reset */
#define RCU_APB1RST_PMURST              BIT(28)                               /*!< PMU reset */
#define RCU_APB1RST_DACRST              BIT(29)                               /*!< DAC reset */

/* RCU_AHBEN */
#define RCU_AHBEN_DMA0EN                BIT(0)                                /*!< DMA0 clock enable */
#define RCU_AHBEN_DMA1EN                BIT(1)                                /*!< DMA1 clock enable */
#define RCU_AHBEN_SRAMSPEN              BIT(2)                                /*!< SRAM clock enable when sleep mode */
#define RCU_AHBEN_DMAMUXEN              BIT(3)                                /*!< DMAMUX clock enable */
#define RCU_AHBEN_FMCSPEN               BIT(4)                                /*!< FMC clock enable when sleep mode */
#define RCU_AHBEN_CRCEN                 BIT(6)                                /*!< CRC clock enable */
#define RCU_AHBEN_MFCOMEN               BIT(14)                               /*!< MFCOM clock enable */
#define RCU_AHBEN_PAEN                  BIT(17)                               /*!< GPIOA clock enable */
#define RCU_AHBEN_PBEN                  BIT(18)                               /*!< GPIOB clock enable */
#define RCU_AHBEN_PCEN                  BIT(19)                               /*!< GPIOC clock enable */
#define RCU_AHBEN_PDEN                  BIT(20)                               /*!< GPIOD clock enable */
#define RCU_AHBEN_PEEN                  BIT(21)                               /*!< GPIOE clock enable */
#define RCU_AHBEN_PFEN                  BIT(22)                               /*!< GPIOF clock enable */

/* RCU_APB2EN */
#define RCU_APB2EN_CFGEN                BIT(0)                                /*!< System configuration clock enable */
#define RCU_APB2EN_CMPEN                BIT(1)                                /*!< Comparator clock enable */
#define RCU_APB2EN_ADC0EN               BIT(9)                                /*!< ADC0 clock enable */
#define RCU_APB2EN_ADC1EN               BIT(10)                               /*!< ADC1 clock enable */
#define RCU_APB2EN_TIMER0EN             BIT(11)                               /*!< TIMER0 clock enable */
#define RCU_APB2EN_SPI0EN               BIT(12)                               /*!< SPI0 clock enable */
#define RCU_APB2EN_TIMER7EN             BIT(13)                               /*!< TIMER7 clock enable */
#define RCU_APB2EN_USART0EN             BIT(14)                               /*!< USART0 clock enable */
#define RCU_APB2EN_TIMER19EN            BIT(20)                               /*!< TIMER19 clock enable */
#define RCU_APB2EN_TIMER20EN            BIT(21)                               /*!< TIMER20 clock enable */
#define RCU_APB2EN_TRIGSELEN            BIT(29)                               /*!< TRIGSEL clock enable */
#define RCU_APB2EN_CAN0EN               BIT(30)                               /*!< CAN0 clock enable */
#define RCU_APB2EN_CAN1EN               BIT(31)                               /*!< CAN1 clock enable */

/* RCU_APB1EN */
#define RCU_APB1EN_TIMER1EN             BIT(0)                                /*!< TIMER1 clock enable */
#define RCU_APB1EN_TIMER5EN             BIT(4)                                /*!< TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN             BIT(5)                                /*!< TIMER6 clock enable */
#define RCU_APB1EN_WWDGTEN              BIT(11)                               /*!< WWDGT clock enable */
#define RCU_APB1EN_SPI1EN               BIT(14)                               /*!< SPI1 clock enable */
#define RCU_APB1EN_USART1EN             BIT(17)                               /*!< USART1 clock enable */
#define RCU_APB1EN_USART2EN             BIT(18)                               /*!< USART2 clock enable */
#define RCU_APB1EN_I2C0EN               BIT(21)                               /*!< I2C0 clock enable */
#define RCU_APB1EN_I2C1EN               BIT(22)                               /*!< I2C1 clock enable */
#define RCU_APB1EN_BKPEN                BIT(27)                               /*!< Back-up interface clock enable */
#define RCU_APB1EN_PMUEN                BIT(28)                               /*!< PMU clock enable */
#define RCU_APB1EN_DACEN                BIT(29)                               /*!< DAC clock enable */

/* RCU_BDCTL */
#define RCU_BDCTL_LXTALEN               BIT(0)                                /*!< LXTAL enable */
#define RCU_BDCTL_LXTALSTB              BIT(1)                                /*!< low speed crystal oscillator stabilization flag */
#define RCU_BDCTL_LXTALBPS              BIT(2)                                /*!< LXTAL bypass mode enable */
#define RCU_BDCTL_LXTALDRI              BITS(3,4)                             /*!< LXTAL drive capability */
#define RCU_BDCTL_RTCSRC                BITS(8,9)                             /*!< RTC clock entry selection */
#define RCU_BDCTL_RTCEN                 BIT(15)                               /*!< RTC clock enable */
#define RCU_BDCTL_BKPRST                BIT(16)                               /*!< backup domain reset */

/* RCU_RSTSCK */
#define RCU_RSTSCK_IRC40KEN             BIT(0)                                /*!< IRC40K enable */
#define RCU_RSTSCK_IRC40KSTB            BIT(1)                                /*!< IRC40K stabilization flag */
#define RCU_RSTSCK_LOCKUPRSTEN          BIT(10)                               /*!< CPU Lock-Up reset enable */
#define RCU_RSTSCK_LVDRSTEN             BIT(11)                               /*!< low voltage detection reset enable */
#define RCU_RSTSCK_ECCRSTEN             BIT(12)                               /*!< ECC 2 bits error reset enable*/
#define RCU_RSTSCK_LOHRSTEN             BIT(13)                               /*!< lost of HXTAL reset enable */
#define RCU_RSTSCK_LOPRSTEN             BIT(14)                               /*!< lost of PLL reset enable */
#define RCU_RSTSCK_BORRSTF              BIT(17)                               /*!< BOR reset flag */
#define RCU_RSTSCK_LOCKUPRSTF           BIT(18)                               /*!< CPU Lock-Up error reset flag */
#define RCU_RSTSCK_LVDRSTF              BIT(19)                               /*!< low Voltage detect error reset flag */
#define RCU_RSTSCK_ECCRSTF              BIT(20)                               /*!< two bit ECC error reset flag */
#define RCU_RSTSCK_LOHRSTF              BIT(21)                               /*!< lost of HXTAL error reset flag */
#define RCU_RSTSCK_LOPRSTF              BIT(22)                               /*!< lost of PLL error reset flag */
#define RCU_RSTSCK_V11RSTF              BIT(23)                               /*!< 1.1V domain power reset flag */
#define RCU_RSTSCK_RSTFC                BIT(24)                               /*!< reset flag clear */
#define RCU_RSTSCK_OBLRSTF              BIT(25)                               /*!< option byte loader reset flag */
#define RCU_RSTSCK_EPRSTF               BIT(26)                               /*!< external pin reset flag */
#define RCU_RSTSCK_PORRSTF              BIT(27)                               /*!< power reset flag */
#define RCU_RSTSCK_SWRSTF               BIT(28)                               /*!< software reset flag */
#define RCU_RSTSCK_FWDGTRSTF            BIT(29)                               /*!< free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF            BIT(30)                               /*!< window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF               BIT(31)                               /*!< low-power reset flag */

/* RCU_AHBRST */
#define RCU_AHBRST_DMA0RST              BIT(0)                                /*!< DMA0 reset */
#define RCU_AHBRST_DMA1RST              BIT(1)                                /*!< DMA1 reset */
#define RCU_AHBRST_DMAMUXRST            BIT(3)                                /*!< DMAMUX reset */
#define RCU_AHBRST_CRCRST               BIT(6)                                /*!< CRC reset */
#define RCU_AHBRST_MFCOMRST             BIT(14)                               /*!< MFCOM reset */
#define RCU_AHBRST_PARST                BIT(17)                               /*!< GPIO port A reset */
#define RCU_AHBRST_PBRST                BIT(18)                               /*!< GPIO port B reset */
#define RCU_AHBRST_PCRST                BIT(19)                               /*!< GPIO port C reset */
#define RCU_AHBRST_PDRST                BIT(20)                               /*!< GPIO port D reset */
#define RCU_AHBRST_PERST                BIT(21)                               /*!< GPIO port E reset */
#define RCU_AHBRST_PFRST                BIT(22)                               /*!< GPIO port F reset */

/* RCU_CFG1 */
#define RCU_CFG1_PREDV                  BITS(0,3)                             /*!< CK_HXTAL divider previous PLL */

/* RCU_CFG2 */
#define RCU_CFG2_USART0SEL              BITS(0,1)                             /*!< USART0 clock source selection */
#define RCU_CFG2_USART1SEL              BITS(4,5)                             /*!< USART1 clock source selection */
#define RCU_CFG2_USART2SEL              BITS(6,7)                             /*!< USART2 clock source selection */
#define RCU_CFG2_CAN0SEL                BITS(12,13)                           /*!< CAN0 clock source selection */
#define RCU_CFG2_CAN1SEL                BITS(14,15)                           /*!< CAN1 clock source selection */
#define RCU_CFG2_ADCPSC                 BITS(27,31)                           /*!< ADC prescaler selection */

/* RCU_VKEY */
#define RCU_VKEY_UNLOCK                 0x1A2B3C4DU                           /*!< the key of RCU_DSV register */
/* RCU_VKEY */
#define RCU_VKEY_KEY                    BITS(0,31)
/* RCU_DSV */
#define RCU_DSV_DSLPVS                  BITS(0,1)                             /*!< Deep-sleep mode voltage selection */

/* constants definitions */
/* define the peripheral clock enable bit position and its register index offset */
#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define RCU_REG_VAL(periph)                 (REG32(RCU + ((uint32_t)(periph) >> 6)))
#define RCU_BIT_POS(val)                    ((uint32_t)(val) & 0x0000001FU)

/* register offset */
/* peripherals enable */
#define AHBEN_REG_OFFSET                0x00000014U                          /*!< AHB enable register offset */
#define APB1EN_REG_OFFSET               0x0000001CU                          /*!< APB1 enable register offset */
#define APB2EN_REG_OFFSET               0x00000018U                          /*!< APB2 enable register offset */

/* peripherals reset */
#define AHBRST_REG_OFFSET               0x00000028U                          /*!< AHB reset register offset */
#define APB1RST_REG_OFFSET              0x00000010U                          /*!< APB1 reset register offset */
#define APB2RST_REG_OFFSET              0x0000000CU                          /*!< APB2 reset register offset */
#define RSTSCK_REG_OFFSET               0x00000024U                          /*!< reset source/clock register offset */

/* clock control */
#define CTL_REG_OFFSET                  0x00000000U                          /*!< control register offset */
#define BDCTL_REG_OFFSET                0x00000020U                          /*!< backup domain control register offset */

/* clock stabilization and stuck interrupt */
#define INT_REG_OFFSET                  0x00000008U                          /*!< clock interrupt register offset */

/* configuration register */
#define CFG0_REG_OFFSET                 0x00000004U                          /*!< clock configuration register 0 offset */
#define CFG1_REG_OFFSET                 0x0000002CU                          /*!< clock configuration register 1 offset */

/* peripheral clock enable */
typedef enum {
    /* AHB peripherals */
    RCU_DMA0      = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 0U),                    /*!< DMA0 clock */
    RCU_DMA1      = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 1U),                    /*!< DMA1 clock */
    RCU_DMAMUX    = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 3U),                    /*!< DMAMUX clock */
    RCU_CRC       = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 6U),                    /*!< CRC clock */
    RCU_MFCOM     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 14U),                   /*!< MFCOM clock */
    RCU_GPIOA     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 17U),                   /*!< GPIOA clock */
    RCU_GPIOB     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 18U),                   /*!< GPIOB clock */
    RCU_GPIOC     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 19U),                   /*!< GPIOC clock */
    RCU_GPIOD     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 20U),                   /*!< GPIOD clock */
    RCU_GPIOE     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 21U),                   /*!< GPIOE clock */
    RCU_GPIOF     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 22U),                   /*!< GPIOF clock */
    /* APB2 peripherals */
    RCU_SYSCFG    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 0U),                   /*!< SYSCFG clock */
    RCU_CMP       = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 1U),                   /*!< CMP clock */
    RCU_ADC0      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 9U),                   /*!< ADC0 clock */
    RCU_ADC1      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 10U),                  /*!< ADC1 clock */
    RCU_TIMER0    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 11U),                  /*!< TIMER0 clock */
    RCU_SPI0      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 12U),                  /*!< SPI0 clock */
    RCU_TIMER7    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 13U),                  /*!< TIMER7 clock */
    RCU_USART0    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 14U),                  /*!< USART0 clock */
    RCU_TIMER19   = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 20U),                  /*!< TIMER19 clock */
    RCU_TIMER20   = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 21U),                  /*!< TIMER20 clock */
    RCU_TRIGSEL   = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 29U),                  /*!< TRIGSEL clock */
    RCU_CAN0      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 30U),                  /*!< CAN0 clock */
    RCU_CAN1      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 31U),                  /*!< CAN1 clock */
    /* APB1 peripherals */
    RCU_TIMER1    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 0U),                   /*!< TIMER1 clock */
    RCU_TIMER5    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 4U),                   /*!< TIMER5 clock */
    RCU_TIMER6    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 5U),                   /*!< TIMER6 clock */
    RCU_WWDGT     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 11U),                  /*!< WWDGT clock */
    RCU_SPI1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 14U),                  /*!< SPI1 clock */
    RCU_USART1    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 17U),                  /*!< USART1 clock */
    RCU_USART2    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 18U),                  /*!< USART2 clock */
    RCU_I2C0      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 21U),                  /*!< I2C0 clock */
    RCU_I2C1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 22U),                  /*!< I2C1 clock */
    RCU_BKP       = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 27U),                  /*!< BKP clock */
    RCU_PMU       = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 28U),                  /*!< PMU clock */
    RCU_DAC       = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 29U),                  /*!< DAC clock */
    RCU_RTC       = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 15U),                   /*!< RTC clock */
} rcu_periph_enum;


/* peripherals reset */
typedef enum {
    /* AHB peripherals */
    RCU_DMA0RST     = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 0U),                 /*!< DMA0 clock reset */
    RCU_DMA1RST     = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 1U),                 /*!< DMA1 clock reset */
    RCU_DMAMUXRST   = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 3U),                 /*!< DMAMUX clock reset */
    RCU_CRCRST      = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 6U),                 /*!< CRC clock reset */
    RCU_MFCOMRST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 14U),                /*!< MFCOM clock reset */
    RCU_GPIOARST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 17U),                /*!< GPIOA clock reset */
    RCU_GPIOBRST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 18U),                /*!< GPIOB clock reset */
    RCU_GPIOCRST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 19U),                /*!< GPIOC clock reset */
    RCU_GPIODRST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 20U),                /*!< GPIOD clock reset */
    RCU_GPIOERST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 21U),                /*!< GPIOE clock reset */
    RCU_GPIOFRST    = RCU_REGIDX_BIT(AHBRST_REG_OFFSET, 22U),                /*!< GPIOF clock reset */
    /* APB2 peripherals */
    RCU_SYSCFGRST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 0U),                /*!< system configuration reset */
    RCU_CMPRST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 1U),                /*!< Comparator reset */
    RCU_ADC0RST     = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 9U),                /*!< ADC0 clock reset */
    RCU_ADC1RST     = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 10U),               /*!< ADC1 clock reset */
    RCU_TIMER0RST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 11U),               /*!< TIMER0 clock reset */
    RCU_SPI0RST     = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 12U),               /*!< SPI0 clock reset */
    RCU_TIMER7RST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 13U),               /*!< TIMER7 clock reset */
    RCU_USART0RST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 14U),               /*!< USART0 clock reset */
    RCU_TIMER19RST  = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 20U),               /*!< TIMER19 clock reset */
    RCU_TIMER20RST  = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 21U),               /*!< TIMER20 clock reset */
    RCU_CAN0RST     = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 30U),               /*!< CAN0 clock reset */
    RCU_CAN1RST     = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 31U),               /*!< CAN1 clock reset */
    /* APB1 peripherals */
    RCU_TIMER1RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 0U),                /*!< TIMER1 clock reset */
    RCU_TIMER5RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 4U),                /*!< TIMER5 clock reset */
    RCU_TIMER6RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 5U),                /*!< TIMER6 clock reset */
    RCU_WWDGTRST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 11U),               /*!< WWDGT clock reset */
    RCU_SPI1RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 14U),               /*!< SPI1 clock reset */
    RCU_USART1RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 17U),               /*!< USART1 clock reset */
    RCU_USART2RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 18U),               /*!< USART2 clock reset */
    RCU_I2C0RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 21U),               /*!< I2C0 clock reset */
    RCU_I2C1RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 22U),               /*!< I2C1 clock reset */
    RCU_PMURST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 28U),               /*!< PMU clock reset */
    RCU_DACRST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 29U),               /*!< DAC clock reset */
} rcu_periph_reset_enum;

/* peripheral clock enable when sleep mode*/
typedef enum {
    /* AHB peripherals */
    RCU_SRAM_SLP     = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 2U),                 /*!< SRAM clock */
    RCU_FMC_SLP      = RCU_REGIDX_BIT(AHBEN_REG_OFFSET, 4U),                 /*!< FMC clock */
} rcu_periph_sleep_enum;

/* clock stabilization and peripheral reset flags */
typedef enum {
    /* clock stabilization flags */
    RCU_FLAG_IRC8MSTB      = RCU_REGIDX_BIT(CTL_REG_OFFSET, 1U),             /*!< IRC8M stabilization flag */
    RCU_FLAG_HXTALSTB      = RCU_REGIDX_BIT(CTL_REG_OFFSET, 17U),            /*!< HXTAL stabilization flag */
    RCU_FLAG_PLLSTB        = RCU_REGIDX_BIT(CTL_REG_OFFSET, 25U),            /*!< PLL stabilization flag */
    RCU_FLAG_LXTALSTB      = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 1U),           /*!< LXTAL stabilization flag */
    /* reset source flags */
    RCU_FLAG_IRC40KSTB     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 1U),          /*!< IRC40K stabilization flag */
    RCU_FLAG_BORRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 17U),         /*!< BOR reset flag */
    RCU_FLAG_LOCKUPRST     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 18U),         /*!< CPU LOCK UP error reset flag */
    RCU_FLAG_LVDRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 19U),         /*!< low voltage detect error reset flag */
    RCU_FLAG_ECCRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 20U),         /*!< 2 bits ECC error reset flag */
    RCU_FLAG_LOHRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 21U),         /*!< lost of HXTAL error reset flag */
    RCU_FLAG_LOPRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 22U),         /*!< lost of PLL error reset flag */
    RCU_FLAG_V11RST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 23U),         /*!< 1.1V domain Power reset flag */
    RCU_FLAG_OBLRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 25U),         /*!< option byte loader reset flag */
    RCU_FLAG_EPRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 26U),         /*!< external PIN reset flag */
    RCU_FLAG_PORRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 27U),         /*!< power reset flag */
    RCU_FLAG_SWRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 28U),         /*!< software reset flag */
    RCU_FLAG_FWDGTRST      = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 29U),         /*!< FWDGT reset flag */
    RCU_FLAG_WWDGTRST      = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 30U),         /*!< WWDGT reset flag */
    RCU_FLAG_LPRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 31U),         /*!< low-power reset flag */
} rcu_flag_enum;

/* clock stabilization and ckm interrupt flags */
typedef enum {
    RCU_INT_FLAG_IRC40KSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 0U),             /*!< IRC40K stabilization interrupt flag */
    RCU_INT_FLAG_LXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 1U),             /*!< LXTAL stabilization interrupt flag */
    RCU_INT_FLAG_IRC8MSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 2U),             /*!< IRC8M stabilization interrupt flag */
    RCU_INT_FLAG_HXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 3U),             /*!< HXTAL stabilization interrupt flag */
    RCU_INT_FLAG_PLLSTB    = RCU_REGIDX_BIT(INT_REG_OFFSET, 4U),             /*!< PLL stabilization interrupt flag */
    RCU_INT_FLAG_LCKM      = RCU_REGIDX_BIT(INT_REG_OFFSET, 5U),             /*!< LXTAL clock monitor interrupt flag */
    RCU_INT_FLAG_PLLM      = RCU_REGIDX_BIT(INT_REG_OFFSET, 6U),             /*!< PLL clock monitor interrupt flag */
    RCU_INT_FLAG_CKM       = RCU_REGIDX_BIT(INT_REG_OFFSET, 7U),             /*!< HXTAL clock stuck interrupt flag */
} rcu_int_flag_enum;

/* clock stabilization and stuck interrupt flags clear */
typedef enum {
    RCU_INT_FLAG_IRC40KSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 16U),        /*!< IRC40K stabilization interrupt flag clear */
    RCU_INT_FLAG_LXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 17U),        /*!< LXTAL stabilization interrupt flag clear */
    RCU_INT_FLAG_IRC8MSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 18U),        /*!< IRC8M stabilization interrupt flag clear */
    RCU_INT_FLAG_HXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 19U),        /*!< HXTAL stabilization interrupt flag clear */
    RCU_INT_FLAG_PLLSTB_CLR    = RCU_REGIDX_BIT(INT_REG_OFFSET, 20U),        /*!< PLL stabilization interrupt flag clear */
    RCU_INT_FLAG_LCKM_CLR      = RCU_REGIDX_BIT(INT_REG_OFFSET, 21U),        /*!< LXTAL clock monitor interrupt clear */
    RCU_INT_FLAG_PLLM_CLR      = RCU_REGIDX_BIT(INT_REG_OFFSET, 22U),        /*!< PLL clock monitor interrupt clear */
    RCU_INT_FLAG_CKM_CLR       = RCU_REGIDX_BIT(INT_REG_OFFSET, 23U),        /*!< CKM interrupt flag clear */
} rcu_int_flag_clear_enum;

/* clock stabilization interrupt enable or disable */
typedef enum {
    RCU_INT_IRC40KSTB          = RCU_REGIDX_BIT(INT_REG_OFFSET, 8U),         /*!< IRC40K stabilization interrupt enable */
    RCU_INT_LXTALSTB           = RCU_REGIDX_BIT(INT_REG_OFFSET, 9U),         /*!< LXTAL stabilization interrupt enable */
    RCU_INT_IRC8MSTB           = RCU_REGIDX_BIT(INT_REG_OFFSET, 10U),        /*!< IRC8M stabilization interrupt enable */
    RCU_INT_HXTALSTB           = RCU_REGIDX_BIT(INT_REG_OFFSET, 11U),        /*!< HXTAL stabilization interrupt enable */
    RCU_INT_PLLSTB             = RCU_REGIDX_BIT(INT_REG_OFFSET, 12U),        /*!< PLL stabilization interrupt enable */
    RCU_INT_LCKM               = RCU_REGIDX_BIT(INT_REG_OFFSET, 13U),        /*!< LXTAL clock monitor interrupt enable */
    RCU_INT_PLLM               = RCU_REGIDX_BIT(INT_REG_OFFSET, 14U),        /*!< PLL clock monitor interrupt enable */
} rcu_int_enum;

/* oscillator types */
typedef enum {
    RCU_HXTAL                  = RCU_REGIDX_BIT(CTL_REG_OFFSET, 16U),        /*!< HXTAL */
    RCU_LXTAL                  = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 0U),       /*!< LXTAL */
    RCU_IRC8M                  = RCU_REGIDX_BIT(CTL_REG_OFFSET, 0U),         /*!< IRC8M */
    RCU_IRC40K                 = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 0U),      /*!< IRC40K */
    RCU_PLL_CK                 = RCU_REGIDX_BIT(CTL_REG_OFFSET, 24U),        /*!< PLL */
} rcu_osci_type_enum;

/* rcu clock frequency */
typedef enum {
    CK_SYS      = 0,                                                         /*!< system clock */
    CK_AHB,                                                                  /*!< AHB clock */
    CK_APB1,                                                                 /*!< APB1 clock */
    CK_APB2,                                                                 /*!< APB2 clock */
    CK_USART0,                                                               /*!< USART0 clock */
    CK_USART1,                                                               /*!< USART1 clock */
    CK_USART2,                                                               /*!< USART2 clock */
} rcu_clock_freq_enum;

/* HXTAL frequency scale select */
#define HXTAL_SCALE_2M_TO_8M            0x00000000U                          /*!< HXTAL scale is 2M-8MHz */
#define HXTAL_SCALE_8M_TO_40M           RCU_CTL_HXTALSCAL                    /*!< HXTAL scale is 8M-40MHz */

/* RCU_CFG0 register bit define */
/* system clock source select */
#define CFG0_SCS(regval)                (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_CKSYSSRC_IRC8M              CFG0_SCS(0)                          /*!< system clock source select IRC8M */
#define RCU_CKSYSSRC_HXTAL              CFG0_SCS(1)                          /*!< system clock source select HXTAL */
#define RCU_CKSYSSRC_PLL                CFG0_SCS(2)                          /*!< system clock source select PLL */

/* system clock source select status */
#define CFG0_SCSS(regval)               (BITS(2,3) & ((uint32_t)(regval) << 2))
#define RCU_SCSS_IRC8M                  CFG0_SCSS(0)                         /*!< system clock source select IRC8M */
#define RCU_SCSS_HXTAL                  CFG0_SCSS(1)                         /*!< system clock source select HXTAL */
#define RCU_SCSS_PLL                    CFG0_SCSS(2)                         /*!< system clock source select PLL */

/* AHB prescaler selection */
#define CFG0_AHBPSC(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))
#define RCU_AHB_CKSYS_DIV1              CFG0_AHBPSC(0)                       /*!< AHB prescaler select CK_SYS */
#define RCU_AHB_CKSYS_DIV2              CFG0_AHBPSC(8)                       /*!< AHB prescaler select CK_SYS/2 */
#define RCU_AHB_CKSYS_DIV4              CFG0_AHBPSC(9)                       /*!< AHB prescaler select CK_SYS/4 */
#define RCU_AHB_CKSYS_DIV8              CFG0_AHBPSC(10)                      /*!< AHB prescaler select CK_SYS/8 */
#define RCU_AHB_CKSYS_DIV16             CFG0_AHBPSC(11)                      /*!< AHB prescaler select CK_SYS/16 */
#define RCU_AHB_CKSYS_DIV64             CFG0_AHBPSC(12)                      /*!< AHB prescaler select CK_SYS/64 */
#define RCU_AHB_CKSYS_DIV128            CFG0_AHBPSC(13)                      /*!< AHB prescaler select CK_SYS/128 */
#define RCU_AHB_CKSYS_DIV256            CFG0_AHBPSC(14)                      /*!< AHB prescaler select CK_SYS/256 */
#define RCU_AHB_CKSYS_DIV512            CFG0_AHBPSC(15)                      /*!< AHB prescaler select CK_SYS/512 */

/* APB1 prescaler selection */
#define CFG0_APB1PSC(regval)            (BITS(8,10) & ((uint32_t)(regval) << 8))
#define RCU_APB1_CKAHB_DIV1             CFG0_APB1PSC(0)                      /*!< APB1 prescaler select CK_AHB */
#define RCU_APB1_CKAHB_DIV2             CFG0_APB1PSC(4)                      /*!< APB1 prescaler select CK_AHB/2 */
#define RCU_APB1_CKAHB_DIV4             CFG0_APB1PSC(5)                      /*!< APB1 prescaler select CK_AHB/4 */
#define RCU_APB1_CKAHB_DIV8             CFG0_APB1PSC(6)                      /*!< APB1 prescaler select CK_AHB/8 */
#define RCU_APB1_CKAHB_DIV16            CFG0_APB1PSC(7)                      /*!< APB1 prescaler select CK_AHB/16 */

/* APB2 prescaler selection */
#define CFG0_APB2PSC(regval)            (BITS(11,13) & ((uint32_t)(regval) << 11))
#define RCU_APB2_CKAHB_DIV1             CFG0_APB2PSC(0)                      /*!< APB2 prescaler select CK_AHB */
#define RCU_APB2_CKAHB_DIV2             CFG0_APB2PSC(4)                      /*!< APB2 prescaler select CK_AHB/2 */
#define RCU_APB2_CKAHB_DIV4             CFG0_APB2PSC(5)                      /*!< APB2 prescaler select CK_AHB/4 */
#define RCU_APB2_CKAHB_DIV8             CFG0_APB2PSC(6)                      /*!< APB2 prescaler select CK_AHB/8 */
#define RCU_APB2_CKAHB_DIV16            CFG0_APB2PSC(7)                      /*!< APB2 prescaler select CK_AHB/16 */

/* RCU system reset */
#define RCU_SYSRST_LOCKUP               RCU_RSTSCK_LOCKUPRSTEN               /*!< CPU lock-up reset */
#define RCU_SYSRST_LVD                  RCU_RSTSCK_LVDRSTEN                  /*!< low voltage detect reset */
#define RCU_SYSRST_ECC                  RCU_RSTSCK_ECCRSTEN                  /*!< ECC 2 bits error reset */
#define RCU_SYSRST_LOH                  RCU_RSTSCK_LOHRSTEN                  /*!< lost of HXTAL error reset */
#define RCU_SYSRST_LOP                  RCU_RSTSCK_LOPRSTEN                  /*!< lost of PLL reset */

/* ADC prescaler select */
#define CFG2_ADCPSC(regval)             (BITS(27,31) & ((uint32_t)(regval) << 27))
#define RCU_CKADC_CKAHB_DIV2            CFG2_ADCPSC(0)                       /*!< ADC prescaler select CK_AHB/2 */
#define RCU_CKADC_CKAHB_DIV3            CFG2_ADCPSC(1)                       /*!< ADC prescaler select CK_AHB/3 */
#define RCU_CKADC_CKAHB_DIV4            CFG2_ADCPSC(2)                       /*!< ADC prescaler select CK_AHB/4 */
#define RCU_CKADC_CKAHB_DIV5            CFG2_ADCPSC(3)                       /*!< ADC prescaler select CK_AHB/5 */
#define RCU_CKADC_CKAHB_DIV6            CFG2_ADCPSC(4)                       /*!< ADC prescaler select CK_AHB/6 */
#define RCU_CKADC_CKAHB_DIV7            CFG2_ADCPSC(5)                       /*!< ADC prescaler select CK_AHB/7 */
#define RCU_CKADC_CKAHB_DIV8            CFG2_ADCPSC(6)                       /*!< ADC prescaler select CK_AHB/8 */
#define RCU_CKADC_CKAHB_DIV9            CFG2_ADCPSC(7)                       /*!< ADC prescaler select CK_AHB/9 */
#define RCU_CKADC_CKAHB_DIV10           CFG2_ADCPSC(8)                       /*!< ADC prescaler select CK_AHB/10 */
#define RCU_CKADC_CKAHB_DIV11           CFG2_ADCPSC(9)                       /*!< ADC prescaler select CK_AHB/11 */
#define RCU_CKADC_CKAHB_DIV12           CFG2_ADCPSC(10)                      /*!< ADC prescaler select CK_AHB/12 */
#define RCU_CKADC_CKAHB_DIV13           CFG2_ADCPSC(11)                      /*!< ADC prescaler select CK_AHB/13 */
#define RCU_CKADC_CKAHB_DIV14           CFG2_ADCPSC(12)                      /*!< ADC prescaler select CK_AHB/14 */
#define RCU_CKADC_CKAHB_DIV15           CFG2_ADCPSC(13)                      /*!< ADC prescaler select CK_AHB/15 */
#define RCU_CKADC_CKAHB_DIV16           CFG2_ADCPSC(14)                      /*!< ADC prescaler select CK_AHB/16 */
#define RCU_CKADC_CKAHB_DIV17           CFG2_ADCPSC(15)                      /*!< ADC prescaler select CK_AHB/17 */
#define RCU_CKADC_CKAHB_DIV18           CFG2_ADCPSC(16)                      /*!< ADC prescaler select CK_AHB/18 */
#define RCU_CKADC_CKAHB_DIV19           CFG2_ADCPSC(17)                      /*!< ADC prescaler select CK_AHB/19 */
#define RCU_CKADC_CKAHB_DIV20           CFG2_ADCPSC(18)                      /*!< ADC prescaler select CK_AHB/20 */
#define RCU_CKADC_CKAHB_DIV21           CFG2_ADCPSC(19)                      /*!< ADC prescaler select CK_AHB/21 */
#define RCU_CKADC_CKAHB_DIV22           CFG2_ADCPSC(20)                      /*!< ADC prescaler select CK_AHB/22 */
#define RCU_CKADC_CKAHB_DIV23           CFG2_ADCPSC(21)                      /*!< ADC prescaler select CK_AHB/23 */
#define RCU_CKADC_CKAHB_DIV24           CFG2_ADCPSC(22)                      /*!< ADC prescaler select CK_AHB/24 */
#define RCU_CKADC_CKAHB_DIV25           CFG2_ADCPSC(23)                      /*!< ADC prescaler select CK_AHB/25 */
#define RCU_CKADC_CKAHB_DIV26           CFG2_ADCPSC(24)                      /*!< ADC prescaler select CK_AHB/26 */
#define RCU_CKADC_CKAHB_DIV27           CFG2_ADCPSC(25)                      /*!< ADC prescaler select CK_AHB/27 */
#define RCU_CKADC_CKAHB_DIV28           CFG2_ADCPSC(26)                      /*!< ADC prescaler select CK_AHB/28 */
#define RCU_CKADC_CKAHB_DIV29           CFG2_ADCPSC(27)                      /*!< ADC prescaler select CK_AHB/29 */
#define RCU_CKADC_CKAHB_DIV30           CFG2_ADCPSC(28)                      /*!< ADC prescaler select CK_AHB/30 */
#define RCU_CKADC_CKAHB_DIV31           CFG2_ADCPSC(29)                      /*!< ADC prescaler select CK_AHB/31 */
#define RCU_CKADC_CKAHB_DIV32           CFG2_ADCPSC(30)                      /*!< ADC prescaler select CK_AHB/32 */

/* PLL clock source selection */
#define RCU_PLLSRC_IRC8M_DIV2           ((uint32_t)0x00000000U)              /*!< IRC8M/2 clock is selected as clock source of PLL */
#define RCU_PLLSRC_HXTAL                RCU_CFG0_PLLSEL                      /*!< HXTAL is selected as clock source of PLL */

/* PLL clock multiplication factor */
#define PLLMF_4                         RCU_CFG0_PLLMF_4                     /*!< bit 4 of PLLMF */
#define CFG0_PLLMF(regval)              (BITS(18,21) & ((uint32_t)(regval) << 18))
#define RCU_PLL_MUL2                    CFG0_PLLMF(0)                        /*!< PLL clock source multiply by 2 */
#define RCU_PLL_MUL3                    CFG0_PLLMF(1)                        /*!< PLL clock source multiply by 3 */
#define RCU_PLL_MUL4                    CFG0_PLLMF(2)                        /*!< PLL clock source multiply by 4 */
#define RCU_PLL_MUL5                    CFG0_PLLMF(3)                        /*!< PLL clock source multiply by 5 */
#define RCU_PLL_MUL6                    CFG0_PLLMF(4)                        /*!< PLL clock source multiply by 6 */
#define RCU_PLL_MUL7                    CFG0_PLLMF(5)                        /*!< PLL clock source multiply by 7 */
#define RCU_PLL_MUL8                    CFG0_PLLMF(6)                        /*!< PLL clock source multiply by 8 */
#define RCU_PLL_MUL9                    CFG0_PLLMF(7)                        /*!< PLL clock source multiply by 9 */
#define RCU_PLL_MUL10                   CFG0_PLLMF(8)                        /*!< PLL clock source multiply by 10 */
#define RCU_PLL_MUL11                   CFG0_PLLMF(9)                        /*!< PLL clock source multiply by 11 */
#define RCU_PLL_MUL12                   CFG0_PLLMF(10)                       /*!< PLL clock source multiply by 12 */
#define RCU_PLL_MUL13                   CFG0_PLLMF(11)                       /*!< PLL clock source multiply by 13 */
#define RCU_PLL_MUL14                   CFG0_PLLMF(12)                       /*!< PLL clock source multiply by 14 */
#define RCU_PLL_MUL15                   CFG0_PLLMF(13)                       /*!< PLL clock source multiply by 15 */
#define RCU_PLL_MUL16                   CFG0_PLLMF(14)                       /*!< PLL clock source multiply by 16 */
#define RCU_PLL_MUL17                   (PLLMF_4 | CFG0_PLLMF(0))            /*!< PLL clock source multiply by 17 */
#define RCU_PLL_MUL18                   (PLLMF_4 | CFG0_PLLMF(1))            /*!< PLL clock source multiply by 18 */
#define RCU_PLL_MUL19                   (PLLMF_4 | CFG0_PLLMF(2))            /*!< PLL clock source multiply by 19 */
#define RCU_PLL_MUL20                   (PLLMF_4 | CFG0_PLLMF(3))            /*!< PLL clock source multiply by 20 */
#define RCU_PLL_MUL21                   (PLLMF_4 | CFG0_PLLMF(4))            /*!< PLL clock source multiply by 21 */
#define RCU_PLL_MUL22                   (PLLMF_4 | CFG0_PLLMF(5))            /*!< PLL clock source multiply by 22 */
#define RCU_PLL_MUL23                   (PLLMF_4 | CFG0_PLLMF(6))            /*!< PLL clock source multiply by 23 */
#define RCU_PLL_MUL24                   (PLLMF_4 | CFG0_PLLMF(7))            /*!< PLL clock source multiply by 24 */
#define RCU_PLL_MUL25                   (PLLMF_4 | CFG0_PLLMF(8))            /*!< PLL clock source multiply by 25 */
#define RCU_PLL_MUL26                   (PLLMF_4 | CFG0_PLLMF(9))            /*!< PLL clock source multiply by 26 */
#define RCU_PLL_MUL27                   (PLLMF_4 | CFG0_PLLMF(10))           /*!< PLL clock source multiply by 27 */
#define RCU_PLL_MUL28                   (PLLMF_4 | CFG0_PLLMF(11))           /*!< PLL clock source multiply by 28 */
#define RCU_PLL_MUL29                   (PLLMF_4 | CFG0_PLLMF(12))           /*!< PLL clock source multiply by 29 */
#define RCU_PLL_MUL30                   (PLLMF_4 | CFG0_PLLMF(13))           /*!< PLL clock source multiply by 30 */
#define RCU_PLL_MUL31                   (PLLMF_4 | CFG0_PLLMF(14))           /*!< PLL clock source multiply by 31 */

/* CKOUT Clock source selection */
#define CFG0_CKOUTSEL(regval)           (BITS(24,26) & ((uint32_t)(regval) << 24))
#define RCU_CKOUTSRC_NONE               CFG0_CKOUTSEL(0)                     /*!< no clock is selected */
#define RCU_CKOUTSRC_IRC40K             CFG0_CKOUTSEL(2)                     /*!< IRC40K is selected as CK_OUT clock source */
#define RCU_CKOUTSRC_LXTAL              CFG0_CKOUTSEL(3)                     /*!< LXTAL is selected as CK_OUT clock source */
#define RCU_CKOUTSRC_CKSYS              CFG0_CKOUTSEL(4)                     /*!< system clock is selected as CK_OUT clock source */
#define RCU_CKOUTSRC_IRC8M              CFG0_CKOUTSEL(5)                     /*!< IRC8M is selected as CK_OUT clock source */
#define RCU_CKOUTSRC_HXTAL              CFG0_CKOUTSEL(6)                     /*!< HXTAL is selected as CK_OUT clock source */
#define RCU_CKOUTSRC_CKPLL_DIV1         (RCU_CFG0_PLLDV | CFG0_CKOUTSEL(7))  /*!< CK_PLL is selected as CK_OUT clock source */
#define RCU_CKOUTSRC_CKPLL_DIV2         CFG0_CKOUTSEL(7)                     /*!< CK_PLL/2 is selected as CK_OUT clock source */

/* CK_OUT divider */
#define CFG0_CKOUTDIV(regval)           (BITS(28,30) & ((uint32_t)(regval) << 28))
#define RCU_CKOUT_DIV1                  CFG0_CKOUTDIV(0)                     /*!< CK_OUT is divided by 1 */
#define RCU_CKOUT_DIV2                  CFG0_CKOUTDIV(1)                     /*!< CK_OUT is divided by 2 */
#define RCU_CKOUT_DIV4                  CFG0_CKOUTDIV(2)                     /*!< CK_OUT is divided by 4 */
#define RCU_CKOUT_DIV8                  CFG0_CKOUTDIV(3)                     /*!< CK_OUT is divided by 8 */
#define RCU_CKOUT_DIV16                 CFG0_CKOUTDIV(4)                     /*!< CK_OUT is divided by 16 */
#define RCU_CKOUT_DIV32                 CFG0_CKOUTDIV(5)                     /*!< CK_OUT is divided by 32 */
#define RCU_CKOUT_DIV64                 CFG0_CKOUTDIV(6)                     /*!< CK_OUT is divided by 64 */
#define RCU_CKOUT_DIV128                CFG0_CKOUTDIV(7)                     /*!< CK_OUT is divided by 128 */

/* LXTAL drive capability */
#define BDCTL_LXTALDRI(regval)          (BITS(3,4) & ((uint32_t)(regval) << 3))
#define RCU_LXTAL_LOWDRI                BDCTL_LXTALDRI(0)                    /*!< lower driving capability */
#define RCU_LXTAL_MED_LOWDRI            BDCTL_LXTALDRI(1)                    /*!< medium low driving capability */
#define RCU_LXTAL_MED_HIGHDRI           BDCTL_LXTALDRI(2)                    /*!< medium high driving capability */
#define RCU_LXTAL_HIGHDRI               BDCTL_LXTALDRI(3)                    /*!< higher driving capability */

/* RTC clock entry selection */
#define BDCTL_RTCSRC(regval)            (BITS(8,9) & ((uint32_t)(regval) << 8))
#define RCU_RTCSRC_NONE                 BDCTL_RTCSRC(0)                     /*!< no clock is selected */
#define RCU_RTCSRC_LXTAL                BDCTL_RTCSRC(1)                     /*!< LXTAL is selected as RTC clock source */
#define RCU_RTCSRC_IRC40K               BDCTL_RTCSRC(2)                     /*!< IRC40K is selected as RTC clock source */
#define RCU_RTCSRC_HXTAL_DIV_128        BDCTL_RTCSRC(3)                     /*!< HXTAL/128 is selected as RTC clock source */

/* PREDV0 division factor */
#define CFG1_PREDV(regval)              (BITS(0,3) & ((uint32_t)(regval) << 0))
#define RCU_PREDV_DIV1                  CFG1_PREDV(0)                        /*!< PREDV input clock source not divided */
#define RCU_PREDV_DIV2                  CFG1_PREDV(1)                        /*!< PREDV input clock source divided by 2 */
#define RCU_PREDV_DIV3                  CFG1_PREDV(2)                        /*!< PREDV input clock source divided by 3 */
#define RCU_PREDV_DIV4                  CFG1_PREDV(3)                        /*!< PREDV input clock source divided by 4 */
#define RCU_PREDV_DIV5                  CFG1_PREDV(4)                        /*!< PREDV input clock source divided by 5 */
#define RCU_PREDV_DIV6                  CFG1_PREDV(5)                        /*!< PREDV input clock source divided by 6 */
#define RCU_PREDV_DIV7                  CFG1_PREDV(6)                        /*!< PREDV input clock source divided by 7 */
#define RCU_PREDV_DIV8                  CFG1_PREDV(7)                        /*!< PREDV input clock source divided by 8 */
#define RCU_PREDV_DIV9                  CFG1_PREDV(8)                        /*!< PREDV input clock source divided by 9 */
#define RCU_PREDV_DIV10                 CFG1_PREDV(9)                        /*!< PREDV input clock source divided by 10 */
#define RCU_PREDV_DIV11                 CFG1_PREDV(10)                       /*!< PREDV input clock source divided by 11 */
#define RCU_PREDV_DIV12                 CFG1_PREDV(11)                       /*!< PREDV input clock source divided by 12 */
#define RCU_PREDV_DIV13                 CFG1_PREDV(12)                       /*!< PREDV input clock source divided by 13 */
#define RCU_PREDV_DIV14                 CFG1_PREDV(13)                       /*!< PREDV input clock source divided by 14 */
#define RCU_PREDV_DIV15                 CFG1_PREDV(14)                       /*!< PREDV input clock source divided by 15 */
#define RCU_PREDV_DIV16                 CFG1_PREDV(15)                       /*!< PREDV input clock source divided by 16 */

/* deep-sleep mode voltage */
#define DSV_DSLPVS(regval)              (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_DEEPSLEEP_V_0_8             DSV_DSLPVS(0)                        /*!< core voltage is 0.8V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_0_9             DSV_DSLPVS(1)                        /*!< core voltage is 0.9V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_0             DSV_DSLPVS(2)                        /*!< core voltage is 1.0V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_1             DSV_DSLPVS(3)                        /*!< core voltage is 1.1V in deep-sleep mode */

/* USART clock source selection */
#define CFG2_USART0SRC(regval)          (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_USARTSRC_HXTAL              CFG2_USART0SRC(0)                    /*!< CK_HXTAL is selected as USART clock source */
#define RCU_USARTSRC_CKSYS              CFG2_USART0SRC(1)                    /*!< CK_SYS is selected as USART clock source */
#define RCU_USARTSRC_LXTAL              CFG2_USART0SRC(2)                    /*!< CK_LXTAL is selected as USART clock source */
#define RCU_USARTSRC_IRC8M              CFG2_USART0SRC(3)                    /*!< CK_IRC8M is selected as USART clock source */

/* CAN clock source selection */
#define CFG2_CAN0SRC(regval)            (BITS(12,13) & ((uint32_t)(regval) << 12))
#define RCU_CANSRC_HXTAL                CFG2_CAN0SRC(0)                      /*!< CK_HXTAL is selected as CAN clock source */
#define RCU_CANSRC_PCLK2                CFG2_CAN0SRC(1)                      /*!< PCLK2 is selected as CAN clock source */
#define RCU_CANSRC_PCLK2_DIV_2          CFG2_CAN0SRC(2)                      /*!< PCLK2/2 is selected as CAN clock source */
#define RCU_CANSRC_IRC8M                CFG2_CAN0SRC(3)                      /*!< CK_IRC8M is selected as CAN clock source */


/* function declarations */
/* peripherals clock configure functions */
/* deinitialize the RCU */
void rcu_deinit(void);
/* enable the peripherals clock */
void rcu_periph_clock_enable(rcu_periph_enum periph);
/* disable the peripherals clock */
void rcu_periph_clock_disable(rcu_periph_enum periph);
/* reset the peripherals */
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset);
/* disable reset the peripheral */
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset);
/* enable the peripherals clock when in sleep mode */
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph);
/* disable the peripherals clock when in sleep mode */
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph);
/* reset the BKP domain control register */
void rcu_bkp_reset_enable(void);
/* disable the BKP domain control register reset */
void rcu_bkp_reset_disable(void);

/* system and peripherals clock source, system reset configure functions */
/* configure the system clock source */
void rcu_system_clock_source_config(uint32_t ck_sys);
/* get the system clock source */
uint32_t rcu_system_clock_source_get(void);
/* configure the AHB prescaler selection */
void rcu_ahb_clock_config(uint32_t ck_ahb);
/* configure the APB1 prescaler selection */
void rcu_apb1_clock_config(uint32_t ck_apb1);
/* configure the APB2 prescaler selection */
void rcu_apb2_clock_config(uint32_t ck_apb2);
/* configure the CK_OUT clock source and divider */
void rcu_ckout_config(uint32_t ckout_src, uint32_t ckout_div);
/* configure the PLL clock source selection and PLL multiply factor */
void rcu_pll_config(uint32_t pll_src, uint32_t pll_mul);
/* enable double PLL clock */
void rcu_double_pll_enable(void);
/* disable double PLL clock */
void rcu_double_pll_disable(void);
/* enable RCU system reset */
void rcu_system_reset_enable(uint32_t reset_source);
/* disable RCU system reset */
void rcu_system_reset_disable(uint32_t reset_source);
/* configure the ADC division factor */
void rcu_adc_clock_config(uint32_t adc_psc);
/* configure the RTC clock source selection */
void rcu_rtc_clock_config(uint32_t rtc_clock_source);
/* configure the USART clock source selection */
void rcu_usart_clock_config(uint32_t usart_periph, uint32_t usart_clock_source);
/* configure the CAN clock source selection */
void rcu_can_clock_config(uint32_t can_periph, uint32_t can_clock_source);

/* LXTAL, IRC8M, PLL and other oscillator configure functions */
/* configure the LXTAL drive capability */
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap);
/* wait for oscillator stabilization flags is SET or oscillator startup is timeout */
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci);
/* turn on the oscillator */
void rcu_osci_on(rcu_osci_type_enum osci);
/* turn off the oscillator */
void rcu_osci_off(rcu_osci_type_enum osci);
/* enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it */
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci);
/* disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it */
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci);
/* configure the HXTAL frequency scale select */
void rcu_hxtal_frequency_scale_select(uint32_t hxtal_scal);
/* configure the HXTAL divider used as input of PLL */
void rcu_hxtal_prediv_config(uint32_t hxtal_prediv);
/* set the IRC8M adjust value */
void rcu_irc8m_adjust_value_set(uint32_t irc8m_adjval);

/* clock monitor configure functions */
/* enable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_enable(void);
/* disable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_disable(void);
/* enable the LXTAL clock monitor */
void rcu_lxtal_clock_monitor_enable(void);
/* disable the LXTAL clock monitor */
void rcu_lxtal_clock_monitor_disable(void);
/* enable the PLL clock monitor */
void rcu_pll_clock_monitor_enable(void);
/* disable the PLL clock monitor */
void rcu_pll_clock_monitor_disable(void);

/* voltage configure and clock frequency get functions */
/* unlock the voltage key */
void rcu_voltage_key_unlock(void);
/* set the deep sleep mode voltage */
void rcu_deepsleep_voltage_set(uint32_t dsvol);
/* get the system clock, bus and peripheral clock frequency */
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock);

/* flag & interrupt functions */
/* get the clock stabilization and peripheral reset flags */
FlagStatus rcu_flag_get(rcu_flag_enum flag);
/* clear the reset flag */
void rcu_all_reset_flag_clear(void);
/* get the clock stabilization interrupt and ckm flags */
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag);
/* clear the interrupt flags */
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag);
/* enable the stabilization interrupt */
void rcu_interrupt_enable(rcu_int_enum interrupt);
/* disable the stabilization interrupt */
void rcu_interrupt_disable(rcu_int_enum interrupt);
#endif /* GD32A50X_RCU_H */
