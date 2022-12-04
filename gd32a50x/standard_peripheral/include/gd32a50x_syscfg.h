/*!
   \file    gd32a50x_syscfg.h
   \brief   definitions for the SYSCFG

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

#ifndef GD32A50X_SYSCFG_H
#define GD32A50X_SYSCFG_H

#include "gd32a50x.h"

/* SYSCFG definitions */
#define SYSCFG                                         SYSCFG_BASE

/* registers definitions */
#define SYSCFG_CFG0                                    REG32(SYSCFG + 0x00000000U)                /*!< system configuration register 0 */
#define SYSCFG_CFG1                                    REG32(SYSCFG + 0x00000004U)                /*!< system configuration register 1 */
#define SYSCFG_EXTISS0                                 REG32(SYSCFG + 0x00000008U)                /*!< EXTI sources selection register 0 */
#define SYSCFG_EXTISS1                                 REG32(SYSCFG + 0x0000000CU)                /*!< EXTI sources selection register 1 */
#define SYSCFG_EXTISS2                                 REG32(SYSCFG + 0x00000010U)                /*!< EXTI sources selection register 2 */
#define SYSCFG_EXTISS3                                 REG32(SYSCFG + 0x00000014U)                /*!< EXTI sources selection register 3 */
#define SYSCFG_CFG2                                    REG32(SYSCFG + 0x00000018U)                /*!< system configuration register 2 */
#define SYSCFG_STAT                                    REG32(SYSCFG + 0x0000001CU)                /*!< system status register */
#define SYSCFG_CFG3                                    REG32(SYSCFG + 0x00000028U)                /*!< system configuration register 3 */
#define SYSCFG_TIMERINSEL                              REG32(SYSCFG + 0x0000002CU)                /*!< TIMER input source selection register */

/* bits definitions */
/* SYSCFG_CFG0 */
#define SYSCFG_CFG0_BOOT_MODE                          BITS(0,1)                                  /*!< SYSCFG memory remap configuration */
#define SYSCFG_CFG0_PA9_PA12_RMP                       BIT(4)                                     /*!< PA9 and PA12 remapping bit for small packages (32 pins) */
#define SYSCFG_CFG0_BOOT0_PF0_RMP                      BIT(6)                                     /*!< BOOT0 and PF0 remapping bit */

/* SYSCFG_CFG1 */
#define SYSCFG_CFG1_ADC1CH14RMP                        BIT(28)                                    /*!< ADC1 channel 14 remapping bit */
#define SYSCFG_CFG1_ADC1CH15RMP                        BIT(29)                                    /*!< ADC1 channel 15 remapping bit */
#define SYSCFG_CFG1_ADC0CH8RMP                         BIT(30)                                    /*!< ADC0 channel 8 remapping bit */
#define SYSCFG_CFG1_ADC0CH9RMP                         BIT(31)                                    /*!< ADC0 channel 9 remapping bit */

/* SYSCFG_EXTISS0 */
#define SYSCFG_EXTISS0_EXTI0_SS                        BITS(0,3)                                  /*!< EXTI 0 configuration */
#define SYSCFG_EXTISS0_EXTI1_SS                        BITS(4,7)                                  /*!< EXTI 1 configuration */
#define SYSCFG_EXTISS0_EXTI2_SS                        BITS(8,11)                                 /*!< EXTI 2 configuration */
#define SYSCFG_EXTISS0_EXTI3_SS                        BITS(12,15)                                /*!< EXTI 3 configuration */

/* SYSCFG_EXTISS1 */
#define SYSCFG_EXTISS1_EXTI4_SS                        BITS(0,3)                                  /*!< EXTI 4 configuration */
#define SYSCFG_EXTISS1_EXTI5_SS                        BITS(4,7)                                  /*!< EXTI 5 configuration */
#define SYSCFG_EXTISS1_EXTI6_SS                        BITS(8,11)                                 /*!< EXTI 6 configuration */
#define SYSCFG_EXTISS1_EXTI7_SS                        BITS(12,15)                                /*!< EXTI 7 configuration */

/* SYSCFG_EXTISS2 */
#define SYSCFG_EXTISS2_EXTI8_SS                        BITS(0,3)                                  /*!< EXTI 8 configuration */
#define SYSCFG_EXTISS2_EXTI9_SS                        BITS(4,7)                                  /*!< EXTI 9 configuration */
#define SYSCFG_EXTISS2_EXTI10_SS                       BITS(8,11)                                 /*!< EXTI 10 configuration */
#define SYSCFG_EXTISS2_EXTI11_SS                       BITS(12,15)                                /*!< EXTI 11 configuration */

/* SYSCFG_EXTISS3 */
#define SYSCFG_EXTISS3_EXTI12_SS                       BITS(0,3)                                  /*!< EXTI 12 configuration */
#define SYSCFG_EXTISS3_EXTI13_SS                       BITS(4,7)                                  /*!< EXTI 13 configuration */
#define SYSCFG_EXTISS3_EXTI14_SS                       BITS(8,11)                                 /*!< EXTI 14 configuration */
#define SYSCFG_EXTISS3_EXTI15_SS                       BITS(12,15)                                /*!< EXTI 15 configuration */

/* SSYSCFG_CFG2 */
#define SYSCFG_CFG2_LOCKUP_LOCK                        BIT(0)                                     /*!< Cortex-M33 LOCKUP output lock bit */
#define SYSCFG_CFG2_SRAM_ECC_ERROR_LOCK                BIT(1)                                     /*!< SRAM ECC check error lock bit */
#define SYSCFG_CFG2_LVD_LOCK                           BIT(2)                                     /*!< LVD lock bit */

/* SYSCFG_STAT */
#define SYSCFG_STAT_SRAMECCMEIF                        BIT(0)                                     /*!< SRAM multi-bits non-correction event flag */
#define SYSCFG_STAT_SRAMECCSEIF                        BIT(1)                                     /*!< SRAM single bit correction event flag */
#define SYSCFG_STAT_FLASHECCIF                         BIT(2)                                     /*!< Flash ECC NMI interrupt flag */
#define SYSCFG_STAT_CKMNMIIF                           BIT(3)                                     /*!< HXTAL clock moniotor NMI interrupt flag */
#define SYSCFG_STAT_NMIPINIF                           BIT(4)                                     /*!< NMI interrupt flag from nmi pin */

/* SSYSCFG_CFG3 */
#define SYSCFG_CFG3_SRAMECCMEIE                        BIT(0)                                     /*!< SRAM multi-bits non-correction NMI interrupt enable */
#define SYSCFG_CFG3_SRAMECCSEIE                        BIT(1)                                     /*!< SRAM single bit correction interrupt enable */
#define SYSCFG_CFG3_FLASHECCIE                         BIT(2)                                     /*!< Flash ECC NMI interrupt enable */
#define SYSCFG_CFG3_CKMNMIIE                           BIT(3)                                     /*!< HXTAL clock moniotor NMI interrupt enable */
#define SYSCFG_CFG3_NMIPINIE                           BIT(4)                                     /*!< NMI pin interrupt enable */
#define SYSCFG_CFG3_SRAMECCSERRBITS                    BITS(12,17)                                /*!< which one bit has an SRAM ECC single-bit correctable error */
#define SYSCFG_CFG3_SRAMECCEADDR                       BITS(18,31)                                /*!< record the faulting system address (Address[15:0] >> 2) where the last SRAM ECC event on SRAM occurred. */

/* SYSCFG_TIMERINSEL */
#define SYSCFG_TIMERINSEL_TIMER7_CH0N_SEL              BIT(0)                                     /*!< TIMER7 channel 0 complementary input selection */
#define SYSCFG_TIMERINSEL_TIMER20_BKIN3_SEL            BIT(2)                                     /*!< TIMER20 break input 3 selection */
#define SYSCFG_TIMERINSEL_TIMER20_BKIN2_SEL            BIT(3)                                     /*!< TIMER20 break input 2 selection */
#define SYSCFG_TIMERINSEL_TIMER20_BKIN1_SEL            BIT(4)                                     /*!< TIMER20 break input 1 selection */
#define SYSCFG_TIMERINSEL_TIMER20_BKIN0_SEL            BIT(5)                                     /*!< TIMER20 break input 0 selection */
#define SYSCFG_TIMERINSEL_TIMER19_BKIN3_SEL            BIT(6)                                     /*!< TIMER19 break input 3 selection */
#define SYSCFG_TIMERINSEL_TIMER19_BKIN2_SEL            BIT(7)                                     /*!< TIMER19 break input 2 selection */
#define SYSCFG_TIMERINSEL_TIMER19_BKIN1_SEL            BIT(8)                                     /*!< TIMER19 break input 1 selection */
#define SYSCFG_TIMERINSEL_TIMER19_BKIN0_SEL            BIT(9)                                     /*!< TIMER19 break input 0 selection */
#define SYSCFG_TIMERINSEL_TIMER7_BKIN3_SEL             BIT(14)                                    /*!< TIMER7 break input 3 selection */
#define SYSCFG_TIMERINSEL_TIMER7_BKIN2_SEL             BIT(15)                                    /*!< TIMER7 break input 2 selection */
#define SYSCFG_TIMERINSEL_TIMER7_BKIN1_SEL             BIT(16)                                    /*!< TIMER7 break input 1 selection */
#define SYSCFG_TIMERINSEL_TIMER7_BKIN0_SEL             BIT(17)                                    /*!< TIMER7 break input 0 selection */
#define SYSCFG_TIMERINSEL_TIMER0_BKIN3_SEL             BIT(18)                                    /*!< TIMER0 break input 3 selection */
#define SYSCFG_TIMERINSEL_TIMER0_BKIN2_SEL             BIT(19)                                    /*!< TIMER0 break input 2 selection */
#define SYSCFG_TIMERINSEL_TIMER0_BKIN1_SEL             BIT(20)                                    /*!< TIMER0 break input 1 selection */
#define SYSCFG_TIMERINSEL_TIMER0_BKIN0_SEL             BIT(21)                                    /*!< TIMER0 break input 0 selection */
#define SYSCFG_TIMERINSEL_TIMER20_ETI_SEL              BITS(22,23)                                /*!< TIMER20 external trigger selection */
#define SYSCFG_TIMERINSEL_TIMER19_ETI_SEL              BITS(24,25)                                /*!< TIMER19 external trigger selection */
#define SYSCFG_TIMERINSEL_TIMER7_ETI_SEL               BITS(28,29)                                /*!< TIMER7 external trigger selection */
#define SYSCFG_TIMERINSEL_TIMER0_ETI_SEL               BITS(30,31)                                /*!< TIMER0 external trigger selection */

/* constants definitions */
/* boot mode definitions */
#define SYSCFG_BOOTMODE_FLASH                         ((uint8_t)0x00U)                            /*!< boot from main flash */
#define SYSCFG_BOOTMODE_SYSTEM                        ((uint8_t)0x01U)                            /*!< boot from system flash memory */
#define SYSCFG_BOOTMODE_SRAM                          ((uint8_t)0x03U)                            /*!< boot from embedded SRAM */

/* PA9/PA12 remap definitions */
#define SYSCFG_PA9_PA12_REMAP                         SYSCFG_CFG0_PA9_PA12_RMP                    /*!< PA9/PA12 pins are mapping on PA10/PA11 pins */

/* PF0/BOOT0 remap definitions */
#define SYSCFG_BOOT0_REMAP_PF0                        SYSCFG_CFG0_BOOT0_PF0_RMP                   /*!< PF0 pin is mapping on the BOOT0 pin */

/* EXTI source select definition */
#define EXTISS0                                       ((uint8_t)0x00U)                            /*!< EXTI source select register 0 */
#define EXTISS1                                       ((uint8_t)0x01U)                            /*!< EXTI source select register 1 */
#define EXTISS2                                       ((uint8_t)0x02U)                            /*!< EXTI source select register 2 */
#define EXTISS3                                       ((uint8_t)0x03U)                            /*!< EXTI source select register 3 */

/* EXTI source select mask bits definition */
#define EXTI_SS_MASK                                  BITS(0,3)                                   /*!< EXTI source select mask */

/* EXTI source select jumping step definition */
#define EXTI_SS_JSTEP                                 ((uint8_t)(0x04U))                          /*!< EXTI source select jumping step */

/* EXTI source select moving step definition */
#define EXTI_SS_MSTEP(pin)                            (EXTI_SS_JSTEP * ((pin) % EXTI_SS_JSTEP))   /*!< EXTI source select moving step */

/* EXTI source port definitions */
#define EXTI_SOURCE_GPIOA                             ((uint8_t)0x00U)                            /*!< EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOB                             ((uint8_t)0x01U)                            /*!< EXTI GPIOB configuration */
#define EXTI_SOURCE_GPIOC                             ((uint8_t)0x02U)                            /*!< EXTI GPIOC configuration */
#define EXTI_SOURCE_GPIOD                             ((uint8_t)0x03U)                            /*!< EXTI GPIOD configuration */
#define EXTI_SOURCE_GPIOE                             ((uint8_t)0x04U)                            /*!< EXTI GPIOE configuration */
#define EXTI_SOURCE_GPIOF                             ((uint8_t)0x05U)                            /*!< EXTI GPIOF configuration */

/* EXTI source pin definitions */
#define EXTI_SOURCE_PIN0                              ((uint8_t)0x00U)                            /*!< EXTI GPIO pin0 configuration */
#define EXTI_SOURCE_PIN1                              ((uint8_t)0x01U)                            /*!< EXTI GPIO pin1 configuration */
#define EXTI_SOURCE_PIN2                              ((uint8_t)0x02U)                            /*!< EXTI GPIO pin2 configuration */
#define EXTI_SOURCE_PIN3                              ((uint8_t)0x03U)                            /*!< EXTI GPIO pin3 configuration */
#define EXTI_SOURCE_PIN4                              ((uint8_t)0x04U)                            /*!< EXTI GPIO pin4 configuration */
#define EXTI_SOURCE_PIN5                              ((uint8_t)0x05U)                            /*!< EXTI GPIO pin5 configuration */
#define EXTI_SOURCE_PIN6                              ((uint8_t)0x06U)                            /*!< EXTI GPIO pin6 configuration */
#define EXTI_SOURCE_PIN7                              ((uint8_t)0x07U)                            /*!< EXTI GPIO pin7 configuration */
#define EXTI_SOURCE_PIN8                              ((uint8_t)0x08U)                            /*!< EXTI GPIO pin8 configuration */
#define EXTI_SOURCE_PIN9                              ((uint8_t)0x09U)                            /*!< EXTI GPIO pin9 configuration */
#define EXTI_SOURCE_PIN10                             ((uint8_t)0x0AU)                            /*!< EXTI GPIO pin10 configuration */
#define EXTI_SOURCE_PIN11                             ((uint8_t)0x0BU)                            /*!< EXTI GPIO pin11 configuration */
#define EXTI_SOURCE_PIN12                             ((uint8_t)0x0CU)                            /*!< EXTI GPIO pin12 configuration */
#define EXTI_SOURCE_PIN13                             ((uint8_t)0x0DU)                            /*!< EXTI GPIO pin13 configuration */
#define EXTI_SOURCE_PIN14                             ((uint8_t)0x0EU)                            /*!< EXTI GPIO pin14 configuration */
#define EXTI_SOURCE_PIN15                             ((uint8_t)0x0FU)                            /*!< EXTI GPIO pin15 configuration */

/* lock definitions */
#define SYSCFG_LOCK_LOCKUP                             SYSCFG_CFG2_LOCKUP_LOCK                    /*!< LOCKUP output lock */
#define SYSCFG_LOCK_SRAM_ECC_ERROR                     SYSCFG_CFG2_SRAM_ECC_ERROR_LOCK            /*!< SRAM ECC error lock */
#define SYSCFG_LOCK_LVD                                SYSCFG_CFG2_LVD_LOCK                       /*!< LVD lock */

/* TIMER external trigger definitions */
#define TIMER_ETI_TRG0                                ((uint8_t)0x00U)                             /*!< TIMER external trigger 0 */
#define TIMER_ETI_TRG1                                ((uint8_t)0x01U)                             /*!< TIMER external trigger 1 */
#define TIMER_ETI_TRG2                                ((uint8_t)0x02U)                             /*!< TIMER external trigger 2 */
#define TIMER_ETI_TRG_NONE                            ((uint8_t)0x03U)                             /*!< do not seclet TIMER external trigger source */

/* TIMERx break input y */
#define TIMER20_BKIN3_TRIG                            SYSCFG_TIMERINSEL_TIMER20_BKIN3_SEL          /*!< TIMER20 break input 3 selection */
#define TIMER20_BKIN2_TRIG                            SYSCFG_TIMERINSEL_TIMER20_BKIN2_SEL          /*!< TIMER20 break input 2 selection */
#define TIMER20_BKIN1_TRIG                            SYSCFG_TIMERINSEL_TIMER20_BKIN1_SEL          /*!< TIMER20 break input 1 selection */
#define TIMER20_BKIN0_TRIG                            SYSCFG_TIMERINSEL_TIMER20_BKIN0_SEL          /*!< TIMER20 break input 0 selection */
#define TIMER19_BKIN3_TRIG                            SYSCFG_TIMERINSEL_TIMER19_BKIN3_SEL          /*!< TIMER19 break input 3 selection */
#define TIMER19_BKIN2_TRIG                            SYSCFG_TIMERINSEL_TIMER19_BKIN2_SEL          /*!< TIMER19 break input 2 selection */
#define TIMER19_BKIN1_TRIG                            SYSCFG_TIMERINSEL_TIMER19_BKIN1_SEL          /*!< TIMER19 break input 1 selection */
#define TIMER19_BKIN0_TRIG                            SYSCFG_TIMERINSEL_TIMER19_BKIN0_SEL          /*!< TIMER19 break input 0 selection */
#define TIMER7_BKIN3_TRIG                             SYSCFG_TIMERINSEL_TIMER7_BKIN3_SEL           /*!< TIMER7 break input 3 selection */
#define TIMER7_BKIN2_TRIG                             SYSCFG_TIMERINSEL_TIMER7_BKIN2_SEL           /*!< TIMER7 break input 2 selection */
#define TIMER7_BKIN1_TRIG                             SYSCFG_TIMERINSEL_TIMER7_BKIN1_SEL           /*!< TIMER7 break input 1 selection */
#define TIMER7_BKIN0_TRIG                             SYSCFG_TIMERINSEL_TIMER7_BKIN0_SEL           /*!< TIMER7 break input 0 selection */
#define TIMER0_BKIN3_TRIG                             SYSCFG_TIMERINSEL_TIMER0_BKIN3_SEL           /*!< TIMER0 break input 3 selection */
#define TIMER0_BKIN2_TRIG                             SYSCFG_TIMERINSEL_TIMER0_BKIN2_SEL           /*!< TIMER0 break input 2 selection */
#define TIMER0_BKIN1_TRIG                             SYSCFG_TIMERINSEL_TIMER0_BKIN1_SEL           /*!< TIMER0 break input 1 selection */
#define TIMER0_BKIN0_TRIG                             SYSCFG_TIMERINSEL_TIMER0_BKIN0_SEL           /*!< TIMER0 break input 0 selection */

/* TIMER7 channel0 complementary input source definitions */
#define TIMER7CH0N_TIMER7CH0_TIMER0CH0_IN              SYSCFG_TIMERINSEL_TIMER7_CH0N_SEL           /*!< exclusive or of TIMER7_CH0_IN,TIMER7_CH0N_IN,and TIMER0_CH0_IN */
#define TIMER7_CH0N_IN                                 (~SYSCFG_TIMERINSEL_TIMER7_CH0N_SEL)        /*!< TIMER7_CH0N_IN */

/* SYSCFG flag definitions */
#define SYSCFG_FLAG_SRAMECCMERR                        SYSCFG_STAT_SRAMECCMEIF                     /*!< SRAM multi-bits non-correction ECC error flag */
#define SYSCFG_FLAG_SRAMECCSERR                        SYSCFG_STAT_SRAMECCSEIF                     /*!< SRAM single bit correction ECC error flag */
#define SYSCFG_FLAG_FLASHECCERR                        SYSCFG_STAT_FLASHECCIF                      /*!< FLASH ECC NMI error flag */
#define SYSCFG_FLAG_CKMNMIERR                          SYSCFG_STAT_CKMNMIIF                        /*!< HXTAL clock monitor NMI error flag */
#define SYSCFG_FLAG_NMIPINERR                          SYSCFG_STAT_NMIPINIF                        /*!< NMI pin error flag */

/* SYSCFG interrupt flag constants definitions */
#define SYSCFG_INT_FLAG_SRAMECCMERR                    SYSCFG_STAT_SRAMECCMEIF                     /*!< SRAM multi-bits non-correction ECC error interrupt flag */
#define SYSCFG_INT_FLAG_SRAMECCSERR                    SYSCFG_STAT_SRAMECCSEIF                     /*!< SRAM single bit correction ECC error interrupt flag */
#define SYSCFG_INT_FLAG_FLASHECCERR                    SYSCFG_STAT_FLASHECCIF                      /*!< FLASH ECC NMI error interrupt flag */
#define SYSCFG_INT_FLAG_CKMNMIERR                      SYSCFG_STAT_CKMNMIIF                        /*!< HXTAL clock monitor NMI error interrupt flag */
#define SYSCFG_INT_FLAG_NMIPINERR                      SYSCFG_STAT_NMIPINIF                        /*!< NMI pin error interrupt flag */

/* SYSCFG interrupt enable/disable constants definitions */
#define SYSCFG_INT_SRAMECCME                          SYSCFG_CFG3_SRAMECCMEIE                      /*!< SRAM multi-bits non-correction ECC error */
#define SYSCFG_INT_SRAMECCSE                          SYSCFG_CFG3_SRAMECCSEIE                      /*!< SRAM single bit correction ECC error */
#define SYSCFG_INT_FLASHECCE                          SYSCFG_CFG3_FLASHECCIE                       /*!< FLASH ECC NMI error */
#define SYSCFG_INT_CKMNMI                             SYSCFG_CFG3_CKMNMIIE                         /*!< receive buffer not empty interrupt */
#define SYSCFG_INT_NMIPIN                             SYSCFG_CFG3_NMIPINIE                         /*!< HXTAL clock monitor NMI error */

typedef enum {
    ADC1_IN14_REMAP = 1,                               /*!< ADC1 channel 14 remapping */
    ADC1_IN15_REMAP,                                   /*!< ADC1 channel 15 remapping */
    ADC0_IN8_REMAP,                                    /*!< ADC0 channel 8 remapping */
    ADC0_IN9_REMAP                                     /*!< ADC0 channel 9 remapping */
} syscfg_adcx_chy_enum;

typedef enum {
    TIMER0SEL = 1,                                     /*!< select TIMER0 */
    TIMER7SEL,                                         /*!< select TIMER7 */
    TIMER19SEL,                                        /*!< select TIMER19 */
    TIMER20SEL,                                        /*!< select TIMER20 */
} syscfg_timersel_enum;

/* function declarations */
/* initialization functions */
/* reset the SYSCFG registers */
void syscfg_deinit(void);

/* configure the GPIO pin as EXTI Line */
void syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

/* enable remap pin function */
void syscfg_pin_remap_enable(uint32_t remap_pin);
/* disable remap pin function */
void syscfg_pin_remap_disable(uint32_t remap_pin);
/* configure ADC channel GPIO pin remap function */
void syscfg_adc_ch_remap_config(syscfg_adcx_chy_enum adcx_iny_remap, ControlStatus newvalue);

/* select TIMER external trigger source */
void syscfg_timer_eti_sel(syscfg_timersel_enum timer_num, uint32_t eti_num);
/* select TRIGSEL as TIMER break input source */
void syscfg_timer_bkin_select_trigsel(uint32_t bkin_source);
/* select GPIO as TIMER break input source */
void syscfg_timer_bkin_select_gpio(uint32_t bkin_source);
/* select TIMER7 channel0 complementary input source */
void syscfg_timer7_ch0n_select(uint32_t timer7_ch0n_in);

/* configure TIMER0/7/19/20 break input to the selected parameter connection */
void syscfg_lock_config(uint32_t syscfg_lock);

/* flag and interrupt functions */
/* get SYSCFG flags */
FlagStatus syscfg_flag_get(uint32_t syscfg_flag);
/* clear SYSCFG flags */
void syscfg_flag_clear(uint32_t syscfg_flag);
/* enable SYSCFG interrupts */
void syscfg_interrupt_enable(uint32_t interrupt);
/* disable SYSCFG interrupts */
void syscfg_interrupt_disable(uint32_t interrupt);
/* get SYSCFG interrupt flag status */
FlagStatus syscfg_interrupt_flag_get(uint32_t interrupt);
/* get the current boot mode */
uint8_t syscfg_bootmode_get(void);
/* get the address where SRAM ECC error occur on */
uint16_t syscfg_sram_ecc_address_get(void);
/* get the bit which has SRAM ECC signle error */
uint8_t syscfg_sram_ecc_bit_get(void);

#endif /* GD32A50X_SYSCFG_H */
