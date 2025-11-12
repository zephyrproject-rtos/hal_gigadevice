/*!
    \file    gd32c2x1.h
    \brief   general definitions for gd32c2x1

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

#ifndef GD32C2X1_H
#define GD32C2X1_H

#ifdef __cplusplus
extern "C" {
#endif

/* define GD32C2x1 */
#if !defined (GD32C2x1)
#define GD32C2x1
#endif /* define GD32C2x1 */
#if !defined (GD32C2x1)
#error "Please select the target gd32c2x1 device used in your application (in gd32c2x1.h file)"
#endif /* undefine GD32C2x1 tip */

/* define value of high speed crystal oscillator (HXTAL) in Hz */
#if !defined  (HXTAL_VALUE)
#define HXTAL_VALUE    ((uint32_t)8000000)
#endif /* high speed crystal oscillator value */

/* define startup timeout value of high speed crystal oscillator (HXTAL) */
#if !defined  (HXTAL_STARTUP_TIMEOUT)
#define HXTAL_STARTUP_TIMEOUT   ((uint16_t)0x0FFFF)
#endif /* high speed crystal oscillator startup timeout */

/* define value of internal 48MHz RC oscillator (IRC48M) in Hz */
#if !defined  (IRC48M_VALUE)
#define IRC48M_VALUE  ((uint32_t)48000000)
#endif /* internal 48MHz RC oscillator value */

/* define startup timeout value of internal 48MHz RC oscillator (IRC48M) */
#if !defined  (IRC48M_STARTUP_TIMEOUT)
#define IRC48M_STARTUP_TIMEOUT   ((uint16_t)0x0500)
#endif /* internal 48MHz RC oscillator startup timeout */

#if !defined  (IRC48M_VALUE)
#define IRC48M_VALUE ((uint32_t)48000000)
#endif /* IRC48M_VALUE */

/* define value of internal 32KHz RC oscillator(IRC32K) in Hz */
#if !defined  (IRC32K_VALUE)
#define IRC32K_VALUE  ((uint32_t)32000)
#endif /* internal 32KHz RC oscillator value */

/* define value of low speed crystal oscillator (LXTAL)in Hz */
#if !defined  (LXTAL_VALUE)
#define LXTAL_VALUE  ((uint32_t)32768)
#endif /* low speed crystal oscillator value */

/* gd32c2x1 firmware library version number V1.0 */
#define __GD32C2X1_STDPERIPH_VERSION_MAIN   (0x01) /*!< [31:24] main version     */
#define __GD32C2X1_STDPERIPH_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version     */
#define __GD32C2X1_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version     */
#define __GD32C2X1_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __GD32C2X1_STDPERIPH_VERSION        ((__GD32C2X1_STDPERIPH_VERSION_MAIN << 24)\
        |(__GD32C2X1_STDPERIPH_VERSION_SUB1 << 16)\
        |(__GD32C2X1_STDPERIPH_VERSION_SUB2 << 8)\
        |(__GD32C2X1_STDPERIPH_VERSION_RC))

/* configuration of the Cortex-M23 processor and core peripherals                                        */
#define __CM23_REV                0x0100U   /*!< Core revision r1p0                                      */
#define __SAUREGION_PRESENT       0U        /*!< SAU regions are not present                             */
#define __MPU_PRESENT             0U        /*!< MPU is present                                          */
#define __VTOR_PRESENT            1U        /*!< VTOR is present                                         */
#define __NVIC_PRIO_BITS          2U        /*!< Number of Bits used for Priority Levels                 */
#define __Vendor_SysTickConfig    0U        /*!< Set to 1 if different SysTick Config is used            */

/* define interrupt number */
typedef enum IRQn {
    /* Cortex-M23 processor exceptions numbers */
    NonMaskableInt_IRQn          = -14,    /*!< non maskable interrupt                                   */
    HardFault_IRQn               = -13,    /*!< hardfault interrupt                                      */
    SVCall_IRQn                  = -5,     /*!< sv call interrupt                                        */
    PendSV_IRQn                  = -2,     /*!< pend sv interrupt                                        */
    SysTick_IRQn                 = -1,     /*!< system tick interrupt                                    */
    /* interruput numbers */
    WWDGT_IRQn                   = 0,      /*!< window watchdog timer interrupt                          */
    TIMESTAMP_IRQn               = 1,      /*!< RTC TimeStamp interrupt                                  */
    FMC_IRQn                     = 3,      /*!< FMC interrupt                                            */
    RCU_IRQn                     = 4,      /*!< RCU interrupt                                            */
    EXTI0_IRQn                   = 5,      /*!< EXTI line 0 interrupts                                   */
    EXTI1_IRQn                   = 6,      /*!< EXTI line 1 interrupts                                   */
    EXTI2_IRQn                   = 7,      /*!< EXTI line 2 interrupts                                   */
    EXTI3_IRQn                   = 8,      /*!< EXTI line 3 interrupts                                   */
    EXTI4_IRQn                   = 9,      /*!< EXTI line 4 interrupts                                   */
    DMA_Channel0_IRQn            = 10,     /*!< DMA channel 0 interrupt                                  */
    DMA_Channel1_IRQn            = 11,     /*!< DMA channel 1 interrupt                                  */
    DMA_Channel2_IRQn            = 12,     /*!< DMA channel 2 interrupt                                  */
    ADC_IRQn                     = 13,     /*!< ADC interrupts                                           */
    USART0_IRQn                  = 14,     /*!< USART0 interrupt                                         */
    USART1_IRQn                  = 15,     /*!< USART1 interrupt                                         */
    USART2_IRQn                  = 16,     /*!< USART2 interrupt                                         */
    I2C0_EV_IRQn                 = 17,     /*!< I2C0 event interrupt                                     */
    I2C0_ER_IRQn                 = 18,     /*!< I2C0 error interrupt                                     */
    I2C1_EV_IRQn                 = 19,     /*!< I2C1 event interrupt                                     */
    I2C1_ER_IRQn                 = 20,     /*!< I2C1 error interrupt                                     */
    SPI0_IRQn                    = 21,     /*!< SPI0 interrupt                                           */
    SPI1_IRQn                    = 22,     /*!< SPI1 interrupt                                           */
    RTC_Alarm_IRQn               = 23,     /*!< RTC Alarm interrupt                                      */
    EXTI5_9_IRQn                 = 24,     /*!< EXTI line 5 to 9 interrupts                              */
    TIMER0_TRG_CMT_UP_BRK_IRQn   = 25,     /*!< TIMER0 Trigger, commutation, Update, Break interrupt   */
    TIMER0_Channel_IRQn          = 26,     /*!< TIMER0 capture compare interrupt                         */
    TIMER2_IRQn                  = 27,     /*!< TIMER2 interrupt                                         */
    TIMER13_IRQn                 = 28,     /*!< TIMER13 interrupt                                        */
    TIMER15_IRQn                 = 29,     /*!< TIMER15 interrupt                                        */
    TIMER16_IRQn                 = 30,     /*!< TIMER16 interrupt                                        */
    EXTI10_15_IRQn               = 31,     /*!< EXTI line 10 to 15 interrupts                            */
    DMAMUX_IRQn                  = 33,     /*!< DMAMUX interrupt                                         */
    CMP0_IRQn                    = 34,     /*!< Comparator 0 interrupt                                   */
    CMP1_IRQn                    = 35,     /*!< Comparator 1 interrupt                                   */
    I2C0_WKUP_IRQn               = 36,     /*!< I2C0 Wakeup interrupt                                    */
    I2C1_WKUP_IRQn               = 37,     /*!< I2C1 Wakeup interrupt                                    */
    USART0_WKUP_IRQn             = 38,     /*!< USART0 Wakeup interrupt                                  */
} IRQn_Type;

/* includes */
#include "core_cm23.h"
#include "system_gd32c2x1.h"
#include <stdint.h>
#ifdef FW_DEBUG_ERR_REPORT
#include "gd32c2x1_err_report.h"
#endif /* FW_DEBUG_ERR_REPORT */

/* enum definitions */
typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
typedef enum {RESET = 0, SET = !RESET} FlagStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;

/* bit operations */
#define REG64(addr)                  (*(volatile uint64_t *)(uint32_t)(addr))
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#ifndef BIT
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
#endif
#define BITS(start, end)             ((0xFFFFFFFFUL << (uint8_t)(start)) & (0xFFFFFFFFUL >> (31U - (uint8_t)(end))))
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))

/* main flash and SRAM memory map */
#define FLASH_BASE            ((uint32_t)0x08000000U)       /*!< main FLASH base address          */
#define SRAM_BASE             ((uint32_t)0x20000000U)       /*!< SRAM  base address               */
/* peripheral memory map */
#define APB_BUS_BASE          ((uint32_t)0x40000000U)       /*!< apb base address                 */
#define AHB1_BUS_BASE         ((uint32_t)0x40020000U)       /*!< ahb1 base address                */
#define AHB2_BUS_BASE         ((uint32_t)0x48000000U)       /*!< ahb2 base address                */
/* advanced peripheral bus 2 memory map */
#define SYSCFG_BASE           (APB_BUS_BASE + 0x00010000U)  /*!< SYSCFG base address              */
#define EXTI_BASE             (APB_BUS_BASE + 0x00010400U)  /*!< EXTI base address                */
#define TIMER_BASE            (APB_BUS_BASE + 0x00000000U)  /*!< TIMER base address               */
#define ADC_BASE              (APB_BUS_BASE + 0x00012400U)  /*!< ADC base address                 */
#define SPI_BASE              (APB_BUS_BASE + 0x00003800U)  /*!< SPI base address                 */
#define USART_BASE            (APB_BUS_BASE + 0x00004400U)  /*!< USART base address               */
#define RTC_BASE              (APB_BUS_BASE + 0x00002800U)  /*!< RTC base address                 */
#define WWDGT_BASE            (APB_BUS_BASE + 0x00002C00U)  /*!< WWDGT base address               */
#define FWDGT_BASE            (APB_BUS_BASE + 0x00003000U)  /*!< FWDGT base address               */
#define I2C_BASE              (APB_BUS_BASE + 0x00005400U)  /*!< I2C base address                 */
#define PMU_BASE              (APB_BUS_BASE + 0x00007000U)  /*!< PMU base address                 */
#define CMP_BASE              (APB_BUS_BASE + 0x00017C00U)  /*!< CMP base address                 */
/* advanced high performance bus 1 memory map */
#define DMA_BASE              (AHB1_BUS_BASE + 0x00000000U) /*!< DMA base address                 */
#define DMA_CHANNEL_BASE      (DMA_BASE + 0x00000008U)      /*!< DMA channel base address         */
#define DMAMUX_BASE           (AHB1_BUS_BASE + 0x00000800U) /*!< DMA base address                 */
#define RCU_BASE              (AHB1_BUS_BASE + 0x00001000U) /*!< RCU base address                 */
#define FMC_BASE              (AHB1_BUS_BASE + 0x00002000U) /*!< FMC base address                 */
#define CRC_BASE              (AHB1_BUS_BASE + 0x00003000U) /*!< CRC base address                 */
/* advanced high performance bus 2 memory map */
#define GPIO_BASE             (AHB2_BUS_BASE + 0x00000000U) /*!< GPIO base address                */
/* option byte and debug memory map */
#define OB_BASE               ((uint32_t)0x1FFFF800U)       /*!< OB base address                  */
#define DBG_BASE              ((uint32_t)0x40015800U)       /*!< DBG base address                 */

#define VREF_BASE             ((uint32_t)0x40010030U)       /*!< VREF base address                */

/* define marco USE_STDPERIPH_DRIVER */
#if !defined USE_STDPERIPH_DRIVER
#define USE_STDPERIPH_DRIVER
#endif 
#ifdef USE_STDPERIPH_DRIVER
#include "gd32c2x1_libopt.h"
#endif /* USE_STDPERIPH_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* gd32c2x1_H */
