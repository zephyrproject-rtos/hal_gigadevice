/*!
    \file    gd32a50x.h
    \brief   general definitions for GD32A50x

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

#ifndef GD32A50X_H
#define GD32A50X_H

#ifdef __cplusplus
 extern "C" {
#endif 

#if !defined (GD32A50X)
#error "Please select the target GD32A50X device used in your application (in gd32a50x.h file)"
#endif /* undefine GD32A50X tip */

/* define value of high speed crystal oscillator (HXTAL) in Hz */
#if !defined  HXTAL_VALUE
#define HXTAL_VALUE    ((uint32_t)8000000) /*!< value of the external oscillator in Hz */
#endif /* high speed crystal oscillator value */

/* define startup timeout value of high speed crystal oscillator (HXTAL) */
#if !defined  (HXTAL_STARTUP_TIMEOUT)
#define HXTAL_STARTUP_TIMEOUT   ((uint16_t)0x0FFFF)
#endif /* high speed crystal oscillator startup timeout */

/* define value of internal 8MHz RC oscillator (IRC8M) in Hz */
#if !defined  (IRC8M_VALUE) 
#define IRC8M_VALUE  ((uint32_t)8000000)
#endif /* internal 8MHz RC oscillator value */

/* define startup timeout value of internal 8MHz RC oscillator (IRC8M) */
#if !defined  (IRC8M_STARTUP_TIMEOUT)
#define IRC8M_STARTUP_TIMEOUT   ((uint16_t)0x0500)
#endif /* internal 8MHz RC oscillator startup timeout */

/* define value of internal 40KHz RC oscillator(IRC40K) in Hz */
#if !defined  (IRC40K_VALUE) 
#define IRC40K_VALUE  ((uint32_t)40000)
#endif /* internal 40KHz RC oscillator value */

/* define value of low speed crystal oscillator (LXTAL)in Hz */
#if !defined  (LXTAL_VALUE) 
#define LXTAL_VALUE  ((uint32_t)32768)
#endif /* low speed crystal oscillator value */

/* GD32E50x firmware library version number V1.0 */
#define __GD32A50X_STDPERIPH_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __GD32A50X_STDPERIPH_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __GD32A50X_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __GD32A50X_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __GD32A50X_STDPERIPH_VERSION        ((__GD32A50X_STDPERIPH_VERSION_MAIN << 24)\
                                            |(__GD32A50X_STDPERIPH_VERSION_SUB1 << 16)\
                                            |(__GD32A50X_STDPERIPH_VERSION_SUB2 << 8)\
                                            |(__GD32A50X_STDPERIPH_VERSION_RC))

/* configuration of the Cortex-M33 processor and core peripherals                                           */
#define __CM33_REV                0x0003U   /*!< Core revision r0p3                                         */
#define __SAUREGION_PRESENT       0U        /*!< SAU regions are not present                                */
#define __MPU_PRESENT             1U        /*!< MPU is present                                             */
#define __VTOR_PRESENT            1U        /*!< VTOR is present                                            */
#define __NVIC_PRIO_BITS          4U        /*!< Number of Bits used for Priority Levels                    */
#define __Vendor_SysTickConfig    0U        /*!< Set to 1 if different SysTick Config is used               */
#define __FPU_PRESENT             1U        /*!< FPU present                                                */
#define __DSP_PRESENT             1U        /*!< DSP present                                                */

/* define interrupt number */
typedef enum IRQn
{
    /* Cortex-M33 processor exceptions numbers */
    NonMaskableInt_IRQn         = -14,      /*!< non mask-able interrupt                                    */
    HardFault_IRQn              = -13,      /*!< hard-fault interrupt                                       */
    MemoryManagement_IRQn       = -12,      /*!< 4 Cortex-M33 memory management interrupt                   */
    BusFault_IRQn               = -11,      /*!< 5 Cortex-M33 bus fault interrupt                           */
    UsageFault_IRQn             = -10,      /*!< 6 Cortex-M33 usage fault interrupt                         */
    SVCall_IRQn                 = -5,       /*!< 11 Cortex-M33 sv call interrupt                            */
    DebugMonitor_IRQn           = -4,       /*!< 12 Cortex-M33 debug monitor interrupt                      */
    PendSV_IRQn                 = -2,       /*!< 14 Cortex-M33 pend sv interrupt                            */
    SysTick_IRQn                = -1,       /*!< 15 Cortex-M33 system tick interrupt                        */
    /* interrupt numbers */    
    WWDGT_IRQn                  = 0,        /*!< window watchdog timer interrupt                            */
    LVD_IRQn                    = 1,        /*!< LVD through EXTI line detect interrupt                     */
    RTC_IRQn                    = 3,        /*!< RTC Wakeup interrupt                                       */
    FMC_IRQn                    = 4,        /*!< FMC interrupt                                              */
    RCU_IRQn                    = 5,        /*!< RCU and CTC interrupt                                      */
    EXTI0_IRQn                  = 6,        /*!< EXTI line 0 interrupts                                     */
    EXTI1_IRQn                  = 7,        /*!< EXTI line 1 interrupts                                     */
    EXTI2_IRQn                  = 8,        /*!< EXTI line 2 interrupts                                     */
    EXTI3_IRQn                  = 9,        /*!< EXTI line 3 interrupts                                     */
    EXTI4_IRQn                  = 10,       /*!< EXTI line 4 interrupts                                     */
    DMA0_Channel0_IRQn          = 11,       /*!< DMA0 channel 0 interrupt                                   */
    DMA0_Channel1_IRQn          = 12,       /*!< DMA0 channel 1 interrupt                                   */
    DMA0_Channel2_IRQn          = 13,       /*!< DMA0 channel 2 interrupt                                   */
    DMA0_Channel3_IRQn          = 14,       /*!< DMA0 channel 3 interrupt                                   */
    DMA0_Channel4_IRQn          = 15,       /*!< DMA0 channel 4 interrupt                                   */
    DMA0_Channel5_IRQn          = 16,       /*!< DMA0 channel 5 interrupt                                   */
    DMA0_Channel6_IRQn          = 17,       /*!< DMA0 channel 6 interrupt                                   */
    ADC0_1_IRQn                 = 18,       /*!< ADC0_1 interrupts                                          */
    CAN0_Message_IRQn           = 19,       /*!< CAN0 message buffer interrupt                              */
    CAN0_Busoff_IRQn            = 20,       /*!< CAN0 bus off interrupt                                     */
    CAN0_Error_IRQn             = 21,       /*!< CAN0 error interrupt                                       */
    CAN0_FastError_IRQn         = 22,       /*!< CAN0 fast transmission error interrupt                     */
    CAN0_TEC_IRQn               = 23,       /*!< CAN0 transmit warning interrupt                            */
    CAN0_REC_IRQn               = 24,       /*!< CAN0 receive  warning interrupt                            */
    CAN0_WKUP_IRQn              = 25,       /*!< CAN0 wakeup through EXTI Line detection interrupt          */
    TIMER0_BRK_UP_TRG_CMT_IRQn  = 26,       /*!< TIMER0 break, update, trigger and commutation interrupt    */
    TIMER0_Channel_IRQn         = 27,       /*!< TIMER0 capture compare interrupt                           */
    TIMER1_IRQn                 = 28,       /*!< TIMER1 interrupt                                           */
    TIMER19_BRK_UP_TRG_CMT_IRQn = 29,       /*!< TIMER19 break, update, trigger and commutation interrupt   */
    TIMER19_Channel_IRQn        = 30,       /*!< TIMER19 capture compare interrupt                          */
    I2C0_EV_IRQn                = 31,       /*!< I2C0 event interrupt                                       */
    I2C0_ER_IRQn                = 32,       /*!< I2C0 error interrupt                                       */
    I2C1_EV_IRQn                = 33,       /*!< I2C1 event interrupt                                       */
    I2C1_ER_IRQn                = 34,       /*!< I2C1 error interrupt                                       */
    SPI0_IRQn                   = 35,       /*!< SPI0 interrupt                                             */
    SPI1_IRQn                   = 36,       /*!< SPI1 interrupt                                             */
    USART0_IRQn                 = 37,       /*!< USART0 interrupt                                           */
    USART1_IRQn                 = 38,       /*!< USART1 interrupt                                           */
    USART2_IRQn                 = 39,       /*!< USART2 interrupt                                           */
    EXTI10_15_IRQn              = 40,       /*!< EXTI line 10 to 15 interrupts                              */
    EXTI5_9_IRQn                = 41,       /*!< EXTI line 5 to 9 interrupts                                */
    TAMPER_IRQn                 = 42,       /*!< BKP Tamper interrupt                                       */
    TIMER20_BRK_UP_TRG_CMT_IRQn = 43,       /*!< TIMER20 break, update, trigger and commutation interrupt   */
    TIMER20_Channel_IRQn        = 44,       /*!< TIMER20 capture compare interrupt                          */
    TIMER7_BRK_UP_TRG_CMT_IRQn  = 45,       /*!< TIMER7 break, update, trigger and commutation interrupt    */
    TIMER7_Channel_IRQn         = 46,       /*!< TIMER7 capture compare interrupt                           */
    DMAMUX_IRQn                 = 47,       /*!< DMAMUX interrupt                                           */
    SRAMC_ECCSE_IRQn            = 48,       /*!< SYSCFG SRAM ECC single err interrupt                       */
    CMP_IRQn                    = 49,       /*!< Comparator interrupt                                       */
    OVD_IRQn                    = 51,       /*!< Over voltage detector interrupt                            */
    TIMER5_DAC_IRQn             = 54,       /*!< TIMER5 and DAC interrupt                                   */
    TIMER6_IRQn                 = 55,       /*!< TIMER6 interrupt                                           */
    DMA1_Channel0_IRQn          = 56,       /*!< DMA1 channel 0 interrupt                                   */
    DMA1_Channel1_IRQn          = 57,       /*!< DMA1 channel 1 interrupt                                   */
    DMA1_Channel2_IRQn          = 58,       /*!< DMA1 channel 2 interrupt                                   */
    DMA1_Channel3_IRQn          = 59,       /*!< DMA1 channel 3 interrupt                                   */
    DMA1_Channel4_IRQn          = 60,       /*!< DMA1 channel 4 interrupt                                   */
    CAN1_WKUP_IRQn              = 62,       /*!< CAN1 wakeup through EXTI Line detection interrupt          */
    CAN1_Message_IRQn           = 63,       /*!< CAN1 message buffer interrupt                              */
    CAN1_Busoff_IRQn            = 64,       /*!< CAN1 bus off interrupt                                     */
    CAN1_Error_IRQn             = 65,       /*!< CAN1 error interrupt                                       */
    CAN1_FastError_IRQn         = 66,       /*!< CAN1 fast transmission error interrupt                     */
    CAN1_TEC_IRQn               = 67,       /*!< CAN1 transmit warning interrupt                            */
    CAN1_REC_IRQn               = 68,       /*!< CAN1 receive  warning interrupt                            */
    FPU_IRQn                    = 69,       /*!< FPU interrupt                                              */
    MFCOM_IRQn                  = 70        /*!< MFCOM interrupt                                            */
} IRQn_Type;

/* includes */
#include "core_cm33.h"
#include "system_gd32a50x.h"
#include <stdint.h>

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
#endif /* BIT */ 
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end)))) 
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))

/* main flash and SRAM memory map */
#define FLASH_BASE            ((uint32_t)0x08000000U)       /*!< main FLASH base address    */
#define SRAM_BASE             ((uint32_t)0x20000000U)       /*!< SRAM base address          */

/* peripheral memory map */
#define APB1_BUS_BASE         ((uint32_t)0x40000000U)       /*!< apb1 base address          */
#define APB2_BUS_BASE         ((uint32_t)0x40010000U)       /*!< apb2 base address          */
#define AHB1_BUS_BASE         ((uint32_t)0x40020000U)       /*!< ahb1 base address          */
#define AHB2_BUS_BASE         ((uint32_t)0x48000000U)       /*!< ahb3 base address          */

/* advanced peripheral bus 1 memory map */
#define TIMER_BASE            (APB1_BUS_BASE + 0x00000000U) /*!< TIMER base address         */
#define RTC_BASE              (APB1_BUS_BASE + 0x00002800U) /*!< RTC base address           */
#define WWDGT_BASE            (APB1_BUS_BASE + 0x00002C00U) /*!< WWDGT base address         */
#define FWDGT_BASE            (APB1_BUS_BASE + 0x00003000U) /*!< FWDGT base address         */
#define SPI_BASE              (APB1_BUS_BASE + 0x00003800U) /*!< SPI base address           */
#define USART_BASE            (APB1_BUS_BASE + 0x00004400U) /*!< USART base address         */
#define I2C_BASE              (APB1_BUS_BASE + 0x00005400U) /*!< I2C base address           */
#define BKP_BASE              (APB1_BUS_BASE + 0x00006C00U) /*!< BKP base address           */
#define PMU_BASE              (APB1_BUS_BASE + 0x00007000U) /*!< PMU base address           */
#define DAC_BASE              (APB1_BUS_BASE + 0x00007400U) /*!< DAC base address           */

/* advanced peripheral bus 2 memory map */
#define SYSCFG_BASE           (APB2_BUS_BASE + 0x00000000U) /*!< SYSCFG base address        */
#define EXTI_BASE             (APB2_BUS_BASE + 0x00000400U) /*!< EXTI base address          */
#define ADC_BASE              (APB2_BUS_BASE + 0x00002400U) /*!< ADC base address           */
#define CMP_BASE              (APB2_BUS_BASE + 0x00007C00U) /*!< CMP base address           */
#define TRIGSEL_BASE          (APB2_BUS_BASE + 0x00008400U) /*!< TRIGSEL base address       */
#define CAN_BASE              (APB2_BUS_BASE + 0x0000A000U) /*!< CAN base address           */

/* advanced high performance bus 1 memory map */
#define DMA_BASE              (AHB1_BUS_BASE + 0x00000000U) /*!< DMA base address           */
#define DMAMUX_BASE           (AHB1_BUS_BASE + 0x00000800U) /*!< DMAMUX base address        */
#define RCU_BASE              (AHB1_BUS_BASE + 0x00001000U) /*!< RCU base address           */
#define FMC_BASE              (AHB1_BUS_BASE + 0x00002000U) /*!< FMC base address           */
#define CRC_BASE              (AHB1_BUS_BASE + 0x00003000U) /*!< CRC base address           */
#define MFCOM_BASE            (AHB1_BUS_BASE + 0x00018400U) /*!< MFCOM base address         */

/* advanced high performance bus 2 memory map */
#define GPIO_BASE             (AHB2_BUS_BASE + 0x00000000U) /*!< GPIO base address          */

/* option byte and debug memory map */
#define OB_BASE               ((uint32_t)0x1FFFF800U)       /*!< OB base address            */

#define DBG_BASE              ((uint32_t)0xE0044000U)       /*!< DBG base address           */

/* define marco USE_STDPERIPH_DRIVER */
#if !defined  USE_STDPERIPH_DRIVER
#define USE_STDPERIPH_DRIVER
#endif 
#ifdef USE_STDPERIPH_DRIVER
#include "gd32a50x_libopt.h"
#endif /* USE_STDPERIPH_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* GD32A50X_H */
