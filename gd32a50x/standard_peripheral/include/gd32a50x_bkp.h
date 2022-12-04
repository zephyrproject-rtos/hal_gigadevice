/*!
    \file    gd32a50x_bkp.h
    \brief   definitions for the BKP

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

#ifndef GD32A50X_BKP_H
#define GD32A50X_BKP_H

#include "gd32a50x.h"

/* BKP definitions */
#define BKP                           BKP_BASE                   /*!< BKP base address */

/* registers definitions */
#define BKP_DATA0                     REG16((BKP) + 0x0004U)     /*!< BKP data register 0 */
#define BKP_DATA1                     REG16((BKP) + 0x0008U)     /*!< BKP data register 1 */
#define BKP_DATA2                     REG16((BKP) + 0x000CU)     /*!< BKP data register 2 */
#define BKP_DATA3                     REG16((BKP) + 0x0010U)     /*!< BKP data register 3 */
#define BKP_DATA4                     REG16((BKP) + 0x0014U)     /*!< BKP data register 4 */
#define BKP_DATA5                     REG16((BKP) + 0x0018U)     /*!< BKP data register 5 */
#define BKP_DATA6                     REG16((BKP) + 0x001CU)     /*!< BKP data register 6 */
#define BKP_DATA7                     REG16((BKP) + 0x0020U)     /*!< BKP data register 7 */
#define BKP_DATA8                     REG16((BKP) + 0x0024U)     /*!< BKP data register 8 */
#define BKP_DATA9                     REG16((BKP) + 0x0028U)     /*!< BKP data register 9 */
#define BKP_OCTL                      REG16((BKP) + 0x002CU)     /*!< RTC signal output control register */
#define BKP_TPCTL                     REG16((BKP) + 0x0030U)     /*!< tamper pin control register */
#define BKP_TPCS                      REG16((BKP) + 0x0034U)     /*!< tamper control and status register */

/* bits definitions */
/* BKP_DATA */
#define BKP_DATA                      BITS(0,15)                 /*!< backup data */

/* BKP_OCTL */
#define BKP_OCTL_RCCV                 BITS(0,6)                  /*!< RTC clock calibration value */
#define BKP_OCTL_COEN                 BIT(7)                     /*!< RTC clock calibration output enable */
#define BKP_OCTL_ASOEN                BIT(8)                     /*!< RTC alarm or second signal output enable */
#define BKP_OCTL_ROSEL                BIT(9)                     /*!< RTC output selection */
#define BKP_OCTL_CCOSEL               BIT(14)                    /*!< RTC clock output selection */
#define BKP_OCTL_CALDIR               BIT(15)                    /*!< RTC clock calibration direction */

/* BKP_TPCTL */
#define BKP_TPCTL_TPEN                BIT(0)                     /*!< tamper detection enable */
#define BKP_TPCTL_TPAL                BIT(1)                     /*!< tamper pin active level */
#define BKP_TPCTL_PCSEL               BIT(15)                    /*!< OSC32IN pin select */

/* BKP_TPCS */
#define BKP_TPCS_TER                  BIT(0)                     /*!< tamper event reset */
#define BKP_TPCS_TIR                  BIT(1)                     /*!< tamper interrupt reset */
#define BKP_TPCS_TPIE                 BIT(2)                     /*!< tamper interrupt enable */
#define BKP_TPCS_TEF                  BIT(8)                     /*!< tamper event flag */
#define BKP_TPCS_TIF                  BIT(9)                     /*!< tamper interrupt flag */

/* constants definitions */
/* BKP register */
#define BKP_DATA0_9(number)           REG16((BKP) + 0x04U + (number) * 0x04U)

/* get data of BKP data register */
#define BKP_DATA_GET(regval)          GET_BITS((uint32_t)(regval), 0, 15)

/* RTC clock calibration value */
#define OCTL_RCCV(regval)             (BITS(0,6) & (uint32_t)(regval))

/* RTC output selection */
#define RTC_OUTPUT_ALARM_PULSE        ((uint16_t)0x0000U)        /*!< RTC alarm pulse is selected as the RTC output */
#define RTC_OUTPUT_SECOND_PULSE       ((uint16_t)0x0200U)        /*!< RTC second pulse is selected as the RTC output */

/* RTC clock output selection */
#define RTC_CLOCK_DIV_64              ((uint16_t)0x0000U)        /*!< RTC clock div 64 */
#define RTC_CLOCK_DIV_1               ((uint16_t)0x4000U)        /*!< RTC clock div 1 */

/* RTC clock calibration direction */                          
#define RTC_CLOCK_SLOWED_DOWN         ((uint16_t)0x0000U)        /*!< RTC clock slow down */
#define RTC_CLOCK_SPEED_UP            ((uint16_t)0x8000U)        /*!< RTC clock speed up */

/* OSC32IN pin select */
#define OSC32IN_PC13                  ((uint16_t)0x0000U)        /*!< OSC32IN pin is PC13 */
#define OSC32IN_PC14                  ((uint16_t)0x8000U)        /*!< OSC32IN pin is PC14 */

/* tamper pin active level */
#define TAMPER_PIN_ACTIVE_HIGH        ((uint16_t)0x0000U)        /*!< the tamper pin is active high */
#define TAMPER_PIN_ACTIVE_LOW         ((uint16_t)0x0002U)        /*!< the tamper pin is active low */

/* tamper flag */
#define BKP_FLAG_TAMPER               BKP_TPCS_TEF               /*!< tamper event flag */

/* tamper interrupt flag */
#define BKP_INT_FLAG_TAMPER           BKP_TPCS_TIF               /*!< tamper interrupt flag */

/* BKP data register number */
typedef enum 
{
    BKP_DATA_0 = 1,                                              /*!< BKP data register 0 */
    BKP_DATA_1,                                                  /*!< BKP data register 1 */
    BKP_DATA_2,                                                  /*!< BKP data register 2 */
    BKP_DATA_3,                                                  /*!< BKP data register 3 */
    BKP_DATA_4,                                                  /*!< BKP data register 4 */
    BKP_DATA_5,                                                  /*!< BKP data register 5 */
    BKP_DATA_6,                                                  /*!< BKP data register 6 */
    BKP_DATA_7,                                                  /*!< BKP data register 7 */
    BKP_DATA_8,                                                  /*!< BKP data register 8 */
    BKP_DATA_9,                                                  /*!< BKP data register 9 */
}bkp_data_register_enum;

/* function declarations */
/* reset BKP registers */
void bkp_deinit(void);
/* write BKP data register */
void bkp_data_write(bkp_data_register_enum register_number, uint16_t data);
/* read BKP data register */
uint16_t bkp_data_read(bkp_data_register_enum register_number);

/* RTC related functions */
/* enable RTC clock calibration output */
void bkp_rtc_calibration_output_enable(void);
/* disable RTC clock calibration output */
void bkp_rtc_calibration_output_disable(void);
/* enable RTC alarm or second signal output */
void bkp_rtc_signal_output_enable(void);
/* disable RTC alarm or second signal output */
void bkp_rtc_signal_output_disable(void);
/* select RTC output */
void bkp_rtc_output_select(uint16_t outputsel);
/* select RTC clock output */
void bkp_rtc_clock_output_select(uint16_t clocksel);
/* RTC clock calibration direction */
void bkp_rtc_clock_calibration_direction(uint16_t direction);
/* set RTC clock calibration value */
void bkp_rtc_calibration_value_set(uint8_t value);

/* select OSC32IN pin */
void bkp_osc32in_pin_select(uint16_t inputpin);

/* tamper pin related functions */
/* enable tamper pin detection */
void bkp_tamper_detection_enable(void);
/* disable tamper pin detection */
void bkp_tamper_detection_disable(void);
/* set tamper pin active level */
void bkp_tamper_active_level_set(uint16_t level);
/* enable tamper pin interrupt */
void bkp_tamper_interrupt_enable(void);
/* disable tamper pin interrupt */
void bkp_tamper_interrupt_disable(void);

/* flag functions */
/* get BKP flag state */
FlagStatus bkp_flag_get(uint16_t flag);
/* clear BKP flag state */
void bkp_flag_clear(uint16_t flag);
/* get BKP interrupt flag state */
FlagStatus bkp_interrupt_flag_get(uint16_t flag);
/* clear BKP interrupt flag state */
void bkp_interrupt_flag_clear(uint16_t flag);

#endif /* GD32A50X_BKP_H */
