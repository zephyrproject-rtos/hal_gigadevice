/*!
    \file    gd32a50x_trigsel.h
    \brief   definitions for the TRIGSEL

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

#ifndef GD32A50X_TRIGSEL_H
#define GD32A50X_TRIGSEL_H

#include "gd32a50x.h"

/* TRIGSEL definitions */
#define TRIGSEL                                    TRIGSEL_BASE                          /*!< TRIGSEL base address */

/* register definitions */
#define TRIGSEL_EXTOUT0                            REG32((TRIGSEL) + 0x00000000U)        /*!< TRIGSEL trigger selection for EXTOUT0 register */
#define TRIGSEL_EXTOUT1                            REG32((TRIGSEL) + 0x00000004U)        /*!< TRIGSEL trigger selection for EXTOUT1 register */
#define TRIGSEL_ADC0                               REG32((TRIGSEL) + 0x00000008U)        /*!< TRIGSEL trigger selection for ADC0 register */
#define TRIGSEL_ADC1                               REG32((TRIGSEL) + 0x0000000CU)        /*!< TRIGSEL trigger selection for ADC1 register */
#define TRIGSEL_DAC                                REG32((TRIGSEL) + 0x00000010U)        /*!< TRIGSEL trigger selection for DAC register */
#define TRIGSEL_TIMER0IN                           REG32((TRIGSEL) + 0x00000014U)        /*!< TRIGSEL trigger selection for TIMER0_ITI register */
#define TRIGSEL_TIMER0BRKIN                        REG32((TRIGSEL) + 0x00000018U)        /*!< TRIGSEL trigger selection for TIMER0_BRKIN register */
#define TRIGSEL_TIMER7IN                           REG32((TRIGSEL) + 0x0000001CU)        /*!< TRIGSEL trigger selection for TIMER7_ITI register */
#define TRIGSEL_TIMER7BRKIN                        REG32((TRIGSEL) + 0x00000020U)        /*!< TRIGSEL trigger selection for TIMER7_BRKIN register */
#define TRIGSEL_TIMER19IN                          REG32((TRIGSEL) + 0x00000024U)        /*!< TRIGSEL trigger selection for TIMER19_ITI register */
#define TRIGSEL_TIMER19BRKIN                       REG32((TRIGSEL) + 0x00000028U)        /*!< TRIGSEL trigger selection for TIMER19_BRKIN register */
#define TRIGSEL_TIMER20IN                          REG32((TRIGSEL) + 0x0000002CU)        /*!< TRIGSEL trigger selection for TIMER20_ITI register */
#define TRIGSEL_TIMER20BRKIN                       REG32((TRIGSEL) + 0x00000030U)        /*!< TRIGSEL trigger selection for TIMER20_BRKIN register */
#define TRIGSEL_TIMER1IN                           REG32((TRIGSEL) + 0x00000034U)        /*!< TRIGSEL trigger selection for TIMER1_ITI register */
#define TRIGSEL_MFCOM                              REG32((TRIGSEL) + 0x00000038U)        /*!< TRIGSEL trigger selection for MFCOM register */
#define TRIGSEL_CAN0                               REG32((TRIGSEL) + 0x0000003CU)        /*!< TRIGSEL trigger selection for CAN0 register */
#define TRIGSEL_CAN1                               REG32((TRIGSEL) + 0x00000040U)        /*!< TRIGSEL trigger selection for CAN1 register */

/* bits definitions */
#define TRIGSEL_TARGET_INSEL0                      BITS(0, 6)                            /*!< trigger input source selection for output0 */
#define TRIGSEL_TARGET_INSEL1                      BITS(8, 14)                           /*!< trigger input source selection for output1 */
#define TRIGSEL_TARGET_INSEL2                      BITS(16, 22)                          /*!< trigger input source selection for output2 */
#define TRIGSEL_TARGET_INSEL3                      BITS(24, 30)                          /*!< trigger input source selection for output3 */
#define TRIGSEL_TARGET_LK                          BIT(31)                               /*!< TRIGSEL register lock */

/* constants definitions */
/* trigger source definitions */
typedef enum
{
    TRIGSEL_INPUT_0                              = ((uint8_t)0x00U),                     /*!< trigger input source 0 */
    TRIGSEL_INPUT_1                              = ((uint8_t)0x01U),                     /*!< trigger input source 1 */
    TRIGSEL_INPUT_TRIGSEL_IN0                    = ((uint8_t)0x02U),                     /*!< trigger input source TRIGSEL_IN0 pin */
    TRIGSEL_INPUT_TRIGSEL_IN1                    = ((uint8_t)0x03U),                     /*!< trigger input source TRIGSEL_IN1 pin */
    TRIGSEL_INPUT_TRIGSEL_IN2                    = ((uint8_t)0x04U),                     /*!< trigger input source TRIGSEL_IN2 pin */
    TRIGSEL_INPUT_TRIGSEL_IN3                    = ((uint8_t)0x05U),                     /*!< trigger input source TRIGSEL_IN3 pin */
    TRIGSEL_INPUT_TRIGSEL_IN4                    = ((uint8_t)0x06U),                     /*!< trigger input source TRIGSEL_IN4 pin */
    TRIGSEL_INPUT_TRIGSEL_IN5                    = ((uint8_t)0x07U),                     /*!< trigger input source TRIGSEL_IN5 pin */
    TRIGSEL_INPUT_TRIGSEL_IN6                    = ((uint8_t)0x08U),                     /*!< trigger input source TRIGSEL_IN6 pin */
    TRIGSEL_INPUT_TRIGSEL_IN7                    = ((uint8_t)0x09U),                     /*!< trigger input source TRIGSEL_IN7 pin */
    TRIGSEL_INPUT_TRIGSEL_IN8                    = ((uint8_t)0x0AU),                     /*!< trigger input source TRIGSEL_IN8 pin */
    TRIGSEL_INPUT_TRIGSEL_IN9                    = ((uint8_t)0x0BU),                     /*!< trigger input source TRIGSEL_IN9 pin */
    TRIGSEL_INPUT_TRIGSEL_IN10                   = ((uint8_t)0x0CU),                     /*!< trigger input source TRIGSEL_IN10 pin */
    TRIGSEL_INPUT_TRIGSEL_IN11                   = ((uint8_t)0x0DU),                     /*!< trigger input source TRIGSEL_IN11 pin */
    TRIGSEL_INPUT_CMP_OUT                        = ((uint8_t)0x0EU),                     /*!< trigger input source CMP_OUT */
    TRIGSEL_INPUT_LXTAL_TRG                      = ((uint8_t)0x10U),                     /*!< trigger input source LXTAL_TRG */
    TRIGSEL_INPUT_TIMER1_CH0                     = ((uint8_t)0x11U),                     /*!< trigger input source timer1 channel 0 */
    TRIGSEL_INPUT_TIMER1_CH1                     = ((uint8_t)0x12U),                     /*!< trigger input source timer1 channel 1 */
    TRIGSEL_INPUT_TIMER1_CH2                     = ((uint8_t)0x13U),                     /*!< trigger input source timer1 channel 2 */
    TRIGSEL_INPUT_TIMER1_CH3                     = ((uint8_t)0x14U),                     /*!< trigger input source timer1 channel 3 */
    TRIGSEL_INPUT_TIMER1_TRGO                    = ((uint8_t)0x15U),                     /*!< trigger input source timer1 TRGO */
    TRIGSEL_INPUT_TIMER0_CH0                     = ((uint8_t)0x16U),                     /*!< trigger input source timer0 channel 0 */
    TRIGSEL_INPUT_TIMER0_CH1                     = ((uint8_t)0x17U),                     /*!< trigger input source timer0 channel 1 */
    TRIGSEL_INPUT_TIMER0_CH2                     = ((uint8_t)0x18U),                     /*!< trigger input source timer0 channel 2 */
    TRIGSEL_INPUT_TIMER0_CH3                     = ((uint8_t)0x19U),                     /*!< trigger input source timer0 channel 3 */
    TRIGSEL_INPUT_TIMER0_MCH0                    = ((uint8_t)0x1AU),                     /*!< trigger input source timer0 multi mode channel 0 */
    TRIGSEL_INPUT_TIMER0_MCH1                    = ((uint8_t)0x1BU),                     /*!< trigger input source timer0 multi mode channel 1 */
    TRIGSEL_INPUT_TIMER0_MCH2                    = ((uint8_t)0x1CU),                     /*!< trigger input source timer0 multi mode channel 2 */
    TRIGSEL_INPUT_TIMER0_MCH3                    = ((uint8_t)0x1DU),                     /*!< trigger input source timer0 multi mode channel 3 */
    TRIGSEL_INPUT_TIMER0_TRGO                    = ((uint8_t)0x1EU),                     /*!< trigger input source timer0 TRGO */
    TRIGSEL_INPUT_TIMER7_CH0                     = ((uint8_t)0x1FU),                     /*!< trigger input source timer7 channel 0 */
    TRIGSEL_INPUT_TIMER7_CH1                     = ((uint8_t)0x20U),                     /*!< trigger input source timer7 channel 1 */
    TRIGSEL_INPUT_TIMER7_CH2                     = ((uint8_t)0x21U),                     /*!< trigger input source timer7 channel 2 */
    TRIGSEL_INPUT_TIMER7_CH3                     = ((uint8_t)0x22U),                     /*!< trigger input source timer7 channel 3 */
    TRIGSEL_INPUT_TIMER7_MCH0                    = ((uint8_t)0x23U),                     /*!< trigger input source timer7 multi mode channel 0 */
    TRIGSEL_INPUT_TIMER7_MCH1                    = ((uint8_t)0x24U),                     /*!< trigger input source timer7 multi mode channel 1 */
    TRIGSEL_INPUT_TIMER7_MCH2                    = ((uint8_t)0x25U),                     /*!< trigger input source timer7 multi mode channel 2 */
    TRIGSEL_INPUT_TIMER7_MCH3                    = ((uint8_t)0x26U),                     /*!< trigger input source timer7 multi mode channel 3 */
    TRIGSEL_INPUT_TIMER7_TRGO                    = ((uint8_t)0x27U),                     /*!< trigger input source timer7 TRGO */
    TRIGSEL_INPUT_TIMER19_CH0                    = ((uint8_t)0x28U),                     /*!< trigger input source timer19 channel 0 */
    TRIGSEL_INPUT_TIMER19_CH1                    = ((uint8_t)0x29U),                     /*!< trigger input source timer19 channel 1 */
    TRIGSEL_INPUT_TIMER19_CH2                    = ((uint8_t)0x2AU),                     /*!< trigger input source timer19 channel 2 */
    TRIGSEL_INPUT_TIMER19_CH3                    = ((uint8_t)0x2BU),                     /*!< trigger input source timer19 channel 3 */
    TRIGSEL_INPUT_TIMER19_MCH0                   = ((uint8_t)0x2CU),                     /*!< trigger input source timer19 multi mode channel 0 */
    TRIGSEL_INPUT_TIMER19_MCH1                   = ((uint8_t)0x2DU),                     /*!< trigger input source timer19 multi mode channel 1 */
    TRIGSEL_INPUT_TIMER19_MCH2                   = ((uint8_t)0x2EU),                     /*!< trigger input source timer19 multi mode channel 2 */
    TRIGSEL_INPUT_TIMER19_MCH3                   = ((uint8_t)0x2FU),                     /*!< trigger input source timer19 multi mode channel 3 */
    TRIGSEL_INPUT_TIMER19_TRGO                   = ((uint8_t)0x30U),                     /*!< trigger input source timer19 TRGO */
    TRIGSEL_INPUT_TIMER20_CH0                    = ((uint8_t)0x31U),                     /*!< trigger input source timer20 channel 0 */
    TRIGSEL_INPUT_TIMER20_CH1                    = ((uint8_t)0x32U),                     /*!< trigger input source timer20 channel 1 */
    TRIGSEL_INPUT_TIMER20_CH2                    = ((uint8_t)0x33U),                     /*!< trigger input source timer20 channel 2 */
    TRIGSEL_INPUT_TIMER20_CH3                    = ((uint8_t)0x34U),                     /*!< trigger input source timer20 channel 3 */
    TRIGSEL_INPUT_TIMER20_MCH0                   = ((uint8_t)0x35U),                     /*!< trigger input source timer20 multi mode channel 0 */
    TRIGSEL_INPUT_TIMER20_MCH1                   = ((uint8_t)0x36U),                     /*!< trigger input source timer20 multi mode channel 1 */
    TRIGSEL_INPUT_TIMER20_MCH2                   = ((uint8_t)0x37U),                     /*!< trigger input source timer20 multi mode channel 2 */
    TRIGSEL_INPUT_TIMER20_MCH3                   = ((uint8_t)0x38U),                     /*!< trigger input source timer20 multi mode channel 3 */
    TRIGSEL_INPUT_TIMER20_TRGO                   = ((uint8_t)0x39U),                     /*!< trigger input source timer20 TRGO */
    TRIGSEL_INPUT_TIMER5_TRGO                    = ((uint8_t)0x3AU),                     /*!< trigger input source timer5 TRGO */
    TRIGSEL_INPUT_TIMER6_TRGO                    = ((uint8_t)0x3BU),                     /*!< trigger input source timer6 TRGO */
    TRIGSEL_INPUT_MFCOM_TRIG0                    = ((uint8_t)0x3CU),                     /*!< trigger input source MFCOM TRIG0 */
    TRIGSEL_INPUT_MFCOM_TRIG1                    = ((uint8_t)0x3DU),                     /*!< trigger input source MFCOM TRIG1 */
    TRIGSEL_INPUT_MFCOM_TRIG2                    = ((uint8_t)0x3EU),                     /*!< trigger input source MFCOM TRIG2 */
    TRIGSEL_INPUT_MFCOM_TRIG3                    = ((uint8_t)0x3FU),                     /*!< trigger input source MFCOM TRIG3 */
    TRIGSEL_INPUT_RTC_ALARM                      = ((uint8_t)0x40U),                     /*!< trigger input source RTC alarm */
    TRIGSEL_INPUT_RTC_SECOND                     = ((uint8_t)0x41U),                     /*!< trigger input source RTC second */
    TRIGSEL_INPUT_TRIGSEL_IN12                   = ((uint8_t)0x42U),                     /*!< trigger input source TRIGSEL_IN12 pin */
    TRIGSEL_INPUT_TRIGSEL_IN13                   = ((uint8_t)0x43U),                     /*!< trigger input source TRIGSEL_IN13 pin */
}trigsel_source_enum;

/* target peripheral definitions */
typedef enum
{
    TRIGSEL_OUTPUT_TRIGSEL_OUT0                  = ((uint8_t)0x00U),                     /*!< output target peripheral TRIGSEL_OUT0 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT1                  = ((uint8_t)0x01U),                     /*!< output target peripheral TRIGSEL_OUT1 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT2                  = ((uint8_t)0x02U),                     /*!< output target peripheral TRIGSEL_OUT2 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT3                  = ((uint8_t)0x03U),                     /*!< output target peripheral TRIGSEL_OUT3 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT4                  = ((uint8_t)0x04U),                     /*!< output target peripheral TRIGSEL_OUT4 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT5                  = ((uint8_t)0x05U),                     /*!< output target peripheral TRIGSEL_OUT5 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT6                  = ((uint8_t)0x06U),                     /*!< output target peripheral TRIGSEL_OUT6 pin */
    TRIGSEL_OUTPUT_TRIGSEL_OUT7                  = ((uint8_t)0x07U),                     /*!< output target peripheral TRIGSEL_OUT7 pin */
    TRIGSEL_OUTPUT_ADC0_RTTRG                    = ((uint8_t)0x08U),                     /*!< output target peripheral ADC0_RTTRG */
    TRIGSEL_OUTPUT_ADC1_RTTRG                    = ((uint8_t)0x0CU),                     /*!< output target peripheral ADC1_RTTRG */
    TRIGSEL_OUTPUT_DAC_EXTRIG                    = ((uint8_t)0x10U),                     /*!< output target peripheral DAC_EXTRIG */
    TRIGSEL_OUTPUT_TIMER0_ITI0                   = ((uint8_t)0x14U),                     /*!< output target peripheral TIMER0_ITI0 */
    TRIGSEL_OUTPUT_TIMER0_ITI1                   = ((uint8_t)0x15U),                     /*!< output target peripheral TIMER0_ITI1 */
    TRIGSEL_OUTPUT_TIMER0_ITI2                   = ((uint8_t)0x16U),                     /*!< output target peripheral TIMER0_ITI2 */
    TRIGSEL_OUTPUT_TIMER0_ITI3                   = ((uint8_t)0x17U),                     /*!< output target peripheral TIMER0_ITI3 */
    TRIGSEL_OUTPUT_TIMER0_BRKIN0                 = ((uint8_t)0x18U),                     /*!< output target peripheral TIMER0_BRKIN0 */
    TRIGSEL_OUTPUT_TIMER0_BRKIN1                 = ((uint8_t)0x19U),                     /*!< output target peripheral TIMER0_BRKIN1 */
    TRIGSEL_OUTPUT_TIMER0_BRKIN2                 = ((uint8_t)0x1AU),                     /*!< output target peripheral TIMER0_BRKIN2 */
    TRIGSEL_OUTPUT_TIMER0_BRKIN3                 = ((uint8_t)0x1BU),                     /*!< output target peripheral TIMER0_BRKIN3 */
    TRIGSEL_OUTPUT_TIMER7_ITI0                   = ((uint8_t)0x1CU),                     /*!< output target peripheral TIMER7_ITI0 */
    TRIGSEL_OUTPUT_TIMER7_ITI1                   = ((uint8_t)0x1DU),                     /*!< output target peripheral TIMER7_ITI1 */
    TRIGSEL_OUTPUT_TIMER7_ITI2                   = ((uint8_t)0x1EU),                     /*!< output target peripheral TIMER7_ITI2 */
    TRIGSEL_OUTPUT_TIMER7_ITI3                   = ((uint8_t)0x1FU),                     /*!< output target peripheral TIMER7_ITI3 */
    TRIGSEL_OUTPUT_TIMER7_BRKIN0                 = ((uint8_t)0x20U),                     /*!< output target peripheral TIMER7_BRKIN0 */
    TRIGSEL_OUTPUT_TIMER7_BRKIN1                 = ((uint8_t)0x21U),                     /*!< output target peripheral TIMER7_BRKIN1 */
    TRIGSEL_OUTPUT_TIMER7_BRKIN2                 = ((uint8_t)0x22U),                     /*!< output target peripheral TIMER7_BRKIN2 */
    TRIGSEL_OUTPUT_TIMER7_BRKIN3                 = ((uint8_t)0x23U),                     /*!< output target peripheral TIMER7_BRKIN3 */
    TRIGSEL_OUTPUT_TIMER19_ITI0                  = ((uint8_t)0x24U),                     /*!< output target peripheral TIMER19_ITI0 */
    TRIGSEL_OUTPUT_TIMER19_ITI1                  = ((uint8_t)0x25U),                     /*!< output target peripheral TIMER19_ITI1 */
    TRIGSEL_OUTPUT_TIMER19_ITI2                  = ((uint8_t)0x26U),                     /*!< output target peripheral TIMER19_ITI2 */
    TRIGSEL_OUTPUT_TIMER19_ITI3                  = ((uint8_t)0x27U),                     /*!< output target peripheral TIMER19_ITI3 */
    TRIGSEL_OUTPUT_TIMER19_BRKIN0                = ((uint8_t)0x28U),                     /*!< output target peripheral TIMER19_BRKIN0 */
    TRIGSEL_OUTPUT_TIMER19_BRKIN1                = ((uint8_t)0x29U),                     /*!< output target peripheral TIMER19_BRKIN1 */
    TRIGSEL_OUTPUT_TIMER19_BRKIN2                = ((uint8_t)0x2AU),                     /*!< output target peripheral TIMER19_BRKIN2 */
    TRIGSEL_OUTPUT_TIMER19_BRKIN3                = ((uint8_t)0x2BU),                     /*!< output target peripheral TIMER19_BRKIN3 */
    TRIGSEL_OUTPUT_TIMER20_ITI0                  = ((uint8_t)0x2CU),                     /*!< output target peripheral TIMER20_ITI0 */
    TRIGSEL_OUTPUT_TIMER20_ITI1                  = ((uint8_t)0x2DU),                     /*!< output target peripheral TIMER20_ITI1 */
    TRIGSEL_OUTPUT_TIMER20_ITI2                  = ((uint8_t)0x2EU),                     /*!< output target peripheral TIMER20_ITI2 */
    TRIGSEL_OUTPUT_TIMER20_ITI3                  = ((uint8_t)0x2FU),                     /*!< output target peripheral TIMER20_ITI3 */
    TRIGSEL_OUTPUT_TIMER20_BRKIN0                = ((uint8_t)0x30U),                     /*!< output target peripheral TIMER20_BRKIN0 */
    TRIGSEL_OUTPUT_TIMER20_BRKIN1                = ((uint8_t)0x31U),                     /*!< output target peripheral TIMER20_BRKIN1 */
    TRIGSEL_OUTPUT_TIMER20_BRKIN2                = ((uint8_t)0x32U),                     /*!< output target peripheral TIMER20_BRKIN2 */
    TRIGSEL_OUTPUT_TIMER20_BRKIN3                = ((uint8_t)0x33U),                     /*!< output target peripheral TIMER20_BRKIN3 */
    TRIGSEL_OUTPUT_TIMER1_ITI0                   = ((uint8_t)0x34U),                     /*!< output target peripheral TIMER1_ITI0 */
    TRIGSEL_OUTPUT_TIMER1_ITI1                   = ((uint8_t)0x35U),                     /*!< output target peripheral TIMER1_ITI1 */
    TRIGSEL_OUTPUT_TIMER1_ITI2                   = ((uint8_t)0x36U),                     /*!< output target peripheral TIMER1_ITI2 */
    TRIGSEL_OUTPUT_TIMER1_ITI3                   = ((uint8_t)0x37U),                     /*!< output target peripheral TIMER1_ITI3 */
    TRIGSEL_OUTPUT_MFCOM_TRG_TIMER0              = ((uint8_t)0x38U),                     /*!< output target peripheral MFCOM_TRG_TIMER0 */
    TRIGSEL_OUTPUT_MFCOM_TRG_TIMER1              = ((uint8_t)0x39U),                     /*!< output target peripheral MFCOM_TRG_TIMER1 */
    TRIGSEL_OUTPUT_MFCOM_TRG_TIMER2              = ((uint8_t)0x3AU),                     /*!< output target peripheral MFCOM_TRG_TIMER2 */
    TRIGSEL_OUTPUT_MFCOM_TRG_TIMER3              = ((uint8_t)0x3BU),                     /*!< output target peripheral MFCOM_TRG_TIMER3 */
    TRIGSEL_OUTPUT_CAN0_EX_TIME_TICK             = ((uint8_t)0x3CU),                     /*!< output target peripheral CAN0_EX_TIME_TICK */
    TRIGSEL_OUTPUT_CAN1_EX_TIME_TICK             = ((uint8_t)0x40U),                     /*!< output target peripheral CAN1_EX_TIME_TICK */
}trigsel_periph_enum;

/* function declarations */
/* set the trigger input signal for target peripheral */
void trigsel_init(trigsel_periph_enum target_periph, trigsel_source_enum trigger_source);
/* get the trigger input signal for target peripheral */
uint8_t trigsel_trigger_source_get(trigsel_periph_enum target_periph);
/* lock the trigger register */
void trigsel_register_lock_set(trigsel_periph_enum target_periph);
/* get the trigger register lock status */
FlagStatus trigsel_register_lock_get(trigsel_periph_enum target_periph);

#endif /* GD32A50X_TRIGSEL_H */
