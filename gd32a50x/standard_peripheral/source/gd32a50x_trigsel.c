/*!
    \file    gd32a50x_trigsel.c
    \brief   TRIGSEL driver

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

#include "gd32a50x_trigsel.h"

/* TRIGSEL target register redefine */
#define TRIGSEL_TARGET_REG(target_periph)          REG32(TRIGSEL + ((uint8_t)(target_periph) & BITS(2, 7)))                                /*!< target peripheral register */
#define TRIGSEL_TARGET_PERIPH_SHIFT(target_periph) (((uint8_t)(target_periph) & BITS(0, 1)) << 3)                                          /*!< bit shift in target peripheral register */
#define TRIGSEL_TARGET_PERIPH_MASK(target_periph)  ((uint32_t)(TRIGSEL_TARGET_INSEL0 << TRIGSEL_TARGET_PERIPH_SHIFT(target_periph)))       /*!< bit mask in target peripheral register */

/*!
    \brief      set the trigger input signal for target peripheral
    \param[in]  target_periph: target peripheral value
                only one parameter can be selected which is shown as below:
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT0: output target peripheral TRIGSEL_OUT0 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT1: output target peripheral TRIGSEL_OUT1 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT2: output target peripheral TRIGSEL_OUT2 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT3: output target peripheral TRIGSEL_OUT3 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT4: output target peripheral TRIGSEL_OUT4 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT5: output target peripheral TRIGSEL_OUT5 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT6: output target peripheral TRIGSEL_OUT6 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT7: output target peripheral TRIGSEL_OUT7 pin
      \arg        TRIGSEL_OUTPUT_ADC0_RTTRG: output target peripheral ADC0_RTTRG
      \arg        TRIGSEL_OUTPUT_ADC1_RTTRG: output target peripheral ADC1_RTTRG
      \arg        TRIGSEL_OUTPUT_DAC_EXTRIG: output target peripheral DAC_EXTRIG
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER1_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER1_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER1_ITI3
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER0 output target peripheral MFCOM_TRG_TIMER0
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER1 output target peripheral MFCOM_TRG_TIMER1
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER2 output target peripheral MFCOM_TRG_TIMER2
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER3 output target peripheral MFCOM_TRG_TIMER3
      \arg        TRIGSEL_OUTPUT_CAN0_EX_TIME_TICK output target peripheral CAN0_EX_TIME_TICK
      \arg        TRIGSEL_OUTPUT_CAN1_EX_TIME_TICK output target peripheral CAN1_EX_TIME_TICK
    \param[in]  trigger_source: trigger source value
                only one parameter can be selected which is shown as below:
      \arg        TRIGSEL_INPUT_0: trigger input source 0
      \arg        TRIGSEL_INPUT_1: trigger input source 1
      \arg        TRIGSEL_INPUT_TRIGSEL_IN0: trigger input source TRIGSEL_IN0 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN1: trigger input source TRIGSEL_IN1 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN2: trigger input source TRIGSEL_IN2 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN3: trigger input source TRIGSEL_IN3 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN4: trigger input source TRIGSEL_IN4 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN5: trigger input source TRIGSEL_IN5 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN6: trigger input source TRIGSEL_IN6 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN7: trigger input source TRIGSEL_IN7 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN8: trigger input source TRIGSEL_IN8 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN9: trigger input source TRIGSEL_IN9 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN10: trigger input source TRIGSEL_IN10 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN11: trigger input source TRIGSEL_IN11 pin
      \arg        TRIGSEL_INPUT_CMP_OUT: trigger input source CMP_OUT
      \arg        TRIGSEL_INPUT_LXTAL_TRG: trigger input source LSE_TRG
      \arg        TRIGSEL_INPUT_TIMER1_CH0: trigger input source timer1 channel 0
      \arg        TRIGSEL_INPUT_TIMER1_CH1: trigger input source timer1 channel 1
      \arg        TRIGSEL_INPUT_TIMER1_CH2: trigger input source timer1 channel 2
      \arg        TRIGSEL_INPUT_TIMER1_CH3: trigger input source timer1 channel 3
      \arg        TRIGSEL_INPUT_TIMER1_TRGO: trigger input source timer1 TRGO
      \arg        TRIGSEL_INPUT_TIMER0_CH0: trigger input source timer0 channel 0
      \arg        TRIGSEL_INPUT_TIMER0_CH1: trigger input source timer0 channel 1
      \arg        TRIGSEL_INPUT_TIMER0_CH2: trigger input source timer0 channel 2
      \arg        TRIGSEL_INPUT_TIMER0_CH3: trigger input source timer0 channel 3
      \arg        TRIGSEL_INPUT_TIMER0_MCH0: trigger input source timer0 channel 0N
      \arg        TRIGSEL_INPUT_TIMER0_MCH1: trigger input source timer0 channel 1N
      \arg        TRIGSEL_INPUT_TIMER0_MCH2: trigger input source timer0 channel 2N
      \arg        TRIGSEL_INPUT_TIMER0_MCH3: trigger input source timer0 channel 3N
      \arg        TRIGSEL_INPUT_TIMER0_TRGO: trigger input source timer0 TRGO
      \arg        TRIGSEL_INPUT_TIMER7_CH0: trigger input source timer7 channel 0
      \arg        TRIGSEL_INPUT_TIMER7_CH1: trigger input source timer7 channel 1
      \arg        TRIGSEL_INPUT_TIMER7_CH2: trigger input source timer7 channel 2
      \arg        TRIGSEL_INPUT_TIMER7_CH3: trigger input source timer7 channel 3
      \arg        TRIGSEL_INPUT_TIMER7_MCH0: trigger input source timer7 channel 0N
      \arg        TRIGSEL_INPUT_TIMER7_MCH1: trigger input source timer7 channel 1N
      \arg        TRIGSEL_INPUT_TIMER7_MCH2: trigger input source timer7 channel 2N
      \arg        TRIGSEL_INPUT_TIMER7_MCH3: trigger input source timer7 channel 3N
      \arg        TRIGSEL_INPUT_TIMER7_TRGO: trigger input source timer7 TRGO
      \arg        TRIGSEL_INPUT_TIMER19_CH0: trigger input source timer19 channel 0
      \arg        TRIGSEL_INPUT_TIMER19_CH1: trigger input source timer19 channel 1
      \arg        TRIGSEL_INPUT_TIMER19_CH2: trigger input source timer19 channel 2
      \arg        TRIGSEL_INPUT_TIMER19_CH3: trigger input source timer19 channel 3
      \arg        TRIGSEL_INPUT_TIMER19_MCH0: trigger input source timer19 channel 0N
      \arg        TRIGSEL_INPUT_TIMER19_MCH1: trigger input source timer19 channel 1N
      \arg        TRIGSEL_INPUT_TIMER19_MCH2: trigger input source timer19 channel 2N
      \arg        TRIGSEL_INPUT_TIMER19_MCH3: trigger input source timer19 channel 3N
      \arg        TRIGSEL_INPUT_TIMER19_TRGO: trigger input source timer19 TRGO
      \arg        TRIGSEL_INPUT_TIMER20_CH0: trigger input source timer20 channel 0
      \arg        TRIGSEL_INPUT_TIMER20_CH1: trigger input source timer20 channel 1
      \arg        TRIGSEL_INPUT_TIMER20_CH2: trigger input source timer20 channel 2
      \arg        TRIGSEL_INPUT_TIMER20_CH3: trigger input source timer20 channel 3
      \arg        TRIGSEL_INPUT_TIMER20_MCH0: trigger input source timer20 channel 0N
      \arg        TRIGSEL_INPUT_TIMER20_MCH1: trigger input source timer20 channel 1N
      \arg        TRIGSEL_INPUT_TIMER20_MCH2: trigger input source timer20 channel 2N
      \arg        TRIGSEL_INPUT_TIMER20_MCH3: trigger input source timer20 channel 3N
      \arg        TRIGSEL_INPUT_TIMER20_TRGO: trigger input source timer20 TRGO
      \arg        TRIGSEL_INPUT_TIMER5_TRGO: trigger input source timer5 TRGO
      \arg        TRIGSEL_INPUT_TIMER6_TRGO: trigger input source timer6 TRGO
      \arg        TRIGSEL_INPUT_MFCOM_TRIG0: trigger input source MFCOM TRIG0
      \arg        TRIGSEL_INPUT_MFCOM_TRIG1: trigger input source MFCOM TRIG1
      \arg        TRIGSEL_INPUT_MFCOM_TRIG2: trigger input source MFCOM TRIG2
      \arg        TRIGSEL_INPUT_MFCOM_TRIG3: trigger input source MFCOM TRIG3
      \arg        TRIGSEL_INPUT_RTC_ALARM: trigger input source RTC alarm
      \arg        TRIGSEL_INPUT_RTC_SECOND: trigger input source RTC second
      \arg        TRIGSEL_INPUT_TRIGSEL_IN12: trigger input source TRIGSEL_IN12 pin
      \arg        TRIGSEL_INPUT_TRIGSEL_IN13: trigger input source TRIGSEL_IN13 pin
    \param[out] none
    \retval     none
*/
void trigsel_init(trigsel_periph_enum target_periph, trigsel_source_enum trigger_source)
{
    /* if register write is enabled, set trigger source to target peripheral */
    if (RESET == trigsel_register_lock_get(target_periph)){
        TRIGSEL_TARGET_REG(target_periph) &= ~TRIGSEL_TARGET_PERIPH_MASK(target_periph);
        TRIGSEL_TARGET_REG(target_periph) |= ((uint32_t)trigger_source << TRIGSEL_TARGET_PERIPH_SHIFT(target_periph)) & TRIGSEL_TARGET_PERIPH_MASK(target_periph);
    }
}

/*!
    \brief      get the trigger input signal for target peripheral
    \param[in]  target_periph: target peripheral value
                only one parameter can be selected which is shown as below:
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT0: output target peripheral TRIGSEL_OUT0 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT1: output target peripheral TRIGSEL_OUT1 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT2: output target peripheral TRIGSEL_OUT2 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT3: output target peripheral TRIGSEL_OUT3 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT4: output target peripheral TRIGSEL_OUT4 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT5: output target peripheral TRIGSEL_OUT5 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT6: output target peripheral TRIGSEL_OUT6 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT7: output target peripheral TRIGSEL_OUT7 pin
      \arg        TRIGSEL_OUTPUT_ADC0_RTTRG: output target peripheral ADC0_RTTRG
      \arg        TRIGSEL_OUTPUT_ADC1_RTTRG: output target peripheral ADC1_RTTRG
      \arg        TRIGSEL_OUTPUT_DAC_EXTRIG: output target peripheral DAC_EXTRIG
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER1_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER1_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER1_ITI3
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER0 output target peripheral MFCOM_TRG_TIMER0
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER1 output target peripheral MFCOM_TRG_TIMER1
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER2 output target peripheral MFCOM_TRG_TIMER2
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER3 output target peripheral MFCOM_TRG_TIMER3
      \arg        TRIGSEL_OUTPUT_CAN0_EX_TIME_TICK output target peripheral CAN0_EX_TIME_TICK
      \arg        TRIGSEL_OUTPUT_CAN1_EX_TIME_TICK output target peripheral CAN1_EX_TIME_TICK
    \param[out] none
    \retval  trigger_source: trigger source value(0~67)
*/
uint8_t trigsel_trigger_source_get(trigsel_periph_enum target_periph)
{
    uint8_t trigger_source;

    trigger_source = (uint8_t)((TRIGSEL_TARGET_REG(target_periph) & TRIGSEL_TARGET_PERIPH_MASK(target_periph)) >> TRIGSEL_TARGET_PERIPH_SHIFT(target_periph));

    return trigger_source;
}

/*!
    \brief      lock the trigger register
    \param[in]  target_periph: target peripheral value
                only one parameter can be selected which is shown as below:
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT0: output target peripheral TRIGSEL_OUT0 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT1: output target peripheral TRIGSEL_OUT1 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT2: output target peripheral TRIGSEL_OUT2 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT3: output target peripheral TRIGSEL_OUT3 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT4: output target peripheral TRIGSEL_OUT4 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT5: output target peripheral TRIGSEL_OUT5 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT6: output target peripheral TRIGSEL_OUT6 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT7: output target peripheral TRIGSEL_OUT7 pin
      \arg        TRIGSEL_OUTPUT_ADC0_RTTRG: output target peripheral ADC0_RTTRG
      \arg        TRIGSEL_OUTPUT_ADC1_RTTRG: output target peripheral ADC1_RTTRG
      \arg        TRIGSEL_OUTPUT_DAC_EXTRIG: output target peripheral DAC_EXTRIG
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER1_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER1_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER1_ITI3
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER0 output target peripheral MFCOM_TRG_TIMER0
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER1 output target peripheral MFCOM_TRG_TIMER1
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER2 output target peripheral MFCOM_TRG_TIMER2
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER3 output target peripheral MFCOM_TRG_TIMER3
      \arg        TRIGSEL_OUTPUT_CAN0_EX_TIME_TICK output target peripheral CAN0_EX_TIME_TICK
      \arg        TRIGSEL_OUTPUT_CAN1_EX_TIME_TICK output target peripheral CAN1_EX_TIME_TICK
    \param[out] none
    \retval     none
*/
void trigsel_register_lock_set(trigsel_periph_enum target_periph)
{
    /*!< lock target peripheral register */
    TRIGSEL_TARGET_REG(target_periph) |= TRIGSEL_TARGET_LK;
}

/*!
    \brief      get the trigger register lock status
    \param[in]  target_periph: target peripheral value
                only one parameter can be selected which is shown as below:
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT0: output target peripheral TRIGSEL_OUT0 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT1: output target peripheral TRIGSEL_OUT1 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT2: output target peripheral TRIGSEL_OUT2 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT3: output target peripheral TRIGSEL_OUT3 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT4: output target peripheral TRIGSEL_OUT4 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT5: output target peripheral TRIGSEL_OUT5 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT6: output target peripheral TRIGSEL_OUT6 pin
      \arg        TRIGSEL_OUTPUT_TRIGSEL_OUT7: output target peripheral TRIGSEL_OUT7 pin
      \arg        TRIGSEL_OUTPUT_ADC0_RTTRG: output target peripheral ADC0_RTTRG
      \arg        TRIGSEL_OUTPUT_ADC1_RTTRG: output target peripheral ADC1_RTTRG
      \arg        TRIGSEL_OUTPUT_DAC_EXTRIG: output target peripheral DAC_EXTRIG
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER0_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER0_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER7_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER7_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER19_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI1: output target peripheral TIMER0_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI2: output target peripheral TIMER0_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER20_ITI3: output target peripheral TIMER0_ITI3
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN0
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN1
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN2
      \arg        TRIGSEL_OUTPUT_TIMER20_BRKIN0: output target peripheral TIMER0_BRKIN3
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI0: output target peripheral TIMER0_ITI0
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI1: output target peripheral TIMER1_ITI1
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI2: output target peripheral TIMER1_ITI2
      \arg        TRIGSEL_OUTPUT_TIMER1_ITI3: output target peripheral TIMER1_ITI3
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER0 output target peripheral MFCOM_TRG_TIMER0
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER1 output target peripheral MFCOM_TRG_TIMER1
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER2 output target peripheral MFCOM_TRG_TIMER2
      \arg        TRIGSEL_OUTPUT_MFCOM_TRG_TIMER3 output target peripheral MFCOM_TRG_TIMER3
      \arg        TRIGSEL_OUTPUT_CAN0_EX_TIME_TICK output target peripheral CAN0_EX_TIME_TICK
      \arg        TRIGSEL_OUTPUT_CAN1_EX_TIME_TICK output target peripheral CAN1_EX_TIME_TICK
    \param[out] none
    \retval     SET or RESET
*/
FlagStatus trigsel_register_lock_get(trigsel_periph_enum target_periph)
{
    if(0U != (TRIGSEL_TARGET_REG(target_periph) & TRIGSEL_TARGET_LK)){
        return SET;
    }else{
        return RESET;
    }
}
