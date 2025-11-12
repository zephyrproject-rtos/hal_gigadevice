/*!
    \file    gd32c2x1_cmp.c
    \brief   CMP driver

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

#include "gd32c2x1_cmp.h"

/*!
    \brief      CMP deinit (API_ID(0x0001U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_deinit(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS &= ((uint32_t)0x00000000U);
    }else if(CMP1 == cmp_periph){
        CMP1_CS &= ((uint32_t)0x00000000U);
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      CMP mode init (API_ID(0x0002U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[in]  operating_mode
      \arg        CMP_MODE_HIGHSPEED: high speed mode
      \arg        CMP_MODE_MIDDLESPEED: medium speed mode
      \arg        CMP_MODE_LOWSPEED: low speed mode
      \arg        CMP_MODE_VERYLOWSPEED: very low speed mode
    \param[in]  inverting_input
      \arg        CMP_INVERTING_INPUT_1_4VREFINT: VREFINT *1/4 input
      \arg        CMP_INVERTING_INPUT_1_2VREFINT: VREFINT *1/2 input
      \arg        CMP_INVERTING_INPUT_3_4VREFINT: VREFINT *3/4 input
      \arg        CMP_INVERTING_INPUT_VREFINT: VREFINT input
      \arg        CMP_INVERTING_INPUT_PB2_PB6: PB2 input for CMP0 or PB6 input for CMP1
      \arg        CMP_INVERTING_INPUT_PA0_PA2: PA0 input for CMP0 or PA2 input for CMP1
      \arg        CMP_INVERTING_INPUT_PB1_PB3: PB1 input for CMP0 or PB3 input for CMP1
      \arg        CMP_INVERTING_INPUT_VSSA_PB4: VSSA input for CMP0 or PB4 input for CMP1
    \param[in]  output_hysteresis
      \arg        CMP_HYSTERESIS_NO: output no hysteresis
      \arg        CMP_HYSTERESIS_LOW: output low hysteresis
      \arg        CMP_HYSTERESIS_MIDDLE: output middle hysteresis
      \arg        CMP_HYSTERESIS_HIGH: output high hysteresis
    \param[out] none
    \retval     none
*/
void cmp_mode_init(cmp_enum cmp_periph, uint32_t operating_mode, uint32_t inverting_input, uint32_t output_hysteresis)
{
    uint32_t temp;

    if(CMP0 == cmp_periph){
        /* initialize comparator 0 mode */
        temp = CMP0_CS;
        temp &= ~(uint32_t)(CMP_CS_CMPXM | CMP_CS_CMPXMSEL | CMP_CS_CMPXHST);
        temp |= (uint32_t)((operating_mode & CMP_CS_CMPXM) | (inverting_input & CMP_CS_CMPXMSEL) | (output_hysteresis & CMP_CS_CMPXHST));
        CMP0_CS = temp;
    }else if(CMP1 == cmp_periph){
        /* initialize comparator 1 mode */
        temp = CMP1_CS;
        temp &= ~(uint32_t)(CMP_CS_CMPXM | CMP_CS_CMPXMSEL | CMP_CS_CMPXHST);
        temp |= (uint32_t)((operating_mode & CMP_CS_CMPXM) | (inverting_input & CMP_CS_CMPXMSEL) | (output_hysteresis & CMP_CS_CMPXHST));
        CMP1_CS = temp;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      CMP output init (API_ID(0x0003U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[in]  output_polarity
      \arg        CMP_OUTPUT_POLARITY_INVERTED: output is inverted
      \arg        CMP_OUTPUT_POLARITY_NONINVERTED: output is not inverted
    \param[out] none
    \retval     none
*/
void cmp_output_init(cmp_enum cmp_periph, uint32_t output_polarity)
{
    uint32_t temp;

    if(CMP0 == cmp_periph){
        /* initialize comparator 0 output */
        temp = CMP0_CS;
        /* output polarity */
        if(CMP_OUTPUT_POLARITY_INVERTED == output_polarity){
            temp |= (uint32_t)CMP_CS_CMPXPL;
        }else{
            temp &= ~(uint32_t)CMP_CS_CMPXPL;
        }
        CMP0_CS = temp;
    }else if(CMP1 == cmp_periph){
        /* initialize comparator 1 output */
        temp = CMP1_CS;
        /* output polarity */
        if(CMP_OUTPUT_POLARITY_INVERTED == output_polarity){
            temp |= (uint32_t)CMP_CS_CMPXPL;
        }else{
            temp &= ~(uint32_t)CMP_CS_CMPXPL;
        }
        CMP1_CS = temp;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      CMP output blanking function init (API_ID(0x0004U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[in]  blanking_source_selection 
      \arg        CMP_BLANKING_NONE: CMP no blanking source
      \arg        CMP_BLANKING_TIMER0_OC1: CMP TIMER0_CH1 output compare signal selected as blanking source
      \arg        CMP_BLANKING_TIMER2_OC1: CMP TIMER2_CH1 output compare signal selected as blanking source
      \arg        CMP_BLANKING_TIMER13_OC0: CMP TIMER13_CH0 output compare signal selected as blanking source
      \arg        CMP_BLANKING_TIMER15_OC0: CMP TIMER15_CH0 output compare signal selected as blanking source
    \param[out] none
    \retval     none
*/
void cmp_blanking_init(cmp_enum cmp_periph, uint32_t blanking_source_selection)
{
    uint32_t temp;

    if(CMP0 == cmp_periph){
        temp = CMP0_CS;
        temp &= ~(uint32_t)CMP_CS_CMPXBLK;
        temp |= (uint32_t)(blanking_source_selection & CMP_CS_CMPXBLK);
        CMP0_CS = temp;
    }else if(CMP1 == cmp_periph){
        temp = CMP1_CS;
        temp &= ~(uint32_t)CMP_CS_CMPXBLK;
        temp |= (uint32_t)(blanking_source_selection & CMP_CS_CMPXBLK);
        CMP1_CS = temp;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      enable CMP (API_ID(0x0005U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_enable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS |= (uint32_t)CMP_CS_CMPXEN;
    }else if(CMP1 == cmp_periph){
        CMP1_CS |= (uint32_t)CMP_CS_CMPXEN;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      disable CMP (API_ID(0x0006U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_disable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS &= ~(uint32_t)CMP_CS_CMPXEN;
    }else if(CMP1 == cmp_periph){
        CMP1_CS &= ~(uint32_t)CMP_CS_CMPXEN;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      enable CMP switch (API_ID(0x0007U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_switch_enable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        /* enable CMP0 switch */
        CMP0_CS |= (uint32_t)CMP_CS_CMPXSW;
    }else if(CMP1 == cmp_periph){
        /* enable CMP1 switch */
        CMP1_CS |= (uint32_t)CMP_CS_CMPXSW;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      disable CMP switch (API_ID(0x0008U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_switch_disable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        /* disable CMP0 switch */
        CMP0_CS &= ~(uint32_t)CMP_CS_CMPXSW;
    }else if(CMP1 == cmp_periph){
        /* disable CMP1 switch */
        CMP1_CS &= ~(uint32_t)CMP_CS_CMPXSW;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      lock the CMP (API_ID(0x0009U))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_lock_enable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        /* lock CMP0 */
        CMP0_CS |= (uint32_t)CMP_CS_CMPXLK;
    }else if(CMP1 == cmp_periph){
        /* lock CMP1 */
        CMP1_CS |= (uint32_t)CMP_CS_CMPXLK;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      enable the voltage scaler (API_ID(0x000AU))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_voltage_scaler_enable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS |= (uint32_t)CMP_CS_CMPXSEN;
    }else if(CMP1 == cmp_periph){
        CMP1_CS |= (uint32_t)CMP_CS_CMPXSEN;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      disable the voltage scaler (API_ID(0x000BU))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_voltage_scaler_disable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS &= ~(uint32_t)CMP_CS_CMPXSEN;
    }else if(CMP1 == cmp_periph){
        CMP1_CS &= ~(uint32_t)CMP_CS_CMPXSEN;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      enable the scaler bridge (API_ID(0x000CU))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_scaler_bridge_enable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS |= (uint32_t)CMP_CS_CMPXBEN;
    }else if(CMP1 == cmp_periph){
        CMP1_CS |= (uint32_t)CMP_CS_CMPXBEN;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      disable the scaler bridge (API_ID(0x000DU))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     none
*/
void cmp_scaler_bridge_disable(cmp_enum cmp_periph)
{
    if(CMP0 == cmp_periph){
        CMP0_CS &= ~(uint32_t)CMP_CS_CMPXBEN;
    }else if(CMP1 == cmp_periph){
        CMP1_CS &= ~(uint32_t)CMP_CS_CMPXBEN;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      get output level (API_ID(0x000EU))
    \param[in]  cmp_periph
      \arg        CMP0: comparator 0
      \arg        CMP1: comparator 1
    \param[out] none
    \retval     the output level
*/
uint32_t cmp_output_level_get(cmp_enum cmp_periph)
{
    uint32_t reval = CMP_OUTPUTLEVEL_LOW;

    if(CMP0 == cmp_periph){
        /* get output level of CMP0 */
        if((uint32_t)RESET != (CMP0_CS & CMP_CS_CMPXO)) {
            reval = CMP_OUTPUTLEVEL_HIGH;
        }
    }else{
        /* get output level of CMP1 */
        if((uint32_t)RESET != (CMP1_CS & CMP_CS_CMPXO)) {
            reval = CMP_OUTPUTLEVEL_HIGH;
        }
    }
    return reval;
}
