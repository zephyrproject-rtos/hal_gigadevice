/*!
    \file    gd32a50x_cmp.c
    \brief   CMP driver

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

#include "gd32a50x_cmp.h"

/*!
    \brief      deinitialize comparator
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_deinit(void)
{
    rcu_periph_reset_enable(RCU_CMPRST);
    rcu_periph_reset_disable(RCU_CMPRST);
}

/*!
    \brief      initialize comparator mode
    \param[in]  operating_mode
      \arg        CMP_HIGHSPEED: high speed mode
      \arg        CMP_MIDDLESPEED: medium speed mode
      \arg        CMP_LOWSPEED: low speed mode
    \param[in]  inverting_input
      \arg        CMP_1_4VREFINT: CMP inverting input VREFINT *1/4
      \arg        CMP_1_2VREFINT: CMP inverting input VREFINT *1/2
      \arg        CMP_3_4VREFINT: CMP inverting input VREFINT *3/4
      \arg        CMP_VREFINT: CMP inverting input VREFINT
      \arg        CMP_DAC_OUT: CMP inverting input DAC_OUT(PA4¡¢PA5)
      \arg        CMP_IM_PC11: CMP inverting input PC11
      \arg        CMP_IM_PC10: CMP inverting input PC10
      \arg        CMP_IM_PB8: CMP inverting input PB8
      \arg        CMP_IM_PA0: CMP inverting input PA0
      \arg        CMP_IM_PA3: CMP inverting input PA3
      \arg        CMP_IM_PA4: CMP inverting input PA4
      \arg        CMP_IM_PA5: CMP inverting input PA5
      \arg        CMP_IM_PA6: CMP inverting input PA6
    \param[in]  plus_input
      \arg        CMP_IP_PC11: CMP plus input PC11
      \arg        CMP_IP_PC10: CMP plus input PC10
      \arg        CMP_IP_PB8: CMP plus input PB8
      \arg        CMP_IP_PA0: CMP plus input PA0
      \arg        CMP_IP_PA3: CMP plus input PA3
      \arg        CMP_IP_PA4: CMP plus input PA4
      \arg        CMP_IP_PA5: CMP plus input PA5
      \arg        CMP_IP_PA6: CMP plus input PA6
    \param[in]  output_hysteresis
      \arg        CMP_HYSTERESIS_NO: output no hysteresis
      \arg        CMP_HYSTERESIS_LOW: output low hysteresis
      \arg        CMP_HYSTERESIS_MIDDLE: output middle hysteresis
      \arg        CMP_HYSTERESIS_HIGH: output high hysteresis
    \param[out] none
    \retval     none
*/
void cmp_mode_init(operating_mode_enum operating_mode, cmp_inverting_input_enum inverting_input, cmp_plus_input_enum plus_input,
                   cmp_hysteresis_enum output_hysteresis)
{
    uint32_t cmp_cs = 0U;
    cmp_cs = CMP_CS;
    /* initialize comparator  mode */
    cmp_cs &= ~(uint32_t)(CMP_CS_PM | CMP_CS_MESEL | CMP_CS_MISEL | CMP_CS_PSEL | CMP_CS_HST);
    cmp_cs |= (uint32_t)(CS_CMPPM(operating_mode) | CS_CMPMSEL(inverting_input) | CS_CMPPSEL(plus_input) | CS_CMPHST(output_hysteresis));
    CMP_CS =  cmp_cs;
}

/*!
    \brief      initialize comparator output
    \param[in]  none
      \param[in]  output_selection
      \arg        CMP_OUTPUT_NONE: output no selection
      \arg        CMP_OUTPUT_TIMER0IC0: TIMER 0 channel0 input capture
      \arg        CMP_OUTPUT_TIMER7IC0: TIMER 7 channel0 input capture
    \param[in]  output_polarity
      \arg        CMP_OUTPUT_POLARITY_INVERTED: output is inverted
      \arg        CMP_OUTPUT_POLARITY_NOINVERTED: output is not inverted
    \param[out] none
    \retval     none
*/
void cmp_output_init(cmp_output_enum output_selection, cmp_output_inv_enum output_polarity)
{
    uint32_t cmp_cs = 0U;
    cmp_cs = CMP_CS;
    /* initialize comparator  output */
    cmp_cs &= ~(uint32_t)CMP_CS_OSEL;
    cmp_cs |= (uint32_t)CS_CMPOSEL(output_selection);
    /* output polarity */
    if(CMP_OUTPUT_POLARITY_INVERTED == output_polarity) {
        cmp_cs |= (uint32_t)CMP_CS_PL;
    } else {
        cmp_cs &= ~(uint32_t)CMP_CS_PL;
    }
    CMP_CS = cmp_cs;
}

/*!
    \brief      initialize comparator blanking function
    \param[in]  none
    \param[in]  blanking_source_selection
      \arg        CMP_BLANKING_NONE: output no selection
      \arg        CMP_BLANKING_TIMER0_OC1: TIMER 0 output channel1
      \arg        CMP_BLANKING_TIMER7_OC1: TIMER 7 output channel1
      \arg        CMP_BLANKING_TIMER1_OC1: TIMER 1 output channel1
    \param[out] none
    \retval     none
*/
void cmp_outputblank_init(blanking_source_enum blanking_source_selection)
{
    uint32_t cmp_cs = 0U;
    cmp_cs = CMP_CS;
    cmp_cs &= ~(uint32_t)CMP_CS_BLK;
    cmp_cs |= (uint32_t)CS_CMPBLK(blanking_source_selection);
    CMP_CS = cmp_cs;
}

/*!
    \brief      enable comparator
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_enable(void)
{
    CMP_CS |= (uint32_t)CMP_CS_EN;
}

/*!
    \brief      disable comparator
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_disable(void)
{
    CMP_CS &= ~(uint32_t)CMP_CS_EN;
}

/*!
    \brief      enable the voltage scaler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_voltage_scaler_enable(void)
{
    CMP_CS |= (uint32_t)CMP_CS_SEN;
}

/*!
    \brief      disable the voltage scaler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_voltage_scaler_disable(void)
{
    CMP_CS &= ~(uint32_t)CMP_CS_SEN;
}

/*!
    \brief      enable the scaler bridge
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_scaler_bridge_enable(void)
{
    CMP_CS |= (uint32_t)CMP_CS_BEN;
}

/*!
    \brief      disable the scaler bridge
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_scaler_bridge_disable(void)
{
    CMP_CS &= ~(uint32_t)CMP_CS_BEN;
}

/*!
    \brief      lock the comparator
    \param[in]  none
    \param[out] none
    \retval     none
*/
void cmp_lock_enable(void)
{
    /* lock CMP */
    CMP_CS |= (uint32_t)CMP_CS_LK;
}

/*!
    \brief      get output level
    \param[in]  none
    \param[out] none
    \retval     the output level
*/
cmp_output_state_enum cmp_output_level_get(void)
{
    /* get output level of CMP */
    if(CMP_CS & CMP_CS_OT) {
        return CMP_OUTPUTLEVEL_HIGH;
    } else {
        return CMP_OUTPUTLEVEL_LOW;
    }
}
