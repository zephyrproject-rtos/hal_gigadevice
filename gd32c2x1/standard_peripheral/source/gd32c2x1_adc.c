/*!
    \file    gd32c2x1_adc.c
    \brief   ADC driver

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

#include "gd32c2x1_adc.h"

/* discontinuous mode macro*/
#define ADC_CHANNEL_LENGTH_SUBTRACT_ONE            ((uint8_t)0x01U)

/* ADC routine channel macro */
#define ADC_ROUTINE_CHANNEL_RANK_SIX               ((uint8_t)0x06U)
#define ADC_ROUTINE_CHANNEL_RANK_TWELVE            ((uint8_t)0x0CU)
#define ADC_ROUTINE_CHANNEL_RANK_LENGTH            ((uint8_t)0x05U)

/* ADC sampling time macro */
#define ADC_CHANNEL_SAMPLE_TEN                     ((uint8_t)0x0AU)
#define ADC_CHANNEL_SAMPLE_LENGTH                  ((uint8_t)0x03U)

/* ADC inserted channel macro */
#define ADC_INSERTED_CHANNEL_RANK_LENGTH           ((uint8_t)0x05U)
#define ADC_INSERTED_CHANNEL_SHIFT_OFFSET          ((uint8_t)0x0FU)

/* ADC inserted channel offset macro */
#define ADC_OFFSET_LENGTH                          ((uint8_t)0x03U)
#define ADC_OFFSET_SHIFT_LENGTH                    ((uint8_t)0x04U)

/* ADC routine sequence length */
#define ROUTINE_LENGTH_HIGH_VALUE                  ((uint32_t)0x00000010U)

#define ADC_CHANNEL_INTERNAL_MASK                  (ADC_CHANNEL_INTERNAL_TEMPSENSOR | ADC_CHANNEL_INTERNAL_VREFINT)
#define ADC_FLAG_MASK                              ((uint32_t)0xC000001FU)
#define ADC_INT_MASK                               ((uint32_t)0xC00000E0U)
#define ADC_INT_FLAG_MASK                          ((uint32_t)0xC0000007U)

/*!
    \brief      reset ADC (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_deinit(void)
{
    rcu_periph_reset_enable(RCU_ADCRST);
    rcu_periph_reset_disable(RCU_ADCRST);
}

/*!
    \brief      enable ADC interface (API_ID(0x0002U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_enable(void)
{
    if(0U == (ADC_CTL1 & ADC_CTL1_ADCON)) {
        /* enable ADC */
        ADC_CTL1 |= (uint32_t)ADC_CTL1_ADCON;
    }
}

/*!
    \brief      disable ADC interface (API_ID(0x0003U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_disable(void)
{
    /* disable ADC */
    ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ADCON);
}

/*!
    \brief      enable DMA request (API_ID(0x0004U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_dma_mode_enable(void)
{
    /* enable DMA request */
    ADC_CTL1 |= (uint32_t)(ADC_CTL1_DMA);
}

/*!
    \brief      disable DMA request (API_ID(0x0005U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_dma_mode_disable(void)
{
    /* disable DMA request */
    ADC_CTL1 &= ~((uint32_t)ADC_CTL1_DMA);
}

/*!
    \brief      configure ADC discontinuous mode (API_ID(0x0006U))
    \param[in]  adc_sequence: select the sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine sequence
      \arg        ADC_INSERTED_CHANNEL: inserted sequence
      \arg        ADC_CHANNEL_DISCON_DISABLE: disable discontinuous mode of routine and inserted sequence
    \param[in]  length: number of conversions in discontinuous mode,the number can be 1..8
                        for routine sequence, the number has no effect for inserted sequence
    \param[out] none
    \retval     none
*/
void adc_discontinuous_mode_config(uint8_t adc_sequence, uint8_t length)
{
    /* disable discontinuous mode of routine & inserted sequence */
    ADC_CTL0 &= ~((uint32_t)(ADC_CTL0_DISRC | ADC_CTL0_DISIC));
    switch(adc_sequence) {
    case ADC_ROUTINE_CHANNEL:
        /* configure the number of conversions in discontinuous mode */
        ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DISNUM);
        if((length <= 8U) && (length >= 1U)) {
            ADC_CTL0 |= CTL0_DISNUM(((uint32_t)length - ADC_CHANNEL_LENGTH_SUBTRACT_ONE));
        }
        /* enable routine sequence discontinuous mode */
        ADC_CTL0 |= (uint32_t)ADC_CTL0_DISRC;
        break;
    case ADC_INSERTED_CHANNEL:
        /* enable inserted sequence discontinuous mode */
        ADC_CTL0 |= (uint32_t)ADC_CTL0_DISIC;
        break;
    case ADC_CHANNEL_DISCON_DISABLE:
        /* disable discontinuous mode of routine & inserted sequence */
    default:
        break;
    }
}

/*!
    \brief      configure ADC special function (API_ID(0x0007U))
    \param[in]  function: the function to configure
                one or more parameters can be selected which is shown as below:
      \arg        ADC_SCAN_MODE: scan mode select
      \arg        ADC_INSERTED_CHANNEL_AUTO: inserted sequence convert automatically
      \arg        ADC_CONTINUOUS_MODE: continuous mode select
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void adc_special_function_config(uint32_t function, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        /* enable ADC scan mode */
        if(0U != (function & ADC_SCAN_MODE)) {
            ADC_CTL0 |= (uint32_t)ADC_SCAN_MODE;
        }
        /* enable ADC inserted sequence convert automatically */
        if(0U != (function & ADC_INSERTED_CHANNEL_AUTO)) {
            ADC_CTL0 |= (uint32_t)ADC_INSERTED_CHANNEL_AUTO;
        }
        /* enable ADC continuous mode */
        if(0U != (function & ADC_CONTINUOUS_MODE)) {
            ADC_CTL1 |= (uint32_t)ADC_CONTINUOUS_MODE;
        }
    } else {
        /* disable ADC scan mode */
        if(0U != (function & ADC_SCAN_MODE)) {
            ADC_CTL0 &= ~((uint32_t)ADC_SCAN_MODE);
        }
        /* disable ADC inserted sequence convert automatically */
        if(0U != (function & ADC_INSERTED_CHANNEL_AUTO)) {
            ADC_CTL0 &= ~((uint32_t)ADC_INSERTED_CHANNEL_AUTO);
        }
        /* disable ADC continuous mode */
        if(0U != (function & ADC_CONTINUOUS_MODE)) {
            ADC_CTL1 &= ~((uint32_t)ADC_CONTINUOUS_MODE);
        }
    }
}

/*!
    \brief      enable or disable ADC internal channels (API_ID(0x0008U))
    \param[in]  internal_channel: the internal channels
                only one parameter can be selected which is shown as below:
      \arg        ADC_CHANNEL_INTERNAL_TEMPSENSOR: temperature sensor channel
      \arg        ADC_CHANNEL_INTERNAL_VREFINT: vrefint channel
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void adc_internal_channel_config(uint32_t internal_channel, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        ADC_CTL1 |= (uint32_t)(internal_channel & ADC_CHANNEL_INTERNAL_MASK);
    } else {
        ADC_CTL1 &= ~((uint32_t)internal_channel & ADC_CHANNEL_INTERNAL_MASK);
    }
}

/*!
    \brief      configure ADC data alignment (API_ID(0x0009U))
    \param[in]  data_alignment: data alignment select
                only one parameter can be selected which is shown as below:
      \arg        ADC_DATAALIGN_RIGHT: right alignment
      \arg        ADC_DATAALIGN_LEFT: left alignment
    \param[out] none
    \retval     none
*/
void adc_data_alignment_config(uint32_t data_alignment)
{
    if(ADC_DATAALIGN_RIGHT != data_alignment) {
        /* left alignment */
        ADC_CTL1 |= ADC_CTL1_DAL;
    } else {
        /* right alignment */
        ADC_CTL1 &= ~((uint32_t)ADC_CTL1_DAL);
    }
}

/*!
    \brief      configure the channel length of routine sequence or inserted sequence (API_ID(0x000AU))
    \param[in]  adc_sequence: select the sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine sequence
      \arg        ADC_INSERTED_CHANNEL: inserted sequence
    \param[in]  length: the channel length of the sequence
                        routine sequence: 1-16
                        inserted sequence 1-4
    \param[out] none
    \retval     none
*/
void adc_channel_length_config(uint8_t adc_sequence, uint32_t length)
{
    switch(adc_sequence) {
    case ADC_ROUTINE_CHANNEL:
        /* configure the length of routine sequence */
        if((length >= 1U) && (length <= ROUTINE_LENGTH_HIGH_VALUE)) {
            ADC_RSQ0 &= ~((uint32_t)ADC_RSQ0_RL);
            ADC_RSQ0 |= RSQ0_RL((uint32_t)(length - ADC_CHANNEL_LENGTH_SUBTRACT_ONE));
        }
        break;
    case ADC_INSERTED_CHANNEL:
        /* configure the length of inserted sequence */
        if((length >= 1U) && (length <= 4U)) {
            ADC_ISQ &= ~((uint32_t)ADC_ISQ_IL);
            ADC_ISQ |= ISQ_IL((uint32_t)(length - ADC_CHANNEL_LENGTH_SUBTRACT_ONE));
        }
        break;
    default:
        break;
    }
}

/*!
    \brief      configure ADC routine channel (API_ID(0x000BU))
    \param[in]  rank: the routine sequence rank, this parameter must be between 0 to 15
    \param[in]  adc_channel: the selected ADC channel
                only one parameter can be selected which is shown as below:
      \arg        ADC_CHANNEL_x: x=0..15
    \param[in]  sample_time: the sample time value
                only one parameter can be selected which is shown as below:
      \arg        ADC_SAMPLETIME_2POINT5: 2.5 cycles
      \arg        ADC_SAMPLETIME_3POINT5: 3.5 cycles
      \arg        ADC_SAMPLETIME_7POINT5: 7.5 cycles
      \arg        ADC_SAMPLETIME_12POINT5: 12.5 cycles
      \arg        ADC_SAMPLETIME_19POINT5: 19.5 cycles
      \arg        ADC_SAMPLETIME_39POINT5: 39.5 cycles
      \arg        ADC_SAMPLETIME_79POINT5: 79.5 cycles
      \arg        ADC_SAMPLETIME_160POINT5: 160.5 cycles
    \param[out] none
    \retval     none
*/
void adc_routine_channel_config(uint8_t rank, uint8_t adc_channel, uint32_t sample_time)
{
    uint32_t rsq, sampt;

    /* configure ADC routine sequence */
    if(rank < ADC_ROUTINE_CHANNEL_RANK_SIX) {
        /* the routine sequence rank is smaller than six */
        rsq = ADC_RSQ2;
        rsq &=  ~((uint32_t)(ADC_RSQX_RSQN << (ADC_ROUTINE_CHANNEL_RANK_LENGTH * rank)));
        /* the channel number is written to these bits to select a channel as the nth conversion in the routine sequence */
        rsq |= ((uint32_t)(adc_channel & ADC_RSQX_RSQN) << (ADC_ROUTINE_CHANNEL_RANK_LENGTH * rank));
        ADC_RSQ2 = rsq;
    } else if(rank < ADC_ROUTINE_CHANNEL_RANK_TWELVE) {
        /* the routine sequence rank is smaller than twelve */
        rsq = ADC_RSQ1;
        rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (ADC_ROUTINE_CHANNEL_RANK_LENGTH * (rank - ADC_ROUTINE_CHANNEL_RANK_SIX))));
        /* the channel number is written to these bits to select a channel as the nth conversion in the routine sequence */
        rsq |= ((uint32_t)(adc_channel & ADC_RSQX_RSQN) << (ADC_ROUTINE_CHANNEL_RANK_LENGTH * (rank - ADC_ROUTINE_CHANNEL_RANK_SIX)));
        ADC_RSQ1 = rsq;
    } else {
        /* the routine sequence rank is larger than twelve */
        rsq = ADC_RSQ0;
        rsq &= ~((uint32_t)(ADC_RSQX_RSQN << (ADC_ROUTINE_CHANNEL_RANK_LENGTH * (rank - ADC_ROUTINE_CHANNEL_RANK_TWELVE))));
        /* the channel number is written to these bits to select a channel as the nth conversion in the routine sequence */
        rsq |= ((uint32_t)(adc_channel & ADC_RSQX_RSQN) << (ADC_ROUTINE_CHANNEL_RANK_LENGTH * (rank - ADC_ROUTINE_CHANNEL_RANK_TWELVE)));
        ADC_RSQ0 = rsq;
    }

    /* configure ADC sampling time */
    if(adc_channel < ADC_CHANNEL_SAMPLE_TEN) {
        /* the routine sequence rank is smaller than ten */
        sampt = ADC_SAMPT1;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (ADC_CHANNEL_SAMPLE_LENGTH * adc_channel)));
        /* channel sample time set*/
        sampt |= (uint32_t)((sample_time & ADC_SAMPTX_SPTN) << (ADC_CHANNEL_SAMPLE_LENGTH * adc_channel));
        ADC_SAMPT1 = sampt;
    } else {
        /* the routine sequence rank is smaller than fiften */
        sampt = ADC_SAMPT0;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (ADC_CHANNEL_SAMPLE_LENGTH * (adc_channel - ADC_CHANNEL_SAMPLE_TEN))));
        /* channel sample time set*/
        sampt |= (uint32_t)((sample_time & ADC_SAMPTX_SPTN) << (ADC_CHANNEL_SAMPLE_LENGTH * (adc_channel - ADC_CHANNEL_SAMPLE_TEN)));
        ADC_SAMPT0 = sampt;
    }
}

/*!
    \brief      configure ADC inserted channel (API_ID(0x000CU))
    \param[in]  rank: the inserted sequencer rank,this parameter must be between 0 to 3
    \param[in]  adc_channel: the selected ADC channel
                only one parameter can be selected which is shown as below:
      \arg        ADC_CHANNEL_x: x=0..15
    \param[in]  sample_time: The sample time value
                only one parameter can be selected which is shown as below:
      \arg        ADC_SAMPLETIME_2POINT5: 2.5 cycles
      \arg        ADC_SAMPLETIME_3POINT5: 3.5 cycles
      \arg        ADC_SAMPLETIME_7POINT5: 7.5 cycles
      \arg        ADC_SAMPLETIME_12POINT5: 12.5 cycles
      \arg        ADC_SAMPLETIME_19POINT5: 19.5 cycles
      \arg        ADC_SAMPLETIME_39POINT5: 39.5 cycles
      \arg        ADC_SAMPLETIME_79POINT5: 79.5 cycles
      \arg        ADC_SAMPLETIME_160POINT5: 160.5 cycles
    \param[out] none
    \retval     none
*/
void adc_inserted_channel_config(uint8_t rank, uint8_t adc_channel, uint32_t sample_time)
{
    uint8_t inserted_length;
    uint32_t isq, sampt;

    /* get inserted sequence length */
    inserted_length = (uint8_t)GET_BITS(ADC_ISQ, 20U, 21U);
    /* the channel number is written to these bits to select a channel as the nth conversion in the inserted sequence */
    if(rank < 4U) {
        isq = ADC_ISQ;
        isq &= ~((uint32_t)(ADC_ISQ_ISQN << (ADC_INSERTED_CHANNEL_SHIFT_OFFSET - (inserted_length - rank) * ADC_INSERTED_CHANNEL_RANK_LENGTH)));
        isq |= ((uint32_t)(adc_channel & ADC_ISQ_ISQN) << (ADC_INSERTED_CHANNEL_SHIFT_OFFSET - (inserted_length - rank) * ADC_INSERTED_CHANNEL_RANK_LENGTH));
        ADC_ISQ = isq;
    }

    /* configure ADC sampling time */
    if(adc_channel < ADC_CHANNEL_SAMPLE_TEN) {
        /* the inserted sequence rank is smaller than ten */
        sampt = ADC_SAMPT1;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (ADC_CHANNEL_SAMPLE_LENGTH * adc_channel)));
        /* channel sample time set*/
        sampt |= (uint32_t)(sample_time & ADC_SAMPTX_SPTN) << (ADC_CHANNEL_SAMPLE_LENGTH * adc_channel);
        ADC_SAMPT1 = sampt;
    } else {
        /* the inserted sequence rank is smaller than fiften */
        sampt = ADC_SAMPT0;
        sampt &= ~((uint32_t)(ADC_SAMPTX_SPTN << (ADC_CHANNEL_SAMPLE_LENGTH * (adc_channel - ADC_CHANNEL_SAMPLE_TEN))));
        /* channel sample time set*/
        sampt |= ((uint32_t)(sample_time & ADC_SAMPTX_SPTN) << (ADC_CHANNEL_SAMPLE_LENGTH * (adc_channel - ADC_CHANNEL_SAMPLE_TEN)));
        ADC_SAMPT0 = sampt;
    }
}

/*!
    \brief      configure ADC inserted channel offset (API_ID(0x000DU))
    \param[in]  inserted_channel: inserted channel select
                only one parameter can be selected which is shown as below:
      \arg        ADC_INSERTED_CHANNEL_0: ADC inserted channel 0
      \arg        ADC_INSERTED_CHANNEL_1: ADC inserted channel 1
      \arg        ADC_INSERTED_CHANNEL_2: ADC inserted channel 2
      \arg        ADC_INSERTED_CHANNEL_3: ADC inserted channel 3
    \param[in]  offset: the offset data
    \param[out] none
    \retval     none
*/
void adc_inserted_channel_offset_config(uint8_t inserted_channel, uint16_t offset)
{
    uint8_t inserted_length;
    uint32_t num;

    inserted_length = (uint8_t)GET_BITS(ADC_ISQ, 20U, 21U);
    num = ((uint32_t)ADC_OFFSET_LENGTH - ((uint32_t)inserted_length - (uint32_t)inserted_channel));

    if(num <= ADC_OFFSET_LENGTH) {
        /* calculate the offset of the register */
        num = num * ADC_OFFSET_SHIFT_LENGTH;
        /* configure the offset of the selected channels */
        REG32(ADC + 0x14U + num) = (uint32_t)(offset & ADC_IOFFX_IOFF);
    }
}

/*!
    \brief      configure ADC external trigger (API_ID(0x000EU))
    \param[in]  adc_sequence: select the sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine sequence
      \arg        ADC_INSERTED_CHANNEL: inserted sequence
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void adc_external_trigger_config(uint8_t adc_sequence, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        /* external trigger enable for routine sequence */
        if(0U != (adc_sequence & ADC_ROUTINE_CHANNEL)) {
            ADC_CTL1 |= (uint32_t)ADC_CTL1_ETERC;
        }
        /* external trigger enable for inserted sequence */
        if(0U != (adc_sequence & ADC_INSERTED_CHANNEL)) {
            ADC_CTL1 |= (uint32_t)ADC_CTL1_ETEIC;
        }
    } else {
        /* external trigger disable for routine sequence */
        if(0U != (adc_sequence & ADC_ROUTINE_CHANNEL)) {
            ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ETERC);
        }
        /* external trigger disable for inserted sequence */
        if(0U != (adc_sequence & ADC_INSERTED_CHANNEL)) {
            ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ETEIC);
        }
    }
}

/*!
    \brief      configure ADC external trigger source (API_ID(0x000FU))
    \param[in]  adc_sequence: select the sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine sequence
      \arg        ADC_INSERTED_CHANNEL: inserted sequence
    \param[in]  external_trigger_source: routine or inserted sequence trigger source
                only one parameter can be selected which is shown as below:
                for routine sequence:
      \arg        ADC_EXTTRIG_ROUTINE_T2_CH1: external trigger TIMER2 CH1 event select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_T0_CH2: external trigger TIMER0 CH2 event select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_T0_CH1: external trigger TIMER0 CH1 event select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_T2_TRGO: external trigger TIMER2 TRGO event select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_T0_CH0: external trigger TIMER0 CH0 event select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_T2_CH0: external trigger TIMER2 CH0 event select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_EXTI_11: external trigger interrupt line 11 select for routine channel
      \arg        ADC_EXTTRIG_ROUTINE_NONE: external trigger software event select for routine channel
                for inserted sequence:
      \arg        ADC_EXTTRIG_INSERTED_T0_TRGO: external trigger TIMER0 TRGO event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_T0_CH3: external trigger TIMER0 CH3 event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_T13_CH0: external trigger TIMER13 CH0 event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_T2_CH3: external trigger TIMER2 CH3 event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_T2_CH2: external trigger TIMER2 CH2 event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_T15_CH0: external trigger TIMER15 CH0 event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_EXTI_15: external trigger interrupt line 15 event select for inserted channel
      \arg        ADC_EXTTRIG_INSERTED_NONE: external trigger software event select for inserted channel
    \param[out] none
    \retval     none
*/
void adc_external_trigger_source_config(uint8_t adc_sequence, uint32_t external_trigger_source)
{
    switch(adc_sequence) {
    case ADC_ROUTINE_CHANNEL:
        /* configure ADC routine sequence external trigger source */
        ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ETSRC);
        ADC_CTL1 |= (uint32_t)(external_trigger_source & ADC_CTL1_ETSRC);
        break;
    case ADC_INSERTED_CHANNEL:
        /* configure ADC inserted sequence external trigger source */
        ADC_CTL1 &= ~((uint32_t)ADC_CTL1_ETSIC);
        ADC_CTL1 |= (uint32_t)(external_trigger_source & ADC_CTL1_ETSIC);
        break;
    default:
        break;
    }
}

/*!
    \brief      enable ADC software trigger (API_ID(0x0010U))
    \param[in]  adc_sequence: select the sequence
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine sequence
      \arg        ADC_INSERTED_CHANNEL: inserted sequence
    \param[out] none
    \retval     none
*/
void adc_software_trigger_enable(uint8_t adc_sequence)
{
    /* enable routine sequence software trigger */
    if(0U != (adc_sequence & ADC_ROUTINE_CHANNEL)) {
        ADC_CTL1 |= (uint32_t)ADC_CTL1_SWRCST;
    }
    /* enable inserted sequence software trigger */
    if(0U != (adc_sequence & ADC_INSERTED_CHANNEL)) {
        ADC_CTL1 |= (uint32_t)ADC_CTL1_SWICST;
    }
}

/*!
    \brief      read ADC routine sequence data register (API_ID(0x0011U))
    \param[in]  none
    \param[out] none
    \retval     the conversion value: 0~0x0FFF
*/
uint16_t adc_routine_data_read(void)
{
    return ((uint16_t)(ADC_RDATA));
}

/*!
    \brief      read ADC inserted sequence data register (API_ID(0x0012U))
    \param[in]  inserted_channel: inserted channel select
                only one parameter can be selected which is shown as below:
      \arg        ADC_INSERTED_CHANNEL_0: ADC inserted channel 0
      \arg        ADC_INSERTED_CHANNEL_1: ADC inserted channel 1
      \arg        ADC_INSERTED_CHANNEL_2: ADC inserted channel 2
      \arg        ADC_INSERTED_CHANNEL_3: ADC inserted channel 3
    \param[out] none
    \retval     the conversion value: 0~0xFFFF
*/
uint16_t adc_inserted_data_read(uint8_t inserted_channel)
{
    uint32_t idata = 0U;

    /* read the data of the selected channel */
    switch(inserted_channel) {
    case ADC_INSERTED_CHANNEL_0:
        /* read the data of channel 0 */
        idata = ADC_IDATA0;
        break;
    case ADC_INSERTED_CHANNEL_1:
        /* read the data of channel 1 */
        idata = ADC_IDATA1;
        break;
    case ADC_INSERTED_CHANNEL_2:
        /* read the data of channel 2 */
        idata = ADC_IDATA2;
        break;
    case ADC_INSERTED_CHANNEL_3:
        /* read the data of channel 3 */
        idata = ADC_IDATA3;
        break;
    default:
        break;
    }
    return ((uint16_t)idata);
}

/*!
    \brief      enable ADC analog watchdog 0 single channel (API_ID(0x0013U))
    \param[in]  adc_channel: the selected ADC channel
                only one parameter can be selected which is shown as below:
      \arg        ADC_CHANNEL_x: x=0..15
    \param[out] none
    \retval     none
*/
void adc_watchdog0_single_channel_enable(uint8_t adc_channel)
{
    ADC_CTL0 &= (uint32_t)~(ADC_CTL0_RWD0EN | ADC_CTL0_IWD0EN | ADC_CTL0_WD0SC | ADC_CTL0_WD0CHSEL);
    ADC_CTL0 |= (uint32_t)(adc_channel & ADC_CTL0_WD0CHSEL);
    ADC_CTL0 |= (uint32_t)(ADC_CTL0_RWD0EN | ADC_CTL0_IWD0EN | ADC_CTL0_WD0SC);
}

/*!
    \brief      enable ADC analog watchdog 0 sequence channel (API_ID(0x0014U))
    \param[in]  adc_sequence: the sequence use analog watchdog
                only one parameter can be selected which is shown as below:
      \arg        ADC_ROUTINE_CHANNEL: routine sequence
      \arg        ADC_INSERTED_CHANNEL: inserted sequence
      \arg        ADC_ROUTINE_INSERTED_CHANNEL: both routine and inserted sequence
    \param[out] none
    \retval     none
*/
void adc_watchdog0_sequence_channel_enable(uint8_t adc_sequence)
{
    ADC_CTL0 &= ~((uint32_t)(ADC_CTL0_RWD0EN | ADC_CTL0_IWD0EN | ADC_CTL0_WD0SC));
    /* select the sequence */
    switch(adc_sequence) {
    case ADC_ROUTINE_CHANNEL:
        /* routine sequence analog watchdog 0 enable */
        ADC_CTL0 |= (uint32_t) ADC_CTL0_RWD0EN;
        break;
    case ADC_INSERTED_CHANNEL:
        /* inserted sequence analog watchdog 0 enable */
        ADC_CTL0 |= (uint32_t) ADC_CTL0_IWD0EN;
        break;
    case ADC_ROUTINE_INSERTED_CHANNEL:
        /* routine and inserted sequence analog watchdog 0 enable */
        ADC_CTL0 |= (uint32_t)(ADC_CTL0_RWD0EN | ADC_CTL0_IWD0EN);
        break;
    default:
        break;
    }
}

/*!
    \brief      disable ADC analog watchdog 0 (API_ID(0x0015U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_watchdog0_disable(void)
{
    ADC_CTL0 &= (uint32_t)~(ADC_CTL0_RWD0EN | ADC_CTL0_IWD0EN | ADC_CTL0_WD0SC | ADC_CTL0_WD0CHSEL);
}

/*!
    \brief      configure ADC analog watchdog 1 channel (API_ID(0x0016U))
    \param[in]  selection_channel: the channel use analog watchdog 1
                one or more parameters can be selected which is shown as below:
      \arg        ADC_AWD1_2_SELECTION_CHANNEL_x, ADC_AWD1_2_SELECTION_CHANNEL_ALL: ADC channel analog watchdog 1/2 selection
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void adc_watchdog1_channel_config(uint32_t selection_channel, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        ADC_WD1SR |= (uint32_t)selection_channel;
    } else {
        ADC_WD1SR &= ~((uint32_t)selection_channel);
    }
}

/*!
    \brief      configure ADC analog watchdog 2 channel (API_ID(0x0017U))
    \param[in]  selection_channel: the channel use analog watchdog 2
                one or more parameters can be selected which is shown as below:
      \arg        ADC_AWD1_2_SELECTION_CHANNEL_x, ADC_AWD1_2_SELECTION_CHANNEL_ALL: ADC channel analog watchdog 1/2 selection
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void adc_watchdog2_channel_config(uint32_t selection_channel, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        ADC_WD2SR |= (uint32_t)selection_channel;
    } else {
        ADC_WD2SR &= ~((uint32_t)selection_channel);
    }
}

/*!
    \brief      disable ADC analog watchdog 1 (API_ID(0x0018U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_watchdog1_disable(void)
{
    ADC_WD1SR &= ~((uint32_t)ADC_WD1SR_AWD1CS);
}

/*!
    \brief      disable ADC analog watchdog 2 (API_ID(0x0019U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_watchdog2_disable(void)
{
    ADC_WD2SR &= ~((uint32_t)ADC_WD2SR_AWD2CS);
}

/*!
    \brief      configure ADC analog watchdog 0 threshold (API_ID(0x001AU))
    \param[in]  low_threshold: analog watchdog 0 low threshold, 0..4095
    \param[in]  high_threshold: analog watchdog 0 high threshold, 0..4095
    \param[out] none
    \retval     none
*/
void adc_watchdog0_threshold_config(uint32_t low_threshold, uint32_t high_threshold)
{
    ADC_WD0LT = (uint32_t)WD0LT_WD0LT(low_threshold);
    ADC_WD0HT = (uint32_t)WD0HT_WD0HT(high_threshold);
}

/*!
    \brief      configure ADC analog watchdog 1 threshold (API_ID(0x001BU))
    \param[in]  low_threshold: analog watchdog 1 low threshold, 0..4095
    \param[in]  high_threshold: analog watchdog 1 high threshold, 0..4095
    \param[out] none
    \retval     none
*/
void adc_watchdog1_threshold_config(uint32_t low_threshold, uint32_t high_threshold)
{
    ADC_WD1LT = (uint32_t)WD1LT_WD1LT(low_threshold);
    ADC_WD1HT = (uint32_t)WD1HT_WD1HT(high_threshold);
}

/*!
    \brief      configure ADC analog watchdog 2 threshold (API_ID(0x001CU))
    \param[in]  low_threshold: analog watchdog 2 low threshold, 0..4095
    \param[in]  high_threshold: analog watchdog 2 high threshold, 0..4095
    \param[out] none
    \retval     none
*/
void adc_watchdog2_threshold_config(uint32_t low_threshold, uint32_t high_threshold)
{
    ADC_WD2LT = (uint32_t)WD2LT_WD2LT(low_threshold);
    ADC_WD2HT = (uint32_t)WD2HT_WD2HT(high_threshold);
}

/*!
    \brief      configure ADC resolution (API_ID(0x001DU))
    \param[in]  resolution: ADC resolution
                only one parameter can be selected which is shown as below:
      \arg        ADC_RESOLUTION_12B: 12-bit ADC resolution
      \arg        ADC_RESOLUTION_10B: 10-bit ADC resolution
      \arg        ADC_RESOLUTION_8B: 8-bit ADC resolution
      \arg        ADC_RESOLUTION_6B: 6-bit ADC resolution
    \param[out] none
    \retval     none
*/
void adc_resolution_config(uint32_t resolution)
{
    ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DRES);
    ADC_CTL0 |= (uint32_t)(resolution & ADC_CTL0_DRES);
}

/*!
    \brief      configure ADC oversample mode (API_ID(0x001EU))
    \param[in]  mode: ADC oversampling mode
                only one parameter can be selected which is shown as below:
      \arg        ADC_OVERSAMPLING_ALL_CONVERT: all oversampled conversions for a channel are done consecutively after a trigger
      \arg        ADC_OVERSAMPLING_ONE_CONVERT: each oversampled conversion for a channel needs a trigger
    \param[in]  shift: ADC oversampling shift
                only one parameter can be selected which is shown as below:
      \arg        ADC_OVERSAMPLING_SHIFT_NONE: no oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_1B: 1-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_2B: 2-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_3B: 3-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_4B: 3-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_5B: 5-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_6B: 6-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_7B: 7-bit oversampling shift
      \arg        ADC_OVERSAMPLING_SHIFT_8B: 8-bit oversampling shift
    \param[in]  ratio: ADC oversampling ratio
                only one parameter can be selected which is shown as below:
      \arg        ADC_OVERSAMPLING_RATIO_MUL2: oversampling ratio multiple 2
      \arg        ADC_OVERSAMPLING_RATIO_MUL4: oversampling ratio multiple 4
      \arg        ADC_OVERSAMPLING_RATIO_MUL8: oversampling ratio multiple 8
      \arg        ADC_OVERSAMPLING_RATIO_MUL16: oversampling ratio multiple 16
      \arg        ADC_OVERSAMPLING_RATIO_MUL32: oversampling ratio multiple 32
      \arg        ADC_OVERSAMPLING_RATIO_MUL64: oversampling ratio multiple 64
      \arg        ADC_OVERSAMPLING_RATIO_MUL128: oversampling ratio multiple 128
      \arg        ADC_OVERSAMPLING_RATIO_MUL256: oversampling ratio multiple 256
    \param[out] none
    \retval     none
*/
void adc_oversample_mode_config(uint32_t mode, uint16_t shift, uint8_t ratio)
{
    /* configure ADC oversampling mode */
    if(ADC_OVERSAMPLING_ONE_CONVERT == mode) {
        ADC_OVSAMPCTL |= (uint32_t)ADC_OVSAMPCTL_TOVS;
    } else {
        ADC_OVSAMPCTL &= ~((uint32_t)ADC_OVSAMPCTL_TOVS);
    }
    /* configure the shift and ratio */
    ADC_OVSAMPCTL &= ~((uint32_t)(ADC_OVSAMPCTL_OVSR | ADC_OVSAMPCTL_OVSS));
    ADC_OVSAMPCTL |= ((uint32_t)(shift & ADC_OVSAMPCTL_OVSS) | (uint32_t)(ratio & ADC_OVSAMPCTL_OVSR));
}

/*!
    \brief      enable ADC oversample mode (API_ID(0x001FU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_oversample_mode_enable(void)
{
    ADC_OVSAMPCTL |= ADC_OVSAMPCTL_OVSEN;
}

/*!
    \brief      disable ADC oversample mode (API_ID(0x0020U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void adc_oversample_mode_disable(void)
{
    ADC_OVSAMPCTL &= ~((uint32_t)ADC_OVSAMPCTL_OVSEN);
}

/*!
    \brief      get ADC flag (API_ID(0x0021U))
    \param[in]  flag: ADC flag
                only one parameter can be selected which is shown as below:
      \arg        ADC_FLAG_WD0E: analog watchdog 0 event flag
      \arg        ADC_FLAG_WD1E: analog watchdog 1 event flag
      \arg        ADC_FLAG_WD2E: analog watchdog 2 event flag
      \arg        ADC_FLAG_EOC: end of sequence conversion flag
      \arg        ADC_FLAG_EOIC: end of inserted sequence conversion flag
      \arg        ADC_FLAG_STIC: start flag of inserted sequence
      \arg        ADC_FLAG_STRC: start flag of routine sequence
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus adc_flag_get(uint32_t flag)
{
    FlagStatus reval = RESET;

    if(0U != (ADC_STAT & flag)) {
        reval = SET;
    }
    return reval;
}

/*!
    \brief      clear ADC flag (API_ID(0x0022U))
    \param[in]  flag: ADC flag
                one or more parameter can be selected which is shown as below:
      \arg        ADC_FLAG_WD0E: analog watchdog 0 event flag
      \arg        ADC_FLAG_WD1E: analog watchdog 1 event flag
      \arg        ADC_FLAG_WD2E: analog watchdog 2 event flag
      \arg        ADC_FLAG_EOC: end of sequence conversion flag
      \arg        ADC_FLAG_EOIC: end of inserted sequence conversion flag
      \arg        ADC_FLAG_STIC: start flag of inserted sequence
      \arg        ADC_FLAG_STRC: start flag of routine sequence
    \param[out] none
    \retval     none
*/
void adc_flag_clear(uint32_t flag)
{
    ADC_STAT &= ~((uint32_t)flag & ADC_FLAG_MASK);
}

/*!
    \brief      enable ADC interrupt (API_ID(0x0023U))
    \param[in]  interrupt: ADC interrupt
                one or more parameter can be selected which is shown as below:
      \arg        ADC_INT_WD0E: analog watchdog 0 interrupt
      \arg        ADC_INT_WD1E: analog watchdog 1 interrupt
      \arg        ADC_INT_WD2E: analog watchdog 2 interrupt
      \arg        ADC_INT_EOC: end of sequence conversion interrupt
      \arg        ADC_INT_EOIC: end of inserted sequence conversion interrupt
    \param[out] none
    \retval     none
*/
void adc_interrupt_enable(uint32_t interrupt)
{
    ADC_CTL0 |= (uint32_t) interrupt & ADC_INT_MASK;
}

/*!
    \brief      disable ADC interrupt (API_ID(0x0024U))
    \param[in]  interrupt: ADC interrupt
                one or more parameter can be selected which is shown as below:
      \arg        ADC_INT_WD0E: analog watchdog 0 interrupt
      \arg        ADC_INT_WD1E: analog watchdog 1 interrupt
      \arg        ADC_INT_WD2E: analog watchdog 2 interrupt
      \arg        ADC_INT_EOC: end of sequence conversion interrupt
      \arg        ADC_INT_EOIC: end of inserted sequence conversion interrupt
    \param[out] none
    \retval     none
*/
void adc_interrupt_disable(uint32_t interrupt)
{
    ADC_CTL0 &= ~((uint32_t)interrupt & ADC_INT_MASK);
}

/*!
    \brief      get ADC interrupt flag (API_ID(0x0025U))
    \param[in]  int_flag: ADC interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        ADC_INT_FLAG_WD0E: analog watchdog 0 interrupt flag
      \arg        ADC_INT_FLAG_WD1E: analog watchdog 1 interrupt flag
      \arg        ADC_INT_FLAG_WD2E: analog watchdog 2 interrupt flag
      \arg        ADC_INT_FLAG_EOC: end of sequence conversion interrupt flag
      \arg        ADC_INT_FLAG_EOIC: end of inserted sequence conversion interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus adc_interrupt_flag_get(uint32_t int_flag)
{
    FlagStatus interrupt_flag = RESET;
    uint32_t state;

    /* check the interrupt flags */
    switch(int_flag) {
    case ADC_INT_FLAG_WD0E:
        /* get the ADC analog watchdog 0 interrupt flags */
        state = ADC_STAT & ADC_STAT_WD0E;
        if((0U != (ADC_CTL0 & ADC_CTL0_WD0EIE)) && (0U != state)) {
            interrupt_flag = SET;
        }
        break;
    case ADC_INT_FLAG_WD1E:
        /* get the ADC analog watchdog 1 interrupt flags */
        state = ADC_STAT & ADC_STAT_WD1E;
        if((0U != (ADC_CTL0 & ADC_CTL0_WD1EIE)) && (0U != state)) {
            interrupt_flag = SET;
        }
        break;
    case ADC_INT_FLAG_WD2E:
        /* get the ADC analog watchdog 2 interrupt flags */
        state = ADC_STAT & ADC_STAT_WD2E;
        if((0U != (ADC_CTL0 & ADC_CTL0_WD2EIE)) && (0U != state)) {
            interrupt_flag = SET;
        }
        break;
    case ADC_INT_FLAG_EOC:
        /* get the ADC end of sequence conversion interrupt flags */
        state = ADC_STAT & ADC_STAT_EOC;
        if((0U != (ADC_CTL0 & ADC_CTL0_EOCIE)) && (0U != state)) {
            interrupt_flag = SET;
        }
        break;
    case ADC_INT_FLAG_EOIC:
        /* get the ADC end of inserted sequence conversion interrupt flags */
        state = ADC_STAT & ADC_STAT_EOIC;
        if((0U != (ADC_CTL0 & ADC_CTL0_EOICIE)) && (0U != state)) {
            interrupt_flag = SET;
        }
        break;
    default:
        break;
    }
    return interrupt_flag;
}

/*!
    \brief      clear ADC interrupt flag (API_ID(0x0026U))
    \param[in]  int_flag: ADC interrupt flag
                one or more parameter can be selected which is shown as below:
      \arg        ADC_INT_FLAG_WD0E: analog watchdog 0 interrupt flag
      \arg        ADC_INT_FLAG_WD1E: analog watchdog 1 interrupt flag
      \arg        ADC_INT_FLAG_WD2E: analog watchdog 2 interrupt flag
      \arg        ADC_INT_FLAG_EOC: end of sequence conversion interrupt flag
      \arg        ADC_INT_FLAG_EOIC: end of inserted sequence conversion interrupt flag
    \param[out] none
    \retval     none
*/
void adc_interrupt_flag_clear(uint32_t int_flag)
{
    ADC_STAT &= ~((uint32_t)int_flag & ADC_INT_FLAG_MASK);
}
