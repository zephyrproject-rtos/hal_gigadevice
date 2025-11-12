/*!
    \file    gd32c2x1_timer.c
    \brief   TIMER driver

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

#include "gd32c2x1_timer.h"

/* TIMER init parameter mask */
#define ALIGNEDMODE_MASK            ((uint32_t)0x00000060U)   /*!< TIMER init parameter aligne dmode mask */
#define COUNTERDIRECTION_MASK       ((uint32_t)0x00000010U)   /*!< TIMER init parameter counter direction mask */
#define CLOCKDIVISION_MASK          ((uint32_t)0x00000300U)   /*!< TIMER init parameter clock division value mask */
#define DEADTIME_MASK               ((uint32_t)0x000000FFU)   /*!< TIMER init parameter deadtime value mask */
#define BREAK0FILTER_MASK           ((uint32_t)0x000F0000U)   /*!< TIMER berak0 filter mask */ 
#define BREAK1FILTER_MASK           ((uint32_t)0x00F00000U)   /*!< TIMER berak1 filter mask */
/*!
    \brief      deinit a TIMER (API_ID(0x0001U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_deinit(uint32_t timer_periph)
{
    switch(timer_periph) {
    case TIMER0:
        /* reset TIMER0 */
        rcu_periph_reset_enable(RCU_TIMER0RST);
        rcu_periph_reset_disable(RCU_TIMER0RST);
        break;
    case TIMER2:
        /* reset TIMER2 */
        rcu_periph_reset_enable(RCU_TIMER2RST);
        rcu_periph_reset_disable(RCU_TIMER2RST);
        break;
    case TIMER13:
        /* reset TIMER13 */
        rcu_periph_reset_enable(RCU_TIMER13RST);
        rcu_periph_reset_disable(RCU_TIMER13RST);
        break;
    case TIMER15:
        /* reset TIMER15 */
        rcu_periph_reset_enable(RCU_TIMER15RST);
        rcu_periph_reset_disable(RCU_TIMER15RST);
        break;
    case TIMER16:
        /* reset TIMER16 */
        rcu_periph_reset_enable(RCU_TIMER16RST);
        rcu_periph_reset_disable(RCU_TIMER16RST);
        break;
    default:
        break;
    }
}

/*!
    \brief      initialize TIMER init parameter struct with a default value (API_ID(0x0002U))
    \param[in]  initpara: init parameter struct
    \param[out] none
    \retval     none
*/
void timer_struct_para_init(timer_parameter_struct *initpara)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(initpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* initialize the init parameter struct member with the default value */
        initpara->prescaler         = 0U;
        initpara->alignedmode       = TIMER_COUNTER_EDGE;
        initpara->counterdirection  = TIMER_COUNTER_UP;
        initpara->period            = 65535U;
        initpara->clockdivision     = TIMER_CKDIV_DIV1;
        initpara->repetitioncounter = 0U;
    }
}

/*!
    \brief      initialize TIMER counter (API_ID(0x0003U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  initpara: init parameter struct
                  prescaler: prescaler value of the counter clock, 0~65535
                  alignedmode: TIMER_COUNTER_EDGE, TIMER_COUNTER_CENTER_DOWN, TIMER_COUNTER_CENTER_UP, TIMER_COUNTER_CENTER_BOTH
                  counterdirection: TIMER_COUNTER_UP, TIMER_COUNTER_DOWN
                  period: counter auto reload value, 0~65535
                  clockdivision: TIMER_CKDIV_DIV1, TIMER_CKDIV_DIV2, TIMER_CKDIV_DIV4
                  repetitioncounter: counter repetition value, 0~65535
    \param[out] none
    \retval     none
*/
void gd32_timer_init(uint32_t timer_periph, timer_parameter_struct *initpara)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(initpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure the counter prescaler value */
        TIMER_PSC(timer_periph) = (uint16_t)initpara->prescaler;

        /* configure the counter direction and aligned mode */
        if((TIMER0 == timer_periph) || (TIMER2 == timer_periph)) {
            TIMER_CTL0(timer_periph) &= (~(uint32_t)(TIMER_CTL0_DIR | TIMER_CTL0_CAM));
            TIMER_CTL0(timer_periph) |= (uint32_t)(initpara->alignedmode & ALIGNEDMODE_MASK);
            TIMER_CTL0(timer_periph) |= (uint32_t)(initpara->counterdirection & COUNTERDIRECTION_MASK);
        }

        /* configure the autoreload value */
        TIMER_CAR(timer_periph) = (uint32_t)initpara->period;
        /* reset the CKDIV bit */
        TIMER_CTL0(timer_periph) &= (~(uint32_t)TIMER_CTL0_CKDIV);
        TIMER_CTL0(timer_periph) |= (uint32_t)((initpara->clockdivision) &CLOCKDIVISION_MASK);

        if((TIMER0 == timer_periph) || (TIMER15 == timer_periph) || (TIMER16 == timer_periph)){
            /* configure the repetition counter value */
            TIMER_CREP(timer_periph) = (uint32_t)initpara->repetitioncounter;
        }

        /* generate an update event */
        TIMER_SWEVG(timer_periph) |= (uint32_t)TIMER_SWEVG_UPG;
    }
}

/*!
    \brief      enable a TIMER (API_ID(0x0004U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_CEN;
}

/*!
    \brief      disable a TIMER (API_ID(0x0005U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_CEN;
}

/*!
    \brief      enable the auto reload shadow function (API_ID(0x0006U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_auto_reload_shadow_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_ARSE;
}

/*!
    \brief      disable the auto reload shadow function (API_ID(0x0007U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_auto_reload_shadow_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_ARSE;
}

/*!
    \brief      enable the update event (API_ID(0x0008U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_update_event_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_UPDIS;
}

/*!
    \brief      disable the update event (API_ID(0x0009U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_update_event_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t) TIMER_CTL0_UPDIS;
}

/*!
    \brief      set TIMER counter alignment mode (API_ID(0x000AU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  aligned:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_COUNTER_EDGE: edge-aligned mode
      \arg        TIMER_COUNTER_CENTER_DOWN: center-aligned and counting down assert mode
      \arg        TIMER_COUNTER_CENTER_UP: center-aligned and counting up assert mode
      \arg        TIMER_COUNTER_CENTER_BOTH: center-aligned and counting up/down assert mode
    \param[out] none
    \retval     none
*/
void timer_counter_alignment(uint32_t timer_periph, uint16_t aligned)
{
    TIMER_CTL0(timer_periph) &= (uint32_t)(~TIMER_CTL0_CAM);
    TIMER_CTL0(timer_periph) |= (uint32_t)(aligned & ALIGNEDMODE_MASK);
}

/*!
    \brief      set TIMER counter up direction (API_ID(0x000BU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[out] none
    \retval     none
*/
void timer_counter_up_direction(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_DIR;
}

/*!
    \brief      set TIMER counter down direction (API_ID(0x000CU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[out] none
    \retval     none
*/
void timer_counter_down_direction(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_DIR;
}

/*!
    \brief      configure TIMER prescaler (API_ID(0x000DU))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  prescaler: prescaler value,0~65535
    \param[in]  pscreload: prescaler reload mode
                only one parameter can be selected which is shown as below:
      \arg        TIMER_PSC_RELOAD_NOW: the prescaler is loaded right now
      \arg        TIMER_PSC_RELOAD_UPDATE: the prescaler is loaded at the next update event
    \param[out] none
    \retval     none
*/
void timer_prescaler_config(uint32_t timer_periph, uint16_t prescaler, uint32_t pscreload)
{
    TIMER_PSC(timer_periph) = (uint32_t)prescaler;

    if(TIMER_PSC_RELOAD_NOW == pscreload) {
        TIMER_SWEVG(timer_periph) |= (uint32_t)TIMER_SWEVG_UPG;
    }
}

/*!
    \brief      configure TIMER repetition register value (API_ID(0x000EU))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  repetition: the counter repetition value, 0~255
    \param[out] none
    \retval     none
*/
void timer_repetition_value_config(uint32_t timer_periph, uint16_t repetition)
{
    TIMER_CREP(timer_periph) = (uint32_t)repetition;
} 
 
/*!
    \brief      configure TIMER autoreload register value (API_ID(0x000FU))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  autoreload: the counter auto-reload value,0~65535
    \param[out] none
    \retval     none
*/
void timer_autoreload_value_config(uint32_t timer_periph, uint16_t autoreload)
{
    TIMER_CAR(timer_periph) = (uint32_t)autoreload;
}

/*!
    \brief      configure TIMER counter register value (API_ID(0x0010U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  counter: the counter value,0~65535
    \param[out] none
    \retval     none
*/
void timer_counter_value_config(uint32_t timer_periph, uint16_t counter)
{
    TIMER_CNT(timer_periph) = (uint32_t)counter;
}

/*!
    \brief      read TIMER counter value (API_ID(0x0011U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     counter value
*/
uint16_t timer_counter_read(uint32_t timer_periph)
{
    uint16_t count_value = 0U;
    count_value = (uint16_t)(TIMER_CNT(timer_periph));
    return (count_value);
}

/*!
    \brief      read TIMER prescaler value (API_ID(0x0012U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     prescaler register value
*/
uint16_t timer_prescaler_read(uint32_t timer_periph)
{
    uint16_t prescaler_value = 0U;
    prescaler_value = (uint16_t)(TIMER_PSC(timer_periph));
    return (prescaler_value);
}

/*!
    \brief      configure TIMER single pulse mode (API_ID(0x0013U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  spmode:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SP_MODE_SINGLE: single pulse mode
      \arg        TIMER_SP_MODE_REPETITIVE: repetitive pulse mode
    \param[out] none
    \retval     none
*/
void timer_single_pulse_mode_config(uint32_t timer_periph, uint32_t spmode)
{
    if(TIMER_SP_MODE_SINGLE == spmode) {
        TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_SPM;
        } else if(TIMER_SP_MODE_REPETITIVE == spmode) {
            TIMER_CTL0(timer_periph) &= ~((uint32_t)TIMER_CTL0_SPM);
        } else {
            /* illegal parameters */
        }
}

/*!
    \brief      configure TIMER update source (API_ID(0x0014U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  update:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_UPDATE_SRC_GLOBAL: update generate by setting of UPG bit or the counter overflow/underflow,or the slave mode controller trigger
      \arg        TIMER_UPDATE_SRC_REGULAR: update generate only by counter overflow/underflow
    \param[out] none
    \retval     none
*/
void timer_update_source_config(uint32_t timer_periph, uint32_t update)
{
    if(TIMER_UPDATE_SRC_REGULAR == update) {
        TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_UPS;
        } else if(TIMER_UPDATE_SRC_GLOBAL == update) {
            TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_UPS;
        } else {
            /* illegal parameters */
        }
}

/*!
    \brief      enable the TIMER DMA (API_ID(0x0015U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  dma: specify which DMA to enable
                only one parameter can be selected which is shown as below:
      \arg        TIMER_DMA_UPD:  update DMA ,TIMERx(x=0,2,15,16)
      \arg        TIMER_DMA_CH0D: channel 0 DMA request,TIMERx(x=0,2,15,16)
      \arg        TIMER_DMA_CH1D: channel 1 DMA request,TIMERx(x=0,2)
      \arg        TIMER_DMA_CH2D: channel 2 DMA request,TIMERx(x=0,2)
      \arg        TIMER_DMA_CH3D: channel 3 DMA request,TIMERx(x=0,2)
      \arg        TIMER_DMA_CMTD: commutation DMA request,TIMERx(x=0)
      \arg        TIMER_DMA_TRGD: trigger DMA request,TIMERx(x=0,2)
    \param[out] none
    \retval     none
*/
void timer_dma_enable(uint32_t timer_periph, uint16_t dma)
{
    TIMER_DMAINTEN(timer_periph) |= ((uint32_t) dma & 0X5F00U);
}

/*!
    \brief      disable the TIMER DMA (API_ID(0x0016U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  dma: specify which DMA to enable
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD:  update DMA ,TIMERx(x=0,2,15,16)
      \arg        TIMER_DMA_CH0D: channel 0 DMA request,TIMERx(x=0,2,15,16)
      \arg        TIMER_DMA_CH1D: channel 1 DMA request,TIMERx(x=0,2)
      \arg        TIMER_DMA_CH2D: channel 2 DMA request,TIMERx(x=0,2)
      \arg        TIMER_DMA_CH3D: channel 3 DMA request,TIMERx(x=0,2)
      \arg        TIMER_DMA_CMTD: commutation DMA request,TIMERx(x=0)
      \arg        TIMER_DMA_TRGD: trigger DMA request,TIMERx(x=0,2)
    \param[out] none
    \retval     none
*/
void timer_dma_disable(uint32_t timer_periph, uint16_t dma)
{
    TIMER_DMAINTEN(timer_periph) &= (~((uint32_t)dma & 0X5F00U));
}

/*!
    \brief      channel DMA request source selection (API_ID(0x0017U))
    \param[in]  timer_periph: TIMERx(x=0,2,15,16)
    \param[in]  dma_request: channel DMA request source selection
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMAREQUEST_CHANNELEVENT: DMA request of channel y is sent when channel y event occurs
       \arg        TIMER_DMAREQUEST_UPDATEEVENT: DMA request of channel y is sent when update event occurs
    \param[out] none
    \retval     none
*/
void timer_channel_dma_request_source_select(uint32_t timer_periph, uint32_t dma_request)
{
    if(TIMER_DMAREQUEST_UPDATEEVENT == dma_request) {
        TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_DMAS;
    } else if(TIMER_DMAREQUEST_CHANNELEVENT == dma_request) {
        TIMER_CTL1(timer_periph) &= ~(uint32_t)TIMER_CTL1_DMAS;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure the TIMER DMA transfer (API_ID(0x0018U))
    \param[in]  timer_periph: TIMERx(x=0,2,15,16)
    \param[in]  dma_baseaddr:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMACFG_DMATA_CTL0: DMA transfer address is TIMER_CTL0
       \arg        TIMER_DMACFG_DMATA_CTL1: DMA transfer address is TIMER_CTL1
       \arg        TIMER_DMACFG_DMATA_SMCFG: DMA transfer address is TIMER_SMCFG
       \arg        TIMER_DMACFG_DMATA_DMAINTEN: DMA transfer address is TIMER_DMAINTEN
       \arg        TIMER_DMACFG_DMATA_INTF: DMA transfer address is TIMER_INTF
       \arg        TIMER_DMACFG_DMATA_SWEVG: DMA transfer address is TIMER_SWEVG
       \arg        TIMER_DMACFG_DMATA_CHCTL0: DMA transfer address is TIMER_CHCTL0
       \arg        TIMER_DMACFG_DMATA_CHCTL1: DMA transfer address is TIMER_CHCTL1
       \arg        TIMER_DMACFG_DMATA_CHCTL2: DMA transfer address is TIMER_CHCTL2
       \arg        TIMER_DMACFG_DMATA_CNT: DMA transfer address is TIMER_CNT
       \arg        TIMER_DMACFG_DMATA_PSC: DMA transfer address is TIMER_PSC
       \arg        TIMER_DMACFG_DMATA_CAR: DMA transfer address is TIMER_CAR
       \arg        TIMER_DMACFG_DMATA_CREP: DMA transfer address is TIMER_CREP
       \arg        TIMER_DMACFG_DMATA_CH0CV: DMA transfer address is TIMER_CH0CV
       \arg        TIMER_DMACFG_DMATA_CH1CV: DMA transfer address is TIMER_CH1CV
       \arg        TIMER_DMACFG_DMATA_CH2CV: DMA transfer address is TIMER_CH2CV
       \arg        TIMER_DMACFG_DMATA_CH3CV: DMA transfer address is TIMER_CH3CV
       \arg        TIMER_DMACFG_DMATA_CCHP: DMA transfer address is TIMER_CCHP
       \arg        TIMER_DMACFG_DMATA_DMACFG: DMA transfer address is TIMER_DMACFG
       \arg        TIMER_DMACFG_DMATA_DMATB: DMA transfer address is TIMER_DMATB
    \param[in]  dma_lenth:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMACFG_DMATC_xTRANSFER(x=1..17): DMA transfer x time
    \param[out] none
    \retval     none
*/
void timer_dma_transfer_config(uint32_t timer_periph, uint32_t dma_baseaddr, uint32_t dma_lenth)
{
    uint32_t ctl;
    ctl = TIMER_DMACFG(timer_periph);
    ctl &= (~(uint32_t)(TIMER_DMACFG_DMATA | TIMER_DMACFG_DMATC));
    ctl |= (uint32_t)((uint32_t)(dma_baseaddr & TIMER_DMACFG_DMATA) | (dma_lenth & TIMER_DMACFG_DMATC));
    TIMER_DMACFG(timer_periph) = ctl;
}

/*!
    \brief      software generate events (API_ID(0x0019U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  event:
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_EVENT_SRC_UPG: update event,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_EVENT_SRC_CH0G: channel 0 capture or compare event generation,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_EVENT_SRC_CH1G: channel 1 capture or compare event generation,TIMERx(x=0,2)
      \arg        TIMER_EVENT_SRC_CH2G: channel 2 capture or compare event generation,TIMERx(x=0,2)
      \arg        TIMER_EVENT_SRC_CH3G: channel 3 capture or compare event generation,TIMERx(x=0,2)
      \arg        TIMER_EVENT_SRC_TRGG: trigger event generation,TIMERx(x=0,2)
      \arg        TIMER_EVENT_SRC_CMTG: channel commutation event generation, TIMERx(x=0,15,16)
      \arg        TIMER_EVENT_SRC_BRK0G: BREAK0 event generation, TIMERx(x=0,15,16)
      \arg        TIMER_EVENT_SRC_BRK1G: BREAK1 event generation, TIMERx(x=0,15,16)
    \param[out] none
    \retval     none
*/
void timer_event_software_generate(uint32_t timer_periph, uint32_t event)
{
    TIMER_SWEVG(timer_periph) |= (uint32_t)event;
}

/*!
    \brief      initialize TIMER break parameter struct with a default value (API_ID(0x001AU))
    \param[in]  breakpara: TIMER break parameter struct
    \param[out] none
    \retval     none
*/
void timer_break_struct_para_init(timer_break_parameter_struct* breakpara)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(breakpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x001AU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* initialize the break parameter struct member with the default value */
        breakpara->runoffstate     = TIMER_ROS_STATE_DISABLE;
        breakpara->ideloffstate    = TIMER_IOS_STATE_DISABLE;
        breakpara->deadtime        = 0U;
        breakpara->outputautostate = TIMER_OUTAUTO_DISABLE;
        breakpara->protectmode     = TIMER_CCHP_PROT_OFF;
        breakpara->break0state     = TIMER_BREAK0_DISABLE;
        breakpara->break0filter    = 0U;
        breakpara->break0polarity  = TIMER_BREAK0_POLARITY_LOW;
        breakpara->break1state     = TIMER_BREAK1_DISABLE;
        breakpara->break1filter    = 0U;
        breakpara->break1polarity  = TIMER_BREAK1_POLARITY_LOW;
    }
}

/*!
    \brief      configure TIMER break function (API_ID(0x001BU))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  breakpara: TIMER break parameter struct
                  runoffstate: TIMER_ROS_STATE_ENABLE,TIMER_ROS_STATE_DISABLE
                  ideloffstate: TIMER_IOS_STATE_ENABLE,TIMER_IOS_STATE_DISABLE
                  deadtime: 0~255
                  outputautostate: TIMER_OUTAUTO_ENABLE,TIMER_OUTAUTO_DISABLE
                  protectmode: TIMER_CCHP_PROT_OFF,TIMER_CCHP_PROT_0,TIMER_CCHP_PROT_1,TIMER_CCHP_PROT_2
                  break0state: TIMER_BREAK0_ENABLE, TIMER_BREAK0_DISABLE
                  break0filter: 0~15
                  break0polarity: TIMER_BREAK0_POLARITY_LOW, TIMER_BREAK0_POLARITY_HIGH
                  break1state: TIMER_BREAK1_ENABLE, TIMER_BREAK1_DISABLE
                  break1filter: 0~15
                  break1polarity: TIMER_BREAK1_POLARITY_LOW, TIMER_BREAK1_POLARITY_HIGH
    \param[out] none
    \retval     none
*/
void timer_break_config(uint32_t timer_periph, timer_break_parameter_struct* breakpara)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(breakpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x001BU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if((TIMER0 == timer_periph) ) {
        TIMER_CCHP(timer_periph) = ((((uint32_t)(breakpara->runoffstate) &TIMER_ROS_STATE_ENABLE))|
                                              (((uint32_t)(breakpara->ideloffstate) &TIMER_IOS_STATE_ENABLE))|
                                              (((uint32_t)(breakpara->deadtime)&DEADTIME_MASK))|
                                              (((uint32_t)(breakpara->outputautostate) &TIMER_OUTAUTO_ENABLE))|
                                              (((uint32_t)(breakpara->break0state) &TIMER_BREAK0_ENABLE)) |
                                              (((uint32_t)(breakpara->break0filter << 16U)&BREAK0FILTER_MASK)) |
                                              (((uint32_t)(breakpara->break0polarity) &TIMER_BREAK0_POLARITY_HIGH))|
                                              (((uint32_t)(breakpara->break1state) &TIMER_BREAK1_ENABLE)) |
                                              (((uint32_t)(breakpara->break1filter << 20U)&BREAK1FILTER_MASK)) |
                                              (((uint32_t)(breakpara->break1polarity) & TIMER_BREAK1_POLARITY_HIGH))|
                                              (((uint32_t)(breakpara->protectmode)&TIMER_CCHP_PROT_2))) ;
        } else if((TIMER15 == timer_periph) || (TIMER16 == timer_periph)) {
            TIMER_CCHP(timer_periph) = ((((uint32_t)(breakpara->runoffstate) &TIMER_ROS_STATE_ENABLE))|
                                              (((uint32_t)(breakpara->ideloffstate) &TIMER_IOS_STATE_ENABLE))|
                                              (((uint32_t)(breakpara->deadtime)&DEADTIME_MASK))|
                                              (((uint32_t)(breakpara->outputautostate) &TIMER_OUTAUTO_ENABLE))|
                                              (((uint32_t)(breakpara->break0state) &TIMER_BREAK0_ENABLE)) |
                                              (((uint32_t)(breakpara->break0filter << 16U)&BREAK0FILTER_MASK)) |
                                              (((uint32_t)(breakpara->break0polarity) &TIMER_BREAK0_POLARITY_HIGH))|
                                              (((uint32_t)(breakpara->protectmode)&TIMER_CCHP_PROT_2))) ;
        } else {
            /* illegal parameters */
        }
        }
}

/*!
    \brief      enable TIMER break function (API_ID(0x001CU))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  break_num: TIMER BREAKx
                only one parameter can be selected which is shown as below:
      \arg        TIMER_BREAK0: BREAK0 input signals, TIMERx(x=0,15,16)
      \arg        TIMER_BREAK1: BREAK1 input signals, TIMERx(x=0)
    \param[out] none
    \retval     none
*/
void timer_break_enable(uint32_t timer_periph, uint16_t break_num)
{
    if(TIMER_BREAK0 == break_num) {
      TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_BRK0EN;
    } else if(TIMER0 == timer_periph) {
        TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_BRK1EN;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      disable TIMER break function (API_ID(0x001DU))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  break_num: TIMER BREAKx
                only one parameter can be selected which is shown as below:
      \arg        TIMER_BREAK0: BREAK0 input signals, TIMERx(x=0,15,16)
      \arg        TIMER_BREAK1: BREAK1 input signals, TIMERx(x=0)
    \param[out] none
    \retval     none
*/
void timer_break_disable(uint32_t timer_periph, uint16_t break_num)
{
    if(TIMER_BREAK0 == break_num) {
      TIMER_CCHP(timer_periph) &= ~(uint32_t)TIMER_CCHP_BRK0EN;
    } else if(TIMER0 == timer_periph) {
        TIMER_CCHP(timer_periph) &= (~(uint32_t)TIMER_CCHP_BRK1EN);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      enable TIMER output automatic function (API_ID(0x001EU))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[out] none
    \retval     none
*/
void timer_automatic_output_enable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_OAEN;
}

/*!
    \brief      disable TIMER output automatic function (API_ID(0x001FU))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[out] none
    \retval     none
*/
void timer_automatic_output_disable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) &= ~(uint32_t)TIMER_CCHP_OAEN;
}

/*!
    \brief      configure TIMER primary output function (API_ID(0x0020U))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void timer_primary_output_config(uint32_t timer_periph, ControlStatus newvalue)
{
    if(ENABLE == newvalue){
        TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_POEN;
    }else{
        TIMER_CCHP(timer_periph) &= (~(uint32_t)TIMER_CCHP_POEN);
    }
}

/*!
    \brief      enable or disable channel capture/compare control shadow register  (API_ID(0x0021U))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  newvalue: ENABLE or DISABLE 
    \param[out] none
    \retval     none
*/
void timer_channel_control_shadow_config(uint32_t timer_periph, ControlStatus newvalue)
{
    if(ENABLE == newvalue){
        TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_CCSE;
    }else{
        TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_CCSE);
    }
}

/*!
    \brief      configure TIMER channel control shadow register update control (API_ID(0x0022U))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  ccuctl: channel control shadow register update control
                only one parameter can be selected which is shown as below:
      \arg        TIMER_UPDATECTL_CCU: the shadow registers update by when CMTG bit is set
      \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers update by when CMTG bit is set or an rising edge of TRGI occurs 
    \param[out] none
    \retval     none
*/              
void timer_channel_control_shadow_update_config(uint32_t timer_periph, uint32_t ccuctl)
{
        if(TIMER_UPDATECTL_CCU == ccuctl){
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_CCUC);
        }else if(TIMER_UPDATECTL_CCUTRI == ccuctl){
            TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_CCUC;
        }else{
            /* illegal parameters */        
        }
}

/*!
    \brief      initialize TIMER channel output parameter struct with a default value (API_ID(0x0023U))
    \param[in]  ocpara: TIMER channel n output parameter struct
    \param[out] none
    \retval     none
*/
void timer_channel_output_struct_para_init(timer_oc_parameter_struct *ocpara)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(ocpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x0023U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* initialize the channel output parameter struct member with the default value */
        ocpara->outputstate  = TIMER_CCX_DISABLE;
        ocpara->outputnstate = TIMER_CCXN_DISABLE;
        ocpara->ocpolarity   = TIMER_OC_POLARITY_HIGH;
        ocpara->ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
        ocpara->ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
        ocpara->ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    }
}

/*!
    \brief      configure TIMER channel output function (API_ID(0x0024U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel 0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel 1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel 2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel 3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel 4(TIMERx(x=0))
    \param[in]  ocpara: TIMER channeln output parameter struct
                  outputstate: TIMER_CCX_ENABLE,TIMER_CCX_DISABLE
                  outputnstate: TIMER_CCXN_ENABLE,TIMER_CCXN_DISABLE
                  ocpolarity: TIMER_OC_POLARITY_HIGH,TIMER_OC_POLARITY_LOW
                  ocnpolarity: TIMER_OCN_POLARITY_HIGH,TIMER_OCN_POLARITY_LOW
                  ocidlestate: TIMER_OC_IDLE_STATE_LOW,TIMER_OC_IDLE_STATE_HIGH
                  ocnidlestate: TIMER_OCN_IDLE_STATE_LOW,TIMER_OCN_IDLE_STATE_HIGH
    \param[out] none
    \retval     none
*/
void timer_channel_output_config(uint32_t timer_periph, uint16_t channel, timer_oc_parameter_struct *ocpara)
{
    uint32_t ctl;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(ocpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x0024U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        switch(channel){
        /* configure TIMER_CH_0 */
        case TIMER_CH_0:
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            TIMER_CHCTL0(timer_periph) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
            /* set the CH0EN bit */
            TIMER_CHCTL2(timer_periph) |= ((uint32_t)ocpara->outputstate & TIMER_CCX_ENABLE);
            /* set the CH0P bit */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL2_CH0P);
            ctl |= ((uint32_t)ocpara->ocpolarity & TIMER_CHCTL2_CH0P);
            TIMER_CHCTL2(timer_periph) = ctl;
            if((TIMER0 == timer_periph) || (TIMER15 == timer_periph) || (TIMER16 == timer_periph)){
                /* reset the CH0NEN bit */
                TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
                /* set the CH0NEN bit */
                TIMER_CHCTL2(timer_periph) |= (uint32_t)(ocpara->outputnstate & TIMER_CHCTL2_CH0NEN);
                /* reset the CH0NP bit */
                TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
                /* set the CH0NP bit */
                TIMER_CHCTL2(timer_periph) |= (uint32_t)(ocpara->ocnpolarity & TIMER_CHCTL2_CH0NP);
                /* reset the ISO0 bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO0);
                /* set the ISO0 bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)(ocpara->ocidlestate & TIMER_CTL1_ISO0);
                /* reset the ISO0N bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO0N);
                /* set the ISO0N bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)(ocpara->ocnidlestate & TIMER_CTL1_ISO0N);
            }
            break;
        /* configure TIMER_CH_1 */
        case TIMER_CH_1:
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
            TIMER_CHCTL0(timer_periph) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;
            /* set the CH1EN bit */
            TIMER_CHCTL2(timer_periph) |= (((uint32_t)ocpara->outputstate & TIMER_CCX_ENABLE) << 4U);
            /* set the CH1P bit */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL2_CH1P);
            ctl |= (uint32_t)(((uint32_t)ocpara->ocpolarity & TIMER_CHCTL2_CH0P) << 4U);
            TIMER_CHCTL2(timer_periph) = ctl;
            if(TIMER0 == timer_periph){
                /* reset the CH1NEN bit */
                TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
                /* set the CH1NEN bit */
                TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->outputnstate & TIMER_CHCTL2_CH0NEN) << 4U);
                /* reset the CH1NP bit */
                TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
                /* set the CH1NP bit */
                TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnpolarity & TIMER_CHCTL2_CH0NP) << 4U);
                /* reset the ISO1 bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO1);
                /* set the ISO1 bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate & TIMER_CTL1_ISO0) << 2U);
                /* reset the ISO1N bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO1N);
                /* set the ISO1N bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnidlestate & TIMER_CTL1_ISO0N) << 2U);
            }
            break;
        /* configure TIMER_CH_2 */
        case TIMER_CH_2:
            /* reset the CH2EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
            TIMER_CHCTL1(timer_periph) &= ~(uint32_t)TIMER_CHCTL1_CH2MS;
            /* set the CH2EN bit */
            TIMER_CHCTL2(timer_periph) |= (((uint32_t)ocpara->outputstate & TIMER_CCX_ENABLE) << 8U);
            /* set the CH2P bit */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL2_CH2P);
            ctl |= (((uint32_t)ocpara->ocpolarity & TIMER_CHCTL2_CH0P) << 8U);
            TIMER_CHCTL2(timer_periph) = ctl;
            if(TIMER0 == timer_periph){
                /* reset the CH2NEN bit */
                TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
                /* set the CH2NEN bit */
                TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->outputnstate & TIMER_CHCTL2_CH0NEN) << 8U);
                /* reset the CH2NP bit */
                TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
                /* set the CH2NP bit */
                TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnpolarity & TIMER_CHCTL2_CH0NP) << 8U);
                /* reset the ISO2 bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO2);
                /* set the ISO2 bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate & TIMER_CTL1_ISO0) << 4U);
                /* reset the ISO2N bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO2N);
                /* set the ISO2N bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnidlestate & TIMER_CTL1_ISO0N) << 4U);
            }
            break;
        /* configure TIMER_CH_3 */
        case TIMER_CH_3:
            /* reset the CH3EN bit */
            TIMER_CHCTL2(timer_periph) &=(~(uint32_t)TIMER_CHCTL2_CH3EN);
            TIMER_CHCTL1(timer_periph) &= ~(uint32_t)TIMER_CHCTL1_CH3MS;
            /* set the CH3EN bit */
            TIMER_CHCTL2(timer_periph) |= (((uint32_t)ocpara->outputstate & TIMER_CCX_ENABLE) << 12U);
            /* set the CH3P bit */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL2_CH3P);
            ctl |= (((uint32_t)ocpara->ocpolarity & TIMER_CHCTL2_CH0P) << 12U);
            TIMER_CHCTL2(timer_periph) = ctl;
            if(TIMER0 == timer_periph){
                /* reset the ISO3 bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO3);
                /* set the ISO3 bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate & TIMER_CTL1_ISO0) << 6U);
            }
            break;
        /* configure TIMER_CH_4 */
        case TIMER_CH_4:
            if(TIMER0 == timer_periph){
                /* reset the CH4EN bit */
                TIMER_CHCTL2(timer_periph) &=(~(uint32_t)TIMER_CHCTL2_CH4EN);
                /* set the CH4EN bit */
                TIMER_CHCTL2(timer_periph) |= (((uint32_t)ocpara->outputstate & TIMER_CCX_ENABLE) << 16U);
                /* set the CH4P bit */
                ctl = TIMER_CHCTL2(timer_periph);
                ctl &= (~(uint32_t)TIMER_CHCTL2_CH4P);
                ctl |= (((uint32_t)ocpara->ocpolarity & TIMER_CHCTL2_CH0P) << 16U);
                TIMER_CHCTL2(timer_periph) = ctl;
                /* reset the ISO4 bit */
                TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO4);
                /* set the ISO4 bit */
                TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate & TIMER_CTL1_ISO0) << 8U);
            }
            break;
        default:
            break;
        }
    }
}

/*!
    \brief      configure TIMER channel output compare mode (API_ID(0x0025U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel4(TIMERx(x=0))
    \param[in]  ocmode: channel output compare mode
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_MODE_TIMING: timing mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_ACTIVE: active mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_INACTIVE: inactive mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_TOGGLE: toggle mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_LOW: force low mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_HIGH: force high mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_PWM0: PWM0 mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_PWM1: PWM1 mode(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_OC_MODE_DELAYABLE_SPM0  : Delayable SPM mode 0(TIMERx(x=0,2))
      \arg        TIMER_OC_MODE_DELAYABLE_SPM1  : Delayable SPM mode 1(TIMERx(x=0,2))
      \arg        TIMER_OC_MODE_COMBINED_PWM0   : Combined PWM mode 0(TIMERx(x=0,2)) 
      \arg        TIMER_OC_MODE_COMBINED_PWM1   : Combined PWM mode 1(TIMERx(x=0,2))
      \arg        TIMER_OC_MODE_ASYMMETRIC_PWM0 : Asymmetric PWM mode 0(TIMERx(x=0,2))
      \arg        TIMER_OC_MODE_ASYMMETRIC_PWM1 : Asymmetric PWM mode 1(TIMERx(x=0,2))
    \param[out] none
    \retval     none
*/
void timer_channel_output_mode_config(uint32_t timer_periph, uint16_t channel, uint32_t ocmode)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0COMCTL);
        ctl |= (uint32_t)(ocmode&TIMER_CHCTL0_CH0COMCTL);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1COMCTL);
        ctl |= (uint32_t)((uint32_t)(ocmode&TIMER_CHCTL0_CH0COMCTL) << 8U);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH2COMCTL);
        ctl |= (uint32_t)(ocmode&TIMER_CHCTL1_CH2COMCTL);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH3COMCTL);
        ctl |= (uint32_t)((uint32_t)(ocmode&TIMER_CHCTL1_CH2COMCTL) << 8U);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        ctl = TIMER_CHCTL3(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL3_CH4COMCTL);
        ctl |= (uint32_t)(ocmode&TIMER_CHCTL3_CH4COMCTL);
        TIMER_CHCTL3(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output combine mode (API_ID(0x0026U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  state: channel combined_3_phase_pwm
                only one parameter can be selected which is shown as below:
      \arg        TIMER_COMBINE_CHANNEL_DISABLE: channel combined 3 phase disable
      \arg        TIMER_COMBINE_CHANNEL_ENABLE: channel combined 3 phase enable
    \param[out] none
    \retval     none
*/
void timer_channel_combined_3_phase_pwm_config(uint32_t timer_periph, uint16_t channel, uint32_t state)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CH4CV(timer_periph);
        ctl &= (~(uint32_t)TIMER_CH4CV_CCH4CH0);
        ctl |= (uint32_t)((state & TIMER_COMBINE_CHANNEL_ENABLE) << 29U);
        TIMER_CH4CV(timer_periph) = ctl;
        break;
    case TIMER_CH_1:
        ctl = TIMER_CH4CV(timer_periph);
        ctl &= (~(uint32_t)TIMER_CH4CV_CCH4CH1);
        ctl |= (uint32_t)((state & TIMER_COMBINE_CHANNEL_ENABLE) << 30U);
        TIMER_CH4CV(timer_periph) = ctl;
        break;
    case TIMER_CH_2:
        ctl = TIMER_CH4CV(timer_periph);
        ctl &= (~(uint32_t)TIMER_CH4CV_CCH4CH2);
        ctl |= (uint32_t)((state & TIMER_COMBINE_CHANNEL_ENABLE) << 31U);
        TIMER_CH4CV(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output pulse value (API_ID(0x0027U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel4(TIMERx(x=0))
    \param[in]  pulse: channel output pulse value
    \param[out] none
    \retval     none
*/
void timer_channel_output_pulse_value_config(uint32_t timer_periph, uint16_t channel, uint32_t pulse)
{
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CH0CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CH1CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CH2CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        TIMER_CH3CV(timer_periph) = (uint32_t)pulse;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        TIMER_CH4CV(timer_periph) = (uint32_t)(pulse & 0xFFFFU);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output shadow function (API_ID(0x0028U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel3(TIMERx(x=0))
    \param[in]  ocshadow: channel output shadow state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_SHADOW_ENABLE: channel output shadow state enable
      \arg        TIMER_OC_SHADOW_DISABLE: channel output shadow state disable
    \param[out] none
    \retval     none
*/
void timer_channel_output_shadow_config(uint32_t timer_periph, uint16_t channel, uint16_t ocshadow)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0COMSEN);
        ctl |= ((uint32_t)ocshadow & TIMER_OC_SHADOW_ENABLE);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1COMSEN);
        ctl |= (((uint32_t)ocshadow & TIMER_OC_SHADOW_ENABLE) << 8U);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH2COMSEN);
        ctl |= ((uint32_t)ocshadow & TIMER_OC_SHADOW_ENABLE);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH3COMSEN);
        ctl |= (((uint32_t)ocshadow & TIMER_OC_SHADOW_ENABLE) << 8U);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        ctl = TIMER_CHCTL3(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL3_CH4COMSEN);
        ctl |= ((uint32_t)ocshadow & TIMER_OC_SHADOW_ENABLE);
        TIMER_CHCTL3(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output fast function (API_ID(0x0029U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel4(TIMERx(x=0))
    \param[in]  ocfast: channel output fast function
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_FAST_ENABLE: channel output fast function enable
      \arg        TIMER_OC_FAST_DISABLE: channel output fast function disable
    \param[out] none
    \retval     none
*/
void timer_channel_output_fast_config(uint32_t timer_periph, uint16_t channel, uint16_t ocfast)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0COMFEN);
        ctl |= ((uint32_t)ocfast & TIMER_OC_FAST_ENABLE);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1COMFEN);
        ctl |= (((uint32_t)ocfast & TIMER_OC_FAST_ENABLE) << 8U);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH2COMFEN);
        ctl |= ((uint32_t)ocfast & TIMER_OC_FAST_ENABLE);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH3COMFEN);
        ctl |= (((uint32_t)ocfast & TIMER_OC_FAST_ENABLE) << 8U);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        ctl = TIMER_CHCTL3(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL3_CH4COMFEN);
        ctl |= ((uint32_t)ocfast & TIMER_OC_FAST_ENABLE);
        TIMER_CHCTL3(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output clear function (API_ID(0x002AU))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0 TIMERx(x=0,2) 
      \arg        TIMER_CH_1: TIMER channel1 TIMERx(x=0,2) 
      \arg        TIMER_CH_2: TIMER channel2 TIMERx(x=0,2) 
      \arg        TIMER_CH_3: TIMER channel3 TIMERx(x=0,2) 
      \arg        TIMER_CH_4: TIMER channel4 TIMERx(x=0)
    \param[in]  occlear: channel output clear function
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_CLEAR_ENABLE: channel output clear function enable
      \arg        TIMER_OC_CLEAR_DISABLE: channel output clear function disable
    \param[out] none
    \retval     none
*/
void timer_channel_output_clear_config(uint32_t timer_periph, uint16_t channel, uint16_t occlear)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0COMCEN);
        ctl |= ((uint32_t)occlear & TIMER_OC_CLEAR_ENABLE);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1COMCEN);
        ctl |= (((uint32_t)occlear & TIMER_OC_CLEAR_ENABLE) << 8U);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH2COMCEN);
        ctl |= ((uint32_t)occlear & TIMER_OC_CLEAR_ENABLE);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH3COMCEN);
        ctl |= (((uint32_t)occlear & TIMER_OC_CLEAR_ENABLE) << 8U);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        ctl = TIMER_CHCTL3(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL3_CH4COMCEN);
        ctl |= ((uint32_t)occlear & TIMER_OC_CLEAR_ENABLE);
        TIMER_CHCTL3(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output polarity (API_ID(0x002BU))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel4(TIMERx(x=0))
    \param[in]  ocpolarity: channel output polarity
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_POLARITY_HIGH: channel output polarity is high
      \arg        TIMER_OC_POLARITY_LOW: channel output polarity is low
    \param[out] none
    \retval     none
*/
void timer_channel_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocpolarity)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH0P);
        ctl |= ((uint32_t)ocpolarity & TIMER_CHCTL2_CH0P);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH1P);
        ctl |= (((uint32_t)ocpolarity & TIMER_CHCTL2_CH0P) << 4U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH2P);
        ctl |= (((uint32_t)ocpolarity & TIMER_CHCTL2_CH0P) << 8U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH3P);
        ctl |= (((uint32_t)ocpolarity & TIMER_CHCTL2_CH0P) << 12U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH4P);
        ctl |= (((uint32_t)ocpolarity & TIMER_CHCTL2_CH0P) << 16U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel complementary output polarity  (API_ID(0x002CU))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: channel to be configured
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  ocnpolarity: channel complementary output polarity 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OCN_POLARITY_HIGH: channel complementary output polarity is high
      \arg        TIMER_OCN_POLARITY_LOW: channel complementary output polarity is low
    \param[out] none
    \retval     none
*/
void timer_channel_complementary_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnpolarity)
{
    switch(channel){
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)(ocnpolarity & TIMER_CHCTL2_CH0NP);
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocnpolarity & TIMER_CHCTL2_CH0NP) << 4U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocnpolarity & TIMER_CHCTL2_CH0NP) << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel enable state (API_ID(0x002DU))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
      \arg        TIMER_CH_4: TIMER channel4(TIMERx(x=0))
    \param[in]  state: TIMER channel enable state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CCX_ENABLE: channel enable
      \arg        TIMER_CCX_DISABLE: channel disable
    \param[out] none
    \retval     none
*/
void timer_channel_output_state_config(uint32_t timer_periph, uint16_t channel, uint32_t state)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
        ctl |= (uint32_t)(state & TIMER_CHCTL2_CH0EN);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
        ctl |= (uint32_t)((uint32_t)(state & TIMER_CHCTL2_CH0EN) << 4U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
        ctl |= (uint32_t)((uint32_t)(state & TIMER_CHCTL2_CH0EN) << 8U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
        ctl |= (uint32_t)((uint32_t)(state & TIMER_CHCTL2_CH0EN) << 12U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_4 */
    case TIMER_CH_4:
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL2_CH4EN);
        ctl |= (uint32_t)((uint32_t)(state & TIMER_CHCTL2_CH0EN) << 16U);
        TIMER_CHCTL2(timer_periph) = ctl;
        break;
    default:
        break;
    }
}


/*!
    \brief      configure TIMER channel complementary output enable state (API_ID(0x002EU))
    \param[in]  timer_periph: TIMERx(x=0)
    \param[in]  channel:
                only one parameter can be selected which is shown as below: 
      \arg        TIMER_CH_0: TIMER channel 0(TIMERx(x=0,15,16))
      \arg        TIMER_CH_1: TIMER channel 1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel 2(TIMERx(x=0))
    \param[in]  ocnstate: TIMER channel complementary output enable state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CCXN_ENABLE: channel complementary enable 
      \arg        TIMER_CCXN_DISABLE: channel complementary disable 
    \param[out] none
    \retval     none
*/
void timer_channel_complementary_output_state_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnstate)
{
    switch(channel){
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
        TIMER_CHCTL2(timer_periph) |= ((uint32_t)ocnstate & TIMER_CCXN_ENABLE);
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
        TIMER_CHCTL2(timer_periph) |= (((uint32_t)ocnstate & TIMER_CCXN_ENABLE) << 4U);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
        TIMER_CHCTL2(timer_periph) |= (uint32_t)(((uint32_t)ocnstate & TIMER_CCXN_ENABLE) << 8U);
        break;
    default:
        break;
    }
}

/*!
    \brief      initialize TIMER channel input parameter struct with a default value (API_ID(0x002FU))
    \param[in]  icpara: TIMER channel intput parameter struct
    \param[out] none
    \retval     none
*/
void timer_channel_input_struct_para_init(timer_ic_parameter_struct *icpara)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(icpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x002FU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* initialize the channel input parameter struct member with the default value */
        icpara->icpolarity  = TIMER_IC_POLARITY_RISING;
        icpara->icselection = TIMER_IC_SELECTION_DIRECTTI;
        icpara->icprescaler = TIMER_IC_PSC_DIV1;
        icpara->icfilter    = 0U;
    }
}

/*!
    \brief      configure TIMER input capture parameter (API_ID(0x0030U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
     \param[in]  icpara: TIMER channel intput parameter struct
                   icpolarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING,TIMER_IC_POLARITY_BOTH_EDGE
                   icselection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI,TIMER_IC_SELECTION_ITS
                   icprescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
                   icfilter: 0~15
    \param[out]  none
    \retval      none
*/
void timer_input_capture_config(uint32_t timer_periph, uint16_t channel, timer_ic_parameter_struct *icpara)
{
    uint32_t ctl;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(icpara)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x0030U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        switch(channel) {
        /* configure TIMER_CH_0 */
        case TIMER_CH_0:
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);

            /* reset the CH0P and CH0NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            ctl |= (((uint32_t)icpara->icpolarity) & TIMER_IC_POLARITY_BOTH_EDGE);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* reset the CH0MS bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
            ctl |= (uint32_t)((icpara->icselection) & TIMER_CHCTL0_CH0MS);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* reset the CH0CAPFLT bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            ctl |= ((((uint32_t)icpara->icfilter) & 0XFU) << 4U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH0EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH0EN;
            break;

        /* configure TIMER_CH_1 */
        case TIMER_CH_1:
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);

            /* reset the CH1P and CH1NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            ctl |= (((uint32_t)(icpara->icpolarity) & TIMER_IC_POLARITY_BOTH_EDGE) << 4U);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* reset the CH1MS bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
            ctl |= (uint32_t)((uint32_t)((icpara->icselection) & TIMER_CHCTL0_CH0MS) << 8U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* reset the CH1CAPFLT bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
            ctl |= ((((uint32_t)icpara->icfilter) & 0XFU) << 12U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH1EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH1EN;
            break;
        /* configure TIMER_CH_2 */
        case TIMER_CH_2:
            /* reset the CH2EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);

            /* reset the CH2P and CH2NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH2P | TIMER_CHCTL2_CH2NP));
            ctl |= ((((uint32_t)icpara->icpolarity) & TIMER_IC_POLARITY_BOTH_EDGE) << 8U);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* reset the CH2MS bit */
            ctl = TIMER_CHCTL1(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL1_CH2MS);
            ctl |= (uint32_t)((uint32_t)((icpara->icselection) & TIMER_CHCTL0_CH0MS));
            TIMER_CHCTL1(timer_periph) = ctl;
            /* reset the CH2CAPFLT bit */
            ctl = TIMER_CHCTL1(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL1_CH2CAPFLT);
            ctl |= ((((uint32_t)icpara->icfilter) & 0XFU) << 4U);
            TIMER_CHCTL1(timer_periph) = ctl;
            /* set the CH2EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH2EN;
            break;
        /* configure TIMER_CH_3 */
        case TIMER_CH_3:
            /* reset the CH3EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);

            /* reset the CH3P and CH3NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH3P | TIMER_CHCTL2_CH3NP));
            ctl |= ((((uint32_t)icpara->icpolarity) & TIMER_IC_POLARITY_BOTH_EDGE) << 12U);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* reset the CH3MS bit */
            ctl = TIMER_CHCTL1(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL1_CH3MS);
            ctl |= (uint32_t)((uint32_t)((icpara->icselection) & TIMER_CHCTL0_CH0MS) << 8U);
            TIMER_CHCTL1(timer_periph) = ctl;
            /* reset the CH3CAPFLT bit */
            ctl = TIMER_CHCTL1(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL1_CH3CAPFLT);
            ctl |= ((((uint32_t)icpara->icfilter) & 0XFU) << 12U);
            TIMER_CHCTL1(timer_periph) = ctl;
            /* set the CH3EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH3EN;
            break;
        default:
            break;
        }
        /* configure TIMER channel input capture prescaler value */
        timer_channel_input_capture_prescaler_config(timer_periph, channel, (uint16_t)(icpara->icprescaler));
    }
}

/*!
    \brief      configure TIMER channel input capture prescaler value (API_ID(0x0031U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  prescaler: channel input capture prescaler value
                only one parameter can be selected which is shown as below:
      \arg        TIMER_IC_PSC_DIV1: no prescaler
      \arg        TIMER_IC_PSC_DIV2: divided by 2
      \arg        TIMER_IC_PSC_DIV4: divided by 4
      \arg        TIMER_IC_PSC_DIV8: divided by 8
    \param[out] none
    \retval     none
*/
void timer_channel_input_capture_prescaler_config(uint32_t timer_periph, uint16_t channel, uint16_t prescaler)
{
    uint32_t ctl;
    switch(channel) {
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0CAPPSC);
        ctl |= ((uint32_t)prescaler & TIMER_CHCTL0_CH0CAPPSC);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1CAPPSC);
        ctl |= (((uint32_t)prescaler & TIMER_CHCTL0_CH0CAPPSC) << 8U);
        TIMER_CHCTL0(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH2CAPPSC);
        ctl |= ((uint32_t)prescaler & TIMER_CHCTL0_CH0CAPPSC);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        ctl = TIMER_CHCTL1(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL1_CH3CAPPSC);
        ctl |= (((uint32_t)prescaler & TIMER_CHCTL0_CH0CAPPSC) << 8U);
        TIMER_CHCTL1(timer_periph) = ctl;
        break;
    default:
        break;
    }
}

/*!
    \brief      read TIMER channel capture compare register value (API_ID(0x0032U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13,15,16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     channel capture compare register value
*/
uint16_t timer_channel_capture_value_register_read(uint32_t timer_periph, uint16_t channel)
{
    uint16_t count_value = 0U;
    switch(channel) {
    /* read TIMER channel 0 capture compare register value */
    case TIMER_CH_0:
        count_value = (uint16_t)TIMER_CH0CV(timer_periph);
        break;
    /* read TIMER channel 1 capture compare register value */
    case TIMER_CH_1:
        count_value = (uint16_t)TIMER_CH1CV(timer_periph);
        break;
    /* read TIMER channel 2 capture compare register value */
    case TIMER_CH_2:
        count_value = (uint16_t)TIMER_CH2CV(timer_periph);
        break;
    /* read TIMER channel 3 capture compare register value */
    case TIMER_CH_3:
        count_value = (uint16_t)TIMER_CH3CV(timer_periph);
        break;
    default:
        break;
    }
    return (count_value);
}

/*!
    \brief      configure TIMER input pwm capture function (API_ID(0x0033U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0
      \arg        TIMER_CH_1: TIMER channel1
     \param[in]  icpwm:TIMER channel intput pwm parameter struct
                 icpolarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING
                 icselection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI
                 icprescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
                 icfilter: 0~15
    \param[out] none
    \retval     none
*/
void timer_input_pwm_capture_config(uint32_t timer_periph, uint16_t channel, timer_ic_parameter_struct *icpwm)
{
    uint32_t ctl;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(icpwm)) {
        fw_debug_report_err(TIMER_MODULE_ID, API_ID(0x0033U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint16_t icpolarity;
        uint16_t icselection;

        /* Set channel input polarity */
        if(TIMER_IC_POLARITY_RISING == icpwm->icpolarity) {
            icpolarity = TIMER_IC_POLARITY_FALLING;
        } else {
            icpolarity = TIMER_IC_POLARITY_RISING;
        }
        /* Set channel input mode selection */
        if(TIMER_IC_SELECTION_DIRECTTI == icpwm->icselection) {
            icselection = TIMER_IC_SELECTION_INDIRECTTI;
        } else {
            icselection = TIMER_IC_SELECTION_DIRECTTI;
        }

        if(TIMER_CH_0 == channel) {
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            /* set the CH0P and CH0NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            ctl |= (((uint32_t)icpwm->icpolarity) & TIMER_IC_POLARITY_BOTH_EDGE);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* set the CH0MS bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
            ctl |= (uint32_t)((icpwm->icselection) & TIMER_CHCTL0_CH0MS);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH0CAPFLT bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            ctl |= ((((uint32_t)icpwm->icfilter) & 0xFU) << 4U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH0EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH0EN;
            /* configure TIMER channel input capture prescaler value */
            timer_channel_input_capture_prescaler_config(timer_periph, TIMER_CH_0, (uint16_t)((icpwm->icprescaler) & TIMER_IC_PSC_DIV8));

            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
            /* set the CH1P and CH1NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            ctl |= (((uint32_t)icpolarity & TIMER_IC_POLARITY_BOTH_EDGE) << 4U);
            TIMER_CHCTL2(timer_periph) = ctl;

            /* set the CH1MS bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
            ctl |= (((uint32_t)icselection & TIMER_CHCTL0_CH0MS) << 8U);
            TIMER_CHCTL0(timer_periph) = ctl;

            /* set the CH1CAPFLT bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
            ctl |= ((((uint32_t)icpwm->icfilter) & 0xFU) << 12U);
            TIMER_CHCTL0(timer_periph) = ctl;

            /* set the CH1EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH1EN;
            /* configure TIMER channel input capture prescaler value */
            timer_channel_input_capture_prescaler_config(timer_periph, TIMER_CH_1, (uint16_t)((icpwm->icprescaler) & TIMER_IC_PSC_DIV8));
        } else {
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
            /* set the CH1P and CH1NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            ctl |= ((((uint32_t)icpwm->icpolarity) & TIMER_IC_POLARITY_BOTH_EDGE) << 4U);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* set the CH1MS bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
            ctl |= ((((uint32_t)icpwm->icselection) &TIMER_CHCTL0_CH0MS) << 8U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH1CAPFLT bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
            ctl |= ((((uint32_t)icpwm->icfilter) & 0xFU) << 12U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH1EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH1EN;
            /* configure TIMER channel input capture prescaler value */
            timer_channel_input_capture_prescaler_config(timer_periph, TIMER_CH_1, (uint16_t)((icpwm->icprescaler)& TIMER_IC_PSC_DIV8));

            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            /* set the CH0P and CH0NP bits */
            ctl = TIMER_CHCTL2(timer_periph);
            ctl &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            ctl |= ((uint32_t)icpolarity & TIMER_IC_POLARITY_BOTH_EDGE);
            TIMER_CHCTL2(timer_periph) = ctl;
            /* set the CH0MS bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
            ctl |= ((uint32_t)icselection & TIMER_CHCTL0_CH0MS);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH0CAPFLT bit */
            ctl = TIMER_CHCTL0(timer_periph);
            ctl &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            ctl |= ((((uint32_t)icpwm->icfilter) & 0xFU) << 4U);
            TIMER_CHCTL0(timer_periph) = ctl;
            /* set the CH0EN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH0EN;
            /* configure TIMER channel input capture prescaler value */
            timer_channel_input_capture_prescaler_config(timer_periph, TIMER_CH_0, (uint16_t)((icpwm->icprescaler) & TIMER_IC_PSC_DIV8));
        }
    }
}

/*!
    \brief      configure TIMER hall sensor mode (API_ID(0x0034U))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  hallmode:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_HALLINTERFACE_ENABLE: TIMER hall sensor mode enable
      \arg        TIMER_HALLINTERFACE_DISABLE: TIMER hall sensor mode disable
    \param[out] none
    \retval     none
*/
void timer_hall_mode_config(uint32_t timer_periph, uint32_t hallmode)
{
    if(TIMER_HALLINTERFACE_ENABLE == hallmode) {
        TIMER_CTL1(timer_periph) |= (uint32_t)TIMER_CTL1_TI0S;
    } else {
        TIMER_CTL1(timer_periph) &= ~(uint32_t)TIMER_CTL1_TI0S;
    }
}

/*!
    \brief      select TIMER input trigger source (API_ID(0x0035U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  intrigger:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SMCFG_TRGSEL_NONE: internal trigger disable
      \arg        TIMER_SMCFG_TRGSEL_ITI0: internal trigger 0
      \arg        TIMER_SMCFG_TRGSEL_ITI2: internal trigger 2
      \arg        TIMER_SMCFG_TRGSEL_ITI3: internal trigger 3
      \arg        TIMER_SMCFG_TRGSEL_CI0F_ED: TI0 edge detector
      \arg        TIMER_SMCFG_TRGSEL_CI0FE0: filtered TIMER input 0
      \arg        TIMER_SMCFG_TRGSEL_CI1FE1: filtered TIMER input 1
      \arg        TIMER_SMCFG_TRGSEL_ETIFP: external trigger
    \param[out] none
    \retval     none
*/
void timer_input_trigger_source_select(uint32_t timer_periph, uint32_t intrigger)
{
    uint32_t TIMERxCFG_temp0 = 0U,TIMERxCFG_temp1 = 0U;
    volatile uint32_t *TIMERxCFG_addr0, *TIMERxCFG_addr1;
    uint8_t i = 0U;
    switch(timer_periph){
    case TIMER0:
        TIMERxCFG_addr0 = &SYSCFG_TIMER0CFG0;
        TIMERxCFG_addr1 = &SYSCFG_TIMER0CFG1;
        break;
    case TIMER2:
        TIMERxCFG_addr0 = &SYSCFG_TIMER2CFG0;
        TIMERxCFG_addr1 = &SYSCFG_TIMER2CFG1;
        break;
    default:
        break;
    }
    TIMERxCFG_temp0 = REG32(TIMERxCFG_addr0);
    TIMERxCFG_temp1 = REG32(TIMERxCFG_addr1);
    if((TIMERxCFG_temp0 & 0xFFFFFFFFU) == 0U){
        TIMERxCFG_temp1 = (BITS(0,2) & intrigger);
        REG32(TIMERxCFG_addr1) = TIMERxCFG_temp1;
    }else{
        for(i = 0U; i < 8U; i++){
          if((TIMERxCFG_temp0 & (BITS(0,3) << (i * 4U))) != 0U){
              break;
          }
        }
        TIMERxCFG_temp0 = ((BITS(0,3) << (i * 4U)) & ((uint32_t)(intrigger) << (i * 4U)));
        REG32(TIMERxCFG_addr0) = TIMERxCFG_temp0;
    }
}

/*!
    \brief      select TIMER master mode output trigger source (API_ID(0x0036U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  outrigger:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_TRI_OUT_SRC_RESET: the UPG bit as trigger output (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_ENABLE: the counter enable signal TIMER_CTL0_CEN as trigger output (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_UPDATE: update event as trigger output (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_CH0: a capture or a compare match occurred in channal0 as trigger output TRGO (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_O0CPRE: O0CPRE as trigger output (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_O1CPRE: O1CPRE as trigger output (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_O2CPRE: O2CPRE as trigger output (TIMERx(x=0,2))
      \arg        TIMER_TRI_OUT_SRC_O3CPRE: O3CPRE as trigger output (TIMERx(x=0,2))
    \param[out] none
    \retval     none
*/
void timer_master_output_trigger_source_select(uint32_t timer_periph, uint32_t outtrigger)
{
    uint32_t ctl;
    ctl = TIMER_CTL1(timer_periph);
    ctl &= (~(uint32_t)TIMER_CTL1_MMC);
    ctl |= (uint32_t)(outtrigger & TIMER_CTL1_MMC);
    TIMER_CTL1(timer_periph) = ctl;
}

/*!
    \brief      select TIMER slave mode (API_ID(0x0037U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  slavemode:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SLAVE_MODE_DISABLE: slave mode disable
      \arg        TIMER_QUAD_DECODER_MODE0: quadrature decoder mode 0 (TIMERx(x=0,2))
      \arg        TIMER_QUAD_DECODER_MODE1: quadrature decoder mode 1 (TIMERx(x=0,2))
      \arg        TIMER_QUAD_DECODER_MODE2: quadrature decoder mode 2 (TIMERx(x=0,2))
      \arg        TIMER_SLAVE_MODE_RESTART: restart mode (TIMERx(x=0,2,13,15,16))
      \arg        TIMER_SLAVE_MODE_PAUSE: pause mode (TIMERx(x=0,2,13,15,16))
      \arg        TIMER_SLAVE_MODE_EVENT: event mode (TIMERx(x=0,2,13,15,16))
      \arg        TIMER_SLAVE_MODE_EXTERNAL0: external clock mode 0 (TIMERx(x=0,2,13,15,16))
      \arg        TIMER_SLAVE_MODE_RESTART_EVENT: restart + event mode (TIMERx(x=0,2,13,15,16))
    \param[out] none
    \retval     none
*/

void timer_slave_mode_select(uint32_t timer_periph, uint32_t slavemode)
{
    uint32_t TIMERxCFG_temp0 = 0U,TIMERxCFG_temp1 = 0U;
    volatile uint32_t *TIMERxCFG_addr0, *TIMERxCFG_addr1;
    uint32_t trigger_temp = 0U;
    uint8_t i = 0U;
    switch(timer_periph){
    case TIMER0:
        TIMERxCFG_addr0 = &SYSCFG_TIMER0CFG0;
        TIMERxCFG_addr1 = &SYSCFG_TIMER0CFG1;
        break;
    case TIMER2:
        TIMERxCFG_addr0 = &SYSCFG_TIMER2CFG0;
        TIMERxCFG_addr1 = &SYSCFG_TIMER2CFG1;
        break;
    default:
         break;
    }
    TIMERxCFG_temp0 = REG32(TIMERxCFG_addr0);
    TIMERxCFG_temp1 = REG32(TIMERxCFG_addr1);
    if((((TIMERxCFG_temp0 & 0xFFFFFFFFU) == 0U)&&(((TIMERxCFG_temp1 & 0xFFFFFFFFU)) == 0U))){
        if(slavemode == TIMER_SLAVE_MODE_DISABLE){
            REG32(TIMERxCFG_addr1) = BITS(0,2);
        }else{
            REG32(TIMERxCFG_addr0) = 0x7U << (slavemode * 4U);
        }
    }else if((TIMERxCFG_temp0 & 0xFFFFFFFFU) == 0U){
        if(slavemode != TIMER_SLAVE_MODE_DISABLE){
            trigger_temp = (TIMERxCFG_temp1 & BITS(0,2));
            TIMERxCFG_temp0 = trigger_temp << (slavemode * 4U);
            REG32(TIMERxCFG_addr0) = TIMERxCFG_temp0;
            REG32(TIMERxCFG_addr1) = 0U;
        }
    }else{
        for(i = 0U; i < 8U; i++){
            if((TIMERxCFG_temp0 & (BITS(0,2) << (i * 4U))) != 0U){
                trigger_temp = (TIMERxCFG_temp0 & (BITS(0,2) << (i * 4U))) >> (i * 4U);
                break;
            }
        }
        if(slavemode != TIMER_SLAVE_MODE_DISABLE){
            TIMERxCFG_temp0 = trigger_temp << (slavemode * 4U);
            REG32(TIMERxCFG_addr0) = TIMERxCFG_temp0;
            REG32(TIMERxCFG_addr1) = 0U;
        }else{
            TIMERxCFG_temp1 = trigger_temp;
            REG32(TIMERxCFG_addr0) = 0U;
            REG32(TIMERxCFG_addr1) = TIMERxCFG_temp1;
        }
    }
}

/*!
    \brief      configure TIMER master slave mode (API_ID(0x0038U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  masterslave:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_MASTER_SLAVE_MODE_ENABLE: master slave mode enable
      \arg        TIMER_MASTER_SLAVE_MODE_DISABLE: master slave mode disable
    \param[out] none
    \retval     none
*/
void timer_master_slave_mode_config(uint32_t timer_periph, uint32_t masterslave)
{
    if(TIMER_MASTER_SLAVE_MODE_ENABLE == masterslave) {
        TIMER_SMCFG(timer_periph) |= (uint32_t)TIMER_SMCFG_MSM;
    } else if(TIMER_MASTER_SLAVE_MODE_DISABLE == masterslave) {
        TIMER_SMCFG(timer_periph) &= ~(uint32_t)TIMER_SMCFG_MSM;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure TIMER external trigger input (API_ID(0x0039U))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  extprescaler:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_EXT_TRI_PSC_OFF: no divided
      \arg        TIMER_EXT_TRI_PSC_DIV2: divided by 2
      \arg        TIMER_EXT_TRI_PSC_DIV4: divided by 4
      \arg        TIMER_EXT_TRI_PSC_DIV8: divided by 8
    \param[in]  extpolarity:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_ETP_FALLING: active low or falling edge active
      \arg        TIMER_ETP_RISING: active high or rising edge active
    \param[in]  extfilter: a value between 0 and 15
    \param[out] none
    \retval     none
*/
void timer_external_trigger_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter)
{
    uint32_t ctl;
    ctl = TIMER_SMCFG(timer_periph);
    ctl &= (~(uint32_t)(TIMER_SMCFG_ETP | TIMER_SMCFG_ETPSC | TIMER_SMCFG_ETFC));
    ctl |= (uint32_t)((extprescaler & TIMER_EXT_TRI_PSC_DIV8) | (extpolarity & TIMER_ETP_FALLING));
    ctl |= (((uint32_t)extfilter & 0xFU) << 8U);
    TIMER_SMCFG(timer_periph) = ctl;
}

/*!
    \brief      configure TIMER quadrature decoder mode (API_ID(0x003AU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  decomode:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_QUAD_DECODER_MODE0: quadrature decoder mode 0 
      \arg        TIMER_QUAD_DECODER_MODE1: quadrature decoder mode 1
      \arg        TIMER_QUAD_DECODER_MODE2: quadrature decoder mode 2
    \param[in]  ic0polarity:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_IC_POLARITY_RISING: capture rising edge
      \arg        TIMER_IC_POLARITY_FALLING: capture falling edge
      \arg        TIMER_IC_POLARITY_BOTH_EDGE: active both edge
    \param[in]  ic1polarity:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_IC_POLARITY_RISING: capture rising edge
      \arg        TIMER_IC_POLARITY_FALLING: capture falling edge
      \arg        TIMER_IC_POLARITY_BOTH_EDGE: active both edge
    \param[out] none
    \retval     none
*/
void timer_quadrature_decoder_mode_config(uint32_t timer_periph, uint32_t decomode, uint16_t ic0polarity, uint16_t ic1polarity)
{
    uint32_t ctl;
    /* configure the quadrature decoder mode */
    timer_slave_mode_select(timer_periph, decomode);
    /* configure input capture selection */
    ctl = TIMER_CHCTL0(timer_periph);
    ctl &= (uint32_t)(((~(uint32_t)TIMER_CHCTL0_CH0MS)) & ((~(uint32_t)TIMER_CHCTL0_CH1MS)));
    ctl |= (uint32_t)(TIMER_IC_SELECTION_DIRECTTI | ((uint32_t)TIMER_IC_SELECTION_DIRECTTI << 8U));
    TIMER_CHCTL0(timer_periph) = ctl;
    /* configure channel input capture polarity */
    ctl = TIMER_CHCTL2(timer_periph);
    ctl &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
    ctl &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
    ctl |= (((uint32_t)ic0polarity & TIMER_IC_POLARITY_BOTH_EDGE) | (((uint32_t)ic1polarity &TIMER_IC_POLARITY_BOTH_EDGE) << 4U));
    TIMER_CHCTL2(timer_periph) = ctl;
}

/*!
    \brief      configure TIMER internal clock mode (API_ID(0x003BU))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[out] none
    \retval     none
*/
void timer_internal_clock_config(uint32_t timer_periph)
{
    timer_slave_mode_select(timer_periph, TIMER_SLAVE_MODE_DISABLE);
}

/*!
    \brief      configure TIMER the internal trigger as external clock input(API_ID(0x003CU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  intrigger: trigger selection
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SMCFG_TRGSEL_ITI0: internal trigger 0
      \arg        TIMER_SMCFG_TRGSEL_ITI1: internal trigger 1
      \arg        TIMER_SMCFG_TRGSEL_ITI2: internal trigger 2
      \arg        TIMER_SMCFG_TRGSEL_ITI3: internal trigger 3
    \param[out] none
    \retval     none
*/
void timer_internal_trigger_as_external_clock_config(uint32_t timer_periph, uint32_t intrigger)
{
    /* select TIMER input trigger source */
    timer_input_trigger_source_select(timer_periph, intrigger);
    /* select TIMER slave mode */
    timer_slave_mode_select(timer_periph, TIMER_SLAVE_MODE_EXTERNAL0);
}

/*!
    \brief      configure TIMER the external trigger as external clock input(API_ID(0x003DU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  extrigger: external trigger selection
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SMCFG_TRGSEL_CI0F_ED: TI0 edge detector
      \arg        TIMER_SMCFG_TRGSEL_CI0FE0: filtered TIMER input 0
      \arg        TIMER_SMCFG_TRGSEL_CI1FE1: filtered TIMER input 1
    \param[in]  extpolarity: external trigger polarity
                only one parameter can be selected which is shown as below:
      \arg        TIMER_IC_POLARITY_RISING: active high or rising edge active
      \arg        TIMER_IC_POLARITY_FALLING: active low or falling edge active
      \arg        TIMER_IC_POLARITY_BOTH_EDGE: active both edge
    \param[in]  extfilter: a value between 0 and 15
    \param[out] none
    \retval     none
*/
void timer_external_trigger_as_external_clock_config(uint32_t timer_periph, uint32_t extrigger, uint16_t extpolarity, uint32_t extfilter)
{
    uint32_t ctl;
    if(TIMER_SMCFG_TRGSEL_CI1FE1 == extrigger) {
        /* reset the CH1EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
        /* set the CH1NP bit */
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
        ctl |= (((uint32_t)extpolarity & TIMER_IC_POLARITY_BOTH_EDGE) << 4U);
        TIMER_CHCTL2(timer_periph) = ctl;
        /* set the CH1MS bit */
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
        ctl |= ((uint32_t)TIMER_IC_SELECTION_DIRECTTI << 8U);
        TIMER_CHCTL0(timer_periph) = ctl;
        /* set the CH1CAPFLT bit */
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
        ctl |= (((uint32_t)extfilter &0xFU) << 12U);
        TIMER_CHCTL0(timer_periph) = ctl;
        /* set the CH1EN bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH1EN;
    } else {
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
        /* set the CH0P and CH0NP bits */
        ctl = TIMER_CHCTL2(timer_periph);
        ctl &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
        ctl |= ((uint32_t)extpolarity & TIMER_IC_POLARITY_BOTH_EDGE);
        TIMER_CHCTL2(timer_periph) = ctl;
        /* set the CH0MS bit */
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
        ctl |= (uint32_t)TIMER_IC_SELECTION_DIRECTTI;
        TIMER_CHCTL0(timer_periph) = ctl;
        /* reset the CH0CAPFLT bit */
        ctl = TIMER_CHCTL0(timer_periph);
        ctl &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
        ctl |= (((uint32_t)extfilter &0xFU) << 4U);
        TIMER_CHCTL0(timer_periph) = ctl;
        /* set the CH0EN bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)TIMER_CHCTL2_CH0EN;
    }
    /* select TIMER slave mode */
    timer_slave_mode_select(timer_periph, TIMER_SLAVE_MODE_EXTERNAL0);
    /* select TIMER input trigger source */
    timer_input_trigger_source_select(timer_periph, extrigger);
}

/*!
    \brief      configure TIMER the external clock mode0 (API_ID(0x003EU))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  extprescaler:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_EXT_TRI_PSC_OFF: no divided
      \arg        TIMER_EXT_TRI_PSC_DIV2: divided by 2
      \arg        TIMER_EXT_TRI_PSC_DIV4: divided by 4
      \arg        TIMER_EXT_TRI_PSC_DIV8: divided by 8
    \param[in]  extpolarity:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_ETP_FALLING: active low or falling edge active
      \arg        TIMER_ETP_RISING: active high or rising edge active
    \param[in]  extfilter: a value between 0 and 15
    \param[out] none
    \retval     none
*/
void timer_external_clock_mode0_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter)
{
    /* configure TIMER external trigger input */
    timer_external_trigger_config(timer_periph, ((uint32_t)extprescaler & TIMER_EXT_TRI_PSC_DIV8), ((uint32_t)extpolarity & TIMER_ETP_FALLING), ((uint32_t)extfilter & 0xFU));
    /* select TIMER slave mode */
    timer_slave_mode_select(timer_periph, TIMER_SLAVE_MODE_EXTERNAL0);
    /* select TIMER input trigger source */
    timer_input_trigger_source_select(timer_periph, TIMER_SMCFG_TRGSEL_ETIFP);
}

/*!
    \brief      configure TIMER the external clock mode1 (API_ID(0x003FU))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[in]  extprescaler:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_EXT_TRI_PSC_OFF: no divided
      \arg        TIMER_EXT_TRI_PSC_DIV2: divided by 2
      \arg        TIMER_EXT_TRI_PSC_DIV4: divided by 4
      \arg        TIMER_EXT_TRI_PSC_DIV8: divided by 8
    \param[in]  extpolarity:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_ETP_FALLING: active low or falling edge active
      \arg        TIMER_ETP_RISING: active high or rising edge active
    \param[in]  extfilter: a value between 0 and 15
    \param[out] none
    \retval     none
*/
void timer_external_clock_mode1_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter)
{
    /* configure TIMER external trigger input */
    timer_external_trigger_config(timer_periph, ((uint32_t)extprescaler & TIMER_EXT_TRI_PSC_DIV8), ((uint32_t)extpolarity & TIMER_ETP_FALLING), (extfilter & 0xFU));
    TIMER_SMCFG(timer_periph) |= (uint32_t)TIMER_SMCFG_SMC1;
}

/*!
    \brief      disable TIMER the external clock mode1 (API_ID(0x0040U))
    \param[in]  timer_periph: TIMERx(x=0,2)
    \param[out] none
    \retval     none
*/
void timer_external_clock_mode1_disable(uint32_t timer_periph)
{
    TIMER_SMCFG(timer_periph) &= ~(uint32_t)TIMER_SMCFG_SMC1;
}


/*!
    \brief      configure TIMER input selection (API_ID(0x0041U))
    \param[in]  timer_periph: TIMERx(x=0,13,15,16)
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0
      \arg        TIMER_CH_1: TIMER channel1
    \param[in]  insel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INSEL_CHx: connect to CH0
      \arg        TIMER_INSEL_CMPx: connect to CMP1
    \param[out] none
    \retval     none
*/
void timer_input_selection_config(uint32_t timer_periph, uint16_t channel, uint16_t insel)
{
    if(TIMER_CH_0 == channel) {
        TIMER_INSEL(timer_periph) &= ~(uint32_t)TIMER_INSEL_CI0_SEL;
        TIMER_INSEL(timer_periph) |= (uint32_t) (insel & TIMER_INSEL_CI0_SEL);
    }else{
        TIMER_INSEL(timer_periph) &= ~(uint32_t)TIMER_INSEL_CI1_SEL;
        TIMER_INSEL(timer_periph) |= (uint32_t) ((uint32_t)(insel & TIMER_INSEL_CI1_SEL) << 8U);
    }
}
/*!
    \brief      configure TIMER write CHxVAL register selection (API_ID(0x0042U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  ccsel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CHVSEL_DISABLE: no effect
      \arg        TIMER_CHVSEL_ENABLE: when write the CHxVAL register, if the write value is same as the CHxVAL value, the write access is ignored
    \param[out] none
    \retval     none
*/
void timer_write_chxval_register_config(uint32_t timer_periph, uint16_t ccsel)
{
    if(TIMER_CHVSEL_ENABLE == ccsel) {
        TIMER_CFG(timer_periph) |= (uint32_t)TIMER_CFG_CHVSEL;
    } else if(TIMER_CHVSEL_DISABLE == ccsel) {
        TIMER_CFG(timer_periph) &= ~(uint32_t)TIMER_CFG_CHVSEL;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure TIMER output value selection (API_ID(0x0043U))
    \param[in]  timer_periph: TIMERx(x=0,2,13,15,16)
    \param[in]  outsel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OUTSEL_DISABLE: no effect
      \arg        TIMER_OUTSEL_ENABLE: if POEN and IOS is 0, the output disabled
    \param[out] none
    \retval     none
*/
void timer_output_value_selection_config(uint32_t timer_periph, uint16_t outsel)
{
    if(TIMER_OUTSEL_ENABLE == outsel) {
        TIMER_CFG(timer_periph) |= (uint32_t)TIMER_CFG_OUTSEL;
    } else if(TIMER_OUTSEL_DISABLE == outsel) {
        TIMER_CFG(timer_periph) &= ~(uint32_t)TIMER_CFG_OUTSEL;
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      configure the TIMER break source (API_ID(0x0044U))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  break_num: TIMER BREAKx
                only one parameter can be selected which is shown as below:
      \arg        TIMER_BREAK0: BREAK0 input signal, TIMERx(x=0,15,16)
      \arg        TIMER_BREAK1: BREAK1 input signal, TIMERx(x=0)
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void timer_break_external_source_config(uint32_t timer_periph, uint16_t break_num, ControlStatus newvalue)
{
    if(ENABLE == newvalue) {
        if(TIMER_BREAK0 == break_num) {
            TIMER_AFCTL0(timer_periph) |= TIMER_AFCTL0_BRK0INEN;
        } else if(TIMER_BREAK1 == break_num) {
            TIMER_AFCTL1(timer_periph) |= TIMER_AFCTL1_BRK1INEN;
        } else {
                /* illegal parameters */
        }
    } else {
        if(TIMER_BREAK0 == break_num) {
            TIMER_AFCTL0(timer_periph) &= (~TIMER_AFCTL0_BRK0INEN);
        } else if(TIMER_BREAK1 == break_num) {
            TIMER_AFCTL1(timer_periph) &= (~TIMER_AFCTL1_BRK1INEN);
        } else {
                /* illegal parameters */
        }
    }
}

/*!
    \brief      configure TIMER break polarity (API_ID(0x0045U))
    \param[in]  timer_periph: TIMERx(x=0,15,16)
    \param[in]  break_num: TIMER BREAKx
                only one parameter can be selected which is shown as below:
      \arg        TIMER_BREAK0: BREAK0 input signal, TIMERx(x=0,15,16)
      \arg        TIMER_BREAK1: BREAK1 input signal, TIMERx(x=0)
    \param[in]  bkinpolarity: break polarity
                only one parameter can be selected which is shown as below:
                  TIMER_BRKIN_POLARITY_LOW: input signal will not be inverted
                  TIMER_BRKIN_POLARITY_HIGH: input signal will be inverted
    \param[out] none
    \retval     none
*/
void timer_break_external_polarity_config(uint32_t timer_periph, uint16_t break_num, uint16_t bkinpolarity)
{
    if(TIMER_BREAK0 == break_num) {
        TIMER_AFCTL0(timer_periph) &= (~(uint32_t)TIMER_AFCTL0_BRK0INP);
        TIMER_AFCTL0(timer_periph) |= (((uint32_t)bkinpolarity &TIMER_BRKIN_POLARITY_HIGH) << 9U);
    }else{
        TIMER_AFCTL1(timer_periph) &= (~(uint32_t)TIMER_AFCTL1_BRK1INP);
        TIMER_AFCTL1(timer_periph) |= (((uint32_t)bkinpolarity &TIMER_BRKIN_POLARITY_HIGH) << 9U);
    }
}


/*!
    \brief      configure TIMER eti source (API_ID(0x0046U))
    \param[in]  timer_periph: TIMERx(x=0)
    \param[in]  eti_source: eti source select
                only one parameter can be selected which is shown as below:
      \arg        TIMER_ETI_LEGACY_MODE : input signal, TIMERx(x=0)
      \arg        TIMER_ETI_ADC_WD0_OUT input signal, TIMERx(x=0)
      \arg        TIMER_ETI_ADC_WD1_OUT input signal, TIMERx(x=0)
      \arg        TIMER_ETI_ADC_WD2_OUT input signal, TIMERx(x=0)
    \param[out] none
    \retval     none
*/
void timer_eti_source_selection_config(uint32_t timer_periph, uint32_t eti_source)
{
   TIMER_AFCTL0(timer_periph) &= (~(uint32_t)TIMER_AFCTL0_ETISEL);
   TIMER_AFCTL0(timer_periph) |= ((uint32_t)eti_source &TIMER_AFCTL0_ETISEL);
}

/*!
    \brief      get TIMER flags (API_ID(0x0047U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  flag: the timer interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        TIMER_FLAG_UP: update flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH0: channel 0 flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH1: channel 1 flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH2: channel 2 flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_CH3: channel 3 flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_CH4: channel 3 flag,TIMERx(x=0)
      \arg        TIMER_FLAG_CMT: channel commutation flag, TIMERx(x=0)
      \arg        TIMER_FLAG_TRG: trigger flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_BRK0: BREAK0 flag, TIMERx(x=0,15,16)
      \arg        TIMER_FLAG_BRK1: BREAK1 flag, TIMERx(x=0)
      \arg        TIMER_FLAG_CH0O: channel 0 overcapture flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH1O: channel 1 overcapture flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH2O: channel 2 overcapture flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_CH3O: channel 3 overcapture flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_SYSB: channel 3 system source break,TIMERx(x=0)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus timer_flag_get(uint32_t timer_periph, uint32_t flag)
{
    FlagStatus timer_flag = RESET;
    if((uint32_t)RESET != (TIMER_INTF(timer_periph) & flag)) {
        timer_flag = SET;
    } else {
        timer_flag = RESET;
    }
    return timer_flag;
}

/*!
    \brief      clear TIMER flags (API_ID(0x0048U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  flag: the timer interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        TIMER_FLAG_UP: update flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH0: channel 0 flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH1: channel 1 flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH2: channel 2 flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_CH3: channel 3 flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_TRG: trigger flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_BRK0: BREAK0 flag, TIMERx(x=0,15,16)
      \arg        TIMER_FLAG_BRK1: BREAK1 flag, TIMERx(x=0)
      \arg        TIMER_FLAG_CH0O: channel 0 overcapture flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH1O: channel 1 overcapture flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_FLAG_CH2O: channel 2 overcapture flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_CH3O: channel 3 overcapture flag,TIMERx(x=0,2)
      \arg        TIMER_FLAG_SYSB: channel 3 system source break,TIMERx(x=0)
    \param[out] none
    \retval     none
*/
void timer_flag_clear(uint32_t timer_periph, uint32_t flag)
{
    TIMER_INTF(timer_periph) = (~(uint32_t)flag);
}

/*!
    \brief      enable the TIMER interrupt (API_ID(0x0049U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  interrupt: timer interrupt enable source
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_UP: update interrupt enable, TIMERx(x=0,2,3,6,8,11)
      \arg        TIMER_INT_CH0: channel 0 interrupt enable, TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_CH1: channel 1 interrupt enable, TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_CH2: channel 2 interrupt enable, TIMERx(x=0,2)
      \arg        TIMER_INT_CH3: channel 3 interrupt enable, TIMERx(x=0,2)
      \arg        TIMER_INT_CMT: commutation interrupt enable, TIMERx(x=0,15,16)
      \arg        TIMER_INT_TRG: trigger interrupt enable, TIMERx(x=0,2)
      \arg        TIMER_INT_BRK: break interrupt enable, TIMERx(x=0,15,16)
    \param[out] none
    \retval     none
*/
void timer_interrupt_enable(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_DMAINTEN(timer_periph) |= (uint32_t) (interrupt & 0xFFU);
}

/*!
    \brief      disable the TIMER interrupt (API_ID(0x0050U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  interrupt: timer interrupt source disable
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_UP: update interrupt enable, TIMERx(x=0,2,3,6,8,11)
      \arg        TIMER_INT_CH0: channel 0 interrupt enable, TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_CH1: channel 1 interrupt enable, TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_CH2: channel 2 interrupt enable, TIMERx(x=0,2)
      \arg        TIMER_INT_CH3: channel 3 interrupt enable, TIMERx(x=0,2)
      \arg        TIMER_INT_CMT: commutation interrupt enable, TIMERx(x=0,15,16)
      \arg        TIMER_INT_TRG: trigger interrupt enable, TIMERx(x=0,2)
      \arg        TIMER_INT_BRK: break interrupt enable, TIMERx(x=0,15,16)
    \param[out] none
    \retval     none
*/
void timer_interrupt_disable(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_DMAINTEN(timer_periph) &= (~(uint32_t)(interrupt & 0xFFU));
}

/*!
    \brief      get timer interrupt flag (API_ID(0x0051U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  int_flag: the timer interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_FLAG_UP: update interrupt flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_FLAG_CH0: channel 0 interrupt flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_FLAG_CH1: channel 1 interrupt flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_FLAG_CH2: channel 2 interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_CH3: channel 3 interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_CMT: trigger interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_TRG: trigger interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_BRK0: trigger interrupt flag,TIMERx(x=0,15,16)
      \arg        TIMER_INT_FLAG_BRK1: trigger interrupt flag,TIMERx(x=0)
      \arg        TIMER_INT_FLAG_SYSB: trigger interrupt flag,TIMERx(x=0,15,16)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus timer_interrupt_flag_get(uint32_t timer_periph, uint32_t int_flag)
{
    FlagStatus timer_interrupt_flag = RESET;
        uint32_t val;
    if( int_flag != TIMER_INT_FLAG_BRK1 ) {
      val = (TIMER_DMAINTEN(timer_periph) & int_flag);
    }
    else {
      val = (TIMER_DMAINTEN(timer_periph) & TIMER_DMAINTEN_BRKIE );
    }
    if(((uint32_t)RESET != (TIMER_INTF(timer_periph) & int_flag)) && ((uint32_t)RESET != val)) {
        timer_interrupt_flag = SET;
    } else {
        timer_interrupt_flag = RESET;
    }
    return timer_interrupt_flag;
}

/*!
    \brief      get timer interrupt flag (API_ID(0x0052U))
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  int_flag: the timer interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        TIMER_INT_FLAG_UP: update interrupt flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_FLAG_CH0: channel 0 interrupt flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_FLAG_CH1: channel 1 interrupt flag,TIMERx(x=0,2,13,15,16)
      \arg        TIMER_INT_FLAG_CH2: channel 2 interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_CH3: channel 3 interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_CMT: trigger interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_TRG: trigger interrupt flag,TIMERx(x=0,2)
      \arg        TIMER_INT_FLAG_BRK0: trigger interrupt flag,TIMERx(x=0,15,16)
      \arg        TIMER_INT_FLAG_BRK1: trigger interrupt flag,TIMERx(x=0)
      \arg        TIMER_INT_FLAG_SYSB: trigger interrupt flag,TIMERx(x=0,15,16)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
void timer_interrupt_flag_clear(uint32_t timer_periph, uint32_t int_flag)
{
    TIMER_INTF(timer_periph) = (~(uint32_t)int_flag);
}
