/*!
    \file    gd32c2x1_dma.c
    \brief   DMA driver

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

#include "gd32c2x1_dma.h"

#define RM_CHXCFG_NBR_OFFSET       ((uint32_t)0x00000013U)               /*!< bit offset of NBR in DMAMUX_RM_CHXCFG */
#define RM_CHXCFG_SYNCID_OFFSET    ((uint32_t)0x00000018U)               /*!< bit offset of SYNCID in RM_CHXCFG_SYNCID */

/* DMA bits mask */
#define DMA_CHXCTL_INT_MASK        ((uint32_t)0x0000001EU)               /*!< DMA_CHXCTL interrupt mask */
/* DMAMUX bits mask */
#define RM_CHXCFG_SYNCID_MASK      ((uint32_t)0x0000001FU)               /*!< RM_CHXCFG_SYNCID mask */

/*!
    \brief      deinitialize DMA a channel registers (API ID: 0x0001U)
    \param[in]  channelx: specify which DMA channel is deinitialized
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_deinit(dma_channel_enum channelx)
{
    /* disable DMA a channel */
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_CHEN;
    /* reset DMA channel registers */
    DMA_CHCTL(channelx) = DMA_CHCTL_RESET_VALUE;
    DMA_CHCNT(channelx) = DMA_CHCNT_RESET_VALUE;
    DMA_CHPADDR(channelx) = DMA_CHPADDR_RESET_VALUE;
    DMA_CHMADDR(channelx) = DMA_CHMADDR_RESET_VALUE;
    DMA_INTC |= DMA_FLAG_ADD(DMA_CHINTF_RESET_VALUE, channelx);
}

/*!
    \brief      initialize the parameters of DMA struct with the default values (API ID: 0x0002U)
    \param[in]  none
    \param[out] init_struct: the initialization data needed to initialize DMA channel
    \retval     none
*/
void dma_struct_para_init(dma_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(DMA_DMAMUX_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* set the DMA struct with the default values */
        init_struct->periph_addr  = 0U;
        init_struct->periph_width = 0U;
        init_struct->periph_inc   = (uint8_t)DMA_PERIPH_INCREASE_DISABLE;
        init_struct->memory_addr  = 0U;
        init_struct->memory_width = 0U;
        init_struct->memory_inc   = (uint8_t)DMA_MEMORY_INCREASE_DISABLE;
        init_struct->number       = 0U;
        init_struct->direction    = (uint8_t)DMA_PERIPHERAL_TO_MEMORY;
        init_struct->priority     = (uint32_t)DMA_PRIORITY_LOW;
        init_struct->request      = DMA_REQUEST_M2M;
    }
}

/*!
    \brief      initialize DMA channel (API ID: 0x0003U)
    \param[in]  channelx: specify which DMA channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  init_struct: the data needed to initialize DMA channel
                  periph_addr: peripheral base address
                  periph_width: DMA_PERIPHERAL_WIDTH_8BIT, DMA_PERIPHERAL_WIDTH_16BIT, DMA_PERIPHERAL_WIDTH_32BIT
                  periph_inc: DMA_PERIPH_INCREASE_ENABLE, DMA_PERIPH_INCREASE_DISABLE
                  memory_addr: memory base address
                  memory_width: DMA_MEMORY_WIDTH_8BIT, DMA_MEMORY_WIDTH_16BIT, DMA_MEMORY_WIDTH_32BIT
                  memory_inc: DMA_MEMORY_INCREASE_ENABLE, DMA_MEMORY_INCREASE_DISABLE
                  direction: DMA_PERIPHERAL_TO_MEMORY, DMA_MEMORY_TO_PERIPHERAL
                  number: the number of remaining data to be transferred by the DMA
                  priority: DMA_PRIORITY_LOW, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_ULTRA_HIGH
                  request: DMA_REQUEST_M2M
                           DMA_REQUEST_GENERATOR0
                           DMA_REQUEST_GENERATOR1
                           DMA_REQUEST_GENERATOR2
                           DMA_REQUEST_GENERATOR3
                           DMA_REQUEST_ADC
                           DMA_REQUEST_I2C0_RX
                           DMA_REQUEST_I2C0_TX
                           DMA_REQUEST_I2C1_RX 
                           DMA_REQUEST_I2C1_TX 
                           DMA_REQUEST_SPI0_RX
                           DMA_REQUEST_SPI0_TX
                           DMA_REQUEST_SPI1_RX
                           DMA_REQUEST_SPI1_TX
                           DMA_REQUEST_TIMER0_CH0
                           DMA_REQUEST_TIMER0_CH1
                           DMA_REQUEST_TIMER0_CH2
                           DMA_REQUEST_TIMER0_CH3
                           DMA_REQUEST_TIMER0_TRIG
                           DMA_REQUEST_TIMER0_UP
                           DMA_REQUEST_TIMER0_COM
                           DMA_REQUEST_TIMER2_CH0
                           DMA_REQUEST_TIMER2_CH1
                           DMA_REQUEST_TIMER2_CH2
                           DMA_REQUEST_TIMER2_CH3
                           DMA_REQUEST_TIMER2_TRIG
                           DMA_REQUEST_TIMER2_UP
                           DMA_REQUEST_TIMER15_CH0
                           DMA_REQUEST_TIMER15_UP
                           DMA_REQUEST_TIMER16_CH0
                           DMA_REQUEST_TIMER16_UP
                           DMA_REQUEST_USART0_RX
                           DMA_REQUEST_USART0_TX
                           DMA_REQUEST_USART1_RX
                           DMA_REQUEST_USART1_TX
    \param[out] none
    \retval     none
*/
void dma_init(dma_channel_enum channelx, dma_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(DMA_DMAMUX_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint32_t ctl;
        dma_channel_disable(channelx);

        /* configure peripheral base address */
        DMA_CHPADDR(channelx) = init_struct->periph_addr;

        /* configure memory base address */
        DMA_CHMADDR(channelx) = init_struct->memory_addr;

        /* configure the number of remaining data to be transferred */
        DMA_CHCNT(channelx) = (init_struct->number & DMA_CHANNEL_CNT_MASK);

        /* configure peripheral transfer width, memory transfer width, channel priority */
        ctl = DMA_CHCTL(channelx);
        ctl &= ~(DMA_CHXCTL_PWIDTH | DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PRIO);
        ctl |= ((init_struct->memory_width & DMA_CHXCTL_MWIDTH) | (init_struct->periph_width & DMA_CHXCTL_PWIDTH ) | (init_struct->priority & DMA_CHXCTL_PRIO));
        DMA_CHCTL(channelx) = ctl;

        /* configure peripheral increasing mode */
        if(DMA_PERIPH_INCREASE_ENABLE == init_struct->periph_inc) {
            DMA_CHCTL(channelx) |= DMA_CHXCTL_PNAGA;
        } else {
            DMA_CHCTL(channelx) &= ~DMA_CHXCTL_PNAGA;
        }

        /* configure memory increasing mode */
        if(DMA_MEMORY_INCREASE_ENABLE == init_struct->memory_inc) {
            DMA_CHCTL(channelx) |= DMA_CHXCTL_MNAGA;
        } else {
            DMA_CHCTL(channelx) &= ~DMA_CHXCTL_MNAGA;
        }

        /* configure the direction of  data transfer */
        if(DMA_PERIPHERAL_TO_MEMORY == init_struct->direction) {
            DMA_CHCTL(channelx) &= ~DMA_CHXCTL_DIR;
        } else {
            DMA_CHCTL(channelx) |= DMA_CHXCTL_DIR;
        }

        DMAMUX_RM_CHXCFG(channelx) &= ~DMAMUX_RM_CHXCFG_MUXID;
        DMAMUX_RM_CHXCFG(channelx) |= (init_struct->request & DMAMUX_RM_CHXCFG_MUXID);
    }
}

/*!
    \brief      enable DMA circulation mode (API ID: 0x0004U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_circulation_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_CMEN;
}

/*!
    \brief      disable DMA circulation mode (API ID: 0x0005U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_circulation_disable(dma_channel_enum channelx)
{

    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_CMEN;
}

/*!
    \brief      enable memory to memory mode (API ID: 0x0006U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_memory_to_memory_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_M2M;
}

/*!
    \brief      disable memory to memory mode (API ID: 0x0007U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_memory_to_memory_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_M2M;
}

/*!
    \brief      enable DMA channel (API ID: 0x0008U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_channel_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_CHEN;
}

/*!
    \brief      disable DMA channel (API ID: 0x0009U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_channel_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_CHEN;
}

/*!
    \brief      set DMA peripheral base address (API ID: 0x000AU)
    \param[in]  channelx: specify which DMA channel to set peripheral base address
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  address: peripheral base address
    \param[out] none
    \retval     none
*/
void dma_periph_address_config(dma_channel_enum channelx, uint32_t address)
{
    DMA_CHPADDR(channelx) = address;
}

/*!
    \brief      set DMA memory base address (API ID: 0x000BU)
    \param[in]  channelx: specify which DMA channel to set memory base address
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  address: memory base address
    \param[out] none
    \retval     none
*/
void dma_memory_address_config(dma_channel_enum channelx, uint32_t address)
{
    DMA_CHMADDR(channelx) = address;
}

/*!
    \brief      set the number of remaining data to be transferred by the DMA (API ID: 0x000CU)
    \param[in]  channelx: specify which DMA channel to set number
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  number: the number of remaining data to be transferred by the DMA
      \arg        0x0 - 0xFFFF
    \param[out] none
    \retval     none
*/
void dma_transfer_number_config(dma_channel_enum channelx, uint32_t number)
{
     DMA_CHCNT(channelx) = (number & DMA_CHANNEL_CNT_MASK);
}

/*!
    \brief      get the number of remaining data to be transferred by the DMA (API ID: 0x000DU)
    \param[in]  channelx: specify which DMA channel to set number
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     the number of remaining data to be transferred by the DMA, 0x0 - 0xFFFF
*/
uint32_t dma_transfer_number_get(dma_channel_enum channelx)
{
    uint32_t reval ;
    reval = (uint32_t)DMA_CHCNT(channelx);
    return reval;
}

/*!
    \brief      configure priority level of DMA channel (API ID: 0x000EU)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  priority: priority level of this channel
                only one parameter can be selected which is shown as below:
      \arg        DMA_PRIORITY_LOW: low priority
      \arg        DMA_PRIORITY_MEDIUM: medium priority
      \arg        DMA_PRIORITY_HIGH: high priority
      \arg        DMA_PRIORITY_ULTRA_HIGH: ultra high priority
    \param[out] none
    \retval     none
*/
void dma_priority_config(dma_channel_enum channelx, uint32_t priority)
{
    uint32_t ctl;

    /* acquire DMA_CHxCTL register */
    ctl = DMA_CHCTL(channelx);
    /* assign regiser */
    ctl &= ~DMA_CHXCTL_PRIO;
    ctl |= (priority & DMA_CHXCTL_PRIO);
    DMA_CHCTL(channelx) = ctl;
}

/*!
    \brief      configure transfer data width of memory (API ID: 0x000FU)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  mwidth: transfer data width of memory
                only one parameter can be selected which is shown as below:
      \arg        DMA_MEMORY_WIDTH_8BIT: transfer data width of memory is 8-bit
      \arg        DMA_MEMORY_WIDTH_16BIT: transfer data width of memory is 16-bit
      \arg        DMA_MEMORY_WIDTH_32BIT: transfer data width of memory is 32-bit
    \param[out] none
    \retval     none
*/
void dma_memory_width_config(dma_channel_enum channelx, uint32_t mwidth)
{
    uint32_t ctl;
    /* acquire DMA_CHxCTL register */
    ctl = DMA_CHCTL(channelx);
    /* assign regiser */
    ctl &= ~DMA_CHXCTL_MWIDTH;
    ctl |= (mwidth & DMA_CHXCTL_MWIDTH);
    DMA_CHCTL(channelx) = ctl;
}

/*!
    \brief      configure transfer data width of peripheral (API ID: 0x0010U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  pwidth: transfer data width of peripheral
                only one parameter can be selected which is shown as below:
      \arg        DMA_PERIPHERAL_WIDTH_8BIT: transfer data width of peripheral is 8-bit
      \arg        DMA_PERIPHERAL_WIDTH_16BIT: transfer data width of peripheral is 16-bit
      \arg        DMA_PERIPHERAL_WIDTH_32BIT: transfer data width of peripheral is 32-bit
    \param[out] none
    \retval     none
*/
void dma_periph_width_config(dma_channel_enum channelx, uint32_t pwidth)
{
    uint32_t ctl;
    /* acquire DMA_CHxCTL register */
    ctl = DMA_CHCTL(channelx);
    /* assign regiser */
    ctl &= ~DMA_CHXCTL_PWIDTH;
    ctl |= (pwidth & DMA_CHXCTL_PWIDTH);
    DMA_CHCTL(channelx) = ctl;
}

/*!
    \brief      enable next address increasement algorithm of memory (API ID: 0x0011U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_memory_increase_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_MNAGA;
}

/*!
    \brief      disable next address increasement algorithm of memory (API ID: 0x0012U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_memory_increase_disable(dma_channel_enum channelx)
{

    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_MNAGA;
}

/*!
    \brief      enable next address increasement algorithm of peripheral (API ID: 0x0013U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_periph_increase_enable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) |= DMA_CHXCTL_PNAGA;
}

/*!
    \brief      disable next address increasement algorithm of peripheral (API ID: 0x0014U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dma_periph_increase_disable(dma_channel_enum channelx)
{
    DMA_CHCTL(channelx) &= ~DMA_CHXCTL_PNAGA;
}

/*!
    \brief      configure the direction of data transfer on the channel (API ID: 0x0015U)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  direction: specify the direction of  data transfer
                only one parameter can be selected which is shown as below:
      \arg        DMA_PERIPHERAL_TO_MEMORY: read from peripheral and write to memory
      \arg        DMA_MEMORY_TO_PERIPHERAL: read from memory and write to peripheral
    \param[out] none
    \retval     none
*/
void dma_transfer_direction_config(dma_channel_enum channelx, uint32_t direction)
{
    if(DMA_PERIPHERAL_TO_MEMORY == direction) {
        DMA_CHCTL(channelx) &= ~DMA_CHXCTL_DIR;
    } else {
        DMA_CHCTL(channelx) |= DMA_CHXCTL_DIR;
    }

}

/*!
    \brief      check DMA flag is set or not (API ID: 0x0016U)
    \param[in]  channelx: specify which DMA channel to get flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_FLAG_G: global interrupt flag of channel
      \arg        DMA_FLAG_FTF: full transfer finish flag of channel
      \arg        DMA_FLAG_HTF: half transfer finish flag of channel
      \arg        DMA_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus dma_flag_get(dma_channel_enum channelx, uint32_t flag)
{
    FlagStatus reval = RESET;
    if((uint32_t)RESET != (DMA_INTF & DMA_FLAG_ADD(flag, channelx))) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear a DMA channel flag (API ID: 0x0017U)
    \param[in]  channelx: specify which DMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_FLAG_G: global interrupt flag of channel
      \arg        DMA_FLAG_FTF: full transfer finish flag of channel
      \arg        DMA_FLAG_HTF: half transfer finish flag of channel
      \arg        DMA_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     none
*/
void dma_flag_clear(dma_channel_enum channelx, uint32_t flag)
{
    DMA_INTC |= DMA_FLAG_ADD(flag, channelx);
}

/*!
    \brief      check DMA flag and interrupt enable bit is set or not (API ID: 0x0018U)
    \param[in]  channelx: specify which DMA channel to get flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_FLAG_FTF: transfer finish flag of channel
      \arg        DMA_INT_FLAG_HTF: half transfer finish flag of channel
      \arg        DMA_INT_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus dma_interrupt_flag_get(dma_channel_enum channelx, uint32_t int_flag)
{
    FlagStatus reval = RESET;
    uint32_t interrupt_enable = 0U, interrupt_flag = 0U;
    switch(int_flag) {
        case DMA_INT_FLAG_FTF:
            interrupt_flag = DMA_INTF & DMA_FLAG_ADD(int_flag, channelx);
            interrupt_enable = DMA_CHCTL(channelx) & DMA_CHXCTL_FTFIE;
            break;
        case DMA_INT_FLAG_HTF:
            interrupt_flag = DMA_INTF & DMA_FLAG_ADD(int_flag, channelx);
            interrupt_enable = DMA_CHCTL(channelx) & DMA_CHXCTL_HTFIE;
            break;
        case DMA_INT_FLAG_ERR:
            interrupt_flag = DMA_INTF & DMA_FLAG_ADD(int_flag, channelx);
            interrupt_enable = DMA_CHCTL(channelx) & DMA_CHXCTL_ERRIE;
            break;
        default:
            break;
    }

    if((0U != interrupt_flag) && (0U != interrupt_enable)) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear a DMA channel interrupt flag (API ID: 0x0019U)
    \param[in]  channelx: specify which DMA channel to clear flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  flag: specify get which flag
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_FLAG_G: global interrupt flag of channel
      \arg        DMA_INT_FLAG_FTF: transfer finish flag of channel
      \arg        DMA_INT_FLAG_HTF: half transfer finish flag of channel
      \arg        DMA_INT_FLAG_ERR: error flag of channel
    \param[out] none
    \retval     none
*/
void dma_interrupt_flag_clear(dma_channel_enum channelx, uint32_t int_flag)
{
    DMA_INTC |= DMA_FLAG_ADD(int_flag, channelx);
}

/*!
    \brief      enable DMA interrupt (API ID: 0x001AU)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  source: specify which interrupt to enable
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_ERR: channel error interrupt
      \arg        DMA_INT_HTF: channel half transfer finish interrupt
      \arg        DMA_INT_FTF: channel full transfer finish interrupt
    \param[out] none
    \retval     none
*/
void dma_interrupt_enable(dma_channel_enum channelx, uint32_t source)
{
    DMA_CHCTL(channelx) |= source & DMA_CHXCTL_INT_MASK;
}

/*!
    \brief      disable DMA interrupt (API ID: 0x001BU)
    \param[in]  channelx: specify which DMA channel to set
                only one parameter can be selected which is shown as below:
      \arg        DMA_CHx(x=0..2)
    \param[in]  source: specify which interrupt to disable
                only one parameter can be selected which is shown as below:
      \arg        DMA_INT_ERR: channel error interrupt
      \arg        DMA_INT_HTF: channel half transfer finish interrupt
      \arg        DMA_INT_FTF: channel full transfer finish interrupt
    \param[out] none
    \retval     none
*/
void dma_interrupt_disable(dma_channel_enum channelx, uint32_t source)
{
    DMA_CHCTL(channelx) &= ~source & DMA_CHXCTL_INT_MASK;
}

/*!
    \brief      initialize the parameters of DMAMUX synchronization mode structure with the default values (API ID: 0x001CU)
    \param[in]  none
    \param[out] init_struct: the initialization data needed to initialize DMAMUX request multiplexer channel synchronization mode
    \retval     none
*/
void dmamux_sync_struct_para_init(dmamux_sync_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(DMA_DMAMUX_MODULE_ID, API_ID(0x001CU), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* set the DMAMUX synchronization struct with the default values */
        init_struct->sync_id        = DMAMUX_SYNC_EVT0_OUT;
        init_struct->sync_polarity  = DMAMUX_SYNC_RISING;
        init_struct->request_number = 1U;
    }
}

/*!
    \brief      initialize DMAMUX request multiplexer channel synchronization mode (API ID: 0x001DU)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[in]  init_struct: the data needed to initialize DMAMUX request multiplexer channel
                  sync_id: DMAMUX_SYNC_EXTI0, DMAMUX_SYNC_EXTI1, DMAMUX_SYNC_EXTI2, DMAMUX_SYNC_EXTI3,
                           DMAMUX_SYNC_EXTI4, DMAMUX_SYNC_EXTI5, DMAMUX_SYNC_EXTI6, DMAMUX_SYNC_EXTI7,
                           DMAMUX_SYNC_EXTI8, DMAMUX_SYNC_EXTI9, DMAMUX_SYNC_EXTI10, DMAMUX_SYNC_EXTI11,
                           DMAMUX_SYNC_EXTI12, DMAMUX_SYNC_EXTI13, DMAMUX_SYNC_EXTI14, DMAMUX_SYNC_EXTI15,
                           DMAMUX_SYNC_EVT0_OUT, DMAMUX_SYNC_EVT1_OUT, DMAMUX_SYNC_EVT2_OUT, DMAMUX_SYNC_TIMER13_O
                  sync_polarity: DMAMUX_SYNC_NO_EVENT, DMAMUX_SYNC_RISING, DMAMUX_SYNC_FALLING, DMAMUX_SYNC_RISING_FALLING
                  request_number: the number of DMA request that will be authorized after a sync event, from 1 to 32
    \param[out] none
    \retval     none
*/
void dmamux_synchronization_init(dmamux_multiplexer_channel_enum channelx,
                                 dmamux_sync_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(DMA_DMAMUX_MODULE_ID, API_ID(0x001DU), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint32_t cfg;
        /* disable synchronization mode and event generation for DMA request forward number configuration */
        DMAMUX_RM_CHXCFG(channelx) &= ~(DMAMUX_RM_CHXCFG_SYNCEN | DMAMUX_RM_CHXCFG_EVGEN);

        /* configure synchronization input identification, synchronization input polarity, number of DMA requests to forward */
        cfg = DMAMUX_RM_CHXCFG(channelx);
        cfg &= ~(DMAMUX_RM_CHXCFG_SYNCID | DMAMUX_RM_CHXCFG_NBR | DMAMUX_RM_CHXCFG_SYNCP);
        cfg |= ((init_struct->sync_polarity & DMAMUX_RM_CHXCFG_SYNCP) | (init_struct->sync_id & DMAMUX_RM_CHXCFG_SYNCID) | RM_CHXCFG_NBR( init_struct->request_number - 1U));
        DMAMUX_RM_CHXCFG(channelx) = cfg;
    }
}

/*!
    \brief      enable synchronization mode (API ID: 0x001EU)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dmamux_synchronization_enable(dmamux_multiplexer_channel_enum channelx)
{
    DMAMUX_RM_CHXCFG(channelx) |= DMAMUX_RM_CHXCFG_SYNCEN;
}

/*!
    \brief      disable synchronization mode (API ID: 0x001FU)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dmamux_synchronization_disable(dmamux_multiplexer_channel_enum channelx)
{
    DMAMUX_RM_CHXCFG(channelx) &= (~DMAMUX_RM_CHXCFG_SYNCEN);
}
/*!
    \brief      enable event generation (API ID: 0x0020U)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dmamux_event_generation_enable(dmamux_multiplexer_channel_enum channelx)
{
    DMAMUX_RM_CHXCFG(channelx) |= DMAMUX_RM_CHXCFG_EVGEN;
}

/*!
    \brief      disable event generation (API ID: 0x0021U)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[out] none
    \retval     none
*/
void dmamux_event_generation_disable(dmamux_multiplexer_channel_enum channelx)
{
    DMAMUX_RM_CHXCFG(channelx) &= (~DMAMUX_RM_CHXCFG_EVGEN);
}

/*!
    \brief      initialize the parameters of DMAMUX request generator structure with the default values (API ID: 0x0022U)
    \param[in]  none
    \param[out] init_struct: the initialization data needed to initialize DMAMUX request generator channel
    \retval     none
*/
void dmamux_gen_struct_para_init(dmamux_gen_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(DMA_DMAMUX_MODULE_ID, API_ID(0x0022U), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* set the DMAMUX request generator structure with the default values */
        init_struct->trigger_id        = DMAMUX_TRIGGER_EXTI0;
        init_struct->trigger_polarity  = DMAMUX_GEN_RISING;
        init_struct->request_number = 1U;
    }
}

/*!
    \brief      initialize DMAMUX request generator channel (API ID: 0x0023U)
    \param[in]  channelx: specify which DMAMUX request generator channel is initialized
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GENCHx(x=0..3)
    \param[in]  init_struct: the data needed to initialize DMAMUX request generator channel
                  trigger_id: DMAMUX_TRIGGER_EXTI0, DMAMUX_TRIGGER_EXTI1, DMAMUX_TRIGGER_EXTI2, DMAMUX_TRIGGER_EXTI3,
                              DMAMUX_TRIGGER_EXTI4, DMAMUX_TRIGGER_EXTI5, DMAMUX_TRIGGER_EXTI6, DMAMUX_TRIGGER_EXTI7,
                              DMAMUX_TRIGGER_EXTI8, DMAMUX_TRIGGER_EXTI9, DMAMUX_TRIGGER_EXTI10, DMAMUX_TRIGGER_EXTI11,
                              DMAMUX_TRIGGER_EXTI12, DMAMUX_TRIGGER_EXTI13, DMAMUX_TRIGGER_EXTI14, DMAMUX_TRIGGER_EXTI15,
                              DMAMUX_TRIGGER_EVT0_OUT, DMAMUX_TRIGGER_EVT1_OUT, DMAMUX_TRIGGER_EVT2_OUT, DMAMUX_TRIGGER_TIMER13_O
                  trigger_polarity: DMAMUX_GEN_NO_EVENT, DMAMUX_GEN_RISING, DMAMUX_GEN_FALLING, DMAMUX_GEN_RISING_FALLING
                  request_number: the number of DMA request that will be generated after a signal event, from 1 to 32
    \param[out] none
    \retval     none
*/
void dmamux_request_generator_init(dmamux_generator_channel_enum channelx,
                                   dmamux_gen_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(DMA_DMAMUX_MODULE_ID, API_ID(0x0023U), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint32_t cfg;
        /* disable DMAMUX request generator channel for DMA request generation number configuration */
        DMAMUX_RG_CHXCFG(channelx) &= ~(DMAMUX_RG_CHXCFG_RGEN);

        /* configure trigger input identification, trigger polarity, number of DMA requests to be generated */
        cfg = DMAMUX_RG_CHXCFG(channelx);
        cfg &= ~(DMAMUX_RG_CHXCFG_TID | DMAMUX_RG_CHXCFG_NBRG | DMAMUX_RG_CHXCFG_RGTP);
        cfg |= (RG_CHXCFG_NBRG(init_struct->request_number - 1U) | (init_struct->trigger_id & DMAMUX_RG_CHXCFG_TID) |
        (init_struct->trigger_polarity & DMAMUX_RG_CHXCFG_RGTP));
        DMAMUX_RG_CHXCFG(channelx) = cfg;
    }
}

/*!
    \brief      enable DMAMUX request generator channel (API ID: 0x0024U)
    \param[in]  channelx: specify which DMAMUX request generator channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GENCHx(x=0..3)
    \param[out] none
    \retval     none
*/
void dmamux_request_generator_channel_enable(dmamux_generator_channel_enum channelx)
{
    DMAMUX_RG_CHXCFG(channelx) |= DMAMUX_RG_CHXCFG_RGEN;
}

/*!
    \brief      disable DMAMUX request generator channel (API ID: 0x0025U)
    \param[in]  channelx: specify which DMAMUX request generator channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GENCHx(x=0..3)
    \param[out] none
    \retval     none
*/
void dmamux_request_generator_channel_disable(dmamux_generator_channel_enum channelx)
{
    DMAMUX_RG_CHXCFG(channelx) &= (~DMAMUX_RG_CHXCFG_RGEN);
}

/*!
    \brief      configure synchronization input polarity (API ID: 0x0026U)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[in]  polarity: synchronization input polarity
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_SYNC_NO_EVENT: no event detection
      \arg        DMAMUX_SYNC_RISING: rising edge
      \arg        DMAMUX_SYNC_FALLING: falling edge
      \arg        DMAMUX_SYNC_RISING_FALLING: rising and falling edges
    \param[out] none
    \retval     none
*/
void dmamux_synchronization_polarity_config(dmamux_multiplexer_channel_enum channelx,
                                            uint32_t polarity)
{

    DMAMUX_RM_CHXCFG(channelx) &= ~DMAMUX_RM_CHXCFG_SYNCP;
    DMAMUX_RM_CHXCFG(channelx) |= (polarity & DMAMUX_RM_CHXCFG_SYNCP);
}

/*!
    \brief      configure number of DMA requests to forward (API ID: 0x0027U)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[in]  number: DMA requests number to forward
                only one parameter can be selected which is shown as below:
      \arg        1 - 32
    \param[out] none
    \retval     none
*/
void dmamux_request_forward_number_config(dmamux_multiplexer_channel_enum channelx, uint32_t number)
{
    DMAMUX_RM_CHXCFG(channelx) &= ~DMAMUX_RM_CHXCFG_NBR;
    DMAMUX_RM_CHXCFG(channelx) |= RM_CHXCFG_NBR(number - 1U) & DMAMUX_RM_CHXCFG_NBR;
}

/*!
    \brief      configure synchronization input identification (API ID: 0x0028U)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[in]  id: synchronization input identification
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_SYNC_EXTI0: synchronization input is EXTI0
      \arg        DMAMUX_SYNC_EXTI1: synchronization input is EXTI1
      \arg        DMAMUX_SYNC_EXTI2: synchronization input is EXTI2
      \arg        DMAMUX_SYNC_EXTI3: synchronization input is EXTI3
      \arg        DMAMUX_SYNC_EXTI4: synchronization input is EXTI4
      \arg        DMAMUX_SYNC_EXTI5: synchronization input is EXTI5
      \arg        DMAMUX_SYNC_EXTI6: synchronization input is EXTI6
      \arg        DMAMUX_SYNC_EXTI7: synchronization input is EXTI7
      \arg        DMAMUX_SYNC_EXTI8: synchronization input is EXTI8
      \arg        DMAMUX_SYNC_EXTI9: synchronization input is EXTI9
      \arg        DMAMUX_SYNC_EXTI10: synchronization input is EXTI10
      \arg        DMAMUX_SYNC_EXTI11: synchronization input is EXTI11
      \arg        DMAMUX_SYNC_EXTI12: synchronization input is EXTI12
      \arg        DMAMUX_SYNC_EXTI13: synchronization input is EXTI13
      \arg        DMAMUX_SYNC_EXTI14: synchronization input is EXTI14
      \arg        DMAMUX_SYNC_EXTI15: synchronization input is EXTI15
      \arg        DMAMUX_SYNC_EVT0_OUT: synchronization input is Evt0_out
      \arg        DMAMUX_SYNC_EVT1_OUT: synchronization input is Evt1_out
      \arg        DMAMUX_SYNC_EVT2_OUT: synchronization input is Evt2_out
      \arg        DMAMUX_TRIGGER_TIMER13_O: synchronization input is TIMER13 O

    \param[out] none
    \retval     none
*/
void dmamux_sync_id_config(dmamux_multiplexer_channel_enum channelx, uint32_t id)
{
    DMAMUX_RM_CHXCFG(channelx) &= ~DMAMUX_RM_CHXCFG_SYNCID;
    DMAMUX_RM_CHXCFG(channelx) |= (uint32_t)(id & RM_CHXCFG_SYNCID_MASK);
}

/*!
    \brief      configure multiplexer input identification (API ID: 0x0029U)
    \param[in]  channelx: specify which DMAMUX request multiplexer channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_MUXCHx(x=0..2)
    \param[in]  id: input DMA request identification
                only one parameter can be selected which is shown as below:
      \arg        DMA_REQUEST_M2M        :    memory to memory transfer
      \arg        DMA_REQUEST_GENERATOR0 :    DMAMUX request generator 0
      \arg        DMA_REQUEST_GENERATOR1 :    DMAMUX request generator 1
      \arg        DMA_REQUEST_GENERATOR2 :    DMAMUX request generator 2
      \arg        DMA_REQUEST_GENERATOR3 :    DMAMUX request generator 3
      \arg        DMA_REQUEST_ADC        :    DMAMUX ADC request
      \arg        DMA_REQUEST_I2C0_RX    :    DMAMUX I2C0 RX request
      \arg        DMA_REQUEST_I2C0_TX    :    DMAMUX I2C0 TX request
      \arg        DMA_REQUEST_I2C1_RX    :    DMAMUX I2C1 RX request
      \arg        DMA_REQUEST_I2C1_TX    :    DMAMUX I2C1 TX request
      \arg        DMA_REQUEST_SPI0_RX    :    DMAMUX SPI0 RX request
      \arg        DMA_REQUEST_SPI0_TX    :    DMAMUX SPI0 TX request
      \arg        DMA_REQUEST_SPI1_RX    :    DMAMUX SPI1 RX request
      \arg        DMA_REQUEST_SPI1_TX    :    DMAMUX SPI1 TX request
      \arg        DMA_REQUEST_TIMER0_CH0 :    DMAMUX TIMER0 CH0 request
      \arg        DMA_REQUEST_TIMER0_CH1 :    DMAMUX TIMER0 CH1 request
      \arg        DMA_REQUEST_TIMER0_CH2 :    DMAMUX TIMER0 CH2 request
      \arg        DMA_REQUEST_TIMER0_CH3 :    DMAMUX TIMER0 CH3 request
      \arg        DMA_REQUEST_TIMER0_TRIG:    DMAMUX TIMER0 TRIG request
      \arg        DMA_REQUEST_TIMER0_UP  :    DMAMUX TIMER0 UP request
      \arg        DMA_REQUEST_TIMER0_COM :    DMAMUX TIMER0 COM request
      \arg        DMA_REQUEST_TIMER2_CH0 :    DMAMUX TIMER2 CH0 request
      \arg        DMA_REQUEST_TIMER2_CH1 :    DMAMUX TIMER2 CH1 request
      \arg        DMA_REQUEST_TIMER2_CH2 :    DMAMUX TIMER2 CH2 request
      \arg        DMA_REQUEST_TIMER2_CH3 :    DMAMUX TIMER2 CH3 request
      \arg        DMA_REQUEST_TIMER2_TRIG:    DMAMUX TIMER2 TRIG request
      \arg        DMA_REQUEST_TIMER2_UP  :    DMAMUX TIMER2 UP request
      \arg        DMA_REQUEST_TIMER15_CH0:    DMAMUX TIMER15 CH0 request
      \arg        DMA_REQUEST_TIMER15_UP :    DMAMUX TIMER15 UP request
      \arg        DMA_REQUEST_TIMER16_CH0:    DMAMUX TIMER16 CH0 request
      \arg        DMA_REQUEST_TIMER16_UP :    DMAMUX TIMER16 UP request
      \arg        DMA_REQUEST_USART0_RX  :    DMAMUX USART0 RX request
      \arg        DMA_REQUEST_USART0_TX  :    DMAMUX USART0 TX request
      \arg        DMA_REQUEST_USART1_RX  :    DMAMUX USART1 RX request
      \arg        DMA_REQUEST_USART1_TX  :    DMAMUX USART1 TX request
    \param[out] none
    \retval     none
*/
void dmamux_request_id_config(dmamux_multiplexer_channel_enum channelx, uint32_t id)
{
    DMAMUX_RM_CHXCFG(channelx) &= ~DMAMUX_RM_CHXCFG_MUXID;
        DMAMUX_RM_CHXCFG(channelx) |= (id & DMAMUX_RM_CHXCFG_MUXID);
}

/*!
    \brief      configure trigger input polarity (API ID: 0x002AU)
    \param[in]  channelx: specify which DMAMUX request generator channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GENCHx(x=0..3)
    \param[in]  polarity: trigger input polarity
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GEN_NO_EVENT: no event detection
      \arg        DMAMUX_GEN_RISING: rising edge
      \arg        DMAMUX_GEN_FALLING: falling edge
      \arg        DMAMUX_GEN_RISING_FALLING: rising and falling edges
    \param[out] none
    \retval     none
*/
void dmamux_trigger_polarity_config(dmamux_generator_channel_enum channelx, uint32_t polarity)
{
    DMAMUX_RG_CHXCFG(channelx) &= ~DMAMUX_RG_CHXCFG_RGTP;
        DMAMUX_RG_CHXCFG(channelx) |= (polarity & DMAMUX_RG_CHXCFG_RGTP);
}

/*!
    \brief      configure number of DMA requests to be generated (API ID: 0x002BU)
    \param[in]  channelx: specify which DMAMUX request generator channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GENCHx(x=0..3)
    \param[in]  number: DMA requests number to be generated
                only one parameter can be selected which is shown as below:
      \arg        1 - 32
    \param[out] none
    \retval     none
*/
void dmamux_request_generate_number_config(dmamux_generator_channel_enum channelx, uint32_t number)
{
    DMAMUX_RG_CHXCFG(channelx) &= ~DMAMUX_RG_CHXCFG_NBRG;
        DMAMUX_RG_CHXCFG(channelx) |= RG_CHXCFG_NBRG((number - 1U) & DMAMUX_RG_CHXCFG_NBRG);
}

/*!
    \brief      configure trigger input identification (API ID: 0x002CU)
    \param[in]  channelx: specify which DMAMUX request generator channel is configured
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_GENCHx(x=0..3)
    \param[in]  id: trigger input identification
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_TRIGGER_EXTI0: trigger input is EXTI0
      \arg        DMAMUX_TRIGGER_EXTI1: trigger input is EXTI1
      \arg        DMAMUX_TRIGGER_EXTI2: trigger input is EXTI2
      \arg        DMAMUX_TRIGGER_EXTI3: trigger input is EXTI3
      \arg        DMAMUX_TRIGGER_EXTI4: trigger input is EXTI4
      \arg        DMAMUX_TRIGGER_EXTI5: trigger input is EXTI5
      \arg        DMAMUX_TRIGGER_EXTI6: trigger input is EXTI6
      \arg        DMAMUX_TRIGGER_EXTI7: trigger input is EXTI7
      \arg        DMAMUX_TRIGGER_EXTI8: trigger input is EXTI8
      \arg        DMAMUX_TRIGGER_EXTI9: trigger input is EXTI9
      \arg        DMAMUX_TRIGGER_EXTI10: trigger input is EXTI10
      \arg        DMAMUX_TRIGGER_EXTI11: trigger input is EXTI11
      \arg        DMAMUX_TRIGGER_EXTI12: trigger input is EXTI12
      \arg        DMAMUX_TRIGGER_EXTI13: trigger input is EXTI13
      \arg        DMAMUX_TRIGGER_EXTI14: trigger input is EXTI14
      \arg        DMAMUX_TRIGGER_EXTI15: trigger input is EXTI15
      \arg        DMAMUX_TRIGGER_EVT0_OUT: trigger input is Evt0_out
      \arg        DMAMUX_TRIGGER_EVT1_OUT: trigger input is Evt1_out
      \arg        DMAMUX_TRIGGER_EVT2_OUT: trigger input is Evt2_out
      \arg        DMAMUX_TRIGGER_TIMER13_O: trigger input is TIMER13 O
    \param[out] none
    \retval     none
*/
void dmamux_trigger_id_config(dmamux_generator_channel_enum channelx, uint32_t id)
{
    DMAMUX_RG_CHXCFG(channelx) &= ~DMAMUX_RG_CHXCFG_TID;
    DMAMUX_RG_CHXCFG(channelx) |= (id & DMAMUX_RG_CHXCFG_TID);
}

/*!
    \brief      get DMAMUX flag (API ID: 0x002CU)
    \param[in]  flag: flag type
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_FLAG_MUXCH0_SO: DMAMUX request multiplexer channel 0 synchronization overrun flag
      \arg        DMAMUX_FLAG_MUXCH1_SO: DMAMUX request multiplexer channel 1 synchronization overrun flag
      \arg        DMAMUX_FLAG_MUXCH2_SO: DMAMUX request multiplexer channel 2 synchronization overrun flag
      \arg        DMAMUX_FLAG_GENCH0_TO: DMAMUX request generator channel 0 trigger overrun flag
      \arg        DMAMUX_FLAG_GENCH1_TO: DMAMUX request generator channel 1 trigger overrun flag
      \arg        DMAMUX_FLAG_GENCH2_TO: DMAMUX request generator channel 2 trigger overrun flag
      \arg        DMAMUX_FLAG_GENCH3_TO: DMAMUX request generator channel 3 trigger overrun flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus dmamux_flag_get(dmamux_flag_enum flag)
{
    FlagStatus reval = RESET;
    if((uint32_t)RESET != (DMAMUX_REG_VAL(flag) & BIT(DMAMUX_BIT_POS(flag)))) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear DMAMUX flag (API ID: 0x002DU)
    \param[in]  flag: flag type
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_FLAG_MUXCH0_SO: DMAMUX request multiplexer channel 0 synchronization overrun flag
      \arg        DMAMUX_FLAG_MUXCH1_SO: DMAMUX request multiplexer channel 1 synchronization overrun flag
      \arg        DMAMUX_FLAG_MUXCH2_SO: DMAMUX request multiplexer channel 2 synchronization overrun flag
      \arg        DMAMUX_FLAG_GENCH0_TO: DMAMUX request generator channel 0 trigger overrun flag
      \arg        DMAMUX_FLAG_GENCH1_TO: DMAMUX request generator channel 1 trigger overrun flag
      \arg        DMAMUX_FLAG_GENCH2_TO: DMAMUX request generator channel 2 trigger overrun flag
      \arg        DMAMUX_FLAG_GENCH3_TO: DMAMUX request generator channel 3 trigger overrun flag
    \param[out] none
    \retval     none
*/
void dmamux_flag_clear(dmamux_flag_enum flag)
{
    DMAMUX_REG_VAL3(flag) = BIT(DMAMUX_BIT_POS(flag));
}

/*!
    \brief      get DMAMUX interrupt flag (API ID: 0x002EU)
    \param[in]  int_flag: flag type
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_INT_FLAG_MUXCH0_SO: DMAMUX request multiplexer channel 0 synchronization overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_MUXCH1_SO: DMAMUX request multiplexer channel 1 synchronization overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_MUXCH2_SO: DMAMUX request multiplexer channel 2 synchronization overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH0_TO: DMAMUX request generator channel 0 trigger overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH1_TO: DMAMUX request generator channel 1 trigger overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH2_TO: DMAMUX request generator channel 2 trigger overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH3_TO: DMAMUX request generator channel 3 trigger overrun interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus dmamux_interrupt_flag_get(dmamux_interrupt_flag_enum int_flag)
{
    FlagStatus reval = RESET;
    uint32_t intenable, flagstatus ;
    /* get the interrupt enable bit status */
    intenable = (DMAMUX_REG_VAL2(int_flag) & BIT(DMAMUX_BIT_POS2(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (DMAMUX_REG_VAL(int_flag) & BIT(DMAMUX_BIT_POS(int_flag)));

    if((0U != flagstatus) && (0U != intenable)) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear DMAMUX interrupt flag (API ID: 0x002FU)
    \param[in]  int_flag: flag type
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_INT_FLAG_MUXCH0_SO: DMAMUX request multiplexer channel 0 synchronization overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_MUXCH1_SO: DMAMUX request multiplexer channel 1 synchronization overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_MUXCH2_SO: DMAMUX request multiplexer channel 2 synchronization overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH0_TO: DMAMUX request generator channel 0 trigger overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH1_TO: DMAMUX request generator channel 1 trigger overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH2_TO: DMAMUX request generator channel 2 trigger overrun interrupt flag
      \arg        DMAMUX_INT_FLAG_GENCH3_TO: DMAMUX request generator channel 3 trigger overrun interrupt flag
    \param[out] none
    \retval     none
*/
void dmamux_interrupt_flag_clear(dmamux_interrupt_flag_enum int_flag)
{
    DMAMUX_REG_VAL3(int_flag) = BIT(DMAMUX_BIT_POS(int_flag));
}

/*!
    \brief      enable DMAMUX interrupt (API ID: 0x0030U)
    \param[in]  interrupt: specify which interrupt to enable
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_INT_MUXCH0_SO: DMAMUX request multiplexer channel 0 synchronization overrun interrupt
      \arg        DMAMUX_INT_MUXCH1_SO: DMAMUX request multiplexer channel 1 synchronization overrun interrupt
      \arg        DMAMUX_INT_MUXCH2_SO: DMAMUX request multiplexer channel 2 synchronization overrun interrupt
      \arg        DMAMUX_INT_GENCH0_TO: DMAMUX request generator channel 0 trigger overrun interrupt
      \arg        DMAMUX_INT_GENCH1_TO: DMAMUX request generator channel 1 trigger overrun interrupt
      \arg        DMAMUX_INT_GENCH2_TO: DMAMUX request generator channel 2 trigger overrun interrupt
      \arg        DMAMUX_INT_GENCH3_TO: DMAMUX request generator channel 3 trigger overrun interrupt
    \param[out] none
    \retval     none
*/
void dmamux_interrupt_enable(dmamux_interrupt_enum interrupt)
{
    DMAMUX_REG_VAL(interrupt) |= BIT(DMAMUX_BIT_POS(interrupt));
}

/*!
    \brief      disable DMAMUX interrupt (API ID: 0x0031U)
    \param[in]  interrupt: specify which interrupt to disable
                only one parameter can be selected which is shown as below:
      \arg        DMAMUX_INT_MUXCH0_SO: DMAMUX request multiplexer channel 0 synchronization overrun interrupt
      \arg        DMAMUX_INT_MUXCH1_SO: DMAMUX request multiplexer channel 1 synchronization overrun interrupt
      \arg        DMAMUX_INT_MUXCH2_SO: DMAMUX request multiplexer channel 2 synchronization overrun interrupt
      \arg        DMAMUX_INT_GENCH0_TO: DMAMUX request generator channel 0 trigger overrun interrupt
      \arg        DMAMUX_INT_GENCH1_TO: DMAMUX request generator channel 1 trigger overrun interrupt
      \arg        DMAMUX_INT_GENCH2_TO: DMAMUX request generator channel 2 trigger overrun interrupt
      \arg        DMAMUX_INT_GENCH3_TO: DMAMUX request generator channel 3 trigger overrun interrupt
    \param[out] none
    \retval     none
*/
void dmamux_interrupt_disable(dmamux_interrupt_enum interrupt)
{
    DMAMUX_REG_VAL(interrupt) &= ~BIT(DMAMUX_BIT_POS(interrupt));
}
