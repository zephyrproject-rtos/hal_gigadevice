/*!
    \file    gd32h73x_75x_tmu.c
    \brief   TMU driver

    \version 2026-07-30, V1.7.0, firmware for GD32H73x_75x
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

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

#include "gd32h73x_75x_tmu.h"

#define MASK_LOW_HALFWORD       ((uint32_t)0xFFFF0000U)
#define MASK_HIGH_HALFWORD      ((uint32_t)0x0000FFFFU)

/*!
    \brief      reset the TMU registers (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_deinit(void)
{
    rcu_periph_reset_enable(RCU_TMURST);
    rcu_periph_reset_disable(RCU_TMURST);
}

/*!
    \brief      initialize the parameters of TMU struct with the default values (API_ID(0x0002U))
    \param[in]  init_struct: pointer to init parameter struct
    \param[out] none
    \retval     none
*/
void tmu_struct_para_init(tmu_parameter_struct* init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* set the struct with the default values */
        init_struct->mode               = TMU_MODE_COS;
        init_struct->scale              = TMU_SCALING_FACTOR_1;
        init_struct->dma_read           = TMU_READ_DMA_DISABLE;
        init_struct->dma_write          = TMU_WRITE_DMA_DISABLE;
        init_struct->read_times         = TMU_READ_TIMES_1;
        init_struct->write_times        = TMU_WRITE_TIMES_1;
        init_struct->output_width       = TMU_OUTPUT_WIDTH_32;
        init_struct->input_width        = TMU_INPUT_WIDTH_32;
    }
}

/*!
    \brief      initialize TMU (API_ID(0x0003U))
    \param[in]  init_struct: pointer to init parameter struct
                  mode: TMU_MODE_COS,TMU_MODE_SIN,TMU_MODE_ATAN2,TMU_MODE_MODULUS,TMU_MODE_ATAN,
                        TMU_MODE_COSH,TMU_MODE_SINH,TMU_MODE_ATANH,TMU_MODE_LN,TMU_MODE_SQRT
                  scale: TMU_SCALING_FACTOR_x(x=1,2,4,8,16,32,64,128)
                  dma_read: TMU_READ_DMA_DISABLE, TMU_READ_DMA_ENABLE
                  dma_write: TMU_WRITE_DMA_DISABLE, TMU_WRITE_DMA_ENABLE
                  read_times: TMU_READ_TIMES_1, TMU_READ_TIMES_2
                  write_times: TMU_WRITE_TIMES_1, TMU_WRITE_TIMES_2
                  output_width: TMU_OUTPUT_WIDTH_32, TMU_OUTPUT_WIDTH_16
                  input_width: TMU_INPUT_WIDTH_32, TMU_INPUT_WIDTH_16
    \param[out] none
    \retval     none
*/
void tmu_init(tmu_parameter_struct* init_struct)
{
    uint32_t reg = 0x50U;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        reg |= ((init_struct->mode & TMU_CS_MODE) | (init_struct->scale & TMU_CS_FACTOR) |
                (init_struct->dma_read & TMU_CS_RDEN) | (init_struct->dma_write & TMU_CS_WDEN) |
                (init_struct->read_times & TMU_CS_ONUM) | (init_struct->write_times & TMU_CS_INUM) |
                (init_struct->output_width & TMU_CS_OWIDTH) | (init_struct->input_width & TMU_CS_IWIDTH));
        TMU_CS = reg;
    }
}

/*!
    \brief      enable TMU read interrupt (API_ID(0x0004U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_read_interrupt_enable(void)
{
    TMU_CS |= TMU_CS_RIE;
}

/*!
    \brief      disable TMU read interrupt (API_ID(0x0005U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_read_interrupt_disable(void)
{
    TMU_CS &= ~TMU_CS_RIE;
}

/*!
    \brief      enable TMU DMA read request (API_ID(0x0006U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_dma_read_enable(void)
{
    TMU_CS |= TMU_CS_RDEN;
}

/*!
    \brief      disable TMU DMA read request (API_ID(0x0007U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_dma_read_disable(void)
{
    TMU_CS &= ~TMU_CS_RDEN;
}

/*!
    \brief      enable TMU DMA write request (API_ID(0x0008U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_dma_write_enable(void)
{
    TMU_CS |= TMU_CS_WDEN;
}

/*!
    \brief      disable TMU DMA write request (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tmu_dma_write_disable(void)
{
    TMU_CS &= ~TMU_CS_WDEN;
}

/*!
    \brief      write one data in q1.31 format (API_ID(0x000AU))
    \param[in]  data: the first input data only
    \param[out] none
    \retval     none
*/
void tmu_one_q31_write(uint32_t data)
{
    TMU_IDATA = data;
}

/*!
    \brief      write two data in q1.31 format (API_ID(0x000BU))
    \param[in]  data1: the first input data
    \param[in]  data2: the second input data
    \param[out] none
    \retval     none
*/
void tmu_two_q31_write(uint32_t data1, uint32_t data2)
{
    TMU_IDATA = data1;
    TMU_IDATA = data2;
}

/*!
    \brief      write two data in q1.15 format (API_ID(0x000CU))
    \param[in]  data1: the first input data
    \param[in]  data2: the second input data (this data is meaningless in mode4 ~ mode9)
    \param[out] none
    \retval     none
*/
void tmu_two_q15_write(uint16_t data1, uint16_t data2)
{
    TMU_IDATA = ((((uint32_t)data1) & MASK_HIGH_HALFWORD)| (((uint32_t)data2 << 16U) & MASK_LOW_HALFWORD));
}

/*!
    \brief      read one data in q1.31 format (API_ID(0x000DU))
    \param[in]  none
    \param[out] p: pointer to the first output data only
    \retval     none
*/
void tmu_one_q31_read(uint32_t* p)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(p)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x000DU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        *p = TMU_ODATA;
    }
}

/*!
    \brief      read two data in q1.31 format (API_ID(0x000EU))
    \param[in]  none
    \param[out] p1: pointer to the first output data
    \param[out] p2: pointer to the second output data
    \retval     none
*/
void tmu_two_q31_read(uint32_t* p1, uint32_t* p2)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(p1)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x000EU), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(p2)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x000EU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        *p1 = TMU_ODATA;
        __DSB();
        *p2 = TMU_ODATA;
    }
}

/*!
    \brief      read two data in q1.15 format (API_ID(0x000FU))
    \param[in]  none
    \param[out] p1: pointer to the first output data
    \param[out] p2: pointer to the second output data (this data is meaningless in mode4, mode7 ~ mode9)
    \retval     none
*/
void tmu_two_q15_read(uint16_t* p1, uint16_t* p2)
{
    uint32_t data;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(p1)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x000FU), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(p2)) {
        fw_debug_report_err(TMU_MODULE_ID, API_ID(0x000FU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        data = TMU_ODATA;
        *p1 = (uint16_t)data;
        *p2 = (uint16_t)(data >> 16U);
    }
}

/*!
    \brief      get TMU flag (API_ID(0x0010U))
    \param[in]  flag: the TMU flags
                one or more parameters can be selected which are shown as below:
      \arg        TMU_FLAG_END: end of TMU operation flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus tmu_flag_get(uint32_t flag)
{
    FlagStatus ret_sta = RESET;

    if(TMU_CS & flag) {
        ret_sta = SET;
    } else {
        ret_sta = RESET;
    }

    return ret_sta;
}
