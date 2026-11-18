/*!
    \file    gd32h73x_75x_rspdif.c
    \brief   RSPDIF driver

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

#include "gd32h73x_75x_rspdif.h"

#define RSPDIF_INIT_MASK                          0xFFC88004U    /*!< RSPDIF parameter initialization mask */
#define RSPDIF_FLAG_GET_MASK                      0x000001FFU    /*!< RSPDIF flag get mask */
#define RSPDIF_FLAG_CLEAR_MASK                    0x0000003CU    /*!< RSPDIF flag clear mask */
#define RSPDIF_INT_MASK                           0x0000007FU    /*!< RSPDIF interrupt mask */

#define RSPDIF_CKCNT5_OFFSET                      (uint32_t)16U  /*!< RSPDIF the number of consecutive time clock cycles offset */
#define RSPDIF_CHANNEL_STAT_OFFSET                (uint32_t)16U  /*!< RSPDIF the channel status offset */
#define RSPDIF_TH_LOW_OFFSET                      (uint32_t)16U  /*!< RSPDIF the low threshold offset */

#define RSPDIF_DATA_F0_PREF_OFFSET                (uint32_t)28U  /*!< RSPDIF data format 0 preamble type offset */
#define RSPDIF_DATA_F0_C_OFFSET                   (uint32_t)27U  /*!< RSPDIF data format 0 channel status offset */
#define RSPDIF_DATA_F0_U_OFFSET                   (uint32_t)26U  /*!< RSPDIF data format 0 user bit offset */
#define RSPDIF_DATA_F0_V_OFFSET                   (uint32_t)25U  /*!< RSPDIF data format 0 validity bit offset */
#define RSPDIF_DATA_F0_P_OFFSET                   (uint32_t)24U  /*!< RSPDIF data format 0 parity error bit offset */

#define RSPDIF_DATA_F1_PREF_OFFSET                (uint32_t)4U   /*!< RSPDIF data format 1 preamble type offset */
#define RSPDIF_DATA_F1_C_OFFSET                   (uint32_t)3U   /*!< RSPDIF data format 1 channel status offset */
#define RSPDIF_DATA_F1_U_OFFSET                   (uint32_t)2U   /*!< RSPDIF data format 1 user bit offset */
#define RSPDIF_DATA_F1_V_OFFSET                   (uint32_t)1U   /*!< RSPDIF data format 1 validity bit offset */
#define RSPDIF_DATA_F1_DATA_OFFSET                (uint32_t)8U   /*!< RSPDIF data format 1 data offset */

/*!
    \brief      reset the RSPDIF (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_deinit(void)
{
    rcu_periph_reset_enable(RCU_RSPDIFRST);
    rcu_periph_reset_disable(RCU_RSPDIFRST);
}

/*!
    \brief      initialize the parameters of RSPDIF structure with the default values (API_ID(0x0002U))
    \param[in]  none
    \param[out] rspdif_parameter_struct: the initialized structure rspdif_parameter_struct pointer
    \retval     none
*/
void rspdif_struct_para_init(rspdif_parameter_struct *rspdif_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(rspdif_struct)) {
        fw_debug_report_err(RSPDIF_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* configure the RSPDIF structure with the default values */
        rspdif_struct->input_sel          = RSPDIF_INPUT_IN0;
        rspdif_struct->max_retrie         = RSPDIF_MAXRETRIES_15;
        rspdif_struct->wait_activity      = RSPDIF_WAIT_FOR_ACTIVITY_ON;
        rspdif_struct->channel_sel        = RSPDIF_CHANNEL_A;
        rspdif_struct->sample_format      = RSPDIF_DATAFORMAT_MSB;
        rspdif_struct->sound_mode         = RSPDIF_STEREOMODE_ENABLE;
        rspdif_struct->pre_type           = RSPDIF_PREAMBLE_TYPE_MASK_OFF;
        rspdif_struct->channel_status_bit = RSPDIF_CHANNEL_STATUS_MASK_OFF;
        rspdif_struct->validity_bit       = RSPDIF_VALIDITY_MASK_OFF;
        rspdif_struct->parity_error_bit   = RSPDIF_PERROR_MASK_OFF;
        rspdif_struct->symbol_clk         = RSPDIF_SYMBOL_CLK_OFF;
        rspdif_struct->bak_symbol_clk     = RSPDIF_BACKUP_SYMBOL_CLK_OFF;
    }
}

/*!
    \brief      initialize the RSPDIF parameters (API_ID(0x0003U))
    \param[in]  rspdif_struct : RSPDIF parameter initialization structure and the member values are shown as below:
                  input_sel          : RSPDIF_INPUT_INx (x = 0 ~ 3)
                  max_retrie         : RSPDIF_MAXRETRIES_NONE, RSPDIF_MAXRETRIES_3, RSPDIF_MAXRETRIES_15, RSPDIF_MAXRETRIES_63
                  wait_activity      : RSPDIF_WAIT_FOR_ACTIVITY_OFF, RSPDIF_WAIT_FOR_ACTIVITY_ON
                  channel_sel        : RSPDIF_CHANNEL_A, RSPDIF_CHANNEL_B
                  sample_format      : RSPDIF_DATAFORMAT_LSB, RSPDIF_DATAFORMAT_MSB, RSPDIF_DATAFORMAT_32BITS
                  sound_mode         : RSPDIF_STEREOMODE_DISABLE, RSPDIF_STEREOMODE_ENABLE
                  pre_type           : RSPDIF_PREAMBLE_TYPE_MASK_OFF, RSPDIF_PREAMBLE_TYPE_MASK_ON
                  channel_status_bit : RSPDIF_CHANNEL_STATUS_MASK_OFF, RSPDIF_CHANNEL_STATUS_MASK_ON
                  validity_bit       : RSPDIF_VALIDITY_MASK_OFF, RSPDIF_VALIDITY_MASK_ON
                  parity_error_bit   : RSPDIF_PERROR_MASK_OFF, RSPDIF_PERROR_MASK_ON
                  symbol_clk         : RSPDIF_BACKUP_SYMBOL_CLK_OFF, RSPDIF_BACKUP_SYMBOL_CLK_ON
                  bak_symbol_clk     : RSPDIF_SYMBOL_CLK_OFF, RSPDIF_SYMBOL_CLK_ON
    \param[out] none
    \retval     none
*/
void rspdif_init(rspdif_parameter_struct *rspdif_struct)
{
    uint32_t reg = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(rspdif_struct)) {
        fw_debug_report_err(RSPDIF_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        reg = RSPDIF_CTL;
        reg &= RSPDIF_INIT_MASK;

        /* select the RSPDIF input */
        reg |= (rspdif_struct->input_sel & RSPDIF_CTL_RXCHSEL);
        /* configure the RSPDIF maximum allowed re-tries during synchronization phase */
        reg |= (rspdif_struct->max_retrie & RSPDIF_CTL_MAXRT);
        /* configure the RSPDIF wait for activity on the selected input */
        reg |= (rspdif_struct->wait_activity & RSPDIF_CTL_WFRXA);
        /* select the channel status from channel A or B */
        reg |= (rspdif_struct->channel_sel & RSPDIF_CTL_CFCHSEL);
        /* configure the RSPDIF data samples format */
        reg |= (rspdif_struct->sample_format & RSPDIF_CTL_RXDF);
        /* select stereo or mono mode */
        reg |= (rspdif_struct->sound_mode & RSPDIF_CTL_RXSTEOMEN);
        /* select whether the preamble type value into the RSPDIF_DATA */
        reg |= (rspdif_struct->pre_type & RSPDIF_CTL_PTNCPEN);
        /* select whether the channel status and user bits are copied or not into the received frame */
        reg |= (rspdif_struct->channel_status_bit & RSPDIF_CTL_CUNCPEN);
        /* select whether the validity bit is copied or not into the received frame */
        reg |= (rspdif_struct->validity_bit & RSPDIF_CTL_VNCPEN);
        /* select whether the parity error bit is copied or not into the received frame */
        reg |= (rspdif_struct->parity_error_bit & RSPDIF_CTL_PNCPEN);
        /* configure the RSPDIF symbol clock generation */
        reg |= (rspdif_struct->symbol_clk & RSPDIF_CTL_SCKEN);
        /* configure the RSPDIF backup symbol clock generation */
        reg |= (rspdif_struct->bak_symbol_clk & RSPDIF_CTL_BKSCKEN);

        /* write to RSPDIF_CTL register */
        RSPDIF_CTL = reg;
    }
}

/*!
    \brief      specifies the RSPDIF peripheral state (API_ID(0x0004U))
    \param[in]  mode :RSPDIF enable mode
                only one parameter can be selected which is shown as below:
      \arg        RSPDIF_STATE_SYNC: enable RSPDIF synchronization only
      \arg        RSPDIF_STATE_RCV : enable RSPDIF receiver
    \param[out] none
    \retval     none
*/
void rspdif_enable(uint32_t mode)
{
    uint32_t reg = 0U;

    reg = RSPDIF_CTL;

    /* clear RSPDIF state */
    reg &= ~RSPDIF_CTL_RXCFG;
    reg |= mode & RSPDIF_CTL_RXCFG;

    /* enable RSPDIF */
    RSPDIF_CTL = reg;
}

/*!
    \brief      disable RSPDIF (API_ID(0x0005U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_disable(void)
{
    /* clear RSPDIF state */
    RSPDIF_CTL &= ~RSPDIF_CTL_RXCFG;
}

/*!
    \brief      enable RSPDIF symbol clock (API_ID(0x0006U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_symbol_clock_enable(void)
{
    RSPDIF_CTL |= RSPDIF_CTL_SCKEN;
}

/*!
    \brief      disable RSPDIF symbol clock (API_ID(0x0007U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_symbol_clock_disable(void)
{
    RSPDIF_CTL &= ~RSPDIF_CTL_SCKEN;
}

/*!
    \brief      enable RSPDIF backup symbol clock (API_ID(0x0008U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_backup_symbol_clock_enable(void)
{
    RSPDIF_CTL |= RSPDIF_CTL_BKSCKEN;
}

/*!
    \brief      disable RSPDIF backup symbol clock (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_backup_symbol_clock_disable(void)
{
    RSPDIF_CTL &= ~RSPDIF_CTL_BKSCKEN;
}

/*!
    \brief      enable the RSPDIF receiver DMA (API_ID(0x000AU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_dma_enable(void)
{
    RSPDIF_CTL |= RSPDIF_CTL_DMAREN;
}

/*!
    \brief      disable the RSPDIF receiver DMA (API_ID(0x000BU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_dma_disable(void)
{
    RSPDIF_CTL &= ~RSPDIF_CTL_DMAREN;
}

/*!
    \brief      enable the RSPDIF control buffer DMA (API_ID(0x000CU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_control_buffer_dma_enable(void)
{
    RSPDIF_CTL |= RSPDIF_CTL_DMACBEN;
}

/*!
    \brief      disable the RSPDIF control buffer DMA (API_ID(0x000DU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rspdif_control_buffer_dma_disable(void)
{
    RSPDIF_CTL &= ~RSPDIF_CTL_DMACBEN;
}

/*!
    \brief      RSPDIF read data (API_ID(0x000EU))
    \param[in]  none
    \param[out] data_struct: RSPDIF data structure and the member values are shown as below:
                  format         : RSPDIF_DATAFORMAT_LSB, RSPDIF_DATAFORMAT_MSB,RSPDIF_DATAFORMAT_32BITS
                  preamble       : RSPDIF_PREAMBLE_NONE,RSPDIF_PREAMBLE_B,RSPDIF_PREAMBLE_M,RSPDIF_PREAMBLE_W
                  channel_status : 0 or 1
                  user_bit       : 0 or 1
                  validity       : 0 or 1
                  parity_err     : 0 or 1
                  data0          : 0 ~ 65535
                  data1          : 0 ~ 65535
    \retval     none
*/
void rspdif_data_read(rspdif_data_struct *data_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(data_struct)) {
        fw_debug_report_err(RSPDIF_MODULE_ID, API_ID(0x000EU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* get data format */
        data_struct->format = RSPDIF_CTL & RSPDIF_CTL_RXDF;

        switch(data_struct->format) {
        /* data format 0 */
        case RSPDIF_DATAFORMAT_LSB:
            /* the preamble type */
            data_struct->preamble       = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F0_PREF) >> RSPDIF_DATA_F0_PREF_OFFSET);
            /* channel status bit */
            data_struct->channel_status = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F0_C)    >> RSPDIF_DATA_F0_C_OFFSET);
            /* user bit */
            data_struct->user_bit       = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F0_U)    >> RSPDIF_DATA_F0_U_OFFSET);
            /* validity bit */
            data_struct->validity       = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F0_V)    >> RSPDIF_DATA_F0_V_OFFSET);
            /* parity error bit */
            data_struct->parity_err     = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F0_P)    >> RSPDIF_DATA_F0_P_OFFSET);
            /* data value 0 */
            data_struct->data0          = (uint32_t)(RSPDIF_DATA & RSPDIF_DATA_F0_DATA0);
            /* data value 1 */
            data_struct->data1          = 0U;
            break;
        /* data format 1 */
        case RSPDIF_DATAFORMAT_MSB:
            /* the preamble type */
            data_struct->preamble       = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F1_PREF)  >> RSPDIF_DATA_F1_PREF_OFFSET);
            /* channel status bit */
            data_struct->channel_status = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F1_C)     >> RSPDIF_DATA_F1_C_OFFSET);
            /* user bit */
            data_struct->user_bit       = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F1_U)     >> RSPDIF_DATA_F1_U_OFFSET);
            /* validity bit */
            data_struct->validity       = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F1_V)     >> RSPDIF_DATA_F1_V_OFFSET);
            /* parity error bit */
            data_struct->parity_err     = (uint32_t)(RSPDIF_DATA & RSPDIF_DATA_F1_P);
            /* data value 0 */
            data_struct->data0          = (uint32_t)((RSPDIF_DATA & RSPDIF_DATA_F1_DATA0) >> RSPDIF_DATA_F1_DATA_OFFSET);
            /* data value 1 */
            data_struct->data1          = 0U;
            break;
        /* data format 2 */
        case RSPDIF_DATAFORMAT_32BITS:
            /* the preamble type */
            data_struct->preamble       = 0U;
            /* channel status bit */
            data_struct->channel_status = 0U;
            /* user bit */
            data_struct->user_bit       = 0U;
            /* validity bit */
            data_struct->validity       = 0U;
            /* parity error bit */
            data_struct->parity_err     = 0U;
            /* data value 0 */
            data_struct->data0          = (uint32_t)(RSPDIF_DATA & RSPDIF_DATA_F2_DATA1);
            /* data value 1 */
            data_struct->data1          = (uint32_t)(RSPDIF_DATA & RSPDIF_DATA_F2_DATA2);
            break;
        default:
            break;
        }
    }
}

/*!
    \brief      get duration of 5 symbols counted using rspdif_ck (API_ID(0x000FU))
    \param[in]  none
    \param[out] none
    \retval     duration of 5 symbols counted using rspdif_ck
*/
uint32_t rspdif_duration_of_symbols_get(void)
{
    return ((RSPDIF_STAT & RSPDIF_STAT_CKCNT5) >> RSPDIF_CKCNT5_OFFSET);
}

/*!
    \brief      get user data information (API_ID(0x0010U))
    \param[in]  none
    \param[out] none
    \retval     user data information
*/
uint32_t rspdif_user_data_get(void)
{
    return (RSPDIF_CHSTAT & RSPDIF_CHSTAT_USER);
}

/*!
    \brief      get channel status information (API_ID(0x0011U))
    \param[in]  none
    \param[out] none
    \retval     channel status information
*/
uint32_t rspdif_channel_status_get(void)
{
    return ((RSPDIF_CHSTAT & RSPDIF_CHSTAT_CHS) >> RSPDIF_CHANNEL_STAT_OFFSET);
}

/*!
    \brief      get start of block (API_ID(0x0012U))
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rspdif_start_block_status_get(void)
{
    FlagStatus reval = RESET;

    if(RESET != (RSPDIF_CHSTAT & RSPDIF_CHSTAT_SOB)) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      get threshold low estimation (API_ID(0x0013U))
    \param[in]  none
    \param[out] none
    \retval     low threshold value
*/
uint32_t rspdif_low_threshold_get(void)
{
    return ((RSPDIF_DTH & RSPDIF_DTH_THLO) >> RSPDIF_TH_LOW_OFFSET);
}

/*!
    \brief      get threshold high estimation (API_ID(0x0014U))
    \param[in]  none
    \param[out] none
    \retval     high threshold value
*/
uint32_t rspdif_high_threshold_get(void)
{
    return (RSPDIF_DTH & RSPDIF_DTH_THHI);
}

/*!
    \brief      get RSPDIF flag status (API_ID(0x0015U))
    \param[in]  flag: RSPDIF flag status
                only one parameter can be selected which is shown as below:
      \arg        RSPDIF_FLAG_RBNE     : RSPDIF RX buffer is not empty
      \arg        RSPDIF_FLAG_CBNE     : RSPDIF RX control buffer is not empty
      \arg        RSPDIF_FLAG_PERR     : RSPDIF parity error
      \arg        RSPDIF_FLAG_RXORERR  : RSPDIF RX overrun error
      \arg        RSPDIF_FLAG_SYNDB    : RSPDIF synchronization block detected
      \arg        RSPDIF_FLAG_SYNDO    : RSPDIF synchronization done
      \arg        RSPDIF_FLAG_FRERR    : RSPDIF frame error
      \arg        RSPDIF_FLAG_SYNERR   : RSPDIF synchronization error
      \arg        RSPDIF_FLAG_TMOUTERR : RSPDIF time out error
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rspdif_flag_get(uint32_t flag)
{
    FlagStatus reval = RESET;

    if(RESET != (RSPDIF_STAT & (flag & RSPDIF_FLAG_GET_MASK))) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      clear RSPDIF flag (API_ID(0x0016U))
    \param[in]  flag: RSPDIF flag status
                one or more parameters can be selected which is shown as below:
      \arg        RSPDIF_FLAG_PERR    : RSPDIF parity error
      \arg        RSPDIF_FLAG_RXORERR : RSPDIF RX overrun error
      \arg        RSPDIF_FLAG_SYNDB   : RSPDIF synchronization block detected
      \arg        RSPDIF_FLAG_SYNDO   : RSPDIF synchronization done
    \param[out] none
    \retval     none
*/
void rspdif_flag_clear(uint32_t flag)
{
    RSPDIF_STATC |= (flag & RSPDIF_FLAG_CLEAR_MASK);
}

/*!
    \brief      enable RSPDIF interrupt (API_ID(0x0017U))
    \param[in]  interrupt: RSPDIF interrupt
                one or more parameters can be selected which is shown as below:
      \arg        RSPDIF_INT_RBNE   : RSPDIF RX buffer is not empty interrupt
      \arg        RSPDIF_INT_CBNE   : RSPDIF RX control buffer is not empty interrupt
      \arg        RSPDIF_INT_PERR   : RSPDIF parity error interrupt
      \arg        RSPDIF_INT_RXORERR: RSPDIF RX overrun error interrupt
      \arg        RSPDIF_INT_SYNDB  : RSPDIF synchronization block detected interrupt
      \arg        RSPDIF_INT_SYNDO  : RSPDIF synchronization done interrupt
      \arg        RSPDIF_INT_RXDCERR: RSPDIF data decoding error interrupt
    \param[out] none
    \retval     none
*/
void rspdif_interrupt_enable(uint32_t interrupt)
{
    RSPDIF_INTEN |= (interrupt & RSPDIF_INT_MASK);
}

/*!
    \brief      disable RSPDIF interrupt (API_ID(0x0018U))
    \param[in]  interrupt: RSPDIF interrupt
                one or more parameters can be selected which is shown as below:
      \arg        RSPDIF_INT_RBNE   : RSPDIF RX buffer is not empty interrupt
      \arg        RSPDIF_INT_CBNE   : RSPDIF RX control buffer is not empty interrupt
      \arg        RSPDIF_INT_PERR   : RSPDIF parity error interrupt
      \arg        RSPDIF_INT_RXORERR: RSPDIF RX overrun error interrupt
      \arg        RSPDIF_INT_SYNDB  : RSPDIF synchronization block detected interrupt
      \arg        RSPDIF_INT_SYNDO  : RSPDIF synchronization done interrupt
      \arg        RSPDIF_INT_RXDCERR: RSPDIF data decoding error interrupt
    \param[out] none
    \retval     none
*/
void rspdif_interrupt_disable(uint32_t interrupt)
{
    RSPDIF_INTEN &= ~(interrupt & RSPDIF_INT_MASK);
}

/*!
    \brief      get RSPDIF interrupt flag status (API_ID(0x0019U))
    \param[in]  int_flag: RSPDIF interrupt flag status
                only one parameter can be selected which is shown as below:
      \arg        RSPDIF_INT_FLAG_RBNE     : RSPDIF RX buffer is not empty interrupt flag
      \arg        RSPDIF_INT_FLAG_CBNE     : RSPDIF RX control buffer is not empty interrupt flag
      \arg        RSPDIF_INT_FLAG_PERR     : RSPDIF parity error interrupt flag
      \arg        RSPDIF_INT_FLAG_RXORERR  : RSPDIF RX overrun error interrupt flag
      \arg        RSPDIF_INT_FLAG_SYNDB    : RSPDIF synchronization block detected interrupt flag
      \arg        RSPDIF_INT_FLAG_SYNDO    : RSPDIF synchronization done interrupt flag
      \arg        RSPDIF_INT_FLAG_FRERR    : RSPDIF frame error interrupt flag
      \arg        RSPDIF_INT_FLAG_SYNERR   : RSPDIF synchronization error interrupt flag
      \arg        RSPDIF_INT_FLAG_TMOUTERR : RSPDIF time out error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rspdif_interrupt_flag_get(rspdif_interrupt_flag_enum int_flag)
{
    FlagStatus reval = RESET;
    uint32_t ret1;
    uint32_t ret2;

    /* get the status of interrupt enable bit */
    ret1 = (RSPDIF_REG_VAL(int_flag) & BIT(RSPDIF_BIT_POS(int_flag)));
    /* get the status of interrupt flag */
    ret2 = (RSPDIF_REG_VAL2(int_flag) & BIT(RSPDIF_BIT_POS2(int_flag)));

    if((RESET != ret1) && (RESET != ret2)) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      clear RSPDIF interrupt flag status (API_ID(0x001AU))
    \param[in]  int_flag: RSPDIF interrupt flag status
                only one parameter can be selected which is shown as below:
      \arg        RSPDIF_INT_FLAG_PERR    : RSPDIF parity error interrupt flag
      \arg        RSPDIF_INT_FLAG_RXORERR : RSPDIF RX overrun error interrupt flag
      \arg        RSPDIF_INT_FLAG_SYNDB   : RSPDIF synchronization block detected interrupt flag
      \arg        RSPDIF_INT_FLAG_SYNDO   : RSPDIF synchronization done interrupt flag
    \param[out] none
    \retval     none
*/
void rspdif_interrupt_flag_clear(rspdif_interrupt_flag_enum int_flag)
{
    RSPDIF_STATC |= BIT(RSPDIF_BIT_POS2(int_flag));
}
