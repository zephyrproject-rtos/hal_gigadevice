/*!
    \file    gd32h73x_75x_lpdts.c
    \brief   LPDTS driver

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

#include "gd32h73x_75x_lpdts.h"

/* LPDTS high threshold value offset macro */
#define LPDTS_IT_INTHT_OFFSET      ((uint32_t)16U)
/* sampling time offset macro */
#define LPDTS_CFG_SPT_OFFSET       ((uint32_t)16U)
/* engineering value offset macro */
#define LPDTS_SDATA_VAL_OFFSET     ((uint32_t)16U)
/* the T0 temperature macro */
#define LPDTS_T0_TMP_VAL0          (25)
#define LPDTS_T0_TMP_VAL1          (-40)
/* bit mask definitions */
#define LPDTS_FLAG_MASK            ((uint32_t)0x00008077U)
#define LPDTS_INT_FLAG_MASK        ((uint32_t)0x00000077U)
#define LPDTS_INTEN_MASK           ((uint32_t)0x00000077U)

/*!
    \brief      reset the LPDTS registers (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lpdts_deinit(void)
{
    rcu_periph_reset_enable(RCU_LPDTSRST);
    rcu_periph_reset_disable(RCU_LPDTSRST);
}

/*!
    \brief      initialize the parameters of LPDTS struct with the default values (API_ID(0x0002U))
    \param[in]  none
    \param[out] init_struct: the initialization data needed to initialize LPDTS
    \retval     none
*/
void lpdts_struct_para_init(lpdts_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(init_struct)) {
        fw_debug_report_err(LPDTS_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else /* FW_DEBUG_ERR_REPORT */
#endif
    {
        /* set the struct with the default values */
        init_struct->ref_clock          = REF_PCLK;
        init_struct->trigger_input      = NO_HARDWARE_TRIGGER;
        init_struct->sampling_time      = SPT_CLOCK_15;
    }

}

/*!
    \brief      initialize the LPDTS (API_ID(0x0003U))
    \param[in]  init_struct: the initialization data needed to initialize LPDTS_CFG
                  ref_clock: REF_PCLK, REF_LXTAL
                  trigger_input: NO_HARDWARE_TRIGGER, LPDTS_TRG
                  sampling_time: SPT_CLOCK_x(x=1..15)
    \param[out] none
    \retval     none
*/
void lpdts_init(lpdts_parameter_struct *init_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_LPDTS_INITIALIZATION_DATA(init_struct)) {
        fw_debug_report_err(LPDTS_MODULE_ID, API_ID(0x0003U), ERR_PARAM_INVALID);
    } else /* FW_DEBUG_ERR_REPORT */
#endif
    {
        uint32_t reg;
        /* configure the LPDTS_CFG */
        reg = LPDTS_CFG;
        reg &= ~(LPDTS_CFG_REFSEL | LPDTS_CFG_ITSEL | LPDTS_CFG_SPT);
        reg |= (init_struct->ref_clock | init_struct->trigger_input | init_struct->sampling_time);
        LPDTS_CFG = reg;
    }
}

/*!
    \brief      enable LPDTS temperature sensor (API_ID(0x0004U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lpdts_enable(void)
{
    LPDTS_CFG |= LPDTS_CFG_TSEN;
}

/*!
    \brief      disable LPDTS temperature sensor (API_ID(0x0005U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lpdts_disable(void)
{
    LPDTS_CFG &= ~LPDTS_CFG_TSEN;
}

/*!
    \brief      enable LPDTS software trigger (API_ID(0x0006U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lpdts_soft_trigger_enable(void)
{
    LPDTS_CFG |= LPDTS_CFG_TRGS;
}

/*!
    \brief      disable LPDTS software trigger (API_ID(0x0007U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lpdts_soft_trigger_disable(void)
{
    LPDTS_CFG &= ~LPDTS_CFG_TRGS;
}

/*!
    \brief      configure LPDTS high threshold value (API_ID(0x0008U))
    \param[in]  value: high threshold value(0~65535)
    \param[out] none
    \retval     none
*/
void lpdts_high_threshold_set(uint16_t value)
{
    uint32_t reg;
    /* configure the LPDTS_IT */
    reg = LPDTS_IT;
    reg &= ~LPDTS_IT_INTHT;
    reg |= (uint32_t)value << LPDTS_IT_INTHT_OFFSET;
    LPDTS_IT = reg;
}

/*!
    \brief      configure LPDTS low threshold value (API_ID(0x0009U))
    \param[in]  value: low threshold value(0~65535)
    \param[out] none
    \retval     none
*/
void lpdts_low_threshold_set(uint16_t value)
{
    uint32_t reg;
    /* configure the LPDTS_IT */
    reg = LPDTS_IT;
    reg &= ~LPDTS_IT_INTLT;
    reg |= (uint32_t)value;
    LPDTS_IT = reg;
}

/*!
    \brief      configure LPDTS reference clock selection (API_ID(0x000AU))
    \param[in]  source: reference clock source
                only one parameter can be selected which is shown as below:
      \arg        REF_PCLK: high speed reference clock (PCLK)
      \arg        REF_LXTAL: low speed reference clock (LXTAL)
    \param[out] none
    \retval     none
*/
void lpdts_ref_clock_source_config(uint32_t source)
{
    uint32_t reg;

    /* configure the LPDTS_CFG */
    reg = LPDTS_CFG;
    reg &= ~LPDTS_CFG_REFSEL;
    reg |= source & LPDTS_CFG_REFSEL;
    LPDTS_CFG = reg;
}

/*!
    \brief      get temperature from LPDTS (API_ID(0x000BU))
    \param[in]  none
    \param[out] none
    \retval     temperature: temperature in deg C
*/
int32_t lpdts_temperature_get(void)
{
    uint32_t freq;
    uint32_t count;
    int32_t t0;
    uint32_t val;
    uint32_t t0_freq;
    uint32_t ramp_coeff;
    uint32_t reg_cfg;
    int32_t temperature;

    /* get the total number of samples */
    count = (LPDTS_DATA & LPDTS_DATA_COVAL);
    /* get LPDTS_CFG configuration */
    reg_cfg = LPDTS_CFG;

    /* get the module frequency on Hz */
    if((reg_cfg & LPDTS_CFG_REFSEL) == LPDTS_CFG_REFSEL) {
        freq = (LXTAL_VALUE * count) / (2U * ((reg_cfg & LPDTS_CFG_SPT) >> LPDTS_CFG_SPT_OFFSET));
    } else {
        freq = (2U * rcu_clock_freq_get(CK_APB1) / count) * ((reg_cfg & LPDTS_CFG_SPT) >> LPDTS_CFG_SPT_OFFSET);
    }

    /* read factory settings */
    val = (LPDTS_SDATA & LPDTS_SDATA_VAL) >> LPDTS_SDATA_VAL_OFFSET;
    if(val == 0U) {
        t0 = LPDTS_T0_TMP_VAL0;
    } else if(val == 1U) {
        t0 = LPDTS_T0_TMP_VAL1;
    } else {
        /* illegal parameters */
    }

    /* get the T0 frequency on Hz */
    t0_freq = (LPDTS_SDATA & LPDTS_SDATA_FREQ) * 100U;
    /* get the ramp coefficient for the temperature sensor on deg C/Hz */
    ramp_coeff = LPDTS_RDATA & LPDTS_RDATA_RCVAL;

    /* figure out the temperature deg C */
    temperature = t0 + (((int32_t)freq - (int32_t)t0_freq) / (int32_t)ramp_coeff);

    return temperature;
}

/*!
    \brief      get LPDTS flag (API_ID(0x000CU))
    \param[in]  flag: LPDTS ready flag
                only one parameter can be selected which is shown as below:
      \arg        LPDTS_FLAG_TSR: temperature sensor ready flag
      \arg        LPDTS_FLAG_EMIF: end of measurement flag
      \arg        LPDTS_FLAG_LTIF: low threshold flag
      \arg        LPDTS_FLAG_HTIF: high threshold flag
      \arg        LPDTS_FLAG_EMAIF: end of measurement asynchronous flag
      \arg        LPDTS_FLAG_LTAIF: low threshold asynchronous flag
      \arg        LPDTS_FLAG_HTAIF: high threshold asynchronous flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus lpdts_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;
    if(LPDTS_STAT & (flag & LPDTS_FLAG_MASK)) {
        status = SET;
    }
    /* return the state of corresponding LPDTS flag */
    return status;
}

/*!
    \brief      enable LPDTS interrupt (API_ID(0x000DU))
    \param[in]  interrupt: the LPDTS interrupt
                one or more parameters can be selected which is shown as below:
      \arg        LPDTS_INT_EM: end of measurement interrupt enable
      \arg        LPDTS_INT_LT: low threshold interrupt enable
      \arg        LPDTS_INT_HT: high threshold interrupt enable
      \arg        LPDTS_INT_EMA: end of measurement asynchronous interrupt enable
      \arg        LPDTS_INT_LTA: low threshold asynchronous interrupt enable
      \arg        LPDTS_INT_HTA: high threshold asynchronous interrupt enable
    \param[out] none
    \retval     none
*/
void lpdts_interrupt_enable(uint32_t interrupt)
{
    LPDTS_INTEN |= (interrupt & LPDTS_INTEN_MASK);
}

/*!
    \brief      disable LPDTS interrupt (API_ID(0x000EU))
    \param[in]  interrupt: the LPDTS interrupt
                one or more parameters can be selected which is shown as below:
      \arg        LPDTS_INT_EM: end of measurement interrupt enable
      \arg        LPDTS_INT_LT: low threshold interrupt enable
      \arg        LPDTS_INT_HT: high threshold interrupt enable
      \arg        LPDTS_INT_EMA: end of measurement asynchronous interrupt enable
      \arg        LPDTS_INT_LTA: low threshold asynchronous interrupt enable
      \arg        LPDTS_INT_HTA: high threshold asynchronous interrupt enable
    \param[out] none
    \retval     none
*/
void lpdts_interrupt_disable(uint32_t interrupt)
{
    LPDTS_INTEN &= ~(interrupt & LPDTS_INTEN_MASK);
}

/*!
    \brief      get LPDTS interrupt flag (API_ID(0x000FU))
    \param[in]  flag: LPDTS interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        LPDTS_INT_FLAG_EM: end of measurement interrupt flag
      \arg        LPDTS_INT_FLAG_LT: low threshold interrupt flag
      \arg        LPDTS_INT_FLAG_HT: high threshold interrupt flag
      \arg        LPDTS_INT_FLAG_EMA: end of measurement asynchronous interrupt flag
      \arg        LPDTS_INT_FLAG_LTA: low threshold asynchronous interrupt flag
      \arg        LPDTS_INT_FLAG_HTA: high threshold asynchronous interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus lpdts_interrupt_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;
    uint32_t interrupt_enable = 0U, interrupt_flag = 0U;

    /* get error interrupt enable bit and flag bit */
    interrupt_enable = (LPDTS_INTEN & (flag & LPDTS_INTEN_MASK));
    interrupt_flag = (LPDTS_STAT & (flag & LPDTS_INT_FLAG_MASK));

    if(interrupt_flag && interrupt_enable) {
        status = SET;
    } else {
        status = RESET;
    }
    /* return the state of corresponding LPDTS flag */
    return status;
}

/*!
    \brief      clear the LPDTS interrupt flag (API_ID(0x0010U))
    \param[in]  flag: LPDTS flag
                only one parameter can be selected which is shown as below:
      \arg        LPDTS_INT_FLAG_EM: end of measurement interrupt flag
      \arg        LPDTS_INT_FLAG_LT: low threshold interrupt flag
      \arg        LPDTS_INT_FLAG_HT: high threshold interrupt flag
      \arg        LPDTS_INT_FLAG_EMA: end of measurement asynchronous interrupt flag
      \arg        LPDTS_INT_FLAG_LTA: low threshold asynchronous interrupt flag
      \arg        LPDTS_INT_FLAG_HTA: high threshold asynchronous interrupt flag
    \param[out] none
    \retval     none
*/
void lpdts_interrupt_flag_clear(uint32_t flag)
{
    /* clear the flags */
    LPDTS_INTC = flag & LPDTS_INT_FLAG_MASK;
}
