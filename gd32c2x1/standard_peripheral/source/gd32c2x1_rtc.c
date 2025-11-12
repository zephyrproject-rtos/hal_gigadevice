/*!
    \file    gd32c2x1_rtc.c
    \brief   RTC driver

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

#include "gd32c2x1_rtc.h"

/* RTC timeout value */
#define RTC_INITM_TIMEOUT                  ((uint32_t)0x00004000U)                    /*!< initialization state flag timeout */
#define RTC_RSYNF_TIMEOUT                  ((uint32_t)0x00008000U)                    /*!< register synchronization flag timeout */
#define RTC_HRFC_TIMEOUT                   ((uint32_t)0x00008000U)                    /*!< recalibration pending flag timeout */
#define RTC_SHIFTCTL_TIMEOUT               ((uint32_t)0x00001000U)                    /*!< shift function operation pending flag timeout */
#define RTC_ALRMXWF_TIMEOUT                ((uint32_t)0x00008000U)                    /*!< alarm configuration can be written flag timeout */

/*!
    \brief      enable RTC bypass shadow registers function (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_bypass_shadow_enable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL |= (uint8_t)RTC_CTL_BPSHAD;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC bypass shadow registers function (API_ID(0x0002U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_bypass_shadow_disable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint8_t)~RTC_CTL_BPSHAD;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      wait until RTC_TIME and RTC_DATE registers are synchronized with APB clock, and the shadow (API_ID(0x0003U))
                registers are updated
    \param[in]  none
    \param[out] none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_register_sync_wait(void)
{
    volatile uint32_t time_index = RTC_RSYNF_TIMEOUT;
    uint32_t flag_status;
    ErrStatus error_status = ERROR;

    if((uint32_t)RESET == (RTC_CTL & RTC_CTL_BPSHAD)) {
        /* disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2;

        /* firstly clear RSYNF flag */
        RTC_STAT &= (uint32_t)(~RTC_STAT_RSYNF);

        /* wait until RSYNF flag to be set */
        do {
            flag_status = RTC_STAT & RTC_STAT_RSYNF;
        } while((--time_index > 0U) && ((uint32_t)0U == flag_status));

        if((uint32_t)RESET != flag_status) {
            error_status = SUCCESS;
        }

        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
    } else {
        error_status = SUCCESS;
    }

    return error_status;
}

/*!
    \brief      enable RTC alarm (API_ID(0x0004U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_alarm_enable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    
    RTC_CTL |= RTC_CTL_ALRM0EN;
    
    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC alarm (API_ID(0x0005U))
    \param[out] none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_alarm_disable(void)
{
    volatile uint32_t time_index = RTC_ALRMXWF_TIMEOUT;
    ErrStatus error_status = ERROR;
    uint32_t flag_status ;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* clear the state of alarm */
    RTC_CTL &= (uint32_t)(~RTC_CTL_ALRM0EN);
    /* wait until ALRM0WF flag to be set after the alarm is disabled */
    do {
        flag_status = RTC_STAT & RTC_STAT_ALRM0WF;
    } while((--time_index > 0U) && ((uint32_t)RESET == flag_status));

    if((uint32_t)RESET != flag_status) {
        error_status = SUCCESS;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      configure RTC alarm (API_ID(0x0006U))
    \param[in]  rtc_alarm_time: pointer to a rtc_alarm_struct structure which contains
                parameters for RTC alarm configuration
                members of the structure and the member values are shown as below:
                  alarm_mask: RTC_ALARM_NONE_MASK, RTC_ALARM_DATE_MASK, RTC_ALARM_HOUR_MASK
                                  RTC_ALARM_MINUTE_MASK, RTC_ALARM_SECOND_MASK, RTC_ALARM_ALL_MASK
                  weekday_or_date: RTC_ALARM_DATE_SELECTED, RTC_ALARM_WEEKDAY_SELECTED
                  alarm_day: 1) 0x1 - 0x31(BCD format) if RTC_ALARM_DATE_SELECTED is set
                                 2) RTC_MONDAY, RTC_TUESDAY, RTC_WEDNESDAY, RTC_THURSDAY, RTC_FRIDAY,
                                    RTC_SATURDAY, RTC_SUNDAY if RTC_ALARM_WEEKDAY_SELECTED is set
                  alarm_hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc display_format
                  alarm_minute: 0x0 - 0x59(BCD format)
                  alarm_second: 0x0 - 0x59(BCD format)
                  am_pm: RTC_AM, RTC_PM
    \param[out] none
    \retval     none
*/
void rtc_alarm_config(rtc_alarm_struct *rtc_alarm_time)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_RTC_VALID_POINTER(rtc_alarm_time)) {
        fw_debug_report_err(RTC_MODULE_ID, API_ID(0x0006U), ERR_PARAM_POINTER);
    }else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t reg_alrmtd;

        /* disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2;

        reg_alrmtd = (rtc_alarm_time->alarm_mask | \
                      rtc_alarm_time->weekday_or_date | \
                      rtc_alarm_time->am_pm | \
                      ALRMTD_DAY(rtc_alarm_time->alarm_day) | \
                      ALRMTD_HR(rtc_alarm_time->alarm_hour) | \
                      ALRMTD_MN(rtc_alarm_time->alarm_minute) | \
                      ALRMTD_SC(rtc_alarm_time->alarm_second));
        
        RTC_ALRM0TD = reg_alrmtd;
        
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
    }
}

/*!
    \brief      configure subsecond of RTC alarm (API_ID(0x0007U))
    \param[in]  mask_subsecond: alarm subsecond mask
                only one parameter can be selected which is shown as below:
      \arg        RTC_MSKSSC_0_14: mask alarm subsecond configuration
      \arg        RTC_MSKSSC_1_14: mask RTC_ALRMXSS_SSC[14:1], and RTC_ALRMXSS_SSC[0] is to be compared
      \arg        RTC_MSKSSC_2_14: mask RTC_ALRMXSS_SSC[14:2], and RTC_ALRMXSS_SSC[1:0] is to be compared
      \arg        RTC_MSKSSC_3_14: mask RTC_ALRMXSS_SSC[14:3], and RTC_ALRMXSS_SSC[2:0] is to be compared
      \arg        RTC_MSKSSC_4_14: mask RTC_ALRMXSS_SSC[14:4]], and RTC_ALRMXSS_SSC[3:0] is to be compared
      \arg        RTC_MSKSSC_5_14: mask RTC_ALRMXSS_SSC[14:5], and RTC_ALRMXSS_SSC[4:0] is to be compared
      \arg        RTC_MSKSSC_6_14: mask RTC_ALRMXSS_SSC[14:6], and RTC_ALRMXSS_SSC[5:0] is to be compared
      \arg        RTC_MSKSSC_7_14: mask RTC_ALRMXSS_SSC[14:7], and RTC_ALRMXSS_SSC[6:0] is to be compared
      \arg        RTC_MSKSSC_8_14: mask RTC_ALRMXSS_SSC[14:8], and RTC_ALRMXSS_SSC[7:0] is to be compared
      \arg        RTC_MSKSSC_9_14: mask RTC_ALRMXSS_SSC[14:9], and RTC_ALRMXSS_SSC[8:0] is to be compared
      \arg        RTC_MSKSSC_10_14: mask RTC_ALRMXSS_SSC[14:10], and RTC_ALRMXSS_SSC[9:0] is to be compared
      \arg        RTC_MSKSSC_11_14: mask RTC_ALRMXSS_SSC[14:11], and RTC_ALRMXSS_SSC[10:0] is to be compared
      \arg        RTC_MSKSSC_12_14: mask RTC_ALRMXSS_SSC[14:12], and RTC_ALRMXSS_SSC[11:0] is to be compared
      \arg        RTC_MSKSSC_13_14: mask RTC_ALRMXSS_SSC[14:13], and RTC_ALRMXSS_SSC[12:0] is to be compared
      \arg        RTC_MSKSSC_14: mask RTC_ALRMXSS_SSC[14], and RTC_ALRMXSS_SSC[13:0] is to be compared
      \arg        RTC_MSKSSC_NONE: mask none, and RTC_ALRMXSS_SSC[14:0] is to be compared
    \param[in]  subsecond: alarm subsecond value(0x000 - 0x7FFF)
    \param[out] none
    \retval     none
*/
void rtc_alarm_subsecond_config(uint32_t mask_subsecond, uint32_t subsecond)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_ALRM0SS = mask_subsecond | subsecond;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      get RTC alarm (API_ID(0x0008U))
    \param[out] rtc_alarm_time: pointer to a rtc_alarm_struct structure which contains
                parameters for RTC alarm configuration
                members of the structure and the member values are shown as below:
                  alarm_mask: RTC_ALARM_NONE_MASK, RTC_ALARM_DATE_MASK, RTC_ALARM_HOUR_MASK
                                  RTC_ALARM_MINUTE_MASK, RTC_ALARM_SECOND_MASK, RTC_ALARM_ALL_MASK
                  weekday_or_date: RTC_ALARM_DATE_SELECTED, RTC_ALARM_WEEKDAY_SELECTED
                  alarm_day: 1) 0x1 - 0x31(BCD format) if RTC_ALARM_DATE_SELECTED is set
                                 2) RTC_MONDAY, RTC_TUESDAY, RTC_WEDNESDAY, RTC_THURSDAY, RTC_FRIDAY,
                                    RTC_SATURDAY, RTC_SUNDAY if RTC_ALARM_WEEKDAY_SELECTED is set
                  alarm_hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc display_format
                  alarm_minute: 0x0 - 0x59(BCD format)
                  alarm_second: 0x0 - 0x59(BCD format)
                  am_pm: RTC_AM, RTC_PM
    \retval     none
*/
void rtc_alarm_get(rtc_alarm_struct *rtc_alarm_time)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_RTC_VALID_POINTER(rtc_alarm_time)){
        fw_debug_report_err(RTC_MODULE_ID, API_ID(0x0008U), ERR_PARAM_POINTER);
    }else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t reg_alrmtd;

        /* get the value of RTC_ALRM0TD register */
        reg_alrmtd = RTC_ALRM0TD;
        
        /* get alarm parameters and construct the rtc_alarm_struct structure */
        rtc_alarm_time->alarm_mask = reg_alrmtd & RTC_ALARM_ALL_MASK;
        rtc_alarm_time->am_pm = (uint32_t)(reg_alrmtd & RTC_ALRMXTD_PM);
        rtc_alarm_time->weekday_or_date = (uint32_t)(reg_alrmtd & RTC_ALRMXTD_DOWS);
        rtc_alarm_time->alarm_day = (uint8_t)GET_ALRMTD_DAY(reg_alrmtd);
        rtc_alarm_time->alarm_hour = (uint8_t)GET_ALRMTD_HR(reg_alrmtd);
        rtc_alarm_time->alarm_minute = (uint8_t)GET_ALRMTD_MN(reg_alrmtd);
        rtc_alarm_time->alarm_second = (uint8_t)GET_ALRMTD_SC(reg_alrmtd);
    }
}

/*!
    \brief      get RTC alarm subsecond (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     RTC alarm subsecond value
*/
uint32_t rtc_alarm_subsecond_get(void)
{
    return ((uint32_t)(RTC_ALRM0SS & RTC_ALRM0SS_SSC));
}

/*!
    \brief      enter RTC init mode (API_ID(0x000AU))
    \param[in]  none
    \param[out] none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_init_mode_enter(void)
{
    volatile uint32_t time_index = RTC_INITM_TIMEOUT;
    uint32_t flag_status ;
    ErrStatus error_status = ERROR;

    /* check whether it has been in init mode */
    if((uint32_t)RESET == (RTC_STAT & RTC_STAT_INITF)) {
        RTC_STAT |= RTC_STAT_INITM;

        /* wait until the INITF flag to be set */
        do {
            flag_status = RTC_STAT & RTC_STAT_INITF;
        } while((--time_index > 0x00U) && ((uint32_t)RESET == flag_status));

        if((uint32_t)RESET != flag_status) {
            error_status = SUCCESS;
        }
    } else {
        error_status = SUCCESS;
    }
    return error_status;
}

/*!
    \brief      initialize RTC registers (API_ID(0x000BU))
    \param[in]  rtc_initpara_struct: pointer to a rtc_parameter_struct structure which contains
                parameters for initialization of the rtc peripheral
                members of the structure and the member values are shown as below:
                  year: 0x0 - 0x99(BCD format)
                  month: RTC_JAN, RTC_FEB, RTC_MAR, RTC_APR, RTC_MAY, RTC_JUN,
                             RTC_JUL, RTC_AUG, RTC_SEP, RTC_OCT, RTC_NOV, RTC_DEC
                  date: 0x1 - 0x31(BCD format)
                  day_of_week: RTC_MONDAY, RTC_TUESDAY, RTC_WEDNESDAY, RTC_THURSDAY
                                   RTC_FRIDAY, RTC_SATURDAY, RTC_SUNDAY
                  hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc display_format chose
                  minute: 0x0 - 0x59(BCD format)
                  second: 0x0 - 0x59(BCD format)
                  factor_asyn: 0x0 - 0x7F
                  factor_syn: 0x0 - 0x7FFF
                  am_pm: RTC_AM, RTC_PM
                  display_format: RTC_24HOUR, RTC_12HOUR
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_init(rtc_parameter_struct *rtc_initpara_struct)
{
    ErrStatus error_status = ERROR;
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_RTC_VALID_POINTER(rtc_initpara_struct)) {
        fw_debug_report_err(RTC_MODULE_ID, API_ID(0x000BU), ERR_PARAM_POINTER);
    }else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t reg_time , reg_date ;

        reg_date = (DATE_YR(rtc_initpara_struct->year) | \
                    DATE_DOW(rtc_initpara_struct->day_of_week) | \
                    DATE_MON(rtc_initpara_struct->month) | \
                    DATE_DAY(rtc_initpara_struct->date));

        reg_time = (rtc_initpara_struct->am_pm | \
                    TIME_HR(rtc_initpara_struct->hour)  | \
                    TIME_MN(rtc_initpara_struct->minute) | \
                    TIME_SC(rtc_initpara_struct->second));

        /* 1st: disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2;

        /* 2nd: enter init mode */
        error_status = rtc_init_mode_enter();

        if(ERROR != error_status) {
            RTC_PSC = (uint32_t)(PSC_FACTOR_A(rtc_initpara_struct->factor_asyn) | \
                                 PSC_FACTOR_S(rtc_initpara_struct->factor_syn));

            RTC_TIME = (uint32_t)reg_time;
            RTC_DATE = (uint32_t)reg_date;

            RTC_CTL &= (uint32_t)(~RTC_CTL_CS);
            RTC_CTL |=  rtc_initpara_struct->display_format;

            /* 3rd: exit init mode */
            rtc_init_mode_exit();

            /* 4th: wait the RSYNF flag to set */
            error_status = rtc_register_sync_wait();
        }

        /* 5th: enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
    }
    return error_status;
}

/*!
    \brief      exit RTC init mode (API_ID(0x000CU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_init_mode_exit(void)
{
    RTC_STAT &= (uint32_t)(~RTC_STAT_INITM);
}

/*!
    \brief      get current time and date (API_ID(0x000DU))
    \param[in]  none
    \param[out] rtc_initpara_struct: pointer to a rtc_parameter_struct structure which contains
                parameters for initialization of the rtc peripheral
                members of the structure and the member values are shown as below:
                  year: 0x0 - 0x99(BCD format)
                  month: RTC_JAN, RTC_FEB, RTC_MAR, RTC_APR, RTC_MAY, RTC_JUN,
                             RTC_JUL, RTC_AUG, RTC_SEP, RTC_OCT, RTC_NOV, RTC_DEC
                  date: 0x1 - 0x31(BCD format)
                  day_of_week: RTC_MONDAY, RTC_TUESDAY, RTC_WEDNESDAY, RTC_THURSDAY
                                   RTC_FRIDAY, RTC_SATURDAY, RTC_SUNDAY
                  hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc display_format chose
                  minute: 0x0 - 0x59(BCD format)
                  second: 0x0 - 0x59(BCD format)
                  factor_asyn: 0x0 - 0x7F
                  factor_syn: 0x0 - 0x7FFF
                  am_pm: RTC_AM, RTC_PM
                  display_format: RTC_24HOUR, RTC_12HOUR
    \retval     none
*/
void rtc_current_time_get(rtc_parameter_struct *rtc_initpara_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_RTC_VALID_POINTER(rtc_initpara_struct)) {
        fw_debug_report_err(RTC_MODULE_ID, API_ID(0x000DU), ERR_PARAM_POINTER);
    }else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t temp_tr, temp_dr, temp_pscr, temp_ctlr;

        temp_tr = (uint32_t)RTC_TIME;
        temp_dr = (uint32_t)RTC_DATE;
        temp_pscr = (uint32_t)RTC_PSC;
        temp_ctlr = (uint32_t)RTC_CTL;

        /* get current time and construct rtc_parameter_struct structure */
        rtc_initpara_struct->year = (uint8_t)GET_DATE_YR(temp_dr);
        rtc_initpara_struct->month = (uint8_t)GET_DATE_MON(temp_dr);
        rtc_initpara_struct->date = (uint8_t)GET_DATE_DAY(temp_dr);
        rtc_initpara_struct->day_of_week = (uint8_t)GET_DATE_DOW(temp_dr);
        rtc_initpara_struct->hour = (uint8_t)GET_TIME_HR(temp_tr);
        rtc_initpara_struct->minute = (uint8_t)GET_TIME_MN(temp_tr);
        rtc_initpara_struct->second = (uint8_t)GET_TIME_SC(temp_tr);
        rtc_initpara_struct->factor_asyn = (uint16_t)GET_PSC_FACTOR_A(temp_pscr);
        rtc_initpara_struct->factor_syn = (uint16_t)GET_PSC_FACTOR_S(temp_pscr);
        rtc_initpara_struct->am_pm = (uint32_t)(temp_tr & RTC_TIME_PM);
        rtc_initpara_struct->display_format = (uint32_t)(temp_ctlr & RTC_CTL_CS);
    }
}

/*!
    \brief      get current subsecond value (API_ID(0x000EU))
    \param[in]  none
    \param[out] none
    \retval     current subsecond value
*/
uint32_t rtc_subsecond_get(void)
{
    uint32_t reg;
    /* if BPSHAD bit is reset, reading RTC_SS will lock RTC_TIME and RTC_DATE automatically */
    reg = (uint32_t)RTC_SS;
    /* read RTC_DATE to unlock the 3 shadow registers */
    (void)(RTC_DATE);

    return reg;
}

/*!
    \brief      reset most of the RTC registers  (API_ID(0x000FU))
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_deinit(void)
{
    /* After Backup domain reset, some of the RTC registers are write-protected: RTC_TIME, RTC_DATE, RTC_PSC,
       RTC_HRFC, RTC_SHIFTCTL, the bit INITM in RTC_STAT and the bits CS, S1H, A1H, REFEN in RTC_CTL. */
    ErrStatus error_status = ERROR;;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* rtc alarmx related registers are not under the protection of RTC_WPK, different from former GD32MCU */
    RTC_CTL &= ((uint32_t)~(RTC_CTL_ALRM0EN));
    /* to write RTC_ALRMxTD and RTC_ALRMxSS register, 1  ALRMxEN bit in RTC_CTL register should be reset as the condition
      2 or in INIT mode */
    RTC_ALRM0TD = RTC_REGISTER_RESET;
    RTC_ALRM0SS = RTC_REGISTER_RESET;

    /* reset RTC_CTL register, this can be done without the init mode */
    RTC_CTL &= RTC_REGISTER_RESET;

    /* enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status) {
        /* before reset RTC_TIME and RTC_DATE, BPSHAD bit in RTC_CTL should be reset as the condition.
           in order to read calendar from shadow register, not the real registers being reset */
        RTC_TIME = RTC_REGISTER_RESET;
        RTC_DATE = RTC_DATE_RESET;

        RTC_PSC = RTC_PSC_RESET;

        /* reset RTC_STAT register, also exit init mode.
           at the same time, RTC_STAT_SOPF bit is reset, as the condition to reset RTC_SHIFTCTL register later */
        RTC_STAT = RTC_STAT_RESET;

        /* to write RTC_ALRM0SS register, ALRM0EN bit in RTC_CTL register should be reset as the condition */
        RTC_ALRM0TD = RTC_REGISTER_RESET;
        RTC_ALRM0SS = RTC_REGISTER_RESET;

        /* reset RTC_SHIFTCTL and RTC_HRFC register, and the bits S1H, A1H, REFEN in RTC_CTL, these can be done without the init mode */
        RTC_SHIFTCTL = RTC_REGISTER_RESET;
        RTC_HRFC = RTC_REGISTER_RESET;

        error_status = rtc_register_sync_wait();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    
    return error_status;
}

/*!
    \brief      adjust the daylight saving time by adding or substracting one hour from the current time (API_ID(0x0010U))
    \param[in]  operation: hour adjustment operation
                only one parameter can be selected which is shown as below:
      \arg        RTC_CTL_A1H: add one hour
      \arg        RTC_CTL_S1H: substract one hour
    \param[out] none
    \retval     none
*/
void rtc_hour_adjust(uint32_t operation)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL |= (uint32_t)((operation)&(RTC_CTL_A1H |RTC_CTL_S1H));

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}


/*!
    \brief      adjust RTC second or subsecond value of current time  (API_ID(0x0011U))
    \param[in]  add: add 1s to current time or not
                only one parameter can be selected which is shown as below:
      \arg        RTC_SHIFT_ADD1S_RESET: no effect
      \arg        RTC_SHIFT_ADD1S_SET: add 1s to current time
    \param[in]  minus: number of subsecond to minus from current time(0x0 - 0x7FFF)
    \param[out] none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_second_adjust(uint32_t add, uint32_t minus)
{
    ErrStatus error_status = ERROR;

    uint32_t time_index = RTC_SHIFTCTL_TIMEOUT;

    uint32_t flag_status;
    uint32_t temp ;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* check if a shift operation is ongoing */
    do {
        flag_status = RTC_STAT & RTC_STAT_SOPF;
    } while((--time_index > 0U) && ((uint32_t)RESET != flag_status));

    /* check if the function of reference clock detection is disabled */
    temp = RTC_CTL & RTC_CTL_REFEN;
    if((0U == flag_status) && (0U == temp)) {
        RTC_SHIFTCTL = (uint32_t)(add | SHIFTCTL_SFS(minus));
        error_status = rtc_register_sync_wait();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    return error_status;
}

/*!
    \brief      enable RTC reference clock detection function  (API_ID(0x0012U))
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_refclock_detection_enable(void)
{
    ErrStatus error_status = ERROR;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status) {
        RTC_CTL |= (uint32_t)RTC_CTL_REFEN;
        /* exit init mode */
        rtc_init_mode_exit();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      disable RTC reference clock detection function  (API_ID(0x0013U))
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_refclock_detection_disable(void)
{
    ErrStatus error_status = ERROR;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status) {
        RTC_CTL &= (uint32_t)~RTC_CTL_REFEN;
        /* exit init mode */
        rtc_init_mode_exit();
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      configure RTC smooth calibration  (API_ID(0x0014U))
    \param[in]  window: select calibration window
      \arg        RTC_CALIBRATION_WINDOW_32S: 2exp20 RTCCLK cycles, 32s if RTCCLK = 32768 Hz
      \arg        RTC_CALIBRATION_WINDOW_16S: 2exp19 RTCCLK cycles, 16s if RTCCLK = 32768 Hz
      \arg        RTC_CALIBRATION_WINDOW_8S: 2exp18 RTCCLK cycles, 8s if RTCCLK = 32768 Hz
    \param[in]  plus: add RTC clock or not
      \arg        RTC_CALIBRATION_PLUS_SET: add one RTC clock every 2048 rtc clock
      \arg        RTC_CALIBRATION_PLUS_RESET: no effect
    \param[in]  minus: the RTC clock to minus during the calibration window(0x0 - 0x1FF)
    \param[out] none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtc_smooth_calibration_config(uint32_t window, uint32_t plus, uint32_t smooth_minus)
{
    ErrStatus error_status = ERROR;

    volatile uint32_t time_index = RTC_HRFC_TIMEOUT;
    uint32_t flag_status;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* check if a smooth calibration operation is ongoing */
    do {
        flag_status = RTC_STAT & RTC_STAT_SCPF;
    } while((--time_index > 0U) && ((uint32_t)RESET != flag_status));

    if((uint32_t)RESET == flag_status) {
        RTC_HRFC = (uint32_t)(window | plus | HRFC_CMSK(smooth_minus));
        error_status = SUCCESS;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return error_status;
}

/*!
    \brief      enable RTC time-stamp  (API_ID(0x0015U))
    \param[in]  edge: specify which edge to detect of time-stamp
                only one parameter can be selected which is shown as below:
      \arg        RTC_TIMESTAMP_RISING_EDGE: rising edge is valid event edge for timestamp event
      \arg        RTC_TIMESTAMP_FALLING_EDGE: falling edge is valid event edge for timestamp event
    \param[out] none
    \retval     none
*/
void rtc_timestamp_enable(uint32_t edge)
{
    uint32_t reg_ctl;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* clear the bits to be configured in RTC_CTL */
    reg_ctl = (uint32_t)(RTC_CTL & (uint32_t)(~(RTC_CTL_TSEG | RTC_CTL_TSEN)));

    /* new configuration */
    reg_ctl |= (uint32_t)((edge&(RTC_CTL_TSEG)) | RTC_CTL_TSEN);

    RTC_CTL = (uint32_t)reg_ctl;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC time-stamp  (API_ID(0x0016U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_timestamp_disable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* clear the TSEN bit */
    RTC_CTL &= (uint32_t)(~ RTC_CTL_TSEN);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      get RTC timestamp time and date (API_ID(0x0017U))
    \param[in]  none
    \param[out] rtc_timestamp: pointer to a rtc_timestamp_struct structure which contains
                parameters for RTC time-stamp configuration
                members of the structure and the member values are shown as below:
                  timestamp_month: RTC_JAN, RTC_FEB, RTC_MAR, RTC_APR, RTC_MAY, RTC_JUN,
                                       RTC_JUL, RTC_AUG, RTC_SEP, RTC_OCT, RTC_NOV, RTC_DEC
                  timestamp_date: 0x1 - 0x31(BCD format)
                  timestamp_day: RTC_MONDAY, RTC_TUESDAY, RTC_WEDNESDAY, RTC_THURSDAY, RTC_FRIDAY,
                                     RTC_SATURDAY, RTC_SUNDAY
                  timestamp_hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc display_format
                  timestamp_minute: 0x0 - 0x59(BCD format)
                  timestamp_second: 0x0 - 0x59(BCD format)
                  am_pm: RTC_AM, RTC_PM
    \retval     none
*/
void rtc_timestamp_get(rtc_timestamp_struct *rtc_timestamp)
{
#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_RTC_VALID_POINTER(rtc_timestamp)) {
        fw_debug_report_err(RTC_MODULE_ID, API_ID(0x0017U), ERR_PARAM_POINTER);
    }else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        uint32_t temp_tts, temp_dts;

        /* get the value of time_stamp registers */
        temp_tts = (uint32_t)RTC_TTS;
        temp_dts = (uint32_t)RTC_DTS;

        /* get timestamp time and construct the rtc_timestamp_struct structure */
        rtc_timestamp->am_pm = (uint32_t)(temp_tts & RTC_TTS_PM);
        rtc_timestamp->timestamp_month = (uint8_t)GET_DTS_MON(temp_dts);
        rtc_timestamp->timestamp_date = (uint8_t)GET_DTS_DAY(temp_dts);
        rtc_timestamp->timestamp_day = (uint8_t)GET_DTS_DOW(temp_dts);
        rtc_timestamp->timestamp_hour = (uint8_t)GET_TTS_HR(temp_tts);
        rtc_timestamp->timestamp_minute = (uint8_t)GET_TTS_MN(temp_tts);
        rtc_timestamp->timestamp_second = (uint8_t)GET_TTS_SC(temp_tts);
    }
}

/*!
    \brief      get RTC time-stamp subsecond (API_ID(0x0018U))
    \param[in]  none
    \param[out] none
    \retval     RTC time-stamp subsecond value
*/
uint32_t rtc_timestamp_subsecond_get(void)
{
    return ((uint32_t)RTC_SSTS);
}

/*!
    \brief      configure rtc calibration output source (API_ID(0x0019U))
    \param[in]  source: specify signal to output
      \arg        RTC_CALIBRATION_512HZ: when the LSE freqency is 32768Hz and the RTC_PSC
                                         is the default value, output 512Hz signal
      \arg        RTC_CALIBRATION_1HZ: when the LSE freqency is 32768Hz and the RTC_PSC
                                       is the default value, output 1Hz signal
    \param[out] none
    \retval     none
*/
void rtc_calibration_output_config(uint32_t source)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint32_t)~(RTC_CTL_COEN | RTC_CTL_COS);

    RTC_CTL |= (uint32_t)(source&((RTC_CTL_COEN | RTC_CTL_COS)));
    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      configure rtc alarm output source (API_ID(0x001AU))
    \param[in]  source: specify signal to output
                only one parameter can be selected which is shown as below:
      \arg        RTC_ALARM0_HIGH: when the  alarm0 flag is set, the output pin is high
      \arg        RTC_ALARM0_LOW: when the  alarm0 flag is set, the output pin is low
    \param[in]  mode: specify the output pin mode when output alarm signal
      \arg        RTC_ALARM_OUTPUT_OD: open drain mode
      \arg        RTC_ALARM_OUTPUT_PP: push pull mode
    \param[out] none
    \retval     none
*/
void rtc_alarm_output_config(uint32_t alarm_output, uint32_t mode)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= ~(RTC_CTL_OS | RTC_CTL_OPOL);
    RTC_TYPE &= ~RTC_TYPE_ALRMOUTTYPE;

    RTC_CTL |= (uint32_t)(alarm_output &((RTC_CTL_OS | RTC_CTL_OPOL)) );
    /* alarm output */
    RTC_TYPE |= (uint32_t)(mode & RTC_TYPE_ALRMOUTTYPE);
    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      select the RTC output pin (API_ID(0x001BU))
    \param[in]  outputpin: specify the rtc output pin is PC13 or not
      \arg        RTC_OUT1_DISABLE: the rtc output pin is RTC_OUT0
      \arg        RTC_OUT1_ENABLE: the rtc output pin is RTC_OUT1
    \param[out] none
    \retval     none
*/
void rtc_output_pin_select(uint32_t outputpin)
{
    uint32_t ctl_tmp = RTC_CTL;
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    ctl_tmp &= (uint32_t)~(RTC_CTL_OUT1EN);
    ctl_tmp |= (uint32_t)(outputpin & RTC_CTL_OUT1EN);
    RTC_CTL = ctl_tmp;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      enable specified RTC interrupt (API_ID(0x001CU))
    \param[in]  interrupt: specify which interrupt source to be enabled
                only one parameter can be selected which is shown as below:
      \arg        RTC_INT_TIMESTAMP: timestamp interrupt
      \arg        RTC_INT_ALARM0: alarm0 interrupt
    \param[out] none
    \retval     none
*/
void rtc_interrupt_enable(rtc_interrupt_flag_enum interrupt)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enable the interrupts in RTC_CTL register */
    RTC_CTL |= (uint32_t)(interrupt);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disble specified RTC interrupt (API_ID(0x001DU))
    \param[in]  interrupt: specify which interrupt source to be disabled
                only one parameter can be selected which is shown as below:
      \arg        RTC_INT_TIMESTAMP: timestamp interrupt
      \arg        RTC_INT_ALARM0: alarm0 interrupt
    \param[out] none
    \retval     none
*/
void rtc_interrupt_disable(rtc_interrupt_flag_enum interrupt)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* disable the interrupts in RTC_CTL register */
    RTC_CTL &= ~(uint32_t)(interrupt);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      check specified flag (API_ID(0x001EU))
    \param[in]  flag: specify which flag to check
                only one parameter can be selected which is shown as below:
      \arg        RTC_FLAG_SCP: smooth calibration pending flag
      \arg        RTC_FLAG_TSOVR: time-stamp overflow flag
      \arg        RTC_FLAG_TS: time-stamp flag
      \arg        RTC_FLAG_ALARM0: alarm0 occurs flag
      \arg        RTC_FLAG_INIT: initialization state flag
      \arg        RTC_FLAG_RSYN: register synchronization flag
      \arg        RTC_FLAG_YCM: year configuration mark status flag
      \arg        RTC_FLAG_SOP: shift function operation pending flag
      \arg        RTC_FLAG_ALARM0W: alarm0 configuration can be written flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rtc_flag_get(rtc_flag_enum flag)
{
    FlagStatus flag_state = RESET;

    if((uint32_t)RESET != (RTC_STAT & (uint32_t)flag)) {
        flag_state = SET;
    }
    return flag_state;
}

/*!
    \brief      clear specified flag (API_ID(0x001FU))
    \param[in]  flag_clear: specify which flag to clear
                only one parameter can be selected which is shown as below:
      \arg        RTC_FLAG_TSOVR: time-stamp overflow flag
      \arg        RTC_FLAG_TS: time-stamp flag
      \arg        RTC_FLAG_ALARM0: alarm0 occurs flag
      \arg        RTC_FLAG_RSYN: register synchronization flag
    \param[out] none
    \retval     none
*/
void rtc_flag_clear(rtc_flag_enum flag_clear)
{
    RTC_STAT &= ~(uint32_t)(flag_clear);
}