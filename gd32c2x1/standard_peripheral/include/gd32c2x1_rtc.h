/*!
    \file    gd32c2x1_rtc.h
    \brief   definitions for the RTC

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

#ifndef GD32C2X1_RTC_H
#define GD32C2X1_RTC_H

#include "gd32c2x1.h"

/* RTC definitions */
#define RTC                                RTC_BASE

/* registers definitions */
#define RTC_TIME                           REG32((RTC) + 0x00000000U)                  /*!< RTC time of day register */
#define RTC_DATE                           REG32((RTC) + 0x00000004U)                  /*!< RTC date register */
#define RTC_CTL                            REG32((RTC) + 0x00000008U)                  /*!< RTC control register */
#define RTC_STAT                           REG32((RTC) + 0x0000000CU)                  /*!< RTC status register */
#define RTC_PSC                            REG32((RTC) + 0x00000010U)                  /*!< RTC time prescaler register */
#define RTC_ALRM0TD                        REG32((RTC) + 0x0000001CU)                  /*!< RTC alarm 0 time and date register */
#define RTC_WPK                            REG32((RTC) + 0x00000024U)                  /*!< RTC write protection key register */
#define RTC_SS                             REG32((RTC) + 0x00000028U)                  /*!< RTC sub second register */
#define RTC_SHIFTCTL                       REG32((RTC) + 0x0000002CU)                  /*!< RTC shift function control register */
#define RTC_TTS                            REG32((RTC) + 0x00000030U)                  /*!< RTC time of timestamp register */
#define RTC_DTS                            REG32((RTC) + 0x00000034U)                  /*!< RTC date of timestamp register */
#define RTC_SSTS                           REG32((RTC) + 0x00000038U)                  /*!< RTC sub second of timestamp register */
#define RTC_HRFC                           REG32((RTC) + 0x0000003CU)                  /*!< RTC high resolution frequency compensation register */
#define RTC_TYPE                           REG32((RTC) + 0x00000040U)                  /*!< RTC type register */
#define RTC_ALRM0SS                        REG32((RTC) + 0x00000044U)                  /*!< RTC alarm 0 sub second register */
#define RTC_BKP0                           REG32((RTC) + 0x00000050U)                  /*!< RTC backup 0 register */
#define RTC_BKP1                           REG32((RTC) + 0x00000054U)                  /*!< RTC backup 1 register */
#define RTC_BKP2                           REG32((RTC) + 0x00000058U)                  /*!< RTC backup 2 register */
#define RTC_BKP3                           REG32((RTC) + 0x0000005CU)                  /*!< RTC backup 3 register */

/* bits definitions */
/* RTC_TIME */
#define RTC_TIME_SCU                       BITS(0,3)                                   /*!< second units in BCD code */
#define RTC_TIME_SCT                       BITS(4,6)                                   /*!< second tens in BCD code */
#define RTC_TIME_MNU                       BITS(8,11)                                  /*!< minute units in BCD code */
#define RTC_TIME_MNT                       BITS(12,14)                                 /*!< minute tens in BCD code */
#define RTC_TIME_HRU                       BITS(16,19)                                 /*!< hour units in BCD code */
#define RTC_TIME_HRT                       BITS(20,21)                                 /*!< hour tens in BCD code */
#define RTC_TIME_PM                        BIT(22)                                     /*!< AM/PM notation */

/* RTC_DATE */
#define RTC_DATE_DAYU                      BITS(0,3)                                   /*!< date units in BCD code */
#define RTC_DATE_DAYT                      BITS(4,5)                                   /*!< date tens in BCD code */
#define RTC_DATE_MONU                      BITS(8,11)                                  /*!< month units in BCD code */
#define RTC_DATE_MONT                      BIT(12)                                     /*!< month tens in BCD code */
#define RTC_DATE_DOW                       BITS(13,15)                                 /*!< day of week units */
#define RTC_DATE_YRU                       BITS(16,19)                                 /*!< year units in BCD code */
#define RTC_DATE_YRT                       BITS(20,23)                                 /*!< year tens in BCD code */

/* RTC_CTL */
#define RTC_CTL_TSEG                       BIT(3)                                      /*!< valid event edge of time-stamp */
#define RTC_CTL_REFEN                      BIT(4)                                      /*!< reference clock detection function enable */
#define RTC_CTL_BPSHAD                     BIT(5)                                      /*!< shadow registers bypass control */
#define RTC_CTL_CS                         BIT(6)                                      /*!< display format of clock system */
#define RTC_CTL_ALRM0EN                    BIT(8)                                      /*!< alarm0 function enable */
#define RTC_CTL_TSEN                       BIT(11)                                     /*!< time-stamp function enable */
#define RTC_CTL_ALRM0IE                    BIT(12)                                     /*!< RTC alarm0 interrupt enable */
#define RTC_CTL_TSIE                       BIT(15)                                     /*!< time-stamp interrupt enable */
#define RTC_CTL_A1H                        BIT(16)                                     /*!< add 1 hour(summer time change) */
#define RTC_CTL_S1H                        BIT(17)                                     /*!< subtract 1 hour(winter time change) */
#define RTC_CTL_DSM                        BIT(18)                                     /*!< daylight saving mark */
#define RTC_CTL_COS                        BIT(19)                                     /*!< calibration output selection */
#define RTC_CTL_OPOL                       BIT(20)                                     /*!< output polarity */
#define RTC_CTL_OS                         BITS(21,22)                                 /*!< output selection */
#define RTC_CTL_COEN                       BIT(23)                                     /*!< calibration output enable */
#define RTC_CTL_OUT1EN                     BIT(31)                                     /*!< RTC_OUT1 pin enable */

/* RTC_STAT */
#define RTC_STAT_ALRM0WF                   BIT(0)                                      /*!< alarm0 configuration can be write flag */
#define RTC_STAT_SOPF                      BIT(3)                                      /*!< shift function operation pending flag */
#define RTC_STAT_YCM                       BIT(4)                                      /*!< year configuration mark status flag */
#define RTC_STAT_RSYNF                     BIT(5)                                      /*!< register synchronization flag */
#define RTC_STAT_INITF                     BIT(6)                                      /*!< initialization state flag */
#define RTC_STAT_INITM                     BIT(7)                                      /*!< enter initialization mode */
#define RTC_STAT_ALRM0F                    BIT(8)                                      /*!< alarm0 occurs flag */
#define RTC_STAT_TSF                       BIT(11)                                     /*!< time-stamp flag */
#define RTC_STAT_TSOVRF                    BIT(12)                                     /*!< time-stamp overflow flag */
#define RTC_STAT_SCPF                      BIT(16)                                     /*!< Smooth calibration pending flag */

/* RTC_PSC */
#define RTC_PSC_FACTOR_S                   BITS(0,14)                                  /*!< synchronous prescaler factor */
#define RTC_PSC_FACTOR_A                   BITS(16,22)                                 /*!< asynchronous prescaler factor */

/* RTC_ALRMxTD */
#define RTC_ALRMXTD_SCU                    BITS(0,3)                                   /*!< second units in BCD code */
#define RTC_ALRMXTD_SCT                    BITS(4,6)                                   /*!< second tens in BCD code */
#define RTC_ALRMXTD_MSKS                   BIT(7)                                      /*!< alarm second mask bit */
#define RTC_ALRMXTD_MNU                    BITS(8,11)                                  /*!< minutes units in BCD code */
#define RTC_ALRMXTD_MNT                    BITS(12,14)                                 /*!< minutes tens in BCD code */
#define RTC_ALRMXTD_MSKM                   BIT(15)                                     /*!< alarm minutes mask bit */
#define RTC_ALRMXTD_HRU                    BITS(16,19)                                 /*!< hour units in BCD code */
#define RTC_ALRMXTD_HRT                    BITS(20,21)                                 /*!< hour tens in BCD code */
#define RTC_ALRMXTD_PM                     BIT(22)                                     /*!< AM/PM flag */
#define RTC_ALRMXTD_MSKH                   BIT(23)                                     /*!< alarm hour mask bit */
#define RTC_ALRMXTD_DAYU                   BITS(24,27)                                 /*!< date units or week day in BCD code */
#define RTC_ALRMXTD_DAYT                   BITS(28,29)                                 /*!< date tens in BCD code */
#define RTC_ALRMXTD_DOWS                   BIT(30)                                     /*!< day of week  selection */
#define RTC_ALRMXTD_MSKD                   BIT(31)                                     /*!< alarm date mask bit */

/* RTC_WPK */
#define RTC_WPK_WPK                        BITS(0,7)                                   /*!< key for write protection */

/* RTC_SS */
#define RTC_SS_SSC                         BITS(0,15)                                  /*!< sub second value */

/* RTC_SHIFTCTL */
#define RTC_SHIFTCTL_SFS                   BITS(0,14)                                  /*!< subtract a fraction of a second */
#define RTC_SHIFTCTL_A1S                   BIT(31)                                     /*!< one second add */

/* RTC_TTS */
#define RTC_TTS_SCU                        BITS(0,3)                                   /*!< second units in BCD code */
#define RTC_TTS_SCT                        BITS(4,6)                                   /*!< second tens in BCD code */
#define RTC_TTS_MNU                        BITS(8,11)                                  /*!< minute units in BCD code */
#define RTC_TTS_MNT                        BITS(12,14)                                 /*!< minute tens in BCD code */
#define RTC_TTS_HRU                        BITS(16,19)                                 /*!< hour units in BCD code */
#define RTC_TTS_HRT                        BITS(20,21)                                 /*!< hour tens in BCD code */
#define RTC_TTS_PM                         BIT(22)                                     /*!< AM/PM notation */

/* RTC_DTS */
#define RTC_DTS_DAYU                       BITS(0,3)                                   /*!< date units in BCD code */
#define RTC_DTS_DAYT                       BITS(4,5)                                   /*!< date tens in BCD code */
#define RTC_DTS_MONU                       BITS(8,11)                                  /*!< month units in BCD code */
#define RTC_DTS_MONT                       BIT(12)                                     /*!< month tens in BCD code */
#define RTC_DTS_DOW                        BITS(13,15)                                 /*!< day of week units */

/* RTC_SSTS */
#define RTC_SSTS_SSC                       BITS(0,15)                                  /*!< timestamp sub second units */

/* RTC_HRFC */
#define RTC_HRFC_CMSK                      BITS(0,8)                                   /*!< calibration mask number */
#define RTC_HRFC_CWND16                    BIT(13)                                     /*!< calibration window select 16 seconds */
#define RTC_HRFC_CWND8                     BIT(14)                                     /*!< calibration window select 8 seconds */
#define RTC_HRFC_FREQI                     BIT(15)                                     /*!< increase RTC frequency by 488.5ppm */

/* RTC_TYPE */
#define RTC_TYPE_DISPU                     BIT(15)                                     /*!< RTC tamp x pull up disable bit */
#define RTC_TYPE_ALRMOUTTYPE               BIT(18)                                     /*!< RTC_ALARM output Type */

/* RTC_ALRM0SS */
#define RTC_ALRM0SS_SSC                    BITS(0,14)                                  /*!< alarm0 sub second value */
#define RTC_ALRM0SS_MASKSSC                BITS(24,27)                                 /*!< mask control bit of SS */

/* RTC_BKP0 */
#define RTC_BKP0_DATA                      BITS(0,15)                                  /*!< backup domain registers */

/* RTC_BKP1 */
#define RTC_BKP1_DATA                      BITS(0,15)                                  /*!< backup domain registers */

/* RTC_BKP2 */
#define RTC_BKP2_DATA                      BITS(0,15)                                  /*!< backup domain registers */

/* RTC_BKP3 */
#define RTC_BKP3_DATA                      BITS(0,15)                                  /*!< backup domain registers */

/* constants definitions */
/* structure for initialization of the RTC */
typedef struct {
    uint8_t year;                                                                      /*!< RTC year value: 0x0 - 0x99(BCD format) */
    uint8_t month;                                                                     /*!< RTC month value */
    uint8_t date;                                                                      /*!< RTC date value: 0x1 - 0x31(BCD format) */
    uint8_t day_of_week;                                                               /*!< RTC weekday value */
    uint8_t hour;                                                                      /*!< RTC hour value */
    uint8_t minute;                                                                    /*!< RTC minute value: 0x0 - 0x59(BCD format) */
    uint8_t second;                                                                    /*!< RTC second value: 0x0 - 0x59(BCD format) */
    uint16_t factor_asyn;                                                              /*!< RTC asynchronous prescaler value: 0x0 - 0x7F */
    uint16_t factor_syn;                                                               /*!< RTC synchronous prescaler value: 0x0 - 0x7FFF */
    uint32_t am_pm;                                                                    /*!< RTC AM/PM value */
    uint32_t display_format;                                                           /*!< RTC time notation */
} rtc_parameter_struct;

/* structure for RTC alarm configuration */
typedef struct {
    uint32_t alarm_mask;                                                               /*!< RTC alarm mask */
    uint32_t weekday_or_date;                                                          /*!< specify RTC alarm is on date or weekday */
    uint8_t alarm_day;                                                                 /*!< RTC alarm date or weekday value*/
    uint8_t alarm_hour;                                                                /*!< RTC alarm hour value */
    uint8_t alarm_minute;                                                              /*!< RTC alarm minute value: 0x0 - 0x59(BCD format) */
    uint8_t alarm_second;                                                              /*!< RTC alarm second value: 0x0 - 0x59(BCD format) */
    uint32_t am_pm;                                                                    /*!< RTC alarm AM/PM value */
} rtc_alarm_struct;

/* structure for RTC time-stamp configuration */
typedef struct {
    uint8_t timestamp_month;                                                           /*!< RTC time-stamp month value */
    uint8_t timestamp_date;                                                            /*!< RTC time-stamp date value: 0x1 - 0x31(BCD format) */
    uint8_t timestamp_day;                                                             /*!< RTC time-stamp weekday value */
    uint8_t timestamp_hour;                                                            /*!< RTC time-stamp hour value */
    uint8_t timestamp_minute;                                                          /*!< RTC time-stamp minute value: 0x0 - 0x59(BCD format) */
    uint8_t timestamp_second;                                                          /*!< RTC time-stamp second value: 0x0 - 0x59(BCD format) */
    uint32_t am_pm;                                                                    /*!< RTC time-stamp AM/PM value */
} rtc_timestamp_struct;

/* time register value */
#define TIME_SC(regval)                    (BITS(0,6) & ((uint32_t)(regval) << 0))     /*!< write value to RTC_TIME_SC bit field */
#define GET_TIME_SC(regval)                GET_BITS((regval),0,6)                      /*!< get value of RTC_TIME_SC bit field */

#define TIME_MN(regval)                    (BITS(8,14) & ((uint32_t)(regval) << 8))    /*!< write value to RTC_TIME_MN bit field */
#define GET_TIME_MN(regval)                GET_BITS((regval),8,14)                     /*!< get value of RTC_TIME_MN bit field */

#define TIME_HR(regval)                    (BITS(16,21) & ((uint32_t)(regval) << 16))  /*!< write value to RTC_TIME_HR bit field */
#define GET_TIME_HR(regval)                GET_BITS((regval),16,21)                    /*!< get value of RTC_TIME_HR bit field */

#define RTC_AM                             ((uint32_t)0x00000000U)                     /*!< AM format */
#define RTC_PM                             RTC_TIME_PM                                 /*!< PM format */

/* date register value */
#define DATE_DAY(regval)                   (BITS(0,5) & ((uint32_t)(regval) << 0))     /*!< write value to RTC_DATE_DAY bit field */
#define GET_DATE_DAY(regval)               GET_BITS((regval),0,5)                      /*!< get value of RTC_DATE_DAY bit field */

#define DATE_MON(regval)                   (BITS(8,12) & ((uint32_t)(regval) << 8))    /*!< write value to RTC_DATE_MON bit field */
#define GET_DATE_MON(regval)               GET_BITS((regval),8,12)                     /*!< get value of RTC_DATE_MON bit field */
#define RTC_JAN                            ((uint8_t)0x01U)                            /*!< Janurary */
#define RTC_FEB                            ((uint8_t)0x02U)                            /*!< February */
#define RTC_MAR                            ((uint8_t)0x03U)                            /*!< March */
#define RTC_APR                            ((uint8_t)0x04U)                            /*!< April */
#define RTC_MAY                            ((uint8_t)0x05U)                            /*!< May */
#define RTC_JUN                            ((uint8_t)0x06U)                            /*!< June */
#define RTC_JUL                            ((uint8_t)0x07U)                            /*!< July */
#define RTC_AUG                            ((uint8_t)0x08U)                            /*!< August */
#define RTC_SEP                            ((uint8_t)0x09U)                            /*!< September */
#define RTC_OCT                            ((uint8_t)0x10U)                            /*!< October */
#define RTC_NOV                            ((uint8_t)0x11U)                            /*!< November */
#define RTC_DEC                            ((uint8_t)0x12U)                            /*!< December */

#define DATE_DOW(regval)                   (BITS(13,15) & ((uint32_t)(regval) << 13))  /*!< write value to RTC_DATE_DOW bit field */
#define GET_DATE_DOW(regval)               GET_BITS((uint32_t)(regval),13,15)          /*!< get value of RTC_DATE_DOW bit field */
#define RTC_MONDAY                         ((uint8_t)0x01)                             /*!< monday */
#define RTC_TUESDAY                        ((uint8_t)0x02)                             /*!< tuesday */
#define RTC_WEDNESDAY                      ((uint8_t)0x03)                             /*!< wednesday */
#define RTC_THURSDAY                       ((uint8_t)0x04)                             /*!< thursday */
#define RTC_FRIDAY                         ((uint8_t)0x05)                             /*!< friday */
#define RTC_SATURDAY                       ((uint8_t)0x06)                             /*!< saturday */
#define RTC_SUNDAY                         ((uint8_t)0x07)                             /*!< sunday */

#define DATE_YR(regval)                    (BITS(16,23) & ((uint32_t)(regval) << 16))  /*!< write value to RTC_DATE_YR bit field */
#define GET_DATE_YR(regval)                GET_BITS((regval),16,23)                    /*!< get value of RTC_DATE_YR bit field */

#define RTC_OUT1_DISABLE                   ((uint32_t)0x00000000U)                     /*!< RTC_OUT1 disable */
#define RTC_OUT1_ENABLE                    RTC_CTL_OUT1EN                              /*!< RTC_OUT1 enable */

#define CTL_OS(regval)                     (BITS(21,22) & ((uint32_t)(regval) << 21))  /*!< write value to RTC_CTL_OS bit field */
#define RTC_OS_DISABLE                     CTL_OS(0)                                   /*!< disable output RTC_ALARM */
#define RTC_OS_ALARM0                      CTL_OS(1)                                   /*!< enable alarm0 flag output */

#define RTC_CALIBRATION_512HZ              RTC_CTL_COEN                                /*!< calibration output of 512Hz is enable */
#define RTC_CALIBRATION_1HZ                (RTC_CTL_COEN | RTC_CTL_COS)                /*!< calibration output of 1Hz is enable */
#define RTC_ALARM0_HIGH                    RTC_OS_ALARM0                               /*!< enable alarm0 flag output with high level */
#define RTC_ALARM0_LOW                     (RTC_OS_ALARM0 | RTC_CTL_OPOL)              /*!< enable alarm0 flag output with low level*/

#define RTC_24HOUR                         ((uint32_t)0x00000000U)                     /*!< 24-hour format */
#define RTC_12HOUR                         RTC_CTL_CS                                  /*!< 12-hour format */

#define RTC_TIMESTAMP_RISING_EDGE          ((uint32_t)0x00000000U)                     /*!< rising edge is valid event edge for time-stamp event */
#define RTC_TIMESTAMP_FALLING_EDGE         RTC_CTL_TSEG                                /*!< falling edge is valid event edge for time-stamp event */

/* psc register value */
#define PSC_FACTOR_S(regval)               (BITS(0,14) & ((uint32_t)(regval) << 0))    /*!< write value to RTC_PSC_FACTOR_S bit field */
#define GET_PSC_FACTOR_S(regval)           GET_BITS((regval),0,14)                     /*!< get value of RTC_PSC_FACTOR_S bit field */

#define PSC_FACTOR_A(regval)               (BITS(16,22) & ((uint32_t)(regval) << 16))  /*!< write value to RTC_PSC_FACTOR_A bit field */
#define GET_PSC_FACTOR_A(regval)           GET_BITS((regval),16,22)                    /*!< get value of RTC_PSC_FACTOR_A bit field */

/* alrmtd register value */
#define ALRMTD_SC(regval)                  (BITS(0,6) & ((uint32_t)(regval)<< 0))      /*!< write value to RTC_ALRMTD_SC bit field */
#define GET_ALRMTD_SC(regval)              GET_BITS((regval),0,6)                      /*!< get value of RTC_ALRMTD_SC bit field */

#define ALRMTD_MN(regval)                  (BITS(8,14) & ((uint32_t)(regval) << 8))    /*!< write value to RTC_ALRMTD_MN bit field */
#define GET_ALRMTD_MN(regval)              GET_BITS((regval),8,14)                     /*!< get value of RTC_ALRMTD_MN bit field */

#define ALRMTD_HR(regval)                  (BITS(16,21) & ((uint32_t)(regval) << 16))  /*!< write value to RTC_ALRMTD_HR bit field */
#define GET_ALRMTD_HR(regval)              GET_BITS((regval),16,21)                    /*!< get value of RTC_ALRMTD_HR bit field */

#define ALRMTD_DAY(regval)                 (BITS(24,29) & ((uint32_t)(regval) << 24))  /*!< write value to RTC_ALRMTD_DAY bit field */
#define GET_ALRMTD_DAY(regval)             GET_BITS((regval),24,29)                    /*!< get value of RTC_ALRMTD_DAY bit field */

#define RTC_ALARM_NONE_MASK                ((uint32_t)0x00000000U)                     /*!< alarm none mask */
#define RTC_ALARM_DATE_MASK                RTC_ALRMXTD_MSKD                            /*!< alarm date mask */
#define RTC_ALARM_HOUR_MASK                RTC_ALRMXTD_MSKH                            /*!< alarm hour mask */
#define RTC_ALARM_MINUTE_MASK              RTC_ALRMXTD_MSKM                            /*!< alarm minute mask */
#define RTC_ALARM_SECOND_MASK              RTC_ALRMXTD_MSKS                            /*!< alarm second mask */
#define RTC_ALARM_ALL_MASK                 (RTC_ALRMXTD_MSKD|RTC_ALRMXTD_MSKH|RTC_ALRMXTD_MSKM|RTC_ALRMXTD_MSKS)   /*!< alarm all mask */

#define RTC_ALARM_DATE_SELECTED            ((uint32_t)0x00000000U)                     /*!< alarm date format selected */
#define RTC_ALARM_WEEKDAY_SELECTED         RTC_ALRMXTD_DOWS                            /*!< alarm weekday format selected */

/* wpk register value */
#define WPK_WPK(regval)                    (BITS(0,7) & ((uint32_t)(regval) << 0))     /*!< write value to RTC_WPK_WPK bit field */

/* ss register value */
#define SS_SSC(regval)                     (BITS(0,15) & ((uint32_t)(regval) << 0))    /*!< write value to RTC_SS_SSC bit field */

/* shiftctl register value */
#define SHIFTCTL_SFS(regval)               (BITS(0,14) & ((uint32_t)(regval) << 0))    /*!< write value to RTC_SHIFTCTL_SFS bit field */

#define RTC_SHIFT_ADD1S_RESET              ((uint32_t)0x00000000U)                     /*!< not add 1 second */
#define RTC_SHIFT_ADD1S_SET                RTC_SHIFTCTL_A1S                            /*!< add one second to the clock */

/* tts register value */
#define TTS_SC(regval)                     (BITS(0,6) & ((uint32_t)(regval) << 0))     /*!< write value to RTC_TTS_SC bit field */
#define GET_TTS_SC(regval)                 GET_BITS((regval),0,6)                      /*!< get value of RTC_TTS_SC bit field */

#define TTS_MN(regval)                     (BITS(8,14) & ((uint32_t)(regval) << 8))    /*!< write value to RTC_TTS_MN bit field */
#define GET_TTS_MN(regval)                 GET_BITS((regval),8,14)                     /*!< get value of RTC_TTS_MN bit field */

#define TTS_HR(regval)                     (BITS(16,21) & ((uint32_t)(regval) << 16))  /*!< write value to RTC_TTS_HR bit field */
#define GET_TTS_HR(regval)                 GET_BITS((regval),16,21)                    /*!< get value of RTC_TTS_HR bit field */

/* dts register value */
#define DTS_DAY(regval)                    (BITS(0,5) & ((uint32_t)(regval) << 0))     /*!< write value to RTC_DTS_DAY bit field */
#define GET_DTS_DAY(regval)                GET_BITS((regval),0,5)                      /*!< get value of RTC_DTS_DAY bit field */

#define DTS_MON(regval)                    (BITS(8,12) & ((uint32_t)(regval) << 8))    /*!< write value to RTC_DTS_MON bit field */
#define GET_DTS_MON(regval)                GET_BITS((regval),8,12)                     /*!< get value of RTC_DTS_MON bit field */

#define DTS_DOW(regval)                    (BITS(13,15) & ((uint32_t)(regval) << 13))  /*!< write value to RTC_DTS_DOW bit field */
#define GET_DTS_DOW(regval)                GET_BITS((regval),13,15)                    /*!< get value of RTC_DTS_DOW bit field */

/* ssts register value */
#define SSTS_SSC(regval)                   (BITS(0,15) & ((uint32_t)(regval) << 0))    /*!< write value to RTC_SSTS_SSC bit field */

/* hrfc register value */
#define HRFC_CMSK(regval)                  (BITS(0,8) & ((uint32_t)(regval) << 0))     /*!< write value to RTC_HRFC_CMSK bit field */

#define RTC_CALIBRATION_WINDOW_32S         ((uint32_t)0x00000000U)                     /*!< 2exp20 RTCCLK cycles, 32s if RTCCLK = 32768 Hz */
#define RTC_CALIBRATION_WINDOW_16S         RTC_HRFC_CWND16                             /*!< 2exp19 RTCCLK cycles, 16s if RTCCLK = 32768 Hz */
#define RTC_CALIBRATION_WINDOW_8S          RTC_HRFC_CWND8                              /*!< 2exp18 RTCCLK cycles, 8s if RTCCLK = 32768 Hz */

#define RTC_CALIBRATION_PLUS_SET           RTC_HRFC_FREQI                              /*!< increase RTC frequency by 488.5ppm */
#define RTC_CALIBRATION_PLUS_RESET         ((uint32_t)0x00000000U)                     /*!< no effect */

#define RTC_ALARM_OUTPUT_OD                ((uint32_t)0x00000000U)                     /*!< RTC alarm output open-drain mode */
#define RTC_ALARM_OUTPUT_PP                RTC_TYPE_ALRMOUTTYPE                        /*!< RTC alarm output push-pull mode */

/* alrm0ss register value */
#define ALRMXSS_SSC(regval)                (BITS(0,14) & ((uint32_t)(regval)<< 0))     /*!< write value to RTC_ALRMXSS_SSC bit field */

#define ALRMXSS_MSKSSC(regval)             (BITS(24,27) & ((uint32_t)(regval) << 24))  /*!< write value to RTC_ALRMXSS_MSKSSC bit field */
#define RTC_MSKSSC_0_14                    ALRMXSS_MSKSSC(0)                           /*!< mask alarm subsecond configuration */
#define RTC_MSKSSC_1_14                    ALRMXSS_MSKSSC(1)                           /*!< mask RTC_ALRMXSS_SSC[14:1], and RTC_ALRMXSS_SSC[0] is to be compared */
#define RTC_MSKSSC_2_14                    ALRMXSS_MSKSSC(2)                           /*!< mask RTC_ALRMXSS_SSC[14:2], and RTC_ALRMXSS_SSC[1:0] is to be compared */
#define RTC_MSKSSC_3_14                    ALRMXSS_MSKSSC(3)                           /*!< mask RTC_ALRMXSS_SSC[14:3], and RTC_ALRMXSS_SSC[2:0] is to be compared */
#define RTC_MSKSSC_4_14                    ALRMXSS_MSKSSC(4)                           /*!< mask RTC_ALRMXSS_SSC[14:4]], and RTC_ALRMXSS_SSC[3:0] is to be compared */
#define RTC_MSKSSC_5_14                    ALRMXSS_MSKSSC(5)                           /*!< mask RTC_ALRMXSS_SSC[14:5], and RTC_ALRMXSS_SSC[4:0] is to be compared */
#define RTC_MSKSSC_6_14                    ALRMXSS_MSKSSC(6)                           /*!< mask RTC_ALRMXSS_SSC[14:6], and RTC_ALRMXSS_SSC[5:0] is to be compared */
#define RTC_MSKSSC_7_14                    ALRMXSS_MSKSSC(7)                           /*!< mask RTC_ALRMXSS_SSC[14:7], and RTC_ALRMXSS_SSC[6:0] is to be compared */
#define RTC_MSKSSC_8_14                    ALRMXSS_MSKSSC(8)                           /*!< mask RTC_ALRMXSS_SSC[14:8], and RTC_ALRMXSS_SSC[7:0] is to be compared */
#define RTC_MSKSSC_9_14                    ALRMXSS_MSKSSC(9)                           /*!< mask RTC_ALRMXSS_SSC[14:9], and RTC_ALRMXSS_SSC[8:0] is to be compared */
#define RTC_MSKSSC_10_14                   ALRMXSS_MSKSSC(10)                          /*!< mask RTC_ALRMXSS_SSC[14:10], and RTC_ALRMXSS_SSC[9:0] is to be compared */
#define RTC_MSKSSC_11_14                   ALRMXSS_MSKSSC(11)                          /*!< mask RTC_ALRMXSS_SSC[14:11], and RTC_ALRMXSS_SSC[10:0] is to be compared */
#define RTC_MSKSSC_12_14                   ALRMXSS_MSKSSC(12)                          /*!< mask RTC_ALRMXSS_SSC[14:12], and RTC_ALRMXSS_SSC[11:0] is to be compared */
#define RTC_MSKSSC_13_14                   ALRMXSS_MSKSSC(13)                          /*!< mask RTC_ALRMXSS_SSC[14:13], and RTC_ALRMXSS_SSC[12:0] is to be compared */
#define RTC_MSKSSC_14                      ALRMXSS_MSKSSC(14)                          /*!< mask RTC_ALRMXSS_SSC[14], and RTC_ALRMXSS_SSC[13:0] is to be compared */
#define RTC_MSKSSC_NONE                    ALRMXSS_MSKSSC(15)                          /*!< mask none, and RTC_ALRMXSS_SSC[14:0] is to be compared */

/* write protect key */
#define RTC_UNLOCK_KEY1                    ((uint8_t)0xCAU)                            /*!< RTC unlock key1 */
#define RTC_UNLOCK_KEY2                    ((uint8_t)0x53U)                            /*!< RTC unlock key2 */
#define RTC_LOCK_KEY                       ((uint8_t)0xFFU)                            /*!< RTC lock key */

/* registers reset value */
#define RTC_REGISTER_RESET                 ((uint32_t)0x00000000U)                     /*!< RTC common register reset value */
#define RTC_DATE_RESET                     ((uint32_t)0x00002101U)                     /*!< RTC_DATE register reset value */
#define RTC_STAT_RESET                     ((uint32_t)0x00000007U)                     /*!< RTC_STAT register reset value */
#define RTC_PSC_RESET                      ((uint32_t)0x007F00FFU)                     /*!< RTC_PSC register reset value */

/* RTC alarm */
#define RTC_ALARM0                         ((uint8_t)0x01U)                            /*!< RTC alarm 0 */

/* RTC interrupt flag definitions */
typedef enum {
    RTC_INT_TIMESTAMP     = ((uint32_t)RTC_CTL_TSIE),       /*!< time-stamp interrupt enable */
    RTC_INT_ALARM0        = ((uint32_t)RTC_CTL_ALRM0IE),      /*!< alarm interrupt enable */
} rtc_interrupt_flag_enum;

/* RTC flag definitions */
typedef enum {
    RTC_FLAG_ALARM0W   = ((uint32_t)RTC_STAT_ALRM0WF),        /*!< alarm0 configuration can be write flag */
    RTC_FLAG_SOP       = ((uint32_t)RTC_STAT_SOPF   ),        /*!< shift function operation pending flag */
    RTC_FLAG_YCM       = ((uint32_t)RTC_STAT_YCM    ),        /*!< year parameter configured event flag */
    RTC_FLAG_RSYN      = ((uint32_t)RTC_STAT_RSYNF  ),        /*!< registers synchronized flag */
    RTC_FLAG_INIT      = ((uint32_t)RTC_STAT_INITF  ),        /*!< init mode event flag */
    RTC_FLAG_SCP       = ((uint32_t)RTC_STAT_SOPF   ),        /*!< smooth calibration pending flag */
    RTC_FLAG_ALARM0    = ((uint32_t)RTC_STAT_ALRM0F ),        /*!< alarm event flag */
    RTC_FLAG_TS        = ((uint32_t)RTC_STAT_TSF    ),        /*!< time-stamp flag */
    RTC_FLAG_TSOVR     = ((uint32_t)RTC_STAT_TSOVRF ),        /*!< time-stamp overflow flag */
} rtc_flag_enum;

/* parameter check definitions */
#ifdef FW_DEBUG_ERR_REPORT

/* check the RTC pointer*/
#define NOT_RTC_VALID_POINTER(pointer)        ((void *) 0 == (pointer))           /*!< check the invalid pointer */
    
#endif

/* function declarations */
/* enable RTC bypass shadow registers function */
void rtc_bypass_shadow_enable(void);
/* disable RTC bypass shadow registers function */
void rtc_bypass_shadow_disable(void);
/* wait until RTC_TIME and RTC_DATE registers are synchronized with APB clock, and the shadow registers are updated */
ErrStatus rtc_register_sync_wait(void);

/* enable RTC alarm */
void rtc_alarm_enable(void);
/* disable RTC alarm */
ErrStatus rtc_alarm_disable(void);
/* alarm configuration functions */
/* configure RTC alarm */
void rtc_alarm_config(rtc_alarm_struct *rtc_alarm_time);
/* configure subsecond of RTC alarm */
void rtc_alarm_subsecond_config(uint32_t mask_subsecond, uint32_t subsecond);
/* get RTC alarm */
void rtc_alarm_get(rtc_alarm_struct *rtc_alarm_time);
/* get RTC alarm subsecond */
uint32_t rtc_alarm_subsecond_get(void);

/* enter RTC init mode */
ErrStatus rtc_init_mode_enter(void);
/* initialize RTC registers */
ErrStatus rtc_init(rtc_parameter_struct *rtc_initpara_struct);
/* exit RTC init mode */
void rtc_init_mode_exit(void);

/* get current time and date */
void rtc_current_time_get(rtc_parameter_struct *rtc_initpara_struct);
/* get current subsecond value */
uint32_t rtc_subsecond_get(void);

/* reset most of the RTC registers */
ErrStatus rtc_deinit(void);

/* adjust the daylight saving time by adding or substracting one hour from the current time */
void rtc_hour_adjust(uint32_t operation);
/* adjust RTC second or subsecond value of current time */
ErrStatus rtc_second_adjust(uint32_t add, uint32_t minus);

/* enable RTC reference clock detection function */
ErrStatus rtc_refclock_detection_enable(void);
/* disable RTC reference clock detection function */
ErrStatus rtc_refclock_detection_disable(void);

/* configure RTC smooth calibration */
ErrStatus rtc_smooth_calibration_config(uint32_t window, uint32_t plus, uint32_t smooth_minus);

/* timestamp and tamper configuration functions */
/* enable RTC time-stamp */
void rtc_timestamp_enable(uint32_t edge);
/* disable RTC time-stamp */
void rtc_timestamp_disable(void);
/* get RTC timestamp time and date */
void rtc_timestamp_get(rtc_timestamp_struct *rtc_timestamp);
/* get RTC time-stamp subsecond */
uint32_t rtc_timestamp_subsecond_get(void);
/* configure RTC calibration output source */
void rtc_calibration_output_config(uint32_t source);
/* configure RTC alarm output source */
void rtc_alarm_output_config(uint32_t alarm_output, uint32_t mode);
/* select the RTC output pin */
void rtc_output_pin_select(uint32_t outputpin);

/* enable specified RTC interrupt */
void rtc_interrupt_enable(rtc_interrupt_flag_enum interrupt);
/* disable specified RTC interrupt */
void rtc_interrupt_disable(rtc_interrupt_flag_enum interrupt);
/* check specified flag */
FlagStatus rtc_flag_get(rtc_flag_enum flag);
/* clear specified flag */
void rtc_flag_clear(rtc_flag_enum flag_clear);

#endif /* GD32C2X1_RTC_H */
