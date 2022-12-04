/*!
    \file    gd32a50x_mfcom.h
    \brief   MFCOM driver

    \version 2022-01-30, V1.0.0, firmware for GD32A50X
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

#ifndef GD32A50X_MFCOM_H
#define GD32A50X_MFCOM_H

#include "gd32a50x.h"

/* MFCOM definitions */
#define MFCOM                           MFCOM_BASE                               /*!< MFCOM base address */

/* registers definitions */
#define MFCOM_CTL                       REG32(MFCOM + 0x00000000U)               /*!< MFCOM control register */
#define MFCOM_PINDATA                   REG32(MFCOM + 0x00000004U)               /*!< MFCOM pin data register */
#define MFCOM_SSTAT                     REG32(MFCOM + 0x00000008U)               /*!< MFCOM shifter status register */
#define MFCOM_SERR                      REG32(MFCOM + 0x0000000CU)               /*!< MFCOM shifter error register */
#define MFCOM_TMSTAT                    REG32(MFCOM + 0x00000010U)               /*!< MFCOM timer status register */
#define MFCOM_SSIEN                     REG32(MFCOM + 0x00000018U)               /*!< MFCOM shifter status interrupt enable register */
#define MFCOM_SEIEN                     REG32(MFCOM + 0x0000001CU)               /*!< MFCOM shifter error interrupt enable register */
#define MFCOM_TMSIEN                    REG32(MFCOM + 0x00000020U)               /*!< MFCOM timer status interrupt enable register */
#define MFCOM_SSDMAEN                   REG32(MFCOM + 0x00000028U)               /*!< MFCOM shifter status dma enable register */
#define MFCOM_SCTL(x)                   REG32(MFCOM + 0x00000080U + (x)*4U)      /*!< MFCOM shifter x control register */
#define MFCOM_SCFG(x)                   REG32(MFCOM + 0x00000100U + (x)*4U)      /*!< MFCOM shifter x configuration register */
#define MFCOM_SBUF(x)                   REG32(MFCOM + 0x00000200U + (x)*4U)      /*!< MFCOM shifter buffer x register */
#define MFCOM_SBUFBIS(x)                REG32(MFCOM + 0x00000280U + (x)*4U)      /*!< MFCOM shifter buffer x bit swapped register  */
#define MFCOM_SBUFBYS(x)                REG32(MFCOM + 0x00000300U + (x)*4U)      /*!< MFCOM shifter buffer x byte swapped register */
#define MFCOM_SBUFBBS(x)                REG32(MFCOM + 0x00000380U + (x)*4U)      /*!< MFCOM shifter buffer x bit byte swapped register */
#define MFCOM_TMCTL(x)                  REG32(MFCOM + 0x00000400U + (x)*4U)      /*!< MFCOM timer x control register */
#define MFCOM_TMCFG(x)                  REG32(MFCOM + 0x00000480U + (x)*4U)      /*!< MFCOM timer x configuration register */
#define MFCOM_TMCMP(x)                  REG32(MFCOM + 0x00000500U + (x)*4U)      /*!< MFCOM timer x compare register */

/* bits definitions */
/* MFCOM_CTL */
#define MFCOM_CTL_MFCOMEN               BIT(0)                                   /*!< MFCOM enable */ 
#define MFCOM_CTL_SWRSTEN               BIT(1)                                   /*!< software reset enable */

/* MFCOM_PINDATA */
#define MFCOM_PINDATA_PDATA             BITS(0,7)                                /*!< input data of pins */

/* MFCOM_SSTAT */
#define MFCOM_SSTAT_SSTAT               BITS(0,3)                                /*!< shifter x status flag */

/* MFCOM_SERR */
#define MFCOM_SERR_SERR                 BITS(0,3)                                /*!< shifter x error flags */

/* MFCOM_TMSTAT */
#define MFCOM_TMSTAT_TMSTAT             BITS(0,3)                                /*!< timer x status flags */

/* MFCOM_SSIEN */
#define MFCOM_SSIEN_SSIEN               BITS(0,3)                                /*!< shifter status interrupt enable */

/* MFCOM_TMSIEN */
#define MFCOM_TMSIEN_TMSIEN             BITS(0,3)                                /*!< timer status interrupt enable */

/* MFCOM_SSDMAEN */
#define MFCOM_SSDMAEN_SSDMAEN           BITS(0,3)                                /*!< shifter status DMA enable */

/* MFCOM_SCTLx */
#define MFCOM_SCTL_SMOD                 BITS(0,2)                                /*!< shifter mode */
#define MFCOM_SCTL_SPPL                 BIT(7)                                   /*!< shifter pin polarity */
#define MFCOM_SCTL_SPSEL                BITS(8,10)                               /*!< shifter pin select */
#define MFCOM_SCTL_SPCFG                BITS(16,17)                              /*!< shifter pin configuration */
#define MFCOM_SCTL_TMPL                 BIT(23)                                  /*!< timer polarity */
#define MFCOM_SCTL_TMSEL                BITS(24,25)                              /*!< timer select */

/* MFCOM_SCFGx */
#define MFCOM_SCFG_SSTART               BITS(0,1)                                /*!< shifter start bit */
#define MFCOM_SCFG_SSTOP                BITS(4,5)                                /*!< shifter stop bit */
#define MFCOM_SCFG_INSRC                BIT(8)                                   /*!< input source */

/* MFCOM_SBUFx */
#define MFCOM_SBUFx_SBUF                BITS(0,31)                               /*!< shift buffer */

/* MFCOM_SBUFBISx */
#define MFCOM_SBUFBISx_SBUFBIS          BITS(0,31)                               /*!< shift buffer bit swapped */

/* MFCOM_SBUFBYSx */
#define MFCOM_SBUFBYSx_SBUFBYS          BITS(0,31)                               /*!< shift buffer byte swapped */

/* MFCOM_SBUFBBSx */
#define MFCOM_SBUFBBSx_SBUFBBS          BITS(0,31)                               /*!< shift buffer bit byte swapped */

/* MFCOM_TMCTLx */
#define MFCOM_TMCTL_TMMOD               BITS(0,1)                                /*!< timer mode */
#define MFCOM_TMCTL_TMPPL               BIT(7)                                   /*!< timer pin polarity */
#define MFCOM_TMCTL_TMPSEL              BITS(8,10)                               /*!< timer pin select */
#define MFCOM_TMCTL_TMPCFG              BITS(16,17)                              /*!< timer pin configuration */
#define MFCOM_TMCTL_TRIGSRC             BIT(22)                                  /*!< trigger source */
#define MFCOM_TMCTL_TRIGPL              BIT(23)                                  /*!< trigger polarity */
#define MFCOM_TMCTL_TRIGSEL             BITS(24,27)                              /*!< trigger select */

/* MFCOM_TMCFGx */
#define MFCOM_TMCFG_TMSTART             BIT(1)                                   /*!< timer start bit */
#define MFCOM_TMCFG_TMSTOP              BITS(4,5)                                /*!< timer stop bit */
#define MFCOM_TMCFG_TMEN                BITS(8,10)                               /*!< timer enable source */
#define MFCOM_TMCFG_TMDIS               BITS(12,14)                              /*!< timer disable source */
#define MFCOM_TMCFG_TMRST               BITS(16,18)                              /*!< timer reset source */
#define MFCOM_TMCFG_TMDEC               BITS(20,21)                              /*!< timer decrement source */
#define MFCOM_TMCFG_TMOUT               BITS(24,25)                              /*!< timer output select */

/* MFCOM_TMCMPx */
#define MFCOM_TMCMPx_TMCVALUE           BITS(0,31)                               /*!< timer compare value */

/* constants definitions */
/* MFCOM timer init parameter struct definitions */
typedef struct
{
    /* trigger */
    uint32_t trigger_select;                                                    /*!< the internal trigger selection */
    uint32_t trigger_polarity;                                                  /*!< trigger polarity */
    /* pin */
    uint32_t pin_config;                                                        /*!< timer pin configuration */
    uint32_t pin_select;                                                        /*!< timer pin number select */
    uint32_t pin_polarity;                                                      /*!< timer pin polarity */
    /* timer */
    uint32_t mode;                                                              /*!< timer work mode */
    uint32_t output;                                                            /*!< configures the initial state of the timer output and
                                                                                whether it is affected by the timer reset */
    uint32_t decrement;                                                         /*!< configures the source of the timer decrement and the
                                                                                source of the shift clock */
    uint32_t reset;                                                             /*!< configures the condition that causes the timer counter
                                                                                (and optionally the timer output) to be reset */
    uint32_t disable;                                                           /*!< configures the condition that causes the timer to be
                                                                                disabled and stop decrementing */
    uint32_t enable;                                                            /*!< configures the condition that causes the timer to be
                                                                                enabled and start decrementing */
    uint32_t stopbit;                                                           /*!< timer stop bit generation */
    uint32_t startbit;                                                          /*!< timer start bit generation */
    uint32_t compare;                                                           /*!< value for timer compare x register */
}mfcom_timer_parameter_struct;

/* MFCOM shifter init parameter struct definitions */
typedef struct
{
    /* timer */
    uint32_t timer_select;                                                      /*!< selects which timer is used for controlling the
                                                                                logic/shift register and generating the shift clock */
    uint32_t timer_polarity;                                                    /*!< timer polarity */
    /* pin */
    uint32_t pin_config;                                                        /*!< shifter pin configuration */
    uint32_t pin_select;                                                        /*!< shifter pin number select */
    uint32_t pin_polarity;                                                      /*!< shifter pin polarity */
    /* shifter */
    uint32_t mode;                                                              /*!< configures the mode of the shifter */
    uint32_t input_source;                                                      /*!< selects the input source for the shifter */
    uint32_t stopbit;                                                           /*!< shifter stop bit */
    uint32_t startbit;                                                          /*!< shifter start bit */
}mfcom_shifter_parameter_struct;

/* MFCOM timer trigger source */
#define TMCTL_TRIGSEL(regval)                    (BITS(24,27) & ((uint32_t)(regval) << 24U))
#define MFCOM_TIMER_TRGSEL_PIN0                  (TMCTL_TRIGSEL(0)|MFCOM_TMCTL_TRIGSRC)                 /*!< pin 0 input selected */
#define MFCOM_TIMER_TRGSEL_SHIFTER0              (TMCTL_TRIGSEL(1)|MFCOM_TMCTL_TRIGSRC)                 /*!< shifter 0 status flag selected */
#define MFCOM_TIMER_TRGSEL_PIN1                  (TMCTL_TRIGSEL(2)|MFCOM_TMCTL_TRIGSRC)                 /*!< pin 1 selected */
#define MFCOM_TIMER_TRGSEL_TIMER0                (TMCTL_TRIGSEL(3)|MFCOM_TMCTL_TRIGSRC)                 /*!< timer 0 trigger output selected */
#define MFCOM_TIMER_TRGSEL_PIN2                  (TMCTL_TRIGSEL(4)|MFCOM_TMCTL_TRIGSRC)                 /*!< pin 2 selected */
#define MFCOM_TIMER_TRGSEL_SHIFTER1              (TMCTL_TRIGSEL(5)|MFCOM_TMCTL_TRIGSRC)                 /*!< shifter 1 status flag selected */
#define MFCOM_TIMER_TRGSEL_PIN3                  (TMCTL_TRIGSEL(6)|MFCOM_TMCTL_TRIGSRC)                 /*!< pin 3 selected */
#define MFCOM_TIMER_TRGSEL_TIMER1                (TMCTL_TRIGSEL(7)|MFCOM_TMCTL_TRIGSRC)                 /*!< timer 1 trigger output selected */
#define MFCOM_TIMER_TRGSEL_PIN4                  (TMCTL_TRIGSEL(8)|MFCOM_TMCTL_TRIGSRC)                 /*!< pin 4 selected */
#define MFCOM_TIMER_TRGSEL_SHIFTER2              (TMCTL_TRIGSEL(9)|MFCOM_TMCTL_TRIGSRC)                 /*!< shifter 2 status flag selected */
#define MFCOM_TIMER_TRGSEL_PIN5                  (TMCTL_TRIGSEL(10)|MFCOM_TMCTL_TRIGSRC)                /*!< pin 5 selected */
#define MFCOM_TIMER_TRGSEL_TIMER2                (TMCTL_TRIGSEL(11)|MFCOM_TMCTL_TRIGSRC)                /*!< timer 2 trigger output selected */
#define MFCOM_TIMER_TRGSEL_PIN6                  (TMCTL_TRIGSEL(12)|MFCOM_TMCTL_TRIGSRC)                /*!< pin 6 selected */
#define MFCOM_TIMER_TRGSEL_SHIFTER3              (TMCTL_TRIGSEL(13)|MFCOM_TMCTL_TRIGSRC)                /*!< shifter 3 status flag selected */
#define MFCOM_TIMER_TRGSEL_PIN7                  (TMCTL_TRIGSEL(14)|MFCOM_TMCTL_TRIGSRC)                /*!< pin 7 selected */
#define MFCOM_TIMER_TRGSEL_TIMER3                (TMCTL_TRIGSEL(15)|MFCOM_TMCTL_TRIGSRC)                /*!< timer 3 trigger output selected */
#define MFCOM_TIMER_TRGSEL_EXTERNAL0             TMCTL_TRIGSEL(0)                                       /*!< external trigger0 selected */
#define MFCOM_TIMER_TRGSEL_EXTERNAL1             TMCTL_TRIGSEL(1)                                       /*!< external trigger1 selected */
#define MFCOM_TIMER_TRGSEL_EXTERNAL2             TMCTL_TRIGSEL(2)                                       /*!< external trigger2 selected */
#define MFCOM_TIMER_TRGSEL_EXTERNAL3             TMCTL_TRIGSEL(3)                                       /*!< external trigger3 selected */

/* MFCOM timer trigger polarity */
#define MFCOM_TIMER_TRGPOL_ACTIVE_HIGH           ((uint32_t)0x00000000U)                                /*!< active high */
#define MFCOM_TIMER_TRGPOL_ACTIVE_LOW            MFCOM_TMCTL_TRIGPL                                     /*!< active low */

/* MFCOM timer pin config */
#define TMCTL_TMPCFG(regval)                     (BITS(16,17) & ((uint32_t)(regval) << 16U))
#define MFCOM_TIMER_PINCFG_INPUT                 TMCTL_TMPCFG(0)                                        /*!< pin input */
#define MFCOM_TIMER_PINCFG_OPENDRAIN             TMCTL_TMPCFG(1)                                        /*!< pin open drain */
#define MFCOM_TIMER_PINCFG_BIDI                  TMCTL_TMPCFG(2)                                        /*!< pin cascade input/output */
#define MFCOM_TIMER_PINCFG_OUTPUT                TMCTL_TMPCFG(3)                                        /*!< pin output */

/* MFCOM timer pin select */
#define TMCTL_TMPSEL(regval)                     (BITS(8,10) & ((uint32_t)(regval) << 8U))
#define MFCOM_TIMER_PINSEL_PIN0                  TMCTL_TMPSEL(0)                                        /*!< timer Pin 0 selected */
#define MFCOM_TIMER_PINSEL_PIN1                  TMCTL_TMPSEL(1)                                        /*!< timer Pin 1 selected */
#define MFCOM_TIMER_PINSEL_PIN2                  TMCTL_TMPSEL(2)                                        /*!< timer Pin 2 selected */
#define MFCOM_TIMER_PINSEL_PIN3                  TMCTL_TMPSEL(3)                                        /*!< timer Pin 3 selected */
#define MFCOM_TIMER_PINSEL_PIN4                  TMCTL_TMPSEL(4)                                        /*!< timer Pin 4 selected */
#define MFCOM_TIMER_PINSEL_PIN5                  TMCTL_TMPSEL(5)                                        /*!< timer Pin 5 selected */
#define MFCOM_TIMER_PINSEL_PIN6                  TMCTL_TMPSEL(6)                                        /*!< timer Pin 6 selected */
#define MFCOM_TIMER_PINSEL_PIN7                  TMCTL_TMPSEL(7)                                        /*!< timer Pin 7 selected */

/* MFCOM timer pin polarity */
#define MFCOM_TIMER_PINPOL_ACTIVE_HIGH           ((uint32_t)0x00000000U)                                /*!< active high */
#define MFCOM_TIMER_PINPOL_ACTIVE_LOW            MFCOM_TMCTL_TMPPL                                      /*!< active low */

/* MFCOM timer mode */
#define TMCTL_TMMOD(regval)                      (BITS(0,1) & ((uint32_t)(regval) << 0U))
#define MFCOM_TIMER_DISABLE                      TMCTL_TMMOD(0)                                         /*!< timer disabled. */
#define MFCOM_TIMER_BAUDMODE                     TMCTL_TMMOD(1)                                         /*!< dual 8-bit counters baud/bit mode */
#define MFCOM_TIMER_PWMMODE                      TMCTL_TMMOD(2)                                         /*!< dual 8-bit counters PWM mode */
#define MFCOM_TIMER_16BITCOUNTER                 TMCTL_TMMOD(3)                                         /*!< single 16-bit counter mode */

/* MFCOM timer output */
#define TMCFG_TMOUT(regval)                      (BITS(24,25) & ((uint32_t)(regval) << 24U))
#define MFCOM_TIMER_OUT_HIGH_EN                  TMCFG_TMOUT(0)                                         /*!< logic one when enabled and is not affected by timer reset */
#define MFCOM_TIMER_OUT_LOW_EN                   TMCFG_TMOUT(1)                                         /*!< logic zero when enabled and is not affected by timer reset */
#define MFCOM_TIMER_OUT_HIGH_EN_RESET            TMCFG_TMOUT(2)                                         /*!< logic one when enabled and on timer reset */
#define MFCOM_TIMER_OUT_LOW_EN_RESET             TMCFG_TMOUT(3)                                         /*!< logic zero when enabled and on timer reset */

/* MFCOM timer decrement */
#define TMCFG_TMDEC(regval)                      (BITS(20,21) & ((uint32_t)(regval) << 20U))
#define MFCOM_TIMER_DEC_CLK_SHIFT_OUT            TMCFG_TMDEC(0)                                         /*!< decrement counter on MFCOM clock, shift clock equals timer output */
#define MFCOM_TIMER_DEC_TRIG_SHIFT_OUT           TMCFG_TMDEC(1)                                         /*!< decrement counter on trigger input (both edges), shift clock equals timer output */
#define MFCOM_TIMER_DEC_PIN_SHIFT_PIN            TMCFG_TMDEC(2)                                         /*!< decrement counter on pin input (both edges), shift clock equals Pin input */
#define MFCOM_TIMER_DEC_TRIG_SHIFT_TRIG          TMCFG_TMDEC(3)                                         /*!< decrement counter on trigger input (both edges), shift clock equals trigger input */

/* MFCOM timer reset */
#define TMCFG_TMRST(regval)                      (BITS(16,18) & ((uint32_t)(regval) << 16U))
#define MFCOM_TIMER_RESET_NEVER                  TMCFG_TMRST(0)                                         /*!< timer never reset */
#define MFCOM_TIMER_RESET_PIN_TIMOUT             TMCFG_TMRST(2)                                         /*!< timer reset on timer pin equal to timer output */
#define MFCOM_TIMER_RESET_TRIG_TIMOUT            TMCFG_TMRST(3)                                         /*!< timer reset on timer trigger equal to timer output */
#define MFCOM_TIMER_RESET_PIN_RISING             TMCFG_TMRST(4)                                         /*!< timer reset on timer pin rising edge */
#define MFCOM_TIMER_RESET_TRIG_RISING            TMCFG_TMRST(6)                                         /*!< timer reset on trigger rising edge */
#define MFCOM_TIMER_RESET_TRIG_BOTH              TMCFG_TMRST(7)                                         /*!< timer reset on trigger rising or falling edge */

/* MFCOM timer disable */
#define TMCFG_TMDIS(regval)                      (BITS(12,14) & ((uint32_t)(regval) << 12U))
#define MFCOM_TIMER_DISMODE_NEVER                TMCFG_TMDIS(0)                                         /*!< timer never disabled */
#define MFCOM_TIMER_DISMODE_PRE_TIMDIS           TMCFG_TMDIS(1)                                         /*!< timer disabled on timer x-1 disable */
#define MFCOM_TIMER_DISMODE_COMPARE              TMCFG_TMDIS(2)                                         /*!< timer disabled on timer compare */
#define MFCOM_TIMER_DISMODE_COMPARE_TRIGLOW      TMCFG_TMDIS(3)                                         /*!< timer disabled on timer compare and trigger Low */
#define MFCOM_TIMER_DISMODE_PINBOTH              TMCFG_TMDIS(4)                                         /*!< timer disabled on pin rising or falling edge */
#define MFCOM_TIMER_DISMODE_PINBOTH_TRIGHIGH     TMCFG_TMDIS(5)                                         /*!< timer disabled on pin rising or falling edge provided trigger is high */
#define MFCOM_TIMER_DISMODE_TRIGFALLING          TMCFG_TMDIS(6)                                         /*!< timer disabled on trigger falling edge */

/* MFCOM timer enable */
#define TMCFG_TMEN(regval)                       (BITS(8,10) & ((uint32_t)(regval) << 8U))
#define MFCOM_TIMER_ENMODE_ALWAYS                TMCFG_TMEN(0)                                          /*!< timer always enabled */
#define MFCOM_TIMER_ENMODE_PRE_TIMEN             TMCFG_TMEN(1)                                          /*!< timer enabled on timer x-1 enable */
#define MFCOM_TIMER_ENMODE_TRIGHIGH              TMCFG_TMEN(2)                                          /*!< timer enabled on trigger high */
#define MFCOM_TIMER_ENMODE_TRIGHIGH_PINHIGH      TMCFG_TMEN(3)                                          /*!< timer enabled on trigger high and Pin high */
#define MFCOM_TIMER_ENMODE_PINRISING             TMCFG_TMEN(4)                                          /*!< timer enabled on pin rising edge */
#define MFCOM_TIMER_ENMODE_PINRISING_TRIGHIGH    TMCFG_TMEN(5)                                          /*!< timer enabled on pin rising edge and trigger high */
#define MFCOM_TIMER_ENMODE_TRIGRISING            TMCFG_TMEN(6)                                          /*!< timer enabled on trigger rising edge */
#define MFCOM_TIMER_ENMODE_TRIGBOTH              TMCFG_TMEN(7)                                          /*!< timer enabled on trigger rising or falling edge */

/* MFCOM timer stopbit */
#define TMCFG_TMSTOP(regval)                    (BITS(4,5) & ((uint32_t)(regval) << 4U))
#define MFCOM_TIMER_STOPBIT_DISABLE             TMCFG_TMSTOP(0)                                         /*!< stop bit disabled */
#define MFCOM_TIMER_STOPBIT_TIMCMP              TMCFG_TMSTOP(1)                                         /*!< stop bit is enabled on timer compare */
#define MFCOM_TIMER_STOPBIT_TIMDIS              TMCFG_TMSTOP(2)                                         /*!< stop bit is enabled on timer disable */
#define MFCOM_TIMER_STOPBIT_TIMCMP_TIMDIS       TMCFG_TMSTOP(3)                                         /*!< stop bit is enabled on timer compare and timer disable */

/* MFCOM timer startbit */
#define MFCOM_TIMER_STARTBIT_DISABLE            ((uint32_t)0x00000000U)                                 /*!< Start bit disabled */
#define MFCOM_TIMER_STARTBIT_ENABLE             MFCOM_TMCFG_TMSTART                                     /*!< Start bit enabled */

/* MFCOM shifter timer select */
#define SCTL_TMSEL(regval)                      (BITS(24,25) & ((uint32_t)(regval) << 24U))
#define MFCOM_SHIFTER_TIMER0                    SCTL_TMSEL(0)                                           /*!< timer0 selected */
#define MFCOM_SHIFTER_TIMER1                    SCTL_TMSEL(1)                                           /*!< timer1 selected */
#define MFCOM_SHIFTER_TIMER2                    SCTL_TMSEL(2)                                           /*!< timer2 selected */
#define MFCOM_SHIFTER_TIMER3                    SCTL_TMSEL(3)                                           /*!< timer3 selected */

/* type of timer polarity for shifter control */
#define MFCOM_SHIFTER_TIMPOL_ACTIVE_HIGH        ((uint32_t)0x00000000U)                                /*!< shift on positive edge of shift clock */
#define MFCOM_SHIFTER_TIMPOL_ACTIVE_LOW         MFCOM_SCTL_TMPL                                        /*!< shift on negative edge of shift clock */

/* MFCOM shifter pin config */
#define SCTL_SPCFG(regval)                      (BITS(16,17) & ((uint32_t)(regval) << 16U))
#define MFCOM_SHIFTER_PINCFG_INPUT              SCTL_SPCFG(0)                                          /*!< pin input */
#define MFCOM_SHIFTER_PINCFG_OPENDRAIN          SCTL_SPCFG(1)                                          /*!< pin open drain */
#define MFCOM_SHIFTER_PINCFG_BIDI               SCTL_SPCFG(2)                                          /*!< pin cascade input/output */
#define MFCOM_SHIFTER_PINCFG_OUTPUT             SCTL_SPCFG(3)                                          /*!< pin output */

/* MFCOM shifter pin select */
#define SCTL_SPSEL(regval)                      (BITS(8,10) & ((uint32_t)(regval) << 8U))
#define MFCOM_SHIFTER_PINSEL_PIN0               SCTL_SPSEL(0)                                          /*!< shifter pin 0 selected */
#define MFCOM_SHIFTER_PINSEL_PIN1               SCTL_SPSEL(1)                                          /*!< shifter pin 1 selected */
#define MFCOM_SHIFTER_PINSEL_PIN2               SCTL_SPSEL(2)                                          /*!< shifter pin 2 selected */
#define MFCOM_SHIFTER_PINSEL_PIN3               SCTL_SPSEL(3)                                          /*!< shifter pin 3 selected */
#define MFCOM_SHIFTER_PINSEL_PIN4               SCTL_SPSEL(4)                                          /*!< shifter pin 4 selected */
#define MFCOM_SHIFTER_PINSEL_PIN5               SCTL_SPSEL(5)                                          /*!< shifter pin 5 selected */
#define MFCOM_SHIFTER_PINSEL_PIN6               SCTL_SPSEL(6)                                          /*!< shifter pin 6 selected */
#define MFCOM_SHIFTER_PINSEL_PIN7               SCTL_SPSEL(7)                                          /*!< shifter pin 7 selected */

/* MFCOM shifter pin polarity */
#define MFCOM_SHIFTER_PINPOL_ACTIVE_HIGH        ((uint32_t)0x00000000U)                                /*!< active high */
#define MFCOM_SHIFTER_PINPOL_ACTIVE_LOW         MFCOM_SCTL_SPPL                                        /*!< active low */

/* MFCOM shifter mode */
#define SCTL_SMOD(regval)                       (BITS(0,2) & ((uint32_t)(regval) << 0U))
#define MFCOM_SHIFTER_DISABLE                   SCTL_SMOD(0)                                           /*!< shifter is disabled */
#define MFCOM_SHIFTER_RECEIVE                   SCTL_SMOD(1)                                           /*!< receive mode */
#define MFCOM_SHIFTER_TRANSMIT                  SCTL_SMOD(2)                                           /*!< transmit mode */
#define MFCOM_SHIFTER_MATCH_STORE               SCTL_SMOD(4)                                           /*!< match store mode */
#define MFCOM_SHIFTER_MATCH_CONTINUOUS          SCTL_SMOD(5)                                           /*!< match continuous mode */

/* MFCOM shifter input source */
#define MFCOM_SHIFTER_INSRC_PIN                 ((uint32_t)0x00000000U)                                /*!< shifter input from pin */
#define MFCOM_SHIFTER_INSRC_NEXTSHIFTER         MFCOM_SCFG_INSRC                                       /*!< shifter input from shifter x+1 */

/* MFCOM shifter stopbit */
#define SCFG_SSTOP(regval)                      (BITS(4,5) & ((uint32_t)(regval) << 4U))
#define MFCOM_SHIFTER_STOPBIT_DISABLE           SCFG_SSTOP(0)                                          /*!< disable shifter stop bit */
#define MFCOM_SHIFTER_STOPBIT_LOW               SCFG_SSTOP(2)                                          /*!< set shifter stop bit to logic low level */
#define MFCOM_SHIFTER_STOPBIT_HIGH              SCFG_SSTOP(3)                                          /*!< set shifter stop bit to logic high level */

/* MFCOM shifter startbit */
#define SCFG_SSTART(regval)                     (BITS(0,1) & ((uint32_t)(regval) << 0U))
#define MFCOM_SHIFTER_STARTBIT_DISABLE          SCFG_SSTART(0)                                         /*!< disable shifter start bit, transmitter loads data on enable */
#define MFCOM_SHIFTER_STARTBIT_DISABLE_TXEN     SCFG_SSTART(1)                                         /*!< disable shifter start bit, transmitter loads data on first shift */
#define MFCOM_SHIFTER_STARTBIT_LOW              SCFG_SSTART(2)                                         /*!< set shifter start bit to logic low level */
#define MFCOM_SHIFTER_STARTBIT_HIGH             SCFG_SSTART(3)                                         /*!< set shifter start bit to logic high level */

/* MFCOM shifter enum */
#define MFCOM_SHIFTER_0                         ((uint32_t)0x00000000U)                                /*!< MFCOM shifter0 */
#define MFCOM_SHIFTER_1                         ((uint32_t)0x00000001U)                                /*!< MFCOM shifter1 */
#define MFCOM_SHIFTER_2                         ((uint32_t)0x00000002U)                                /*!< MFCOM shifter2 */
#define MFCOM_SHIFTER_3                         ((uint32_t)0x00000003U)                                /*!< MFCOM shifter3 */

/* MFCOM timer enum */
#define MFCOM_TIMER_0                           ((uint32_t)0x00000000U)                                /*!< MFCOM timer0 */
#define MFCOM_TIMER_1                           ((uint32_t)0x00000001U)                                /*!< MFCOM timer1 */
#define MFCOM_TIMER_2                           ((uint32_t)0x00000002U)                                /*!< MFCOM timer2 */
#define MFCOM_TIMER_3                           ((uint32_t)0x00000003U)                                /*!< MFCOM timer3 */

/* MFCOM read write mode enum */
#define MFCOM_RWMODE_NORMAL                     ((uint32_t)0x00000000U)                                /*!< read and write in normal mode */
#define MFCOM_RWMODE_BITSWAP                    ((uint32_t)0x00000001U)                                /*!< read and write in bit swapped mode */
#define MFCOM_RWMODE_BYTESWAP                   ((uint32_t)0x00000002U)                                /*!< read and write in byte swapped mode */
#define MFCOM_RWMODE_BITBYTESWAP                ((uint32_t)0x00000003U)                                /*!< read and write in bit byte swapped mode */

/* function declarations */
/* reset MFCOM */
void mfcom_deinit(void);
/* software reset */
void mfcom_software_reset(void);
/* enable MFCOM function */
void mfcom_enable(void);
/* disable MFCOM function */
void mfcom_disable(void);
/* initialize mfcom_timer_parameter_struct with the default values */
void mfcom_timer_struct_para_init(mfcom_timer_parameter_struct* init_struct);
/* initialize mfcom_shifter_parameter_struct with the default values */
void mfcom_shifter_struct_para_init(mfcom_shifter_parameter_struct* init_struct);

/* initialize MFCOM timer */
void mfcom_timer_init(uint32_t timer, mfcom_timer_parameter_struct* init_struct);
/* initialize MFCOM shifter */
void mfcom_shifter_init(uint32_t shifter, mfcom_shifter_parameter_struct* init_struct);

/* configure timer pin mode */
void mfcom_timer_pin_config(uint32_t timer, uint32_t mode);
/* configure shifter pin mode */
void mfcom_shifter_pin_config(uint32_t shifter, uint32_t mode);
/* enable MFCOM timer in specific mode */
void mfcom_timer_enable(uint32_t timer, uint32_t timermode);
/* enable MFCOM shifter in specific mode */
void mfcom_shifter_enable(uint32_t shifter, uint32_t shiftermode);
/* disable MFCOM timer */
void mfcom_timer_disable(uint32_t timer);
/* disable MFCOM shifter */
void mfcom_shifter_disable(uint32_t shifter);

/* set the timer compare value */
void mfcom_timer_cmpvalue_set(uint32_t timer, uint32_t compare);
/* get the timer compare value */
uint32_t mfcom_timer_cmpvalue_get(uint32_t timer);
/* set the timer disable source */
void mfcom_timer_dismode_set(uint32_t timer, uint32_t dismode);

/* set the shifter stopbit */
void mfcom_shifter_stopbit_set(uint32_t shifter, uint32_t stopbit);
/* write MFCOM shifter buffer */
void mfcom_buffer_write(uint32_t shifter, uint32_t data, uint32_t rwmode);
/* read MFCOM shifter buffer */
uint32_t mfcom_buffer_read(uint32_t shifter, uint32_t rwmode);
/* get MFCOM shifter flag */
FlagStatus mfcom_shifter_flag_get(uint32_t shifter);
/* get MFCOM shifter error flag */
FlagStatus mfcom_shifter_error_flag_get(uint32_t shifter);
/* get MFCOM timer flag */
FlagStatus mfcom_timer_flag_get(uint32_t timer);

/* get MFCOM shifter interrupt flag */
FlagStatus mfcom_shifter_interrupt_flag_get(uint32_t shifter);
/* get MFCOM shifter error interrupt flag */
FlagStatus mfcom_shifter_error_interrupt_flag_get(uint32_t shifter);
/* get MFCOM timer interrupt flag */
FlagStatus mfcom_timer_interrupt_flag_get(uint32_t timer);

/* clear MFCOM shifter flag */
void mfcom_shifter_flag_clear(uint32_t shifter);
/* clear MFCOM shifter error flag */
void mfcom_shifter_error_flag_clear(uint32_t shifter);
/* clear MFCOM timer flag */
void mfcom_timer_flag_clear(uint32_t timer);

/* enable MFCOM shifter interrupt */
void mfcom_shifter_interrupt_enable(uint32_t shifter);
/* enable MFCOM shifter error interrupt */
void mfcom_shifter_error_interrupt_enable(uint32_t shifter);
/* enable MFCOM timer interrupt */
void mfcom_timer_interrupt_enable(uint32_t timer);
/* enable MFCOM shifter dma */
void mfcom_shifter_dma_enable(uint32_t shifter);

/* disable MFCOM shifter interrupt */
void mfcom_shifter_interrupt_disable(uint32_t shifter);
/* disable MFCOM shifter error interrupt */
void mfcom_shifter_error_interrupt_disable(uint32_t shifter);
/* disable MFCOM timer interrupt */
void mfcom_timer_interrupt_disable(uint32_t timer);
/* disable MFCOM shifter dma */
void mfcom_shifter_dma_disable(uint32_t shifter);

#endif /* GD32A50X_MFCOM_H */
