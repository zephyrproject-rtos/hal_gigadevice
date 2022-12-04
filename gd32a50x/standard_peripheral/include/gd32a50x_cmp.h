/*!
    \file    gd32a50x_cmp.h
    \brief   definitions for the CMP

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

#ifndef GD32A50X_CMP_H
#define GD32A50X_CMP_H

#include "gd32a50x.h"

/* CMP definitions */
#define CMP                                      CMP_BASE + 0x00000000U         /*!< CMP base address */

/* registers definitions */
#define CMP_CS                                   REG32(CMP + 0x00000000U)       /*!< CMP control and status register */

/* bits definitions */
/* CMP_CS */
#define CMP_CS_EN                                BIT(0)                         /*!< CMP enable  */
#define CMP_CS_PM                                BITS(2,3)                      /*!< CMP mode */
#define CMP_CS_MESEL                             BITS(4,6)                      /*!< CMP_IM external input selection */
#define CMP_CS_MISEL                             BITS(7,9)                      /*!< CMP_IM internal input selection */
#define CMP_CS_PSEL                              BITS(10,12)                    /*!< CMP_IP input selection */
#define CMP_CS_OSEL                              BITS(13,14)                    /*!< CMP output selection */
#define CMP_CS_PL                                BIT(15)                        /*!< polarity of CMP output */
#define CMP_CS_HST                               BITS(16,17)                    /*!< CMP hysteresis */
#define CMP_CS_BLK                               BITS(18,20)                    /*!< CMP output blanking source */
#define CMP_CS_BEN                               BIT(22)                        /*!< scaler bridge enable bit */
#define CMP_CS_SEN                               BIT(23)                        /*!< voltage input scaler */
#define CMP_CS_OT                                BIT(30)                        /*!< CMP output */
#define CMP_CS_LK                                BIT(31)                        /*!< CMP lock */

/* constants definitions */
/* operating mode */
typedef enum {
    CMP_HIGHSPEED      = 0,                                                     /*!< high speed mode */
    CMP_MIDDLESPEED    = 1,                                                     /*!< middle speed mode */
    CMP_LOWSPEED       = 3                                                      /*!< low speed mode */
} operating_mode_enum;

/* inverting input */
typedef enum {
    CMP_1_4VREFINT     = 0,                                                     /*!< VREFINT /4 input */
    CMP_1_2VREFINT     = 8,                                                     /*!< VREFINT /2 input */
    CMP_3_4VREFINT     = 16,                                                    /*!< VREFINT *3/4 input */
    CMP_VREFINT        = 24,                                                    /*!< VREFINT input */
    CMP_DAC_OUT        = 32,                                                    /*!< DAC_OUT input */
    CMP_IM_PC11        = 40,                                                    /*!< PC11*/
    CMP_IM_PC10        = 41,                                                    /*!< PC10 */
    CMP_IM_PB8         = 42,                                                    /*!< PB8 */
    CMP_IM_PA0         = 43,                                                    /*!< PA0 */
    CMP_IM_PA3         = 44,                                                    /*!< PA3 */
    CMP_IM_PA4         = 45,                                                    /*!< PA4 */
    CMP_IM_PA5         = 46,                                                    /*!< PA5 */
    CMP_IM_PA6         = 47,                                                    /*!< PA6 */
} cmp_inverting_input_enum;

/* plus input */
typedef enum {
    CMP_IP_PC11 = 0,                                                            /*!< PC11*/
    CMP_IP_PC10,                                                                /*!< PC10 */
    CMP_IP_PB8,                                                                 /*!< PB8 */
    CMP_IP_PA0,                                                                 /*!< PA0 */
    CMP_IP_PA3,                                                                 /*!< PA3 */
    CMP_IP_PA4,                                                                 /*!< PA4 */
    CMP_IP_PA5,                                                                 /*!< PA5 */
    CMP_IP_PA6                                                                  /*!< PA6 */
} cmp_plus_input_enum;

/* hysteresis */
typedef enum {
    CMP_HYSTERESIS_NO = 0,                                                      /*!< output no hysteresis */
    CMP_HYSTERESIS_LOW,                                                         /*!< output low hysteresis */
    CMP_HYSTERESIS_MIDDLE,                                                      /*!< output middle hysteresis */
    CMP_HYSTERESIS_HIGH                                                         /*!< output high hysteresis */
} cmp_hysteresis_enum;

/* output */
typedef enum {
    CMP_OUTPUT_NONE = 0,                                                        /*!< output no selection */
    CMP_OUTPUT_TIMER0IC0,                                                       /*!< TIMER 0 channel0 input capture */
    CMP_OUTPUT_TIMER7IC0                                                        /*!< TIMER 7 channel0 input capture */
} cmp_output_enum;

/* output inv */
typedef enum {
    CMP_OUTPUT_POLARITY_NOINVERTED = 0,                                         /*!< output is not inverted */
    CMP_OUTPUT_POLARITY_INVERTED                                                /*!< output is inverted */
} cmp_output_inv_enum;

/* blanking_source*/
typedef enum {
    CMP_BLANKING_NONE = 0,                                                      /*!< no blanking */
    CMP_BLANKING_TIMER0_OC1,                                                    /*!< select TIMER0_CH1 as blanking source */
    CMP_BLANKING_TIMER7_OC1,                                                    /*!< select TIMER7_CH1 as blanking source */
    CMP_BLANKING_TIMER1_OC1                                                     /*!< select TIMER1_CH1 as blanking source */
} blanking_source_enum;

/* output state */
typedef enum {
    CMP_OUTPUTLEVEL_LOW = 0,                                                    /*!< the output is low */
    CMP_OUTPUTLEVEL_HIGH                                                        /*!< the output is high */
} cmp_output_state_enum;

/* CMP mode */
#define CS_CMPPM(regval)                         (BITS(2,3) & ((uint32_t)(regval) << 2))
#define CS_CMPPM_HIGHSPEED                       CS_CMPPM(0)                    /*!< CMP mode high speed */
#define CS_CMPPM_MIDDLESPEED                     CS_CMPPM(1)                    /*!< CMP mode middle speed */
#define CS_CMPPM_LOWSPEED                        CS_CMPPM(3)                    /*!< CMP mode low speed */

/* CMP  input */
/* IM input */
#define CS_CMPMSEL(regval)                       (BITS(4,9) & ((uint32_t)(regval) << 4))
#define CS_CMPMSEL_1_4VREFINT                    CS_CMPMSEL(0)                  /*!< CMP inverting internal input VREFINT *1/4 */
#define CS_CMPMSEL_1_2VREFINT                    CS_CMPMSEL(8)                  /*!< CMP inverting internal input VREFINT *1/2 */
#define CS_CMPMSEL_3_4VREFINT                    CS_CMPMSEL(16)                 /*!< CMP inverting internal input VREFINT *3/4 */
#define CS_CMPMSEL_VREFINT                       CS_CMPMSEL(24)                 /*!< CMP inverting internal input VREFINT */
#define CS_CMPMSEL_DAC_OUT                       CS_CMPMSEL(32)                 /*!< CMP inverting internal input DAC_OUT */
#define CS_CMPMSEL_PC11                          CS_CMPMSEL(40)                 /*!< CMP inverting external input PC11 */
#define CS_CMPMSEL_PC10                          CS_CMPMSEL(41)                 /*!< CMP inverting external input PC10 */
#define CS_CMPMSEL_PB8                           CS_CMPMSEL(42)                 /*!< CMP inverting external input PB8 */
#define CS_CMPMSEL_PA0                           CS_CMPMSEL(43)                 /*!< CMP inverting external input PA0 */
#define CS_CMPMSEL_PA3                           CS_CMPMSEL(44)                 /*!< CMP inverting external input PA3 */
#define CS_CMPMSEL_PA4                           CS_CMPMSEL(45)                 /*!< CMP inverting external input PA4 */
#define CS_CMPMSEL_PA5                           CS_CMPMSEL(46)                 /*!< CMP inverting external input PA5 */
#define CS_CMPMSEL_PA6                           CS_CMPMSEL(47)                 /*!< CMP inverting external input PA6 */

/* IP input */
#define CS_CMPPSEL(regval)                       (BITS(10,12) & ((uint32_t)(regval) << 10))
#define CS_CMPPSEL_PC11                          CS_CMPPSEL(0)                  /*!< CMP plus input PC11 */
#define CS_CMPPSEL_PC10                          CS_CMPPSEL(1)                  /*!< CMP plus input PC10 */
#define CS_CMPPSEL_PB8                           CS_CMPPSEL(2)                  /*!< CMP plus input PB8 */
#define CS_CMPPSEL_PA0                           CS_CMPPSEL(3)                  /*!< CMP plus input PA0 */
#define CS_CMPPSEL_PA3                           CS_CMPPSEL(4)                  /*!< CMP plus input PA3 */
#define CS_CMPPSEL_PA4                           CS_CMPPSEL(5)                  /*!< CMP plus input PA4 */
#define CS_CMPPSEL_PA5                           CS_CMPPSEL(6)                  /*!< CMP plus input PA5 */
#define CS_CMPPSEL_PA6                           CS_CMPPSEL(7)                  /*!< CMP plus input PA6 */

/* CMP output selection */
#define CS_CMPOSEL(regval)                       (BITS(13,14) & ((uint32_t)(regval) << 13))
#define CS_CMPOSEL_NOSELECTION                   CS_CMPOSEL(0)                  /*!< CMP no output selection */
#define CS_CMPOSEL_TIMER0IC0                     CS_CMPOSEL(1)                  /*!< CMP as TIMER0 channel0 input capture source */
#define CS_CMPOSEL_TIMER7IC0                     CS_CMPOSEL(2)                  /*!< CMP as TIMER7 channel0 input capture source */

/* CMP output polarity*/
#define CS_CMPPL(regval)                         (BIT(15) & ((uint32_t)(regval) << 15))
#define CS_CMPPL_NOT                             CS_CMPPL(0)                    /*!< CMP output not inverted */
#define CS_CMPPL_INV                             CS_CMPPL(1)                    /*!< CMP output inverted */

/* CMP hysteresis */
#define CS_CMPHST(regval)                        (BITS(16,17) & ((uint32_t)(regval) << 16))
#define CS_CMPHST_HYSTERESIS_NO                  CS_CMPHST(0)                   /*!< CMP output no hysteresis */
#define CS_CMPHST_HYSTERESIS_LOW                 CS_CMPHST(1)                   /*!< CMP output low hysteresis */
#define CS_CMPHST_HYSTERESIS_MIDDLE              CS_CMPHST(2)                   /*!< CMP output middle hysteresis */
#define CS_CMPHST_HYSTERESIS_HIGH                CS_CMPHST(3)                   /*!< CMP output high hysteresis */

/* CMP blanking suorce */
#define CS_CMPBLK(regval)                        (BITS(18,20) & ((uint32_t)(regval) << 18))
#define CS_CMPBLK_NOBLK                          CS_CMPBLK(0)                   /*!< CMP no blanking */
#define CS_CMPBLK_TIMER0_OC1                     CS_CMPBLK(1)                   /*!< CMP TIMER0 OC1 selected as blanking source */
#define CS_CMPBLK_TIMER7_OC1                     CS_CMPBLK(2)                   /*!< CMP TIMER7 OC1 selected as blanking source */
#define CS_CMPBLK_TIMER1_OC1                     CS_CMPBLK(4)                   /*!< CMP TIMER1 OC1 selected as blanking source */

/* CMP bridge enable*/
#define CS_CMPBEN(regval)                        (BIT(22) & ((uint32_t)(regval) << 22))
#define CS_CMPBEN_DISABLE                        CS_CMPBEN(0)                   /*!< CMP bridge enable */
#define CS_CMPBEN_ENABLE                         CS_CMPBEN(1)                   /*!< CMP bridge disable */

/* CMP voltage scaler*/
#define CS_CMPSEN(regval)                        (BIT(23) & ((uint32_t)(regval) << 23))
#define CS_CMPSEN_DISABLE                        CS_CMPSEN(0)                   /*!< CMP voltage scaler enable */
#define CS_CMPSEN_ENABLE                         CS_CMPSEN(1)                   /*!< CMP voltage scaler disable */

/* CMP lock bit*/
#define CS_CMPLK(regval)                         (BIT(31) & ((uint32_t)(regval) << 31))
#define CS_CMPLK_DISABLE                         CS_CMPLK(0)                    /*!< CMP_CS bits are read-write */
#define CS_CMPLK_ENABLE                          CS_CMPLK(1)                    /*!< CMP_CS bits are read-only */

/* function declarations */
/* initialization functions */
/* deinitialize comparator */
void cmp_deinit(void);
/* initialize comparator mode */
void cmp_mode_init(operating_mode_enum operating_mode, cmp_inverting_input_enum inverting_input, cmp_plus_input_enum plus_input,
                   cmp_hysteresis_enum output_hysteresis);
/* initialize comparator output */
void cmp_output_init(cmp_output_enum output_selection, cmp_output_inv_enum output_polarity);
/* initialize comparator blanking function */
void cmp_outputblank_init(blanking_source_enum blanking_source_selection);

/* enable functions */
/* enable comparator */
void cmp_enable(void);
/* disable comparator */
void cmp_disable(void);
/* enable the voltage scaler */
void cmp_voltage_scaler_enable(void);
/* disable the voltage scaler */
void cmp_voltage_scaler_disable(void);
/* enable the scaler bridge */
void cmp_scaler_bridge_enable(void);
/* disable the scaler bridge */
void cmp_scaler_bridge_disable(void);
/* lock the comparator */
void cmp_lock_enable(void);

/* output functions */
/* get output level */
cmp_output_state_enum cmp_output_level_get(void);

#endif /* GD32A50X_CMP_H */
