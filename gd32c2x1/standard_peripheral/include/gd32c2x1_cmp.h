/*!
    \file    gd32c2x1_cmp.h
    \brief   definitions for the CMP

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

#ifndef GD32C2X1_CMP_H
#define GD32C2X1_CMP_H

#include "gd32c2x1.h"

/* CMP definitions */
#define CMP                                      CMP_BASE                         /*!< CMP base address */

/* registers definitions */
#define CMP0_CS                                  REG32((CMP) + 0x00000000U)       /*!< CMP0 control and status register */
#define CMP1_CS                                  REG32((CMP) + 0x00000004U)       /*!< CMP1 control and status register */

/* bits definitions */
/* CMPx_CS */
#define CMP_CS_CMPXEN                            BIT(0)                           /*!< CMPx enable */
#define CMP_CS_CMPXM                             BITS(2,3)                        /*!< CMPx mode */
#define CMP_CS_CMPXMSEL                          BITS(4,6)                        /*!< CMP_IM input selection */
#define CMP_CS_CMPXSW                            BIT(11)                          /*!< CMPx switch mode enable */
#define CMP_CS_CMPXPL                            BIT(15)                          /*!< CMPx output polarity */
#define CMP_CS_CMPXHST                           BITS(16,17)                      /*!< CMPx hysteresis */
#define CMP_CS_CMPXBLK                           BITS(18,20)                      /*!< CMPx output blanking source */
#define CMP_CS_CMPXBEN                           BIT(22)                          /*!< CMPx scaler bridge enable bit */
#define CMP_CS_CMPXSEN                           BIT(23)                          /*!< CMPx voltage input scaler */
#define CMP_CS_CMPXO                             BIT(30)                          /*!< CMPx output state bit */
#define CMP_CS_CMPXLK                            BIT(31)                          /*!< CMPx lock */

/* constants definitions */
/* CMP units */
typedef enum{
    CMP0,                                                                         /*!< comparator 0 */
    CMP1                                                                          /*!< comparator 1 */
}cmp_enum;

/* CMP operating mode */
#define CS_CMPXM(regval)                         (BITS(2,3) & ((uint32_t)(regval) << 2U))
#define CMP_MODE_HIGHSPEED                       CS_CMPXM(0)                      /*!< CMP mode high speed */
#define CMP_MODE_MIDDLESPEED                     CS_CMPXM(1)                      /*!< CMP mode middle speed */
#define CMP_MODE_LOWSPEED                        CS_CMPXM(2)                      /*!< CMP mode low speed */
#define CMP_MODE_VERYLOWSPEED                    CS_CMPXM(3)                      /*!< CMP mode very low speed */

/* CMP hysteresis */
#define CS_CMPXHST(regval)                       (BITS(16,17) & ((uint32_t)(regval) << 16U))
#define CMP_HYSTERESIS_NO                        CS_CMPXHST(0)                    /*!< CMP output no hysteresis */
#define CMP_HYSTERESIS_LOW                       CS_CMPXHST(1)                    /*!< CMP output low hysteresis */
#define CMP_HYSTERESIS_MIDDLE                    CS_CMPXHST(2)                    /*!< CMP output middle hysteresis */
#define CMP_HYSTERESIS_HIGH                      CS_CMPXHST(3)                    /*!< CMP output high hysteresis */

/* CMP inverting input */
#define CS_CMPXMSEL(regval)                      (BITS(4,6) & ((uint32_t)(regval) << 4U))
#define CMP_INVERTING_INPUT_1_4VREFINT           CS_CMPXMSEL(0)                   /*!< CMP inverting input 1/4 Vrefint */
#define CMP_INVERTING_INPUT_1_2VREFINT           CS_CMPXMSEL(1)                   /*!< CMP inverting input 1/2 Vrefint */
#define CMP_INVERTING_INPUT_3_4VREFINT           CS_CMPXMSEL(2)                   /*!< CMP inverting input 3/4 Vrefint */
#define CMP_INVERTING_INPUT_VREFINT              CS_CMPXMSEL(3)                   /*!< CMP inverting input Vrefint */
#define CMP_INVERTING_INPUT_PB2_PB6              CS_CMPXMSEL(4)                   /*!< CMP inverting input PB2 for CMP0 or PB6 for CMP1 */
#define CMP_INVERTING_INPUT_PA0_PA2              CS_CMPXMSEL(5)                   /*!< CMP inverting input PA0 for CMP0 or PA2 for CMP1 */
#define CMP_INVERTING_INPUT_PB1_PB3              CS_CMPXMSEL(6)                   /*!< CMP inverting input PB1 for CMP0 or PB3 for CMP1 */
#define CMP_INVERTING_INPUT_VSSA_PB4             CS_CMPXMSEL(7)                   /*!< CMP inverting input VSSA for CMP0 or PB4 for CMP1 */

/* CMP output polarity*/
#define CS_CMPXPL(regval)                        (BIT(15) & ((uint32_t)(regval) << 15U))
#define CMP_OUTPUT_POLARITY_NONINVERTED          CS_CMPXPL(0)                     /*!< CMP output not inverted */
#define CMP_OUTPUT_POLARITY_INVERTED             CS_CMPXPL(1)                     /*!< CMP output inverted */

/* CMP blanking suorce */
#define CS_CMPXBLK(regval)                       (BITS(18,20) & ((uint32_t)(regval) << 18U))
#define CMP_BLANKING_NONE                        CS_CMPXBLK(0)                    /*!< CMP no blanking source */
#define CMP_BLANKING_TIMER0_OC1                  CS_CMPXBLK(1)                    /*!< CMP TIMER0_CH1 output compare signal selected as blanking source */
#define CMP_BLANKING_TIMER2_OC1                  CS_CMPXBLK(2)                    /*!< CMP TIMER2_CH1 output compare signal selected as blanking source */
#define CMP_BLANKING_TIMER13_OC0                 CS_CMPXBLK(4)                    /*!< CMP TIMER13_CH0 output compare signal selected as blanking source */
#define CMP_BLANKING_TIMER15_OC0                 CS_CMPXBLK(5)                    /*!< CMP TIMER15_CH0 output compare signal selected as blanking source */

/* CMP output level */
#define CMP_OUTPUTLEVEL_HIGH                     ((uint32_t)0x00000001U)           /*!< CMP output high */
#define CMP_OUTPUTLEVEL_LOW                      ((uint32_t)0x00000000U)           /*!< CMP output low */

/* function declarations */
/* initialization functions */
/* CMP deinit */
void cmp_deinit(cmp_enum cmp_periph);
/* CMP mode init */
void cmp_mode_init(cmp_enum cmp_periph, uint32_t operating_mode, uint32_t inverting_input, uint32_t output_hysteresis);
/* CMP output init */
void cmp_output_init(cmp_enum cmp_periph, uint32_t output_polarity);
/* CMP output blanking function init */
void cmp_blanking_init(cmp_enum cmp_periph,uint32_t blanking_source_selection);

/* enable functions */
/* enable CMP */
void cmp_enable(cmp_enum cmp_periph);
/* disable CMP */
void cmp_disable(cmp_enum cmp_periph);
/* enable CMP switch */
void cmp_switch_enable(cmp_enum cmp_periph);
/* disable CMP switch */
void cmp_switch_disable(cmp_enum cmp_periph);
/* lock the CMP */
void cmp_lock_enable(cmp_enum cmp_periph);
/* enable the voltage scaler */
void cmp_voltage_scaler_enable(cmp_enum cmp_periph);
/* disable the voltage scaler */
void cmp_voltage_scaler_disable(cmp_enum cmp_periph);
/* enable the scaler bridge */
void cmp_scaler_bridge_enable(cmp_enum cmp_periph);
/* disable the scaler bridge */
void cmp_scaler_bridge_disable(cmp_enum cmp_periph);

/* get state related functions */
/* get output level */
uint32_t cmp_output_level_get(cmp_enum cmp_periph);

#endif /* GD32C23X_CMP_H */
