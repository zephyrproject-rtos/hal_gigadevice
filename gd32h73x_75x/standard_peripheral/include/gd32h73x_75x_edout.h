/*!
    \file    gd32h73x_75x_edout.c
    \brief   definitions for the EDOUT

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

#ifndef GD32H73X_75X_EDOUT_H
#define GD32H73X_75X_EDOUT_H

#include "gd32h73x_75x.h"

/* EDOUT definition */
#define EDOUT                            EDOUT_BASE                        /*!< EDOUT base address */

/* registers definitions */
#define EDOUT_CTL                        REG32((EDOUT) + 0x00000000U)      /*!< EDOUT control register */
#define EDOUT_ENABLE                     REG32((EDOUT) + 0x00000004U)      /*!< EDOUT enable register */
#define EDOUT_LOC                        REG32((EDOUT) + 0x00000008U)      /*!< EDOUT location register */
#define EDOUT_OCNT                       REG32((EDOUT) + 0x0000000CU)      /*!< EDOUT output counter register */
#define EDOUT_LCNT                       REG32((EDOUT) + 0x00000010U)      /*!< EDOUT location counter register */
#define EDOUT_ZCR                        REG32((EDOUT) + 0x00000014U)      /*!< EDOUT Z-phase configure register */

/* bits definitions */
/* EDOUT_CTL */
#define EDOUT_CTL_POL                    BIT(0)                            /*!< active polarity of the B-phase output signal selection */

/* EDOUT_ENABLE */
#define EDOUT_ENABLE_EDOUTEN             BIT(0)                            /*!< EDOUT enable bit */

/* EDOUT_LOC */
#define EDOUT_LOC_LOCMAX                 BITS(0,15)                        /*!< maximum location value for one rotation */

/* EDOUT_OCNT */
#define EDOUT_OCNT_EDGC                  BITS(0,15)                        /*!< number of edges of the A-phase and the B-phase signal for the next update period */
#define EDOUT_OCNT_PDC                   BITS(16,31)                       /*!< phase difference between the A-phase and the B-phase signal for the next update period */

/* EDOUT_LCNT */
#define EDOUT_LCNT_LOCCNT                BITS(0,15)                        /*!< current location value */

/* EDOUT_ZCR */
#define EDOUT_ZCR_ZOSP                   BITS(0,15)                        /*!< Z-phase output start location */
#define EDOUT_ZCR_ZOWH                   BITS(16,23)                       /*!< Z-phase output width */
#define EDOUT_ZCR_ZOMD                   BIT(24)                           /*!< Z-phase output mode */

/* constants definitions */
/* EDOUT_CTL */
#define EDOUT_POL_POSITIVE               ((uint32_t)0x00000000U)           /*!< active polarity is positive */
#define EDOUT_POL_NEGATIVE               EDOUT_CTL_POL                     /*!< active polarity is negative */

/* EDOUT_ENABLE */
#define EDOUT_STATE_DISABLE              ((uint32_t)0x00000000U)           /*!< disable EDOUT */
#define EDOUT_STATE_ENABLE               EDOUT_ENABLE_EDOUTEN              /*!< enable EDOUT */

/* EDOUT_ZCR */
#define EDOUT_Z_OUTPUT_MODE0             ((uint32_t)0x00000000U)           /*!< Z-phase output according to the current location */
#define EDOUT_Z_OUTPUT_MODE1             EDOUT_ZCR_ZOMD                    /*!< Z-phase output according to the number of edges */

/* parameter check definitions */
#ifdef FW_DEBUG_ERR_REPORT

/* check the maximum location value */
#define LOCATION_LOW_VALUE                                  ((uint32_t)0x0000000FU)    /*!< minimum location value */
#define LOCATION_HIGH_VALUE                                 ((uint32_t)0x0000FFFFU)    /*!< maximum location value */
#define NOT_EDOUT_MAX_LOCATION_VAL(loc)                     ((LOCATION_HIGH_VALUE < (loc)) || \
                                                             (LOCATION_LOW_VALUE > (loc)) || \
                                                             (0U != (((loc) + 1U) % 4U)))

/* check the current location value */
#define NOT_EDOUT_CUR_LOCATION_VAL(loc)                     (LOCATION_HIGH_VALUE < (loc))

/* check the phase difference value */
#define PHASE_DIFF_LOW_VALUE                                ((uint16_t)0x0002U)    /*!< minimum phase difference value */
#define NOT_EDOUT_PHASE_DIFF_VAL(phase_diff)                (PHASE_DIFF_LOW_VALUE > (phase_diff))

/* check the Z-phase output start location value */
#define START_LOCATION_HIGH_VALUE                           ((uint32_t)0x0000FFFFU)    /*!< maximum start location value */
#define NOT_EDOUT_Z_PHASE_START_LOCATION_VAL(loc)           (START_LOCATION_HIGH_VALUE < (loc))

/* check the Z-phase output width value */
#define WIDTH_HIGH_VALUE                                    ((uint32_t)0x0000FFFFU)    /*!< maximum width value */
#define NOT_EDOUT_Z_PHASE_WIDTH_VAL(width)                  (WIDTH_HIGH_VALUE < (width))

#endif /* FW_DEBUG_ERR_REPORT */

/* function declarations */
/* deinitialize EDOUT */
void edout_deinit(void);
/* initialize EDOUT */
void edout_init(uint32_t pol, uint32_t max_loc, uint32_t cur_loc);
/* enable EDOUT */
void edout_enable(void);
/* disable EDOUT */
void edout_disable(void);
/* set B-phase active polarity */
void edout_polarity_config(uint32_t pol);
/* set the maximum location value for one rotation */
void edout_max_location_value_config(uint32_t max_loc);
/* update the output counter, used to set the phase difference and the number of edges for the next update period */
void edout_output_counter_update(int16_t num_edges, uint16_t phase_diff);
/* set the current location value */
void edout_current_location_config(uint32_t cur_loc);
/* get the current location value */
uint16_t edout_current_location_get(void);
/* configure Z-phase output mode */
void edout_z_output_mode_config(uint32_t mode);
/* configure Z-phase output start location and width */
void edout_z_output_start_loc_and_width_config(uint32_t start_loc, uint32_t width);

#endif /* GD32H73X_75X_EDOUT_H */
