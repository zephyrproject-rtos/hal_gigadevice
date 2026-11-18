/*!
    \file    gd32h73x_75x_exti.c
    \brief   EXTI driver

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

#include "gd32h73x_75x_exti.h"

#define EXTI_REG_RESET_VALUE            ((uint32_t)0x00000000U)

/*!
    \brief      deinitialize the EXTI (API_ID(0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void exti_deinit(void)
{
    /* reset the value of all the EXTI registers */
    EXTI_INTEN0 = EXTI_REG_RESET_VALUE;
    EXTI_EVEN0  = EXTI_REG_RESET_VALUE;
    EXTI_RTEN0  = EXTI_REG_RESET_VALUE;
    EXTI_FTEN0  = EXTI_REG_RESET_VALUE;
    EXTI_SWIEV0 = EXTI_REG_RESET_VALUE;
    EXTI_INTEN1 = EXTI_REG_RESET_VALUE;
    EXTI_EVEN1  = EXTI_REG_RESET_VALUE;
    EXTI_RTEN1  = EXTI_REG_RESET_VALUE;
    EXTI_FTEN1  = EXTI_REG_RESET_VALUE;
    EXTI_SWIEV1 = EXTI_REG_RESET_VALUE;
}

/*!
    \brief      initialize the EXTI line x (API_ID(0x0002U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[in]  mode: interrupt or event mode, refer to exti_mode_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_INTERRUPT: interrupt mode
      \arg        EXTI_EVENT: event mode
    \param[in]  trig_type: interrupt and event trigger type, refer to exti_trig_type_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_TRIG_RISING: rising edge trigger
      \arg        EXTI_TRIG_FALLING: falling trigger
      \arg        EXTI_TRIG_BOTH: rising and falling trigger
      \arg        EXTI_TRIG_NONE: without rising edge or falling edge trigger
    \param[out] none
    \retval     none
*/
void exti_init(exti_line_enum linex, exti_mode_enum mode, exti_trig_type_enum trig_type)
{
    /* reset the EXTI line x */
    EXTI_INTEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
    EXTI_EVEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
    EXTI_RTEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
    EXTI_FTEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);

    /* set the EXTI mode and enable the interrupts or events from EXTI line x */
    switch(mode) {
    case EXTI_INTERRUPT:
        EXTI_INTEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
        break;
    case EXTI_EVENT:
        EXTI_EVEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
        break;
    default:
        break;
    }

    /* set the EXTI trigger type */
    switch(trig_type) {
    case EXTI_TRIG_RISING:
        EXTI_RTEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
        EXTI_FTEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
        break;
    case EXTI_TRIG_FALLING:
        EXTI_RTEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
        EXTI_FTEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
        break;
    case EXTI_TRIG_BOTH:
        EXTI_RTEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
        EXTI_FTEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
        break;
    case EXTI_TRIG_NONE:
    default:
        break;
    }
}

/*!
    \brief      enable the interrupts from EXTI line x (API_ID(0x0003U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_interrupt_enable(exti_line_enum linex)
{
    EXTI_INTEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
}

/*!
    \brief      disable the interrupts from EXTI line x (API_ID(0x0004U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_interrupt_disable(exti_line_enum linex)
{
    EXTI_INTEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
}

/*!
    \brief      enable the events from EXTI line x (API_ID(0x0005U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_event_enable(exti_line_enum linex)
{
    EXTI_EVEN(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
}

/*!
    \brief      disable the events from EXTI line x (API_ID(0x0006U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_event_disable(exti_line_enum linex)
{
    EXTI_EVEN(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
}

/*!
    \brief      enable the software interrupt event from EXTI line x (API_ID(0x0007U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_software_interrupt_enable(exti_line_enum linex)
{
    EXTI_SWIEV(EXTI_REG_VAL(linex)) |= EXTI_BIT_POS(linex);
}

/*!
    \brief      disable the software interrupt event from EXTI line x (API_ID(0x0008U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_software_interrupt_disable(exti_line_enum linex)
{
    EXTI_SWIEV(EXTI_REG_VAL(linex)) &= ~EXTI_BIT_POS(linex);
}

/*!
    \brief      get EXTI line x interrupt pending flag (API_ID(0x0009U))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     FlagStatus: status of flag (RESET or SET)
*/
FlagStatus exti_flag_get(exti_line_enum linex)
{
    FlagStatus status;

    if(RESET != (EXTI_PD(EXTI_REG_VAL(linex)) & EXTI_BIT_POS(linex))) {
        status = SET;
    } else {
        status = RESET;
    }

    return status;
}

/*!
    \brief      clear EXTI line x interrupt pending flag (API_ID(0x000AU))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_flag_clear(exti_line_enum linex)
{
    EXTI_PD(EXTI_REG_VAL(linex)) = EXTI_BIT_POS(linex);
}

/*!
    \brief      get EXTI line x interrupt pending flag (API_ID(0x000BU))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     FlagStatus: status of flag (RESET or SET)
*/
FlagStatus exti_interrupt_flag_get(exti_line_enum linex)
{
    FlagStatus status;

    if(RESET != (EXTI_PD(EXTI_REG_VAL(linex)) & EXTI_BIT_POS(linex))) {
        status = SET;
    } else {
        status = RESET;
    }

    return status;
}

/*!
    \brief      clear EXTI line x interrupt pending flag (API_ID(0x000CU))
    \param[in]  linex: EXTI line number, refer to exti_line_enum
                only one parameter can be selected which is shown as below:
      \arg        EXTI_x (x=0..37): EXTI line x
    \param[out] none
    \retval     none
*/
void exti_interrupt_flag_clear(exti_line_enum linex)
{
    EXTI_PD(EXTI_REG_VAL(linex)) = EXTI_BIT_POS(linex);
}
