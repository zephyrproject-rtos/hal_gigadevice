/*!
    \file    gd32c2x1_dbg.c
    \brief   DBG driver

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

#include "gd32c2x1_dbg.h"

#define DBG_RESET_VAL       ((uint32_t)0x00000000U)   /*!< DBG reset value */
#define DBG_LOWPOWER_MASK   ((uint32_t)0x00000007U)   /*!< DBG low power mask */

/*!
    \brief      deinitialize the DBG (API_ID: (0x0001U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dbg_deinit(void)
{
    DBG_CTL0 = DBG_RESET_VAL;
    DBG_CTL1 = DBG_RESET_VAL;
}

/*!
    \brief      read DBG_ID code register (API_ID: (0x0002U))
    \param[in]  none
    \param[out] none
    \retval     DBG_ID code
*/
uint32_t dbg_id_get(void)
{
    return DBG_ID;
}

/*!
    \brief      enable low power behavior when the MCU is in debug mode (API_ID: (0x0003U))
    \param[in]  dbg_low_power:
                one or more parameters can be selected which are shown as below:
      \arg        DBG_LOW_POWER_SLEEP: keep debugger connection during sleep mode
      \arg        DBG_LOW_POWER_DEEPSLEEP: keep debugger connection during deepsleep mode
      \arg        DBG_LOW_POWER_STANDBY: keep debugger connection during standby mode
    \param[out] none
    \retval     none
*/
void dbg_low_power_enable(uint32_t dbg_low_power)
{
    DBG_CTL0 |= (dbg_low_power & DBG_LOWPOWER_MASK);
}

/*!
    \brief      disable low power behavior when the MCU is in debug mode (API_ID: (0x0004U))
    \param[in]  dbg_low_power:
                one or more parameters can be selected which are shown as below:
      \arg        DBG_LOW_POWER_SLEEP: keep debugger connection during sleep mode
      \arg        DBG_LOW_POWER_DEEPSLEEP: keep debugger connection during deepsleep mode
      \arg        DBG_LOW_POWER_STANDBY: keep debugger connection during standby mode
    \param[out] none
    \retval     none
*/
void dbg_low_power_disable(uint32_t dbg_low_power)
{
    DBG_CTL0 &= ~(dbg_low_power & DBG_LOWPOWER_MASK);
}

/*!
    \brief      enable peripheral behavior when the MCU is in debug mode (API_ID: (0x0005U))
    \param[in]  dbg_periph: DBG peripheral
                only one parameter can be selected which is shown as below:
      \arg        DBG_FWDGT_HOLD: hold FWDGT counter when core is halted
      \arg        DBG_WWDGT_HOLD: hold WWDGT counter when core is halted
      \arg        DBG_TIMER0_HOLD: hold TIMER0 counter when core is halted
      \arg        DBG_TIMER2_HOLD: hold TIMER2 counter when core is halted
      \arg        DBG_TIMER13_HOLD: hold TIMER13 counter when core is halted
      \arg        DBG_TIMER15_HOLD: hold TIMER15 counter when core is halted
      \arg        DBG_TIMER16_HOLD: hold TIMER16 counter when core is halted
      \arg        DBG_I2C0_HOLD: hold I2C0 SMBUS when core is halted
      \arg        DBG_I2C1_HOLD: hold I2C1 SMBUS when core is halted
      \arg        DBG_RTC_HOLD: hold RTC calendar and wakeup counter when core is halted
    \param[out] none
    \retval     none
*/
void dbg_periph_enable(dbg_periph_enum dbg_periph)
{
    DBG_REG_VAL(dbg_periph) |= BIT(DBG_BIT_POS(dbg_periph));
}

/*!
    \brief      disable peripheral behavior when the MCU is in debug mode (API_ID: (0x0006U))
    \param[in]  dbg_periph: DBG peripheral
                only one parameter can be selected which is shown as below:
      \arg        DBG_FWDGT_HOLD: hold FWDGT counter when core is halted
      \arg        DBG_WWDGT_HOLD: hold WWDGT counter when core is halted
      \arg        DBG_TIMER0_HOLD: hold TIMER0 counter when core is halted
      \arg        DBG_TIMER2_HOLD: hold TIMER2 counter when core is halted
      \arg        DBG_TIMER13_HOLD: hold TIMER13 counter when core is halted
      \arg        DBG_TIMER15_HOLD: hold TIMER15 counter when core is halted
      \arg        DBG_TIMER16_HOLD: hold TIMER16 counter when core is halted
      \arg        DBG_I2C0_HOLD: hold I2C0 SMBUS when core is halted
      \arg        DBG_I2C1_HOLD: hold I2C1 SMBUS when core is halted
      \arg        DBG_RTC_HOLD: hold RTC calendar and wakeup counter when core is halted
    \param[out] none
    \retval     none
*/
void dbg_periph_disable(dbg_periph_enum dbg_periph)
{
    DBG_REG_VAL(dbg_periph) &= ~BIT(DBG_BIT_POS(dbg_periph));
}
