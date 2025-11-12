/*!
    \file    gd32c2x1_fmc.c
    \brief   FMC driver

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

#include "gd32c2x1_fmc.h"

#define FMC_FLAG_MASK                              ((uint32_t)0x0000C2FBU)                          /*!< FMC flag mask */
#define FMC_INTERRUPT_CFG_MASK                     ((uint32_t)0x07000000U)                          /*!< FMC interrupt config mask */
#define FMC_INTERRUPT_FLAG_MASK                    ((uint32_t)0x00004003U)                          /*!< FMC interrupt flag mask */
#define FMC_DFLASH_WP_CFG_MASK                     ((uint32_t)0x0000000FU)                          /*!< FMC data flash config mask */

/*!
    \brief      unlock the main FMC operation (API_ID(0x0001U))
                it is better to used in pairs with fmc_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_unlock(void)
{
    if(0U != (FMC_CTL & FMC_CTL_LK)) {
        /* write the FMC unlock key */
        FMC_KEY = FMC_UNLOCK_KEY0;
        FMC_KEY = FMC_UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the main FMC operation (API_ID(0x0002U))
                it is better to used in pairs with fmc_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_lock(void)
{
    /* set the LK bit */
    FMC_CTL |= FMC_CTL_LK;
}

/*!
    \brief      get the main flash empty check status (API_ID(0x0003U))
    \param[in]  none
    \param[out] none
    \retval     empty check status
*/
FlagStatus fmc_main_flash_empty_stat_get(void)
{
    FlagStatus status;
    
    if(0U!=(FMC_WS & FMC_WS_MFPE)) {
        status = SET;
    } else {
        status = RESET;
    }

    return status;
}

/**
  * \brief      modify the main flash empty check status (API_ID(0x0004U))
    \param[in]  empty_check_status: status of check
                only one parameter can be selected which is shown as below:
      \arg        FMC_WS_MFPE_PROGRAMMED: the first location of main flash is programmed
      \arg        FMC_WS_MFPE_EMPTY: the first location of main flash is empty
    \param[out] none
    \retval     none
*/
void fmc_main_flash_empty_stat_modify(uint32_t empty_check_status)
{
    FMC_WS &= ~FMC_WS_MFPE;
    FMC_WS |= empty_check_status & FMC_WS_MFPE;
}

/*!
    \brief      set the wait state (API_ID(0x0005U))
    \param[in]  wscnt: wait state
                only one parameter can be selected which is shown as below:
      \arg        FMC_WAIT_STATE_0: 0 wait state added
      \arg        FMC_WAIT_STATE_1: 1 wait state added
    \param[out] none
    \retval     none
*/
void fmc_wscnt_set(uint32_t wscnt)
{
    uint32_t reg = 0U;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_FMC_WAIT_STATE(wscnt)) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x0005U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* set the wait state counter value */
        reg = FMC_WS;
        reg &= ~FMC_WS_WSCNT;
        reg |= wscnt & FMC_WS_WSCNT;
        FMC_WS = reg;
    }
}

/*!
    \brief      enable pre-fetch (API_ID(0x0006U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_enable(void)
{
    FMC_WS |= FMC_WS_PFEN;
}

/*!
    \brief      disable pre-fetch (API_ID(0x0007U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_disable(void)
{
    FMC_WS &= ~FMC_WS_PFEN;
}

/*!
    \brief      enable IBUS cache (API_ID(0x0008U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_icache_enable(void)
{
    FMC_WS |= FMC_WS_ICEN;
}

/*!
    \brief      disable IBUS cache (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_icache_disable(void)
{
    FMC_WS &= ~FMC_WS_ICEN;
}

/*!
    \brief      reset IBUS cache (API_ID(0x000AU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_icache_reset(void)
{
    FMC_WS |= FMC_WS_ICRST;
}

/*!
    \brief      erase page (API_ID(0x000BU))
    \param[in]  page_number: page offset
                only one parameter can be selected which is shown as below:
      \arg        0 ~ (max count of pages)-1
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum fmc_page_erase(uint32_t page_number)
{
    fmc_state_enum fmc_state;
#ifdef FW_DEBUG_ERR_REPORT
    if(page_number >= MAIN_FLASH_PAGE_TOTAL_NUM) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x000BU), ERR_PARAM_OUT_OF_RANGE);
        fmc_state = FMC_UNDEFINEDERR;
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* select the bank page in */
            FMC_CTL &= ~FMC_CTL_PN;
            FMC_CTL |= page_number << CTL_PN_OFFSET;
            FMC_CTL |= FMC_CTL_PER;

            /* start page erase */
            FMC_CTL |= FMC_CTL_START;

            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

            FMC_CTL &= ~FMC_CTL_PER;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      erase whole chip (API_ID(0x000CU))
    \param[in]  none
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
*/
fmc_state_enum fmc_mass_erase(void)
{
    fmc_state_enum fmc_state;

    /* wait for the FMC ready */
    fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start chip erase */
        FMC_CTL |= FMC_CTL_MER;
        FMC_CTL |= FMC_CTL_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MER bit */
        FMC_CTL &= ~FMC_CTL_MER;
    }
    /* return the fmc state */
    return fmc_state;
}

/*!
    \brief      program a double word at the given address in main flash (API_ID(0x000DU))
    \param[in]  address: address to program
    \param[in]  data: double word to program
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data)
{
    uint32_t data0, data1;
    fmc_state_enum fmc_state;
#ifdef FW_DEBUG_ERR_REPORT
    if((address % 8U) != 0U) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x000DU), ERR_PARAM_OUT_OF_RANGE);
        fmc_state = FMC_UNDEFINEDERR;
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        
        data0 = (uint32_t)(data & 0xFFFFFFFFU);
        data1 = (uint32_t)((data >> 32U) & 0xFFFFFFFFU);
        
        if(FMC_READY == fmc_state) {
            /* set the PG bit to start program */
            FMC_CTL |= FMC_CTL_PG;
            REG32(address) = data0;
            REG32(address + 4U) = data1;
        
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        
            /* reset the PG bit */
            FMC_CTL &= ~FMC_CTL_PG;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      fast program a row at the corresponding address (API_ID(0x000EU))
    \param[in]  address: address to program
    \param[in]  data_buf: data buffer to program
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum fmc_fast_program(uint32_t address, uint32_t data_buf)
{
    uint8_t index;
    fmc_state_enum fmc_state;
#ifdef FW_DEBUG_ERR_REPORT
    if((address % 16U) != 0U) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x000EU), ERR_PARAM_OUT_OF_RANGE);
        fmc_state = FMC_UNDEFINEDERR;
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        
        if(FMC_READY == fmc_state) {
            /* set the FSTPG bit to start program */
            FMC_CTL |= FMC_CTL_FSTPG;
        
            /* program the row data */
            for(index = 0U; index < DOUBLEWORD_CNT_IN_ROW; index++) {
                REG32(address) = REG32(data_buf);
                REG32(address + 4U) = REG32(data_buf + 4U);
                address += 8U;
                data_buf += 8U;
            }
        
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        
            /* reset the FSTPG bit */
            FMC_CTL &= ~FMC_CTL_FSTPG;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      enable debugger (API_ID(0x000FU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_debugger_enable(void)
{
    FMC_WS |= FMC_WS_DBGEN;
}

/*!
  * \brief      disable debugger (API_ID(0x0010U))
    \param[in]  none
    \param[out] none
    \retval     none
  */
void fmc_debugger_disable(void)
{
    FMC_WS &= ~FMC_WS_DBGEN;
}

/*!
    \brief      enable secure area protection (API_ID(0x0011U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_scr_area_enable(void)
{
    FMC_CTL |= FMC_CTL_SCR;
}

/*!
    \brief      unlock the option bytes operation (API_ID(0x0012U))
                it is better to used in pairs with ob_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_unlock(void)
{
    if(0U != (FMC_CTL & FMC_CTL_OBLK)) {
        /* write the FMC ob unlock key */
        FMC_OBKEY = OB_UNLOCK_KEY0;
        FMC_OBKEY = OB_UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the option bytes operation (API_ID(0x0013U))
                it is better to used in pairs with ob_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_lock(void)
{
    /* reset the OBWEN bit */
    FMC_CTL &= ~FMC_CTL_OBLK;
}

/*!
    \brief      reload the option bytes operation (API_ID(0x0014U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_reload(void)
{
    /* set the OBRLD bit */
    FMC_CTL |= FMC_CTL_OBRLD;
}

/*!
    \brief      program the option bytes USER (API_ID(0x0015U))
                programmer must ensure FMC & option bytes are both unlocked before calling this function
                this function can only clear the corresponding bits to be 0 rather than 1.
    \param[in]  ob_user: user option bytes. When write one parameter, the corresponding mask should be set.
                one or more (bitwise OR) parameters can be selected which are shown as below:
      \arg        OB_NRST_PIN_INPUT_MODE/OB_NRST_PIN_NORMAL_GPIO/OB_NRST_PIN_INPUT_OUTPUT_MODE: reset pin mode
      \arg        OB_NBOOT0_VALUE_0/OB_NBOOT0_VALUE_1: boot0 value 0 or value 1
      \arg        OB_NBOOT1_VALUE_0/OB_NBOOT1_VALUE_1: boot1 value 0 or value 1
      \arg        OB_SWBOOT0_FROM_OB_BOOT0/OB_SWBOOT0_FROM_PIN: boot by option byte value or by PB8/BOOT0 pin
      \arg        OB_SRAM_PARITY_CHECK_ENABLE/OB_SRAM_PARITY_CHECK_DISABLE: SRAM parity check enable configuration
      \arg        OB_SRAM_ECC_ENABLE/OB_SRAM_ECC_DISABLE: SRAM ECC enable configuration
      \arg        OB_HXTAL_REMAP_ENABLE/OB_HXTAL_REMAP_DISABLE: HXTAL remapping
      \arg        OB_WWDGT_HW/OB_WWDGT_SW: hardware or software window watchdog timer
      \arg        OB_FWDGT_HW/OB_FWDGT_SW: hardware or software free watchdog timer
      \arg        OB_STDBY_RST/OB_STDBY_NRST: reset or not entering standby mode
      \arg        OB_DEEPSLEEP_RST/OB_DEEPSLEEP_NRST: reset or not entering deep sleep mode
      \arg        OB_BORR_TH_VALUE0/OB_BORR_TH_VALUE1/OB_BORR_TH_VALUE2/OB_BORR_TH_VALUE3: BOR rising threshold value
      \arg        OB_BORF_TH_VALUE0/OB_BORF_TH_VALUE1/OB_BORF_TH_VALUE2/OB_BORF_TH_VALUE3: BOR falling threshold value
      \arg        OB_BORST_DISABLE/OB_BORST_ENABLE: BOR reset configuration
    \param[in]  ob_user_mask: user bits mask. They correspond to the above parameter value one by one.
                  one or more (bitwise OR) parameters can be selected which are shown as below:
      \arg        FMC_OBCTL_NRST_MDSEL: reset pin mode bit mask
      \arg        FMC_OBCTL_NBOOT0: NBOOT0 option bit mask
      \arg        FMC_OBCTL_NBOOT1: NBOOT1 option bit mask
      \arg        FMC_OBCTL_SWBT0: software BOOT0 bit mask
      \arg        FMC_OBCTL_SRAM_ECC_EN: SRAM ECC disable bit mask
      \arg        FMC_OBCTL_HXTAL_REMAP: HXTAL remapping bit mask
      \arg        FMC_OBCTL_NWWDG_HW: window watchdog configuration bit mask
      \arg        FMC_OBCTL_NFWDG_HW: free watchdog configuration bit mask
      \arg        FMC_OBCTL_NRST_STDBY: option byte standby reset bit mask
      \arg        FMC_OBCTL_NRST_DPSLP: option byte deepsleep reset bit mask
      \arg        FMC_OBCTL_BORR_TH: BORR threshold bits mask
      \arg        FMC_OBCTL_BORF_TH: BORF threshold bits mask
      \arg        FMC_OBCTL_BORST_EN: brown out reset bit mask
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: high security protection
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum ob_user_write(uint32_t ob_user, uint32_t ob_user_mask)
{
    uint32_t obctl_reg;
    fmc_state_enum fmc_state = FMC_UNDEFINEDERR;
#ifdef FW_DEBUG_ERR_REPORT
    /* ob_user_mask is not supported */
    if(NOT_FMC_OB_USER_MASK(ob_user_mask)){
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x0015U), ERR_PARAM_INVALID);
    /* ob_user doesn't match ob_user_mask */
    } else if( 0u != (ob_user & (~ob_user_mask))){
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x0015U), ERR_PARAM_INVALID);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* check the option bytes security protection value */
        if(FMC_HSPC == (FMC_OBCTL & FMC_OBCTL_SPC)) {
            fmc_state =  FMC_OB_HSPC;
        } else {
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            
            if(FMC_READY == fmc_state) {
                obctl_reg = FMC_OBCTL;
                FMC_OBCTL = (obctl_reg & (~ob_user_mask)) | ob_user;
                FMC_CTL |= FMC_CTL_OBSTART;
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            }
        }
    }
    return fmc_state;
}

/*!
    \brief      configure security protection level (API_ID(0x0016U))
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  ob_spc: specify security protection
                only one parameter can be selected which is shown as below:
      \arg        FMC_NSPC: no security protection
      \arg        FMC_LSPC: low security protection
      \arg        FMC_HSPC: high security protection
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: high security protection
*/
fmc_state_enum ob_security_protection_level_config(uint8_t ob_spc)
{
    uint32_t obctl_reg;
    fmc_state_enum fmc_state;

    /* wait for the FMC ready */
    fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        obctl_reg = FMC_OBCTL;

        /* reset the SPC, set according to ob_spc */
        obctl_reg &= ~FMC_OBCTL_SPC;
        obctl_reg |= (uint32_t)ob_spc;
        FMC_OBCTL = obctl_reg;
        FMC_CTL |= FMC_CTL_OBSTART;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure the option bytes DCRP area (API_ID(0x0017U))
    \param[in]  dcrp_area: DCRP area
                only one parameter can be selected which is shown as below:
      \arg        DCRP_AREA_0: the DCRP area 0
      \arg        DCRP_AREA_1: the DCRP area 1
    \param[in]  dcrp_eren: DCRP area erase enable bit
      \arg        OB_DCRP_AREA_ERASE_DISABLE: DCRP area is not erased when low security protection to no security protection
      \arg        OB_DCRP_AREA_ERASE_ENABLE: DCRP area is erased when low security protection to no security protection
    \param[in]  dcrp_start: the first page offset of the DCRP area
    \param[in]  dcrp_end: the last page offset of the DCRP area
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: high security protection
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum ob_dcrp_area_config(uint32_t dcrp_area, uint32_t dcrp_eren, uint32_t dcrp_start,
                                   uint32_t dcrp_end)
{
    uint32_t reg_value;
    fmc_state_enum fmc_state;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_FMC_DCRP_AREA_VALID_CFG(dcrp_area,dcrp_eren,dcrp_start,dcrp_end)) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x0017U), ERR_PARAM_OUT_OF_RANGE);
        fmc_state = FMC_UNDEFINEDERR;
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* check the option bytes security protection value */
        if(FMC_HSPC == (FMC_OBCTL & FMC_OBCTL_SPC)) {
            fmc_state = FMC_OB_HSPC;
        } else {
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            
            if(FMC_READY == fmc_state) {
                /* configure DCRP area */
                if(DCRP_AREA_0 == dcrp_area) {
                    FMC_DCRP_SADDR0 = dcrp_start << DCRP_SADDR_OFFSET;
                    reg_value = FMC_DCRP_EADDR0;
                    reg_value &= ~FMC_DCRP_EADDR0_DCRP_EREN;
                    reg_value &= ~FMC_DCRP_EADDR0_DCRP0_EADDR;
                    reg_value |= (dcrp_end << DCRP_EADDR_OFFSET);
                    FMC_DCRP_EADDR0 = reg_value;
                } else{
                    FMC_DCRP_SADDR1 = dcrp_start << DCRP_SADDR_OFFSET;
                    FMC_DCRP_EADDR1 = dcrp_end << DCRP_EADDR_OFFSET;
                }
            
                /* configure EREN */
                reg_value = FMC_DCRP_EADDR0;
                reg_value &= ~FMC_DCRP_EADDR0_DCRP_EREN;
                reg_value |= dcrp_eren;
                FMC_DCRP_EADDR0 = reg_value;
            
                /* set the OBSTART bit in FMC_CTL register */
                FMC_CTL |= FMC_CTL_OBSTART;
            
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure the option bytes write protection area (API_ID(0x0018U))
    \param[in]  wp_area: write protection area
                only one parameter can be selected which is shown as below:
      \arg        WP_AREA_0: the first area
      \arg        WP_AREA_1: the second area
    \param[in]  wp_start: first page of write protection area
    \param[in]  wp_end: last page of write protection area
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: high security protection
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum ob_write_protection_area_config(uint32_t wp_area, uint32_t wp_start, uint32_t wp_end)
{
    uint32_t value;
    fmc_state_enum fmc_state;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_FMC_WP_AREA_VALID_CFG(wp_area,wp_start,wp_end)){
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x0018U), ERR_PARAM_OUT_OF_RANGE);
        fmc_state = FMC_UNDEFINEDERR;
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* check the option bytes security protection value */
        if(FMC_HSPC == (FMC_OBCTL & FMC_OBCTL_SPC)) {
            fmc_state =  FMC_OB_HSPC;
        } else 
        {
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            if(FMC_READY == fmc_state) {
                value = (wp_start << WP_SADDR_OFFSET) | (wp_end << WP_EADDR_OFFSET);
                /* configure the write protected area */
                if(WP_AREA_0 == wp_area) {
                    FMC_WP0 = value;
                } else {
                    FMC_WP1 = value;
                }
                /* set the OBSTART bit in FMC_CTL register */
                FMC_CTL |= FMC_CTL_OBSTART;
                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure the option bytes secure area (API_ID(0x0019U))
    \param[in]  secure_size: size of secure area in page unit
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: high security protection
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum ob_scr_area_config(uint32_t secure_size)
{
    uint32_t reg_value;
    fmc_state_enum fmc_state;
#ifdef FW_DEBUG_ERR_REPORT
    if(secure_size > MAIN_FLASH_PAGE_TOTAL_NUM) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x0019U), ERR_PARAM_OUT_OF_RANGE);
        fmc_state = FMC_UNDEFINEDERR;
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* check the option bytes security protection value */
        if(FMC_HSPC == (FMC_OBCTL & FMC_OBCTL_SPC)) {
            fmc_state = FMC_OB_HSPC;
        } else {
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
            
            if(FMC_READY == fmc_state) {
                /* configure secure area */
                reg_value = FMC_SCR;
                reg_value &= ~FMC_SCR_SCR_PAGE_CNT;
                reg_value |= secure_size << SCR_PAGE_CNT_OFFSET;
                FMC_SCR = reg_value;
            }
            
            /* set the OBSTART bit in FMC_CTL register */
            FMC_CTL |= FMC_CTL_OBSTART;
            
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure the option bytes boot lock (API_ID(0x001AU))
    \param[in]  boot_config: boot configuration
                only one parameter can be selected which is shown as below:
      \arg        OB_BOOT_LOCK_FROM_MAIN_FLASH: boot from main flash
      \arg        OB_BOOT_UNLOCK: unlock boot
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: high security protection
      \arg        FMC_UNDEFINEDERR: undefined error for function input parameter checking
*/
fmc_state_enum ob_boot_lock_config(uint32_t boot_config)
{
    uint32_t reg_value;
    fmc_state_enum fmc_state;

    /* check the option bytes security protection value */
    if(FMC_HSPC == (FMC_OBCTL & FMC_OBCTL_SPC)) {
        fmc_state = FMC_OB_HSPC;
    } else {
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        
        if(FMC_READY == fmc_state) {
            reg_value = FMC_SCR;
            reg_value &= ~FMC_SCR_BOOTLK;
            reg_value |= boot_config & FMC_SCR_BOOTLK;
            FMC_SCR = reg_value;
        
            /* set the OBSTART bit in FMC_CTL register */
            FMC_CTL |= FMC_CTL_OBSTART;
        
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get the value of option bytes USER (API_ID(0x001BU))
    \param[in]  user_data_extract_info:user data extract information which include mask and start bit position
      \arg        OBCTL_USER_DATA_BORST_EN: brown out reset enable mask and it's start bit position
      \arg        OBCTL_USER_DATA_BOR_TH: BOR threshold mask and it's start bit position
      \arg        OBCTL_USER_DATA_BORR_TH: BOR threshold at rising VDD supply mask and it's start bit position
      \arg        OBCTL_USER_DATA_BORF_TH: BOR threshold at falling VDD supply mask and it's start bit position
      \arg        OBCTL_USER_DATA_NRST_DPSLP: option byte deepsleep reset value bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_NRST_STDBY: option byte standby reset value bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_NFWDG_HW: free watchdog configuration bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_NWWDG_HW: window watchdog configuration bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_HXTAL_REMAP: HXTAL remapping bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_SRAM_ECC_EN: SRAM ECC enable bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_SRAM_PARITY_CHECK: SRAM parity check configuration bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_MLT_BND_SEC_EN: Multiple-bonding security enable bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_SWBT0: software BOOT0 bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_NBOOT1: NBOOT1 option bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_NBOOT0: NBOOT0 option bit mask and it's start bit position
      \arg        OBCTL_USER_DATA_NRST_MDSEL: NRST pin mode bit mask and it's start bit position
    \param[out] ob_user_data: the user data
    \retval     state of return user value
      \arg        INVLD_RETURN_VALUE: the area address is invalid
      \arg        VLD_RETURN_VALUE: the area address is valid
*/
uint32_t ob_user_get(ob_user_data_extract_info_enum user_data_extract_info, uint8_t * ob_user_data)
{
    uint32_t ret = INVLD_RETURN_VALUE;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(ob_user_data)){
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x001BU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* return the FMC user option bytes value */
        uint32_t user_data_mask = (uint32_t)user_data_extract_info & 0xFFF0U;
        uint8_t  user_data_start_position = (uint8_t)((uint32_t)user_data_extract_info & 0x000FU);
        *ob_user_data = (uint8_t)((FMC_OBCTL & user_data_mask) >> user_data_start_position);
        ret = VLD_RETURN_VALUE;
    }
    return ret;
}

/*!
    \brief      get the value of option bytes security protection level in FMC_OBCTL register (API_ID(0x001CU))
    \param[in]  none
    \param[out] none
    \retval     protection level
      \arg        FMC_NSPC: no protection
      \arg        FMC_LSPC: protection level low
      \arg        FMC_HSPC: protection level high
*/
uint8_t ob_security_protection_level_get(void)
{
    return (uint8_t)((FMC_OBCTL & FMC_OBCTL_SPC) >> OBCTL_SPC_OFFSET);
}

/*!
    \brief      get configuration of DCRP area (API_ID(0x001DU))
    \param[in]  dcrp_area: DCRP area
                only one parameter can be selected which is shown as below:
      \arg        DCRP_AREA_0: the DCRP area 0
      \arg        DCRP_AREA_1: the DCRP area 1
    \param[out] dcrp_erase_option: erase option of DCRP area
    \param[out] dcrp_start: start address of DCRP area
    \param[out] dcrp_end: end address of DCRP area
    \retval     state of address
      \arg        INVLD_RETURN_VALUE: the area address is invalid
      \arg        VLD_RETURN_VALUE: the area address is valid
*/
uint32_t ob_dcrp_area_get(uint32_t dcrp_area, uint32_t *dcrp_erase_option,
                          uint32_t *dcrp_start, uint32_t *dcrp_end)
{
    uint32_t dcrp_area_start, dcrp_area_end;
    uint32_t ret = INVLD_RETURN_VALUE;
#ifdef FW_DEBUG_ERR_REPORT
    if((dcrp_area != DCRP_AREA_0) && (dcrp_area != DCRP_AREA_1)) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x001DU), ERR_PARAM_OUT_OF_RANGE);
    } else if((NOT_VALID_POINTER(dcrp_erase_option))    ||
              (NOT_VALID_POINTER(dcrp_start)) ||
              (NOT_VALID_POINTER(dcrp_end))) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x001DU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(DCRP_AREA_0 == dcrp_area) {
            dcrp_area_start = (FMC_DCRP_SADDR0 & FMC_DCRP_SADDR0_DCRP0_SADDR) >> DCRP_SADDR_OFFSET;
            dcrp_area_end = (FMC_DCRP_EADDR0 & FMC_DCRP_EADDR0_DCRP0_EADDR) >> DCRP_EADDR_OFFSET;
        
        } else {
            dcrp_area_start = (FMC_DCRP_SADDR1 & FMC_DCRP_SADDR1_DCRP1_SADDR) >> DCRP_SADDR_OFFSET;
            dcrp_area_end = (FMC_DCRP_EADDR1 & FMC_DCRP_EADDR1_DCRP1_EADDR) >> DCRP_EADDR_OFFSET;
        }
        /* get erase option of DCRP area */
        *dcrp_erase_option = (FMC_DCRP_EADDR0 & FMC_DCRP_EADDR0_DCRP_EREN) >> DCRP_EREN_OFFSET;
        /* get start address and end address of DCRP area */
        *dcrp_start = MAIN_FLASH_BASE_ADDRESS + dcrp_area_start * DCRP_AREA_SUBPAGE_SIZE;
        *dcrp_end = MAIN_FLASH_BASE_ADDRESS + (dcrp_area_end + 1U) * DCRP_AREA_SUBPAGE_SIZE - 1U;
        
        if(dcrp_area_start <= dcrp_area_end) {
            /* the DCRP area is valid */
            ret = VLD_RETURN_VALUE;
        } else {
            /* the DCRP area is invalid */
            ret = INVLD_RETURN_VALUE;
        }
    }
    return ret;
}

/*!
    \brief      get address of write protection area (API_ID(0x001EU))
    \param[in]  wp_area: write protection area
                only one parameter can be selected which is shown as below:
      \arg        WP_AREA_0: write protection area 0
      \arg        WP_AREA_1: write protection area 1
    \param[out] wp_start: start address of write protection area
    \param[out] wp_end: end address of write protection area
    \retval     state of address
      \arg        INVLD_RETURN_VALUE: the area address is invalid
      \arg        VLD_RETURN_VALUE: the area address is valid
*/
uint32_t ob_write_protection_area_get(uint32_t wp_area, uint32_t *wp_start,
                                      uint32_t *wp_end)
{
    uint32_t wp_area_start , wp_area_end ;
    uint32_t ret = INVLD_RETURN_VALUE;
#ifdef FW_DEBUG_ERR_REPORT
    if((wp_area != WP_AREA_0) && (wp_area != WP_AREA_1)) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x001EU), ERR_PARAM_OUT_OF_RANGE);
    } else if((NOT_VALID_POINTER(wp_start))    ||
              (NOT_VALID_POINTER(wp_end))) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x001EU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        if(WP_AREA_0 == wp_area) {
            wp_area_start = (FMC_WP0 & FMC_WP0_WP0_SADDR) >> WP_SADDR_OFFSET;
            wp_area_end = (FMC_WP0 & FMC_WP0_WP0_EADDR) >> WP_EADDR_OFFSET;
        } else {
            wp_area_start = (FMC_WP1 & FMC_WP1_WP1_SADDR) >> WP_SADDR_OFFSET;
            wp_area_end = (FMC_WP1 & FMC_WP1_WP1_EADDR) >> WP_EADDR_OFFSET;
        }
        
        *wp_start = MAIN_FLASH_BASE_ADDRESS + wp_area_start * WP_AREA_SUBPAGE_SIZE;
        *wp_end = MAIN_FLASH_BASE_ADDRESS + (wp_area_end + 1U) * WP_AREA_SUBPAGE_SIZE - 1U;
        
        if(wp_area_start <= wp_area_end) {
            /* the write protected area is valid */
            ret =  VLD_RETURN_VALUE;
        } else {
            /* the write protected area is invalid */
            ret =  INVLD_RETURN_VALUE;
        }
    }
    return ret;
}
/*!
    \brief      get size of secure area (API_ID(0x001FU))
    \param[out] scr_area_byte_cnt: secure area size in byte unit
    \retval     none
*/
uint32_t ob_scr_area_get(uint32_t *scr_area_byte_cnt)
{
    uint32_t scr_area_byte_size;
    uint32_t ret = INVLD_RETURN_VALUE;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(scr_area_byte_cnt)) {
        fw_debug_report_err(FMC_MODULE_ID, API_ID(0x001FU), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        scr_area_byte_size = (FMC_SCR & FMC_SCR_SCR_PAGE_CNT) >> SCR_PAGE_CNT_OFFSET;
        *scr_area_byte_cnt = scr_area_byte_size * MAIN_FLASH_PAGE_SIZE;
        ret =  VLD_RETURN_VALUE;
    }
    return ret;
}

/*!
    \brief      get boot lock configuration (API_ID(0x0020U))
    \param[in]  none
    \param[out] none
    \retval     boot configuration
      \arg        OB_BOOT_LOCK_FROM_MAIN_FLASH: boot from main flash
      \arg        OB_BOOT_UNLOCK: unlock boot
*/
uint32_t ob_boot_lock_get(void)
{
    return (uint32_t)(FMC_SCR & FMC_SCR_BOOTLK);
}

/*!
    \brief      get FMC flag status (API_ID(0x0021U))
    \param[in]  flag: FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_FLAG_BUSY: FMC operation is in progress
      \arg        FMC_FLAG_OBERR: option byte read error
      \arg        FMC_FLAG_WPERR: erase/program protection error
      \arg        FMC_FLAG_PGSERR: program sequence error
      \arg        FMC_FLAG_PGMERR: program size error
      \arg        FMC_FLAG_PGAERR: program alignment error
      \arg        FMC_FLAG_FSTPERR: fast programming error
      \arg        FMC_FLAG_RPERR: read protection error
      \arg        FMC_FLAG_PGERR: program error
      \arg        FMC_FLAG_OPRERR: operation error
      \arg        FMC_FLAG_ENDF: FMC end of operation flag error
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;
    
    flag &= FMC_FLAG_MASK | FMC_STAT_BUSY;
    if(0U!=(FMC_STAT & flag)) {
        status = SET;
    }

    /* return the state of corresponding FMC flag */
    return status;
}

/*!
    \brief      clear the FMC flag (API_ID(0x0022U))
    \param[in]  flag: FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_FLAG_OBERR: option byte read error
      \arg        FMC_FLAG_WPERR: erase/program protection error
      \arg        FMC_FLAG_PGSERR: program sequence error
      \arg        FMC_FLAG_PGMERR: program size error
      \arg        FMC_FLAG_PGAERR: program alignment error
      \arg        FMC_FLAG_FSTPERR: fast programming error
      \arg        FMC_FLAG_RPERR: read protection error
      \arg        FMC_FLAG_PGERR: program error
      \arg        FMC_FLAG_OPRERR: operation error
      \arg        FMC_FLAG_ENDF: FMC end of operation flag error
    \param[out] none
    \retval     none
*/
void fmc_flag_clear(uint32_t flag)
{
    /* clear the flags */
    FMC_STAT = flag & FMC_FLAG_MASK;
}

/*!
    \brief      enable FMC interrupt (API_ID(0x0023U))
    \param[in]  interrupt: the FMC interrupt
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_END: FMC end of operation interrupt
      \arg        FMC_INT_ERR: FMC error interrupt
      \arg        FMC_INT_RPERR: read protection error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_enable(uint32_t interrupt)
{
    FMC_CTL |= interrupt & FMC_INTERRUPT_CFG_MASK;
}

/*!
    \brief      disable FMC interrupt (API_ID(0x0024U))
    \param[in]  interrupt: the FMC interrupt
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_END: FMC end of operation interrupt
      \arg        FMC_INT_ERR: FMC error interrupt
      \arg        FMC_INT_RPERR: read protection error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_disable(uint32_t interrupt)
{
    FMC_CTL &= ~(interrupt & FMC_INTERRUPT_CFG_MASK);
}

/*!
    \brief      get FMC interrupt flag (API_ID(0x0025U))
    \param[in]  flag: FMC interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_INT_FLAG_END: FMC end of operation interrupt flag
      \arg        FMC_INT_FLAG_OPRERR: FMC error interrupt flag
      \arg        FMC_INT_FLAG_RPERR: read protection error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_interrupt_flag_get(uint32_t flag)
{
    FlagStatus status = RESET;

    flag &= FMC_INTERRUPT_FLAG_MASK;
    
    if(0U!=(FMC_STAT & flag)) {
        status = SET;
    }
    /* return the state of corresponding FMC flag */
    return status;
}

/*!
    \brief      clear FMC interrupt flag (API_ID(0x0026U))
    \param[in]  flag: FMC interrupt flag
                one or more parameters can be selected which is shown as below:
      \arg        FMC_INT_FLAG_END: FMC end of operation interrupt flag
      \arg        FMC_INT_FLAG_OPRERR: FMC error interrupt flag
      \arg        FMC_INT_FLAG_RPERR: read protection error interrupt flag
    \param[out] none
    \retval     none
*/
void fmc_interrupt_flag_clear(uint32_t flag)
{
    /* clear the flag */
    FMC_STAT = flag & FMC_INTERRUPT_FLAG_MASK;
}

/*!
    \brief      get the FMC state (API_ID(0x0027U))
    \param[in]  none
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
*/
fmc_state_enum fmc_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;

    if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_BUSY)) {
        fmc_state = FMC_BUSY;
    } else {
        if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_WPERR)) {
            fmc_state = FMC_WPERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_PGERR)) {
            fmc_state = FMC_PGERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_PGSERR)) {
            fmc_state = FMC_PGSERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_PGAERR)) {
            fmc_state = FMC_PGAERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_RPERR)) {
            fmc_state = FMC_RPERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_PGMERR)) {
            fmc_state = FMC_PGMERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_FSTPERR)) {
            fmc_state = FMC_FSTPERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_OPRERR)) {
            fmc_state = FMC_OPRERR;
        } else if((uint32_t)0x00U != (FMC_STAT & FMC_STAT_OBERR)) {
            fmc_state = FMC_OBERR;
        } else {
            /* illegal parameters */
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC is ready or not  (API_ID(0x0028U))
    \param[in]  timeout: timeout count
    \param[out] none
    \retval     state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_OBERR: option byte read error
      \arg        FMC_RPERR: read protection error
      \arg        FMC_FSTPERR: fast programming error
      \arg        FMC_PGSERR: program sequence error
      \arg        FMC_PGMERR: program size error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_PGERR: program error
      \arg        FMC_OPRERR: operation error
      \arg        FMC_TOERR: timeout error
    \note       This function includes timeout exit scenarios.
                Modify according to the user's actual usage scenarios.
*/
fmc_state_enum fmc_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state;

    /* wait for FMC ready */
    do {
        /* get FMC state */
        fmc_state = fmc_state_get();
        timeout--;
    } while((FMC_BUSY == fmc_state) && (0x00U != timeout));

    if(FMC_BUSY == fmc_state) {
        fmc_state = FMC_TOERR;
    }
    /* return the FMC state */
    return fmc_state;
}
