/*!
    \file    gd32a50x_fmc.c
    \brief   FMC driver

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

#include "gd32a50x_fmc.h"

/* FMC mask */
#define LOW_8BITS_MASK                  ((uint32_t)0x000000FFU)               /*!< the 0-7 bits mask of a word */
#define HIGH_8BITS_MASK                 ((uint32_t)0x0000FF00U)               /*!< the 8-15 bits mask of a word */
#define LOW_8BITS_MASK1                 ((uint32_t)0x00FF0000U)               /*!< the 16-23 bits mask of a word */
#define HIGH_8BITS_MASK1                ((uint32_t)0xFF000000U)               /*!< the 24-31 bits mask of a word */
#define LOW_16BITS_MASK                 ((uint32_t)0x0000FFFFU)               /*!< the 0-15 bits mask of a word */
#define HIGH_16BITS_MASK                ((uint32_t)0xFFFF0000U)               /*!< the 16-31 bits mask of a word */
#define ECCCS_FLAG_MASK                 ((uint32_t)0xEC000000U)               /*!< flag mask in ECCCS register */
#define ECCCS_ERROR_MASK                ((uint32_t)0x00F10000U)               /*!< error flag mask in ECCCS register */

/* FMC register bit offset */
#define CTL_CBCMDLEN_OFFSET             ((uint32_t)0x0000001DU)               /*!< CBCMDLEN offset in FMC_CTL register */
#define OBSTAT_PLEVEL_OFFSET            ((uint32_t)0x00000001U)               /*!< PLEVEL offset in FMC_OBSTAT register */
#define OBSTAT_USER_OFFSET              ((uint32_t)0x00000008U)               /*!< USER offset in FMC_OBSTAT register */
#define OBSTAT_DATA_OFFSET              ((uint32_t)0x00000010U)               /*!< DATA offset in FMC_OBSTAT register */
#define WP1_BK1WP_OFFSET                ((uint32_t)0x00000000U)               /*!< BK1WP offset in FMC_WP1 register */
#define WP1_DFWP_OFFSET                 ((uint32_t)0x00000008U)               /*!< DFWP offset in FMC_WP1 register */
#define WP1_EPWP_OFFSET                 ((uint32_t)0x00000010U)               /*!< EPWP offset in FMC_WP1 register */
#define ECCCS_REG_OFFSET                ((uint32_t)0x00000004U)               /*!< ECCCS register offset */

/* return the FMC bank0 state */
static fmc_state_enum fmc_bank0_state_get(void);
/* return the FMC bank1 state */
static fmc_state_enum fmc_bank1_state_get(void);
/* check FMC bank0 ready or not */
static fmc_state_enum fmc_bank0_ready_wait(uint32_t timeout);
/* check FMC bank1 ready or not */
static fmc_state_enum fmc_bank1_ready_wait(uint32_t timeout);
/* wait shared SRAM mode to be ready */
static void fmc_sram_mode_ready_wait(uint32_t ready_flag);

/*!
    \brief      unlock the main flash operation
                it is better to used in pairs with fmc_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_unlock(void)
{
    if((0U != (FMC_CTL0 & FMC_CTL0_LK))) {
        /* write the FMC bank0 key */
        FMC_KEY0 = UNLOCK_KEY0;
        FMC_KEY0 = UNLOCK_KEY1;
    }

    if((0U != (FMC_CTL1 & FMC_CTL1_LK))) {
        /* write the FMC bank1 key */
        FMC_KEY1 = UNLOCK_KEY0;
        FMC_KEY1 = UNLOCK_KEY1;
    }
}

/*!
    \brief      unlock the main flash bank0 operation
                it is better to used in pairs with fmc_bank0_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_bank0_unlock(void)
{
    if((0U != (FMC_CTL0 & FMC_CTL0_LK))) {
        /* write the FMC bank0 key */
        FMC_KEY0 = UNLOCK_KEY0;
        FMC_KEY0 = UNLOCK_KEY1;
    }
}

/*!
    \brief      unlock the main flash bank1 operation
                it is better to used in pairs with fmc_bank1_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_bank1_unlock(void)
{
    if((0U != (FMC_CTL1 & FMC_CTL1_LK))) {
        /* write the FMC bank1 key */
        FMC_KEY1 = UNLOCK_KEY0;
        FMC_KEY1 = UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the main flash operation
                it is better to used in pairs with fmc_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_lock(void)
{
    /* set the bank0 LK bit*/
    FMC_CTL0 |= FMC_CTL0_LK;

    /* set the bank1 LK bit*/
    FMC_CTL1 |= FMC_CTL1_LK;
}

/*!
    \brief      lock the main flash bank0 operation
                it is better to used in pairs with fmc_bank0_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_bank0_lock(void)
{
    /* set the LK bit*/
    FMC_CTL0 |= FMC_CTL0_LK;
}

/*!
    \brief      lock the main flash bank1 operation
                it is better to used in pairs with fmc_bank1_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_bank1_lock(void)
{
    /* set the LK bit*/
    FMC_CTL1 |= FMC_CTL1_LK;
}

/*!
    \brief      set the wait state counter value
    \param[in]  wscnt: wait state counter value
                only one parameter can be selected which is shown as below:
      \arg        WS_WSCNT_0: 0 wait state added
      \arg        WS_WSCNT_1: 1 wait state added
      \arg        WS_WSCNT_2: 2 wait state added
      \arg        WS_WSCNT_3: 3 wait state added
    \param[out] none
    \retval     none
*/
void fmc_wscnt_set(uint8_t wscnt)
{
    uint32_t reg;

    reg = FMC_WS;
    /* set the wait state counter value */
    reg &= ~FMC_WS_WSCNT;
    FMC_WS = (reg | wscnt);
}

/*!
    \brief      enable pre-fetch
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_enable(void)
{
    FMC_WS |= FMC_WS_PFEN;
}

/*!
    \brief      disable pre-fetch
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_prefetch_disable(void)
{
    FMC_WS &= ~FMC_WS_PFEN;
}

/*!
    \brief      enable cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_cache_enable(void)
{
    FMC_WS |= FMC_WS_IDCEN;
}

/*!
    \brief      disable cache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_cache_disable(void)
{
    FMC_WS &= ~FMC_WS_IDCEN;
}

/*!
    \brief      enable cache reset if cache is disabled
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_cache_reset_enable(void)
{
    FMC_WS |= FMC_WS_IDRST;
}

/*!
    \brief      disable cache reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_cache_reset_disable(void)
{
    FMC_WS &= ~FMC_WS_IDRST;
}

/*!
    \brief      flash goto power-down mode when MCU enters deepsleep mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_powerdown_mode_set(void)
{
    FMC_WS &= ~FMC_WS_SLEEP_SLP;
}

/*!
    \brief      flash goto sleep mode when MCU enters deepsleep mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_sleep_mode_set(void)
{
    FMC_WS |= FMC_WS_SLEEP_SLP;
}

/*!
    \brief      configure shared SRAM mode
    \param[in]  sram_mode: shared SRAM mode
                only one parameter can be selected which is shown as below:
      \arg        FASTPG_SRAM_MODE: fast program SRAM mode
      \arg        BASIC_SRAM_MODE: basic SRAM mode
      \arg        EEPROM_SRAM_MODE: EEPROM SRAM mode
    \param[out] none
    \retval     none
*/
void fmc_sram_mode_config(fmc_sram_mode_enum sram_mode)
{
    fmc_sram_mode_enum curr_mode;

    curr_mode = fmc_sram_mode_get();
    FMC_CTL1 &= ~FMC_CTL1_SRAMCMD;
    if((EEPROM_SRAM_MODE == sram_mode) && (EEPROM_SRAM_MODE != curr_mode)) {
        /* set shared SRAM to EEPROM SRAM mode*/
        FMC_CTL1 |= EEPROM_SRAM_CMD;
        fmc_sram_mode_ready_wait(FMC_WS_ERAMRDY);
    } else if((BASIC_SRAM_MODE == sram_mode) && (BASIC_SRAM_MODE != curr_mode)) {
        /* set shared SRAM to basic SRAM mode*/
        FMC_CTL1 |= BASIC_SRAM_CMD;
        fmc_sram_mode_ready_wait(FMC_WS_BRAMRDY);
    } else if((FASTPG_SRAM_MODE == sram_mode) && (FASTPG_SRAM_MODE != curr_mode)) {
        /* set shared SRAM to fast program SRAM mode*/
        FMC_CTL1 |= FASTPG_SRAM_CMD;
        fmc_sram_mode_ready_wait(FMC_WS_PRAMRDY);
    } else {
        /* illegal parameters */
    }
}

/*!
    \brief      get shared SRAM mode
    \param[in]  none
    \param[out] none
    \retval     sram_mode: shared SRAM mode
      \arg        NO_SRAM_MODE: SRAM mode is not configured
      \arg        FASTPG_SRAM_MODE: fast PG SRAM mode
      \arg        BASIC_SRAM_MODE: basic SRAM mode
      \arg        EEPROM_SRAM_MODE: EEPROM SRAM mode
*/
fmc_sram_mode_enum fmc_sram_mode_get(void)
{
    fmc_sram_mode_enum sram_mode;

    if(0U != (FMC_WS & FMC_WS_ERAMRDY)) {
        /* SRAM is in EEPROM SRAM mode*/
        sram_mode = EEPROM_SRAM_MODE;
    } else if(0U != (FMC_WS & FMC_WS_BRAMRDY)) {
        /* SRAM is in basic SRAM mode*/
        sram_mode = BASIC_SRAM_MODE;
    } else if(0U != (FMC_WS & FMC_WS_PRAMRDY)) {
        /* SRAM is in fast program SRAM mode*/
        sram_mode = FASTPG_SRAM_MODE;
    } else {
        sram_mode = NO_SRAM_MODE;
    }

    return sram_mode;
}

/*!
    \brief      check whether flash page is blank or not by check blank command
    \param[in]  address: start address to check
    \param[in]  length: the read length is 2^length double words, the flash area to be checked must be in one page and should not exceed 1KB boundary
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_blank_check(uint32_t address, uint8_t length)
{
    fmc_state_enum fmc_state;

    if((address >= BANK0_BASE_ADDRESS) && (address < BANK1_BASE_ADDRESS)) {
        /* start check blank command */
        FMC_ADDR0 = address;
        FMC_CTL0 &= ~FMC_CTL0_CBCMDLEN;
        FMC_CTL0 |= (uint32_t)length << CTL_CBCMDLEN_OFFSET;
        FMC_CTL0 |= FMC_CTL0_CBCMD;
        FMC_CTL0 |= FMC_CTL0_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the CBCMD bit */
        FMC_CTL0 &= ~FMC_CTL0_CBCMD;
    } else {
        /* start check blank command */
        FMC_ADDR1 = address;
        FMC_CTL1 &= ~FMC_CTL1_CBCMDLEN;
        FMC_CTL1 |= (uint32_t)length << CTL_CBCMDLEN_OFFSET;
        FMC_CTL1 |= FMC_CTL1_CBCMD;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the CBCMD bit */
        FMC_CTL1 &= ~FMC_CTL1_CBCMD;
    }

    return fmc_state;
}

/*!
    \brief      erase main flash page
    \param[in]  page_address: target page start address
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_page_erase(uint32_t page_address)
{
    fmc_state_enum fmc_state = FMC_READY;

    if((page_address >= BANK0_BASE_ADDRESS) && (page_address < BANK1_BASE_ADDRESS)) {
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* start page erase */
            FMC_CTL0 |= FMC_CTL0_PER;
            FMC_ADDR0 = page_address;
            FMC_CTL0 |= FMC_CTL0_START;

            /* wait for the FMC ready */
            fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the PER bit */
            FMC_CTL0 &= ~FMC_CTL0_PER;
        }
    } else {
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
        if(FMC_READY == fmc_state) {
            /* start page erase */
            FMC_CTL1 |= FMC_CTL1_PER;
            FMC_ADDR1 = page_address;
            FMC_CTL1 |= FMC_CTL1_START;

            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the PER bit */
            FMC_CTL1 &= ~FMC_CTL1_PER;
        }
    }

    /* return the FMC state  */
    return fmc_state;
}

/*!
    \brief      erase flash bank0
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_bank0_mass_erase(void)
{
    fmc_state_enum fmc_state;

    /* wait for the FMC ready */
    fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start chip erase */
        FMC_CTL0 |= FMC_CTL0_MER;
        FMC_CTL0 |= FMC_CTL0_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MER bit */
        FMC_CTL0 &= ~FMC_CTL0_MER;
    }
    /* return the fmc state */
    return fmc_state;
}

/*!
    \brief      erase flash bank1
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_bank1_mass_erase(void)
{
    fmc_state_enum fmc_state;

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start chip erase */
        FMC_CTL1 |= FMC_CTL1_MER;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MER bit */
        FMC_CTL1 &= ~FMC_CTL1_MER;
    }
    /* return the fmc state  */
    return fmc_state;
}

/*!
    \brief      erase the data flash
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_dflash_mass_erase(void)
{
    fmc_state_enum fmc_state;

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start data flash erase */
        FMC_CTL1 |= FMC_CTL1_MERDF;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MERDF bit */
        FMC_CTL1 &= ~FMC_CTL1_MERDF;
    }

    return fmc_state;
}

/*!
    \brief      erase whole chip
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_mass_erase(void)
{
    fmc_state_enum fmc_state;

    /* mass erase flash bank0 */
    fmc_state = fmc_bank0_mass_erase();

    /* return the bank0 abnormal state */
    if(FMC_READY != fmc_state) {
        return fmc_state;
    }

    /* mass erase flash bank1 */
    fmc_state = fmc_bank1_mass_erase();

    /* return the bank1 abnormal state */
    if(FMC_READY != fmc_state) {
        return fmc_state;
    }

    /* mass erase data flash */
    fmc_state = fmc_dflash_mass_erase();

    /* return the fmc state */
    return fmc_state;
}

/*!
    \brief      program a double word at the corresponding address in main flash
    \param[in]  address: address to program, apply to bank0, bank1 and data flash
    \param[in]  data: double word to program
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_doubleword_program(uint32_t address, uint64_t data)
{
    uint32_t data0, data1;
    fmc_state_enum fmc_state;

    if((address >= BANK0_BASE_ADDRESS) && (address < BANK1_BASE_ADDRESS)) {
        /* wait for the FMC ready */
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

        data0 = (uint32_t)(data & 0xFFFFFFFFU);
        data1 = (uint32_t)((data >> 32U) & 0xFFFFFFFFU);

        if(FMC_READY == fmc_state) {
            /* set the PG bit to start program */
            FMC_CTL0 |= FMC_CTL0_PG;
            REG32(address) = data0;
            REG32(address + 4U) = data1;

            /* wait for the FMC ready */
            fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the PG bit */
            FMC_CTL0 &= ~FMC_CTL0_PG;
        }
    } else {
        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        data0 = (uint32_t)(data & 0xFFFFFFFFU);
        data1 = (uint32_t)((data >> 32U) & 0xFFFFFFFFU);

        if(FMC_READY == fmc_state) {
            /* set the PG bit to start program */
            FMC_CTL1 |= FMC_CTL1_PG;
            REG32(address) = data0;
            REG32(address + 4U) = data1;

            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the PG bit */
            FMC_CTL1 &= ~FMC_CTL1_PG;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      FMC fast program one row data (32 double-word) starting at the corresponding address
    \param[in]  address: address to program
    \param[in]  data: data buffer to program
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum fmc_fast_program(uint32_t address, uint64_t data[])
{
    uint8_t index;
    fmc_state_enum fmc_state;
    fmc_sram_mode_enum sram_mode;
    uint8_t cache_en = 0U;

    /* get shared SRAM mode */
    sram_mode = fmc_sram_mode_get();

    /* set shared SRAM to fast program mode */
    if(FASTPG_SRAM_MODE != sram_mode) {
        fmc_sram_mode_config(FASTPG_SRAM_MODE);
    }

    /* check the row (32 double-word) in flash to confirm all data in flash is all 0xFF */
    fmc_state = fmc_blank_check(address, CBCMDLEN_OF_ONE_ROW);
    if(FMC_CBCMDERR == fmc_state) {
        /* flash is not erased */
        return FMC_PGERR;
    }
    /* flush the cache if it is enabled */
    if(FMC_WS & FMC_WS_IDCEN) {
        cache_en = 1U;
        fmc_cache_disable();
    }
    fmc_cache_reset_enable();
    fmc_cache_reset_disable();

    if((address >= BANK0_BASE_ADDRESS) && (address < BANK1_BASE_ADDRESS)) {
        /* wait for the FMC ready */
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* set the FSTPG bit to start program */
            FMC_CTL0 |= FMC_CTL0_FSTPG;

            /* program the row data */
            for(index = 0U; index < DOUBLEWORD_CNT_IN_ROW; index++) {
                REG32(address) = (uint32_t)(data[index] & 0x00000000FFFFFFFFU);
                REG32(address + 4U) = (uint32_t)(data[index] >> 32U);
                address += 8U;
            }
            /* set START bit to launch fast program operation to flash */
            FMC_CTL0 |= FMC_CTL0_START;

            /* wait for the FMC ready */
            fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the FSTPG bit */
            FMC_CTL0 &= ~FMC_CTL0_FSTPG;
        }
    } else {
        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* set the FSTPG bit to start program */
            FMC_CTL1 |= FMC_CTL1_FSTPG;

            /* program the row data */
            for(index = 0U; index < DOUBLEWORD_CNT_IN_ROW; index++) {
                REG32(address) = (uint32_t)(data[index] & 0x00000000FFFFFFFFU);
                REG32(address + 4U) = (uint32_t)(data[index] >> 32U);
                address += 8U;
            }

            /* set START bit to launch fast program operation to flash */
            FMC_CTL1 |= FMC_CTL1_START;

            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the FSTPG bit */
            FMC_CTL1 &= ~FMC_CTL1_FSTPG;
        }
    }
    if(1U == cache_en) {
        fmc_cache_enable();
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program a double word at the corresponding address in OTP
    \param[in]  address: address to program
    \param[in]  data: double word to write
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum otp_doubleword_program(uint32_t address, uint64_t data)
{
    uint32_t data0, data1;
    fmc_state_enum fmc_state;

    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
    data0 = (uint32_t)(data & 0xFFFFFFFFU);
    data1 = (uint32_t)((data >> 32U) & 0xFFFFFFFFU);

    /* configure program width */
    if(FMC_READY == fmc_state) {
        /* set the PG bit to start program */
        FMC_CTL1 |= FMC_CTL1_PG;
        REG32(address) = data0;
        REG32(address + 4U) = data1;
        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the PG bit */
        FMC_CTL1 &= ~FMC_CTL1_PG;
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program a word at the corresponding address in EEPROM
    \param[in]  address: address to program
    \param[in]  data: word to write
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
fmc_state_enum eeprom_word_program(uint32_t address, uint32_t data)
{
    fmc_sram_mode_enum sram_mode;
    fmc_state_enum fmc_state;

    /* get shared SRAM mode */
    sram_mode = fmc_sram_mode_get();

    /* set Shared SRAM to EEPROM RAM mode */
    if(EEPROM_SRAM_MODE != sram_mode) {
        fmc_sram_mode_config(EEPROM_SRAM_MODE);
    }

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    /* configure program width */
    if(FMC_READY == fmc_state) {
        REG32(address) = data;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      read a word at the corresponding address in EEPROM
    \param[in]  address: address to read
    \param[in]  none
    \param[out] none
    \retval     word data readout
*/
uint32_t eeprom_word_read(uint32_t address)
{
    uint32_t data;
    fmc_sram_mode_enum sram_mode;

    /* get shared SRAM mode */
    sram_mode = fmc_sram_mode_get();

    /* set shared SRAM to EEPROM RAM mode */
    if(EEPROM_SRAM_MODE != sram_mode) {
        fmc_sram_mode_config(EEPROM_SRAM_MODE);
    }

    data = REG32(address);

    /* return data */
    return data;
}

/*!
    \brief      unlock the option bytes 0 operation
                it is better to used in pairs with ob_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_unlock(void)
{
    if(0U == (FMC_CTL1 & FMC_CTL1_OBWEN)) {
        /* write the FMC key */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
    }

    /* wait until OBWEN bit is set by hardware */
    while(0U == (FMC_CTL1 & FMC_CTL1_OBWEN)) {
    }
}

/*!
    \brief      lock the option bytes 0 operation
                it is better to used in pairs with ob_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_lock(void)
{
    /* reset the OBWEN bit */
    FMC_CTL1 &= ~FMC_CTL1_OBWEN;
}

/*!
    \brief      force to reload the option bytes 0
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_reset(void)
{
    /* set the OBRLD bit */
    FMC_CTL1 |= FMC_CTL1_OBRLD;
}

/*!
    \brief      erase the option bytes 0
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB_HSPC: FMC is under high security protection
*/
fmc_state_enum ob_erase(void)
{
    uint32_t temp_spc_user;
    fmc_state_enum fmc_state;

    temp_spc_user = OB_SPC_USER;

    /* check the option bytes security protection value */
    if(OB_OBSTAT_PLEVEL_HIGH == ob_plevel_get()) {
        return FMC_OB_HSPC;
    }

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start erase the option bytes */
        FMC_CTL1 |= FMC_CTL1_OB0ER;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;

            /* set the OB0PG bit */
            FMC_CTL1 |= FMC_CTL1_OB0PG;

            /* restore the previous security protection configuration */
            OB_SPC_USER = (temp_spc_user & LOW_8BITS_MASK) | LOW_8BITS_MASK1;
            OB_DATA = 0xFFFFFFFFU;

            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the OB0PG bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0PG;
        } else {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      enable option bytes 0 write protection
    \param[in]  wp_area: write protection area
                only one parameter can be selected which is shown as below:
      \arg        BANK0_AREA: main flash bank0 write protection area
      \arg        BANK1_AREA: main flash bank1 write protection area
      \arg        DATA_FLASH_AREA: data flash write protection area
      \arg        EEPROM_AREA: EEPROM write protection area
    \param[in]  ob_wp: write protection configuration data. Notice that set the bit to 1 if you want to protect
                the corresponding pages. The lowest 8 bits is valid in area except bank0.
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB_HSPC: FMC is under high security protection
*/
fmc_state_enum ob_write_protection_enable(fmc_area_enum wp_area, uint32_t ob_wp)
{
    uint32_t i;
    uint32_t op_byte[OB_WORD_CNT] = {0U};
    fmc_state_enum fmc_state;

    /* check the option bytes security protection value */
    if(OB_OBSTAT_PLEVEL_HIGH == ob_plevel_get()) {
        return FMC_OB_HSPC;
    }

    /* read option bytes */
    for(i = 0U; i < OB_WORD_CNT; i++) {
        op_byte[i] = OP_BYTE(i);
    }

    ob_wp = (uint32_t)(~ob_wp);
    if(BANK0_AREA == wp_area) {
        /* configure write protection to main flash bank0 area */
        op_byte[2] &= (ob_wp & LOW_8BITS_MASK) | ((ob_wp & HIGH_8BITS_MASK) << 8);
        op_byte[3] &= ((ob_wp & LOW_8BITS_MASK1) >> 16) | ((ob_wp & HIGH_8BITS_MASK1) >> 8);
    } else if(BANK1_AREA == wp_area) {
        /* configure write protection to main flash bank1 area */
        op_byte[4] &= (uint32_t)((ob_wp & LOW_8BITS_MASK) | HIGH_16BITS_MASK);
    } else if(DATA_FLASH_AREA == wp_area) {
        /* configure write protection to data flash area */
        op_byte[4] &= (uint32_t)(((ob_wp & LOW_8BITS_MASK) << 16U) | LOW_16BITS_MASK);
    } else if(EEPROM_AREA == wp_area) {
        /* configure write protection to EEPROM area */
        op_byte[5] &= (uint32_t)(ob_wp & LOW_8BITS_MASK);
    } else {
        /* illegal parameters */
    }

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start erase the option bytes */
        FMC_CTL1 |= FMC_CTL1_OB0ER;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;

            /* enable the option bytes programming */
            FMC_CTL1 |= FMC_CTL1_OB0PG;

            /* write option bytes */
            for(i = 0U; i < OB_DOUBLEWORD_CNT; i++) {
                OP_BYTE(i * 2U) = op_byte[i * 2U];
                OP_BYTE(i * 2U + 1U) = op_byte[i * 2U + 1U];
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state) {
                    break;
                }
            }
            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the OB0PG bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0PG;
        } else {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure security protection
    \param[in]  ob_spc: specify security protection code
                only one parameter can be selected which is shown as below:
      \arg        FMC_NSPC: no security protection
      \arg        FMC_LSPC: low security protection
      \arg        FMC_HSPC: high security protection
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB_HSPC: FMC is under high security protection
*/
fmc_state_enum ob_security_protection_config(uint16_t ob_spc)
{
    uint32_t i;
    uint32_t op_byte[OB_WORD_CNT] = {0U};
    fmc_state_enum fmc_state;

    /* check the option bytes security protection value */
    if(OB_OBSTAT_PLEVEL_HIGH == ob_plevel_get()) {
        return FMC_OB_HSPC;
    }
    /* read option bytes */
    for(i = 0U; i < OB_WORD_CNT; i++) {
        op_byte[i] = OP_BYTE(i);
    }

    op_byte[0] = ((uint32_t)(ob_spc)) | ((op_byte[0] & HIGH_16BITS_MASK));

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start erase the option bytes */
        FMC_CTL1 |= FMC_CTL1_OB0ER;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {

            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;

            /* enable the option bytes programming */
            FMC_CTL1 |= FMC_CTL1_OB0PG;

            /* write option bytes */
            for(i = 0U; i < OB_DOUBLEWORD_CNT; i++) {
                OP_BYTE(i * 2U) = op_byte[i * 2U];
                OP_BYTE(i * 2U + 1U) = op_byte[i * 2U + 1U];
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state) {
                    break;
                }
            }

            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the OB0PG bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0PG;
        } else {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program the FMC user option bytes 0
                programmer must ensure FMC & option bytes are both unlocked before calling this function
                this function can only clear the corresponding bits to be 0 rather than 1.
                the function ob_erase is used to set all the bits to be 1.
    \param[in]  ob_user: user option bytes
                one or more (bitwise OR) parameters can be selected which are shown as below:
      \arg        OB_FWDGT_HW/OB_FWDGT_SW: free watchdog mode
      \arg        OB_DEEPSLEEP_RST/OB_DEEPSLEEP_NRST: generate a reset or enter deep-sleep mode
      \arg        OB_STDBY_RST/OB_STDBY_NRST: generate a reset or enter standby mode
      \arg        OB_BOOT_FROM_BANK1/OB_BOOT_FROM_BANK0: boot mdoe
      \arg        OB_BOOT_OTA_ENABLE/OB_BOOT_OTA_DISABLE: OTA mdoe
      \arg        OB_BOR_DISABLE/OB_BOR_ENABLE: BOR on/off
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB_HSPC: FMC is under high security protection
*/
fmc_state_enum ob_user_write(uint16_t ob_user)
{
    uint32_t i;
    uint32_t op_byte[OB_WORD_CNT] = {0U};
    uint8_t ob_user_0, ob_user_1, ob_user_temp;

    /* check the option bytes security protection value */
    if(OB_OBSTAT_PLEVEL_HIGH == ob_plevel_get()) {
        return FMC_OB_HSPC;
    }

    ob_user_0 = (uint8_t)(ob_user >> 8U);
    ob_user_1 = (uint8_t)(ob_user & 0xFFU);

    /* read option bytes */
    for(i = 0U; i < OB_WORD_CNT; i++) {
        op_byte[i] = OP_BYTE(i);
    }

    ob_user_temp = (uint8_t)(op_byte[0] >> 16U);
    ob_user_temp |= ob_user_1;
    ob_user_temp &= (uint8_t)(~ob_user_0);

    op_byte[0] = ((uint32_t)ob_user_temp << 16U) | ((op_byte[0] & LOW_16BITS_MASK));

    /* check whether FMC is ready or not */
    fmc_state_enum fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start erase the option bytes */
        FMC_CTL1 |= FMC_CTL1_OB0ER;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {

            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;

            /* enable the option bytes programming */
            FMC_CTL1 |= FMC_CTL1_OB0PG;

            /* write option bytes */
            for(i = 0U; i < OB_DOUBLEWORD_CNT; i++) {
                OP_BYTE(i * 2U) = op_byte[i * 2U];
                OP_BYTE(i * 2U + 1U) = op_byte[i * 2U + 1U];
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state) {
                    break;
                }
            }

            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the OB0PG bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0PG;
        } else {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program the FMC data option bytes 0
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  ob_data: the data to be programmed, OB_DATA[0:15]
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB_HSPC: FMC is under high security protection
*/
fmc_state_enum ob_data_program(uint16_t ob_data)
{
    uint32_t i;
    uint32_t val;
    uint32_t op_byte[OB_WORD_CNT] = {0U};
    fmc_state_enum fmc_state = FMC_READY;

    val = OB_SPC;
    /* check the option bytes security protection value */
    if(OB_OBSTAT_PLEVEL_HIGH == ob_plevel_get()) {
        return FMC_OB_HSPC;
    }

    /* read option bytes */
    for(i = 0U; i < OB_WORD_CNT; i++) {
        op_byte[i] = OP_BYTE(i);
    }

    val = (uint32_t)(ob_data & LOW_8BITS_MASK);
    val |= ((uint32_t)ob_data & HIGH_8BITS_MASK) << 8U;
    op_byte[1] = val;
    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* start erase the option bytes */
        FMC_CTL1 |= FMC_CTL1_OB0ER;
        FMC_CTL1 |= FMC_CTL1_START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_READY == fmc_state) {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;
            /* set the OB0PG bit */
            FMC_CTL1 |= FMC_CTL1_OB0PG;

            /* write option bytes */
            for(i = 0U; i < OB_DOUBLEWORD_CNT; i++) {
                OP_BYTE(i * 2U) = op_byte[i * 2U];
                OP_BYTE(i * 2U + 1U) = op_byte[i * 2U + 1U];
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
                if(FMC_READY != fmc_state) {
                    break;
                }
            }
            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the OB0PG bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0PG;
        } else {
            /* reset the OB0ER bit */
            FMC_CTL1 &= ~FMC_CTL1_OB0ER;
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get the value of FMC option bytes OB_USER in FMC_OBSTAT register
    \param[in]  none
    \param[out] none
    \retval     the option bytes USER value
*/
uint8_t ob_user_get(void)
{
    return (uint8_t)((FMC_OBSTAT & FMC_OBSTAT_USER) >> OBSTAT_USER_OFFSET);
}

/*!
    \brief      get the value of FMC option bytes OB_DATA in FMC_OBSTAT register
    \param[in]  none
    \param[out] none
    \retval     the option bytes DATA value
*/
uint16_t ob_data_get(void)
{
    return (uint16_t)((FMC_OBSTAT & FMC_OBSTAT_DATA) >> OBSTAT_DATA_OFFSET);
}

/*!
    \brief      get the value of FMC option bytes BK0WP in FMC_WP0 register
    \param[in]  none
    \param[out] none
    \retval     the option bytes BK0WP value
*/
uint32_t ob_write_protection_get(void)
{
    return (uint32_t)(FMC_WP0);
}

/*!
    \brief      get the value of FMC option bytes BK1WP in FMC_WP1 register
    \param[in]  none
    \param[out] none
    \retval     the option bytes BK1WP value
*/
uint8_t ob_bk1_write_protection_get(void)
{
    return (uint8_t)((FMC_WP1 & FMC_WP1_BK1WP) >> WP1_BK1WP_OFFSET);
}

/*!
    \brief      get the value of FMC option bytes DFWP in FMC_WP1 register
    \param[in]  none
    \param[out] none
    \retval     the option bytes DFWP value
*/
uint8_t ob_df_write_protection_get(void)
{
    return (uint8_t)((FMC_WP1 & FMC_WP1_DFWP) >> WP1_DFWP_OFFSET);
}

/*!
    \brief      get the value of FMC option bytes EPWP in FMC_WP1 register
    \param[in]  none
    \param[out] none
    \retval     the option bytes EPWP value
*/
uint8_t ob_ep_write_protection_get(void)
{
    return (uint8_t)((FMC_WP1 & FMC_WP1_EPWP) >> WP1_EPWP_OFFSET);
}

/*!
    \brief      get the value of FMC option bytes 0 security protection level (PLEVEL) in FMC_OBSTAT register
    \param[in]  none
    \param[out] none
    \retval     the value of PLEVEL
      \arg        OB_OBSTAT_PLEVEL_NO: no security protection
      \arg        OB_OBSTAT_PLEVEL_LOW: low security protection
      \arg        OB_OBSTAT_PLEVEL_HIGH: high security protection
*/
uint8_t ob_plevel_get(void)
{
    return (uint8_t)((FMC_OBSTAT & FMC_OBSTAT_PLEVEL) >> OBSTAT_PLEVEL_OFFSET);
}

/*!
    \brief      configure lock value in option bytes 1
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  lk_value: the LK value to be programmed (OB1_LK_VALUE)
                only one parameter can be selected which is shown as below:
      \arg        OB1CS_OB1_LK: when configured as OB1CS_OB1_LK, the option bytes 1 cannot be modified any more
      \arg        OB1CS_OB1_NOT_LK: option bytes 1 is not locked
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB1_LK: option bytes 1 is locked
*/
fmc_state_enum ob1_lock_config(uint32_t lk_value)
{
    uint32_t reg;

    /* check the option bytes 1 lock status */
    if(0U != (FMC_OB1CS & FMC_OB1CS_OB1LK)) {
        return FMC_OB1_LK;
    }

    reg = FMC_OB1CS;
    reg &= ~FMC_OB1CS_LKVAL;
    reg |= lk_value;

    /* wait for the FMC ready */
    fmc_state_enum fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* write the value to LKVAL in FMC_OB1CS register */
        FMC_OB1CS = reg;

        /* set the OB1START bit */
        FMC_OB1CS |= FMC_OB1CS_OB1START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure the EPLOAD value of option bytes 1 loaded after the system reset
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  epload: EPLOAD value to be programmed
                only one parameter can be selected which is shown as below:
      \arg        OB1CS_EPLOAD_NOT_LOAD_EPDATA: shared SRAM is not loaded with valid EEPROM data during FMC reset
      \arg        OB1CS_EPLOAD_LOAD_EPDATA: shared SRAM is loaded with valid EEPROM data during FMC reset
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB1_LK: option bytes 1 is locked
*/
fmc_state_enum ob1_epload_config(uint32_t epload)
{
    uint32_t reg;
    fmc_state_enum fmc_state = FMC_READY;

    /* check the option bytes 1 lock status */
    if(0U != (FMC_OB1CS & FMC_OB1CS_OB1LK)) {
        return FMC_OB1_LK;
    }

    reg = FMC_OB1CS;
    reg &= ~FMC_OB1CS_EPLOAD;
    reg |= epload;

    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* write the value to EPLOAD in FMC_OB1CS register */
        FMC_OB1CS = reg;

        /* set the OB1START bit */
        FMC_OB1CS |= FMC_OB1CS_OB1START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
        /* reset the OB1START bit */
        FMC_OB1CS &= ~FMC_OB1CS_OB1START;
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure option bytes 1 EEPROM parameters
                programmer must ensure FMC & option bytes are both unlocked before calling this function
    \param[in]  efalc: EFALC value to be programmed
                only one parameter can be selected which is shown as below:
      \arg        OB1CS_DF_64K_EF_0K: data flash size is 64KB, EEPROM backup size is 0KB
      \arg        OB1CS_DF_48K_EF_16K: data flash size is 48KB, EEPROM backup size is 16KB
      \arg        OB1CS_DF_32K_EF_32K: data flash size is 32KB, EEPROM backup size is 32KB
      \arg        OB1CS_DF_16K_EF_48K: data flash size is 16KB, EEPROM backup size is 48KB
      \arg        OB1CS_DF_0K_EF_64K: data flash size is 0KB, EEPROM backup size is 64KB
      \arg        OB1CS_DF_EF_INVALID: data flash and EEPROM backup are invalid
      \arg        OB1CS_DF_32K_EF_0K: data flash size is 32KB, EEPROM backup size is 0KB
      \arg        OB1CS_DF_8K_EF_24K: data flash size is 8KB, EEPROM backup size is 24KB
      \arg        OB1CS_DF_0K_EF_32K: data flash size is 0KB, EEPROM backup size is 32KB
      \arg        OB1CS_DF_16K_EF_16K: data flash size is 16KB, EEPROM backup size is 16KB
      \arg        OB1CS_DF_24K_EF_8K: data flash size is 24KB, EEPROM backup size is 8KB
      \arg        OB1CS_DF_16K_EF_0K: data flash size is 16KB, EEPROM backup size is 0KB
      \arg        OB1CS_DF_4K_EF_12K: data flash size is 4KB, EEPROM backup size is 12KB
      \arg        OB1CS_DF_0K_EF_16K: data flash size is 0KB, EEPROM backup size is 16KB
      \arg        OB1CS_DF_8K_EF_8K: data flash size is 8KB, EEPROM backup size is 8KB
      \arg        OB1CS_DF_12K_EF_4K: data flash size is 12KB, EEPROM backup size is 4KB
    \param[in]  epsize: EPSIZE value to be programmed
                only one parameter can be selected which is shown as below:
      \arg        OB1CS_EPSIZE_NONE: no EEPROM
      \arg        OB1CS_EPSIZE_4K: 4KB EEPROM
      \arg        OB1CS_EPSIZE_2K: 2KB EEPROM
      \arg        OB1CS_EPSIZE_1K: 1KB EEPROM
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
      \arg        FMC_OB1_LK: option bytes 1 is locked
*/
fmc_state_enum ob1_eeprom_parameter_config(uint32_t efalc, uint32_t epsize)
{
    uint32_t reg;
    fmc_state_enum fmc_state;

    /* check the option bytes security protection value */
    if(0U != (FMC_OB1CS & FMC_OB1CS_OB1LK)) {
        return FMC_OB1_LK;
    }
    /* configure the EFALC and EPSIZE */
    reg = FMC_OB1CS;
    reg &= ~FMC_OB1CS_EFALC;
    reg |= efalc;
    reg &= ~FMC_OB1CS_EPSIZE;
    reg |= epsize;

    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state) {
        /* write the value to EPSIZE and EFALC in FMC_OB1CS register */
        FMC_OB1CS = reg;

        /* set the OB1START bit */
        FMC_OB1CS |= FMC_OB1CS_OB1START;

        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
        /* reset the OB1START bit */
        FMC_OB1CS &= ~FMC_OB1CS_OB1START;
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get data flash size in byte unit
    \param[in]  none
    \param[out] none
    \retval     data flash byte count
*/
uint32_t dflash_size_get(void)
{
    uint32_t efalc_value;
    uint32_t dflash_size = 0U;

    /* get EFALC value in option bytes 1 */
    efalc_value = FMC_OB1CS & FMC_OB1CS_EFALC;

    switch(efalc_value) {
    case OB1CS_DF_64K_EF_0K:
        dflash_size = 64U * 0x400U;
        break;
    case OB1CS_DF_48K_EF_16K:
        dflash_size = 48U * 0x400U;
        break;
    case OB1CS_DF_32K_EF_32K:
        dflash_size = 32U * 0x400U;
        break;
    case OB1CS_DF_16K_EF_48K:
        dflash_size = 16U * 0x400U;
        break;
    case OB1CS_DF_0K_EF_64K:
        dflash_size = 0x00000000U;
        break;
    case OB1CS_DF_EF_INVALID:
        dflash_size = 0x00000000U;
        break;
    case OB1CS_DF_32K_EF_0K:
        dflash_size = 32U * 0x400U;
        break;
    case OB1CS_DF_8K_EF_24K:
        dflash_size = 8U * 0x400U;
        break;
    case OB1CS_DF_0K_EF_32K:
        dflash_size = 0x00000000U;
        break;
    case OB1CS_DF_16K_EF_16K:
        dflash_size = 16U * 0x400U;
        break;
    case OB1CS_DF_24K_EF_8K:
        dflash_size = 24U * 0x400U;
        break;
    case OB1CS_DF_16K_EF_0K:
        dflash_size = 16U * 0x400U;
        break;
    case OB1CS_DF_4K_EF_12K:
        dflash_size = 4U * 0x400U;
        break;
    case OB1CS_DF_0K_EF_16K:
        dflash_size = 0x00000000U;
        break;
    case OB1CS_DF_8K_EF_8K:
        dflash_size = 8U * 0x400U;
        break;
    case OB1CS_DF_12K_EF_4K:
        dflash_size = 12U * 0x400U;
        break;
    default:
        break;
    }

    return dflash_size;
}

/*!
    \brief      get EEPROM backup size in byte unit
    \param[in]  none
    \param[out] none
    \retval     EEPROM backup byte count
*/
uint32_t eeprom_backup_size_get(void)
{
    uint32_t efalc_value;
    uint32_t eeprom_backup_size = 0U;

    /* get EFALC value in option bytes 1 */
    efalc_value = FMC_OB1CS & FMC_OB1CS_EFALC;

    switch(efalc_value) {
    case OB1CS_DF_64K_EF_0K:
        eeprom_backup_size = 0x00000000U;
        break;
    case OB1CS_DF_48K_EF_16K:
        eeprom_backup_size = 16U * 0x400U;
        break;
    case OB1CS_DF_32K_EF_32K:
        eeprom_backup_size = 32U * 0x400U;
        break;
    case OB1CS_DF_16K_EF_48K:
        eeprom_backup_size = 48U * 0x400U;
        break;
    case OB1CS_DF_0K_EF_64K:
        eeprom_backup_size = 64U * 0x400U;
        break;
    case OB1CS_DF_EF_INVALID:
        eeprom_backup_size = 0x00000000U;
        break;
    case OB1CS_DF_32K_EF_0K:
        eeprom_backup_size = 0x00000000U;
        break;
    case OB1CS_DF_8K_EF_24K:
        eeprom_backup_size = 24U * 0x400U;
        break;
    case OB1CS_DF_0K_EF_32K:
        eeprom_backup_size = 0x00000000U;
        break;
    case OB1CS_DF_16K_EF_16K:
        eeprom_backup_size = 16U * 0x400U;
        break;
    case OB1CS_DF_24K_EF_8K:
        eeprom_backup_size = 8U * 0x400U;
        break;
    case OB1CS_DF_16K_EF_0K:
        eeprom_backup_size = 0x00000000U;
        break;
    case OB1CS_DF_4K_EF_12K:
        eeprom_backup_size = 12U * 0x400U;
        break;
    case OB1CS_DF_0K_EF_16K:
        eeprom_backup_size = 16U * 0x400U;
        break;
    case OB1CS_DF_8K_EF_8K:
        eeprom_backup_size = 8U * 0x400U;
        break;
    case OB1CS_DF_12K_EF_4K:
        eeprom_backup_size = 4U * 0x400U;
        break;
    default:
        break;
    }

    return eeprom_backup_size;
}

/*!
    \brief      get EEPROM size in byte unit
    \param[in]  none
    \param[out] none
    \retval     EEPROM byte count
*/
uint32_t eeprom_size_get(void)
{
    uint32_t epsize_value;
    uint32_t eeprom_size = 0U;

    /* get EPSIZE value in option bytes 1 */
    epsize_value = FMC_OB1CS & FMC_OB1CS_EPSIZE;

    switch(epsize_value) {
    case OB1CS_EPSIZE_NONE:
        eeprom_size = 0x000000000U;
        break;
    case OB1CS_EPSIZE_4K:
        eeprom_size = 4U * 0x400U;
        break;
    case OB1CS_EPSIZE_2K:
        eeprom_size = 2U * 0x400U;
        break;
    case OB1CS_EPSIZE_1K:
        eeprom_size = 0x400U;
        break;
    default:
        break;
    }

    return eeprom_size;
}

/*!
    \brief      get FMC flag status
    \param[in]  flag: FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_BANK0_FLAG_BUSY: flash bank0 busy flag
      \arg        FMC_BANK0_FLAG_PGSERR: flash bank0 program sequence error flag
      \arg        FMC_BANK0_FLAG_PGERR: flash bank0 program error flag
      \arg        FMC_BANK0_FLAG_PGAERR: flash bank0 program alignment error flag
      \arg        FMC_BANK0_FLAG_WPERR: flash bank0 erase/program protection error flag
      \arg        FMC_BANK0_FLAG_END: flash bank0 end of operation flag
      \arg        FMC_BANK0_FLAG_CBCMDERR: flash bank0 checked area by the check blank command is all 0xFF or not flag
      \arg        FMC_BANK0_FLAG_RSTERR: flash bank0 BOR/POR or system reset during erase/program flag
      \arg        FMC_BANK1_FLAG_BUSY: flash bank1 busy flag
      \arg        FMC_BANK1_FLAG_PGSERR: flash bank1 program sequence error flag
      \arg        FMC_BANK1_FLAG_PGERR: flash bank1 program error flag
      \arg        FMC_BANK1_FLAG_PGAERR: flash bank1 program alignment error flag
      \arg        FMC_BANK1_FLAG_WPERR: flash bank1 erase/program protection error flag
      \arg        FMC_BANK1_FLAG_END: flash bank1 end of operation flag
      \arg        FMC_BANK1_FLAG_CBCMDERR: flash bank1 checked area by the check blank command is all 0xFF or not flag
      \arg        FMC_BANK1_FLAG_RSTERR: flash bank1 BOR/POR or system reset during erase/program flag
      \arg        FMC_FLAG_BK0ECC: an ECC bit error is detected in bank 0 flag
      \arg        FMC_FLAG_OB0ECC: an ECC bit error is detected in option byte 0 flag
      \arg        FMC_FLAG_BK1ECC: an ECC bit error is detected in bank 1 flag
      \arg        FMC_FLAG_SYSECC: an ECC bit error is detected in system memory flag
      \arg        FMC_FLAG_DFECC: an ECC bit error is detected in data flash flag
      \arg        FMC_FLAG_OTPECC: an ECC bit error is detected in OTP flag
      \arg        FMC_FLAG_OB1ECCDET: option bytes 1 two bit error detect flag
      \arg        FMC_FLAG_OB0ECCDET: option bytes 0 two bit error detect flag
      \arg        FMC_FLAG_EPECCDET: EEPROM two bits error detect flag
      \arg        FMC_FLAG_ECCCOR: one bit error detected and correct flag
      \arg        FMC_FLAG_ECCDET: OTP/data flash/system memory/bank1 two bit error detect flag
      \arg        FMC_FLAG_OBERR: option bytes 0 error flag
      \arg        FMC_FLAG_OB1ERR: option bytes 1 read error flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_flag_get(fmc_flag_enum flag)
{
    /* check if an ECC bit error is detected in bank 0 */
    if(FMC_FLAG_BK0ECC == flag) {
        if(0U != (FMC_REG_VAL(flag) & ECCCS_ERROR_MASK)) {
            return RESET;
        } else {
            return SET;
        }
    } else {
        /* get other flag */
        if(0U != (FMC_REG_VAL(flag) & BIT(FMC_BIT_POS(flag)))) {
            return SET;
        } else {
            return RESET;
        }
    }
}

/*!
    \brief      clear FMC flag status
    \param[in]  flag: FMC flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_BANK0_FLAG_PGSERR: flash bank0 program sequence error flag
      \arg        FMC_BANK0_FLAG_PGERR: flash bank0 program error flag
      \arg        FMC_BANK0_FLAG_PGAERR: flash bank0 program alignment error flag
      \arg        FMC_BANK0_FLAG_WPERR: flash bank0 erase/program protection error flag
      \arg        FMC_BANK0_FLAG_END: flash bank0 end of operation flag
      \arg        FMC_BANK0_FLAG_CBCMDERR: flash bank0 checked area by the check blank command is all 0xFF or not flag
      \arg        FMC_BANK0_FLAG_RSTERR: flash bank0 BOR/POR or system reset during erase/program flag
      \arg        FMC_BANK1_FLAG_PGSERR: flash bank1 program sequence error flag
      \arg        FMC_BANK1_FLAG_PGERR: flash bank1 program error flag
      \arg        FMC_BANK1_FLAG_PGAERR: flash bank1 program alignment error flag
      \arg        FMC_BANK1_FLAG_WPERR: flash bank1 erase/program protection error flag
      \arg        FMC_BANK1_FLAG_END: flash bank1 end of operation flag
      \arg        FMC_BANK1_FLAG_CBCMDERR: flash bank1 checked area by the check blank command is all 0xFF or not flag
      \arg        FMC_BANK1_FLAG_RSTERR: flash bank1 BOR/POR or system reset during erase/program flag
      \arg        FMC_FLAG_OB1ECCDET: option bytes 1 two bit error detect flag
      \arg        FMC_FLAG_OB0ECCDET: option bytes 0 two bit error detect flag
      \arg        FMC_FLAG_EPECCDET: EEPROM two bits error detect flag
      \arg        FMC_FLAG_ECCCOR: one bit error detected and correct flag
      \arg        FMC_FLAG_ECCDET: OTP/data flash/system memory/bank1 two bit error detect flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
void fmc_flag_clear(fmc_flag_enum flag)
{
    uint32_t reg_offset, reg;

    reg_offset = ((uint32_t)(flag) & 0x0000FFFFU) >> 6U;
    /* clear the flags in ECCCS register */
    if(ECCCS_REG_OFFSET == reg_offset) {
        reg = FMC_REG_VAL(flag);
        reg &= ~ECCCS_FLAG_MASK;
        reg |= BIT(FMC_BIT_POS(flag));
        FMC_REG_VAL(flag) = reg;
    } else {
        /* clear the flags in STAT0/STAT1 register */
        FMC_REG_VAL(flag) = BIT(FMC_BIT_POS(flag));
    }
}

/*!
    \brief      enable FMC interrupt
    \param[in]  interrupt: the FMC interrupt source
                only one parameter can be selected which is shown as below:
      \arg        FMC_BANK0_INT_ERR: FMC bank0 error interrupt
      \arg        FMC_BANK0_INT_END: FMC bank0 end of operation interrupt
      \arg        FMC_BANK1_INT_ERR: FMC bank1 error interrupt
      \arg        FMC_BANK1_INT_END: FMC bank1 end of operation interrupt
      \arg        FMC_INT_ECCCOR: FMC one bit error correct interrupt
      \arg        FMC_INT_ECCDET: FMC two bits error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_enable(fmc_interrupt_enum interrupt)
{
    uint32_t reg_offset, reg;

    reg_offset = ((uint32_t)(interrupt) & 0x0000FFFFU) >> 6U;
    /* enable interrupt in ECCCS register */
    if(ECCCS_REG_OFFSET == reg_offset) {
        reg = FMC_REG_VAL(interrupt);
        reg &= ~ECCCS_FLAG_MASK;
        reg |= BIT(FMC_BIT_POS(interrupt));
        FMC_REG_VAL(interrupt) = reg;
    } else {
        /* enable interrupt in CTL0/CTL1 register */
        FMC_REG_VAL(interrupt) |= BIT(FMC_BIT_POS(interrupt));
    }
}

/*!
    \brief      disable FMC interrupt
    \param[in]  interrupt: the FMC interrupt source
                only one parameter can be selected which is shown as below:
      \arg        FMC_BANK0_INT_ERR: FMC bank0 error interrupt
      \arg        FMC_BANK0_INT_END: FMC bank0 end of operation interrupt
      \arg        FMC_BANK1_INT_ERR: FMC bank1 error interrupt
      \arg        FMC_BANK1_INT_END: FMC bank1 end of operation interrupt
      \arg        FMC_INT_ECCCOR: FMC one bit error correct interrupt
      \arg        FMC_INT_ECCDET: FMC two bits error interrupt
    \param[out] none
    \retval     none
*/
void fmc_interrupt_disable(fmc_interrupt_enum interrupt)
{
    uint32_t reg_offset, reg;

    reg_offset = ((uint32_t)(interrupt) & 0x0000FFFFU) >> 6U;
    /* disable interrupt in ECCCS register */
    if(ECCCS_REG_OFFSET == reg_offset) {
        reg = FMC_REG_VAL(interrupt);
        reg &= ~ECCCS_FLAG_MASK;
        reg &= ~BIT(FMC_BIT_POS(interrupt));
        FMC_REG_VAL(interrupt) = reg;
    } else {
        /* disable interrupt in CTL0/CTL1 register */
        FMC_REG_VAL(interrupt) &= ~BIT(FMC_BIT_POS(interrupt));
    }
}

/*!
    \brief      get FMC interrupt flag status
    \param[in]  flag: FMC interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_BANK0_INT_FLAG_PGSERR: flash bank0 program sequence error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_PGERR: flash bank0 program error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_PGAERR: flash bank0 program alignment error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_WPERR: flash bank0 erase/program protection error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_END: flash bank0 end of operation intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_CBCMDERR: flash bank0 checked area by the check blank command is all 0xFF or not intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_RSTERR: flash bank0 BOR/POR or system reset during erase/program intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_PGSERR: flash bank1 program sequence error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_PGERR: flash bank1 program error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_PGAERR: flash bank1 program alignment error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_WPERR: flash bank1 erase/program protection error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_END: flash bank1 end of operation intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_CBCMDERR: flash bank1 checked area by the check blank command is all 0xFF or not intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_RSTERR: flash bank1 BOR/POR or system reset during erase/program intrrupt flag
      \arg        FMC_INT_FLAG_OB1ECCDET: option bytes 1 two bit error detect intrrupt flag
      \arg        FMC_INT_FLAG_OB0ECCDET: option bytes 0 two bit error detect intrrupt flag
      \arg        FMC_INT_FLAG_EPECCDET: EEPROM two bits error detect intrrupt flag
      \arg        FMC_INT_FLAG_ECCCOR: one bit error detected and correct intrrupt flag
      \arg        FMC_INT_FLAG_ECCDET: OTP/data flash/system memory/bank1 two bit error detect intrrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_interrupt_flag_get(fmc_interrupt_flag_enum int_flag)
{
    uint32_t intenable = 0U, flagstatus = 0U;
    /* get the interrupt enable bit status */
    intenable = (FMC_REG_VAL(int_flag) & BIT(FMC_BIT_POS(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (FMC_REG_VAL2(int_flag) & BIT(FMC_BIT_POS2(int_flag)));

    if(flagstatus && intenable) {
        return SET;
    } else {
        return RESET;
    }
}

/*!
    \brief      clear FMC interrupt flag status
    \param[in]  flag: FMC interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        FMC_BANK0_INT_FLAG_PGSERR: flash bank0 program sequence error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_PGERR: flash bank0 program error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_PGAERR: flash bank0 program alignment error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_WPERR: flash bank0 erase/program protection error intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_END: flash bank0 end of operation intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_CBCMDERR: flash bank0 checked area by the check blank command is all 0xFF or not intrrupt flag
      \arg        FMC_BANK0_INT_FLAG_RSTERR: flash bank0 BOR/POR or system reset during erase/program intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_PGSERR: flash bank1 program sequence error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_PGERR: flash bank1 program error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_PGAERR: flash bank1 program alignment error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_WPERR: flash bank1 erase/program protection error intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_END: flash bank1 end of operation intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_CBCMDERR: flash bank1 checked area by the check blank command is all 0xFF or not intrrupt flag
      \arg        FMC_BANK1_INT_FLAG_RSTERR: flash bank1 BOR/POR or system reset during erase/program intrrupt flag
      \arg        FMC_INT_FLAG_OB1ECCDET: option bytes 1 two bit error detect intrrupt flag
      \arg        FMC_INT_FLAG_OB0ECCDET: option bytes 0 two bit error detect intrrupt flag
      \arg        FMC_INT_FLAG_EPECCDET: EEPROM two bits error detect intrrupt flag
      \arg        FMC_INT_FLAG_ECCCOR: one bit error detected and correct intrrupt flag
      \arg        FMC_INT_FLAG_ECCDET: OTP/data flash/system memory/bank1 two bit error detect intrrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
void fmc_interrupt_flag_clear(fmc_interrupt_flag_enum int_flag)
{
    uint32_t reg_offset, reg;

    reg_offset = (uint32_t)(int_flag) >> 22U;
    /* clear the intrrupt flag in ECCCS register */
    if(ECCCS_REG_OFFSET == reg_offset) {
        reg = FMC_REG_VAL2(int_flag);
        reg &= ~ECCCS_FLAG_MASK;
        reg |= BIT(FMC_BIT_POS2(int_flag));
        FMC_REG_VAL2(int_flag) = reg;
    } else {
        /* clear the intrrupt flag in STAT0/STAT1 register */
        FMC_REG_VAL2(int_flag) = BIT(FMC_BIT_POS2(int_flag));
    }
}

/*!
    \brief      get FMC bank0 state
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
static fmc_state_enum fmc_bank0_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;

    if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_BUSY)) {
        fmc_state = FMC_BUSY;
    } else {
        if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_WPERR)) {
            fmc_state = FMC_WPERR;
        } else if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_PGERR)) {
            fmc_state = FMC_PGERR;
        } else if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_PGSERR)) {
            fmc_state = FMC_PGSERR;
        } else if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_PGAERR)) {
            fmc_state = FMC_PGAERR;
        } else if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_CBCMDERR)) {
            fmc_state = FMC_CBCMDERR;
        } else if((uint32_t)0x00U != (FMC_STAT0 & FMC_STAT0_RSTERR)) {
            fmc_state = FMC_RSTERR;
        } else {
            /* illegal parameters */
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get FMC bank1 state
    \param[in]  none
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
static fmc_state_enum fmc_bank1_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;

    if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_BUSY)) {
        fmc_state = FMC_BUSY;
    } else {
        if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_WPERR)) {
            fmc_state = FMC_WPERR;
        } else if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_PGERR)) {
            fmc_state = FMC_PGERR;
        } else if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_PGSERR)) {
            fmc_state = FMC_PGSERR;
        } else if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_PGAERR)) {
            fmc_state = FMC_PGAERR;
        } else if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_CBCMDERR)) {
            fmc_state = FMC_CBCMDERR;
        } else if((uint32_t)0x00U != (FMC_STAT1 & FMC_STAT1_RSTERR)) {
            fmc_state = FMC_RSTERR;
        } else {
            /* illegal parameters */
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC bank0 is ready or not
    \param[in]  timeout: timeout count
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
static fmc_state_enum fmc_bank0_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;

    /* wait for FMC ready */
    do {
        /* get FMC state */
        fmc_state = fmc_bank0_state_get();
        timeout--;
    } while((FMC_BUSY == fmc_state) && (0U != timeout));

    if(FMC_BUSY == fmc_state) {
        fmc_state = FMC_TOERR;
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC bank1 is ready or not
    \param[in]  timeout: timeout count
    \param[out] none
    \retval     state of FMC, refer to fmc_state_enum
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGSERR:  program sequence error
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_CBCMDERR: the checked area not blank error
      \arg        FMC_RSTERR: BOR/POR or system reset during flash erase/program error
*/
static fmc_state_enum fmc_bank1_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;

    /* wait for FMC ready */
    do {
        /* get FMC state */
        fmc_state = fmc_bank1_state_get();
        timeout--;
    } while((FMC_BUSY == fmc_state) && (0U != timeout));

    if(FMC_BUSY == fmc_state) {
        fmc_state = FMC_TOERR;
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      wait shared SRAM mode to be ready
    \param[in]  ready_flag: sram mode ready bit
      \arg        FMC_WS_ERAMRDY: EEPROM SRAM ready bit
      \arg        FMC_WS_BRAMRDY: basic SRAM ready bit
      \arg        FMC_WS_PRAMRDY: fast PG SRAM ready bit
    \param[out] none
    \retval     none
*/
static void fmc_sram_mode_ready_wait(uint32_t ready_flag)
{
    while(1) {
        if(0U != (FMC_WS & ready_flag)) {
            break;
        }
    }
}
