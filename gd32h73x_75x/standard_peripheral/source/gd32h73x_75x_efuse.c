/*!
    \file    gd32h73x_75x_efuse.c
    \brief   EFUSE driver

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

#include "gd32h73x_75x_efuse.h"

/* FMC register bit offset */
#define EFUSE_CTL_AES_KEY_CRC_OFFSET    ((uint32_t)0x00000018U)               /*!< bit offset of AES_KEY_CRC in EFUSE_CTL register*/
#define EFUSE_CTL_MPVEN_OFFSET          ((uint32_t)0x0000000FU)               /*!< bit offset of MPVEN in EFUSE_CTL register*/
#define EFUSE_STAT_LDO_RDY_OFFSET       ((uint32_t)0x00000004U)               /*!< bit offset of LDO_RDY in EFUSE_STAT register*/
#define EFUSE_ADDR_EFSIZE_OFFSET        ((uint32_t)0x0000000AU)               /*!< EFSIZE OFFSET in register EFUSE_ADDR */

#define EFUSE_TIMEOUT                   ((uint32_t)0x0000FFFFU)              /*!< EFUSE operation timeout value */
#define USER_CTL_NDBG0                  BIT(8)                                /*!< debug mode setting bit0 in register EFUSE_USER_CTL */

/* EFUSE parameter mask */
#define EFUSE_FLAG_MASK                 (EFUSE_FLAG_ILLEGAL_ACCESS_ERR | EFUSE_FLAG_PROGRAM_COMPLETE | \
                                         EFUSE_FLAG_READ_COMPLETE | EFUSE_FLAG_PROGRAM_VOLTAGE_ERR)
#define EFUSE_FLAG_CLEAR_MASK           (EFUSE_FLAG_ILLEGAL_ACCESS_ERR_CLR | EFUSE_FLAG_PROGRAM_COMPLETE_CLR | \
                                         EFUSE_FLAG_READ_COMPLETE_CLR | EFUSE_FLAG_PROGRAM_VOLTAGE_ERR_CLR)
#define EFUSE_INT_MASK                  (EFUSE_INT_ILLEGAL_ACCESS_ERR | EFUSE_INT_PROGRAM_COMPLETE | \
                                         EFUSE_INT_READ_COMPLETE | EFUSE_INT_PROGRAM_VOLTAGE_ERR)
#define EFUSE_INT_FLAG_CLEAR_MASK       (EFUSE_INT_FLAG_ILLEGAL_ACCESS_ERR_CLR | EFUSE_INT_FLAG_PROGRAM_COMPLETE_CLR | \
                                         EFUSE_INT_FLAG_READ_COMPLETE_CLR | EFUSE_INT_FLAG_PROGRAM_VOLTAGE_ERR_CLR)

static uint32_t para_start_efaddr[EFUSE_PARA_CNT] = {USER_CTL_EFADDR, MCU_RESERVED_EFADDR, DP_EFADDR, AES_KEY_EFADDR, USER_DATA_EFADDR};
static uint32_t para_reg_start_addr[EFUSE_PARA_CNT] = {EFUSE_USER_CTL_REG_ADDR, EFUSE_MCU_RSV_REG_ADDR, EFUSE_DP_REG_ADDR, EFUSE_AES_KEY_REG_ADDR, EFUSE_USER_DATA_REG_ADDR};
static efuse_state_enum efuse_ready_wait(uint32_t efuse_flag, uint32_t timeout);

/*!
    \brief      read system parameters from EFUSE macro to registers (API_ID(0x0001U))
    \param[in]  ef_addr: start address of the system parameters to be read
                only one parameter can be selected which is shown as below:
      \arg        USER_CTL_EFADDR: user control parameter start address
      \arg        MCU_RESERVED_EFADDR: MCU reserved parameter start address
      \arg        DP_EFADDR: debug password parameter start address
      \arg        USER_DATA_EFADDR: user data parameter start address
    \param[in]  size: size of the system parameters to be read
                only one parameter can be selected which is shown as below:
      \arg        USER_CTL_MCU_RESERVED_SIZE: user control parameter size, MCU reserved parameter size
      \arg        DP_SIZE: debug password parameter size
      \arg        USER_DATA_SIZE: user data parameter size
    \param[out] buf: the buffer for data read from EFUSE macro
    \retval     ErrStatus: ERROR or SUCCESS
*/

ErrStatus efuse_read(uint32_t ef_addr, uint32_t size, uint32_t buf[])
{
    ErrStatus status = ERROR;
    uint32_t timeout = EFUSE_TIMEOUT;
    efuse_state_enum efuse_state;
    uint32_t reg_addr = 0U;
    uint32_t i = 0U;
    uint32_t number = 0U;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_EFUSE_ADDR(ef_addr)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0001U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_SIZE(size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0001U), ERR_PARAM_INVALID);
    } else if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0001U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        switch(ef_addr) {
        case USER_CTL_EFADDR:
            break;
        case MCU_RESERVED_EFADDR:
            /* read MCU reserved data */
            reg_addr = EFUSE_MCU_RSV_REG_ADDR;
            number = 1U;
            status = SUCCESS;
            break;
        case DP_EFADDR:
            status = SUCCESS;
            /* read debug password */
            if(RESET != (EFUSE_USER_CTL & EFUSE_USER_CTL_DPLK)) {
                if(RESET != (EFUSE_USER_CTL & EFUSE_USER_CTL_JTAGNSW)) {
                    if((RESET != (EFUSE_USER_CTL & USER_CTL_NDBG0))) {
                        status = ERROR;
                    }
                }
            }
            if(SUCCESS == status) {
                reg_addr = EFUSE_DP_REG_ADDR;
                number = 2U;
            }
            break;
        case USER_DATA_EFADDR:
            /* read user data */
            reg_addr = EFUSE_USER_DATA_REG_ADDR;
            number = 4U;
            status = SUCCESS;
            break;
        default:
            break;
        }

        if(SUCCESS == status) {
            /* clear the RDIF bit if it is SET */
            efuse_flag_clear(EFUSE_FLAG_READ_COMPLETE_CLR);
            /* reset the EFRW bit in EFUSE_CTL */
            EFUSE_CTL &= ~EFUSE_CTL_EFRW;
            /* write the desired efuse address and size to the EFUSE_ADDR register */
            EFUSE_ADDR = (uint32_t)(((size << EFUSE_ADDR_EFSIZE_OFFSET) & EFUSE_ADDR_EFSIZE) | (ef_addr & EFUSE_ADDR_EFADDR));
            /* start array read EFUSE operation */
            EFUSE_CTL |= EFUSE_CTL_EFSTR;
            /* wait for the operation to complete */
            efuse_state = efuse_ready_wait(EFUSE_FLAG_READ_COMPLETE, timeout);
            if(EFUSE_READY != efuse_state) {
                status = ERROR;
            }
            /* read EFUSE register */
            for(i = 0U; i < number; i++) {
                buf[i] = REG32(reg_addr + (4U * i));
            }
        }
    }

    return status;
}

/*!
    \brief      program register values to EFUSE macro system parameters (API_ID(0x0002U))
    \param[in]  ef_addr: the EFUSE address to be programmed, pgm_addr cannot exceed 384, and must be an integral multiple of 8
    \param[in]  size: byte count to program, (1~16)
    \param[in]  buf: the buffer for data written to EFUSE macro
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus efuse_write(uint32_t ef_addr, uint32_t size, uint8_t *buf)
{
    uint32_t i;
    uint32_t reg_addr;
    uint32_t byte_offset_in_reg;
    uint32_t cnt;
    ErrStatus status = ERROR;
    uint32_t para_index = 0U;
    uint32_t tmp_buf_8;
    uint32_t buf_addr;
    uint32_t timeout = EFUSE_TIMEOUT;
    efuse_state_enum efuse_state;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_EFUSE_VALID_SIZE(size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR(ef_addr)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR_CONFIG5(ef_addr, size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR_CONFIG4(ef_addr, size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR_CONFIG3(ef_addr, size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR_CONFIG2(ef_addr, size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR_CONFIG1(ef_addr, size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_EFUSE_VALID_ADDR_CONFIG0(ef_addr, size)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_INVALID);
    } else if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        /* parameter validation */
        if(0U == size) {
            status = ERROR;
        } else if(ef_addr % 8U) {
            /* the address should be on byte address boundary */
            status = ERROR;
        } else if(MAX_EFADDR < ef_addr) {
            status = ERROR;
        } else {
            status = SUCCESS;
            for(i = EFUSE_PARA_CNT; i > 0U; i--) {
                if(ef_addr >= para_start_efaddr[i - 1U]) {
                    break;
                }
            }
            /* get the index of parameter to be programmed */
            para_index = i - 1U;
            /* program range should not over parameter boundary */
            if(para_index == (EFUSE_PARA_CNT - 1U)) {
                if((ef_addr + size * 8U - 1U) > MAX_EFADDR) {
                    status = ERROR;
                }
            } else {
                if((ef_addr + size * 8U - 1U) > para_start_efaddr[para_index + 1U]) {
                    status = ERROR;
                }
            }
            if((AES_KEY_IDX == para_index) && (AES_KEY_SIZE != size)) {
                /* AES key should be programmed in one time */
                status = ERROR;
            }
        }

        /* proceed with programming if validation passed */
        if(SUCCESS == status) {
            reg_addr = (unsigned int)para_reg_start_addr[para_index] + (ef_addr - para_start_efaddr[para_index]) / 32U * 4U;
            byte_offset_in_reg = ((ef_addr - para_start_efaddr[para_index]) / 8U) % 4U;
            /* clear the PGIF bit if it is SET */
            efuse_flag_clear(EFUSE_FLAG_PROGRAM_COMPLETE_CLR);
            /* set the EFRW bit in EFUSE_CTL */
            EFUSE_CTL |= EFUSE_CTL_EFRW;
            /* write the desired efuse address and size to the EFUSE_ADDR register */
            EFUSE_ADDR = (uint32_t)(((size << EFUSE_ADDR_EFSIZE_OFFSET) & EFUSE_ADDR_EFSIZE) | (ef_addr & EFUSE_ADDR_EFADDR));
            buf_addr = (uint32_t)buf;

            while(size) {
                if((0U != byte_offset_in_reg) || ((0U == byte_offset_in_reg) && (size < 4U))) {
                    cnt = size < (4U - byte_offset_in_reg) ? size : 4U - byte_offset_in_reg;
                    for(i = 0U; i < cnt; i++) {
                        tmp_buf_8 = buf_addr;
                        /* write the data to the corresponding register */
                        tmp_buf_8 += i;
                        REG32(reg_addr) |= (((uint32_t)(*(uint8_t *)(tmp_buf_8))) << ((byte_offset_in_reg + i) * 8U));
                    }
                    size -= cnt;
                    reg_addr += 4U;
                    byte_offset_in_reg = 0U;
                    buf_addr += cnt;
                } else {
                    cnt = size / 4U;
                    for(i = 0U; i < cnt; i++) {
                        tmp_buf_8 = buf_addr;
                        /* write the data to the corresponding register */
                        tmp_buf_8 += (i * 4U);
                        REG32(reg_addr) = (uint32_t)(*(uint32_t *)(tmp_buf_8));
                        reg_addr += 4U;
                    }
                    size -= cnt * 4U;
                    buf_addr += cnt * 4U;
                }
            }
            /* start EFUSE program operation */
            EFUSE_CTL |= EFUSE_CTL_EFSTR;
            /* wait for the operation to complete */
            efuse_state = efuse_ready_wait(EFUSE_FLAG_PROGRAM_COMPLETE, timeout);
            if(EFUSE_READY != efuse_state) {
                status = ERROR;
            }
        }
    }

    return status;
}

/*!
    \brief      program all user control parameters (API_ID(0x0003U))
    \param[in]  buf: the buffer of data written to efuse
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus efuse_user_control_write(uint8_t *buf)
{
    ErrStatus status = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        status = efuse_write(USER_CTL_EFADDR, USER_CTL_MCU_RESERVED_SIZE, buf);
    }

    return status;
}

/*!
    \brief      program all MCU reserved parameters (API_ID(0x0004U))
    \param[in]  buf: the buffer of data written to efuse
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus efuse_mcu_reserved_write(uint8_t *buf)
{
    ErrStatus status = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0004U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        status = efuse_write(MCU_RESERVED_EFADDR, USER_CTL_MCU_RESERVED_SIZE, buf);
    }

    return status;
}

/*!
    \brief      program all debug password parameters (API_ID(0x0005U))
    \param[in]  buf: the buffer of data written to efuse
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus efuse_dp_write(uint8_t *buf)
{
    ErrStatus status = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0005U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        status = efuse_write(DP_EFADDR, DP_SIZE, buf);
    }

    return status;
}

/*!
    \brief      program all AES key parameters (API_ID(0x0006U))
    \param[in]  buf: the buffer of data written to efuse
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus efuse_aes_key_write(uint8_t *buf)
{
    ErrStatus status = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0006U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        status = efuse_write(AES_KEY_EFADDR, AES_KEY_SIZE, buf);
    }

    return status;
}

/*!
    \brief      program all user data parameters (API_ID(0x0007U))
    \param[in]  buf: the buffer of data written to efuse
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus efuse_user_data_write(uint8_t *buf)
{
    ErrStatus status = ERROR;

#ifdef FW_DEBUG_ERR_REPORT
    /* check parameter */
    if(NOT_VALID_POINTER(buf)) {
        fw_debug_report_err(EFUSE_MODULE_ID, API_ID(0x0007U), ERR_PARAM_POINTER);
    } else
#endif /* FW_DEBUG_ERR_REPORT */
    {
        status = efuse_write(USER_DATA_EFADDR, USER_DATA_SIZE, buf);
    }

    return status;
}

/*!
    \brief      get 8-bits CRC calculation result value of AES key (API_ID(0x0008U))
    \param[in]  none
    \param[out] none
    \retval     8-bits CRC calculation result value of AES key
*/
uint8_t efuse_aes_key_crc_get(void)
{
    return (uint8_t)((EFUSE_CTL & EFUSE_CTL_AES_KEY_CRC) >> EFUSE_CTL_AES_KEY_CRC_OFFSET);
}

/*!
    \brief      enable monitor program voltage function (API_ID(0x0009U))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void efuse_monitor_program_voltage_enable(void)
{
    uint32_t ctl_reg;
    ctl_reg = EFUSE_CTL;
    /* enable monitor program voltage function */
    ctl_reg |= EFUSE_CTL_MPVEN;
    EFUSE_CTL = ctl_reg;
}

/*!
    \brief      disable monitor program voltage function (API_ID(0x000AU))
    \param[in]  none
    \param[out] none
    \retval     none
*/
void efuse_monitor_program_voltage_disable(void)
{
    uint32_t ctl_reg;
    ctl_reg = EFUSE_CTL;
    /* disable monitor program voltage function */
    ctl_reg &= ~EFUSE_CTL_MPVEN;
    EFUSE_CTL = ctl_reg;
}

/*!
    \brief      get monitor program voltage function (API_ID(0x000BU))
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus efuse_monitor_program_voltage_get(void)
{
    FlagStatus mpven_state = RESET;

    if(EFUSE_CTL_MPVEN == (uint32_t)(EFUSE_CTL & EFUSE_CTL_MPVEN)) {
        mpven_state = SET;
    } else {
        mpven_state = RESET;
    }
    return mpven_state;
}

/*!
    \brief      get ldo ready signal (API_ID(0x000CU))
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus efuse_ldo_ready_get(void)
{
    FlagStatus ldo_ready_state = RESET;

    if(EFUSE_STAT_LDO_RDY == (uint32_t)(EFUSE_STAT & EFUSE_STAT_LDO_RDY)) {
        ldo_ready_state = SET;
    } else {
        ldo_ready_state = RESET;
    }
    return ldo_ready_state;
}

/*!
    \brief      check EFUSE flag is set or not (API_ID(0x000DU))
    \param[in]  flag: specifies to get a flag
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_FLAG_ILLEGAL_ACCESS_ERR: illegal access error flag
      \arg        EFUSE_FLAG_PROGRAM_COMPLETE: programming operation completion flag
      \arg        EFUSE_FLAG_READ_COMPLETE: read operation completion flag
      \arg        EFUSE_FLAG_PROGRAM_VOLTAGE_ERR: program voltage setting error flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus efuse_flag_get(uint32_t flag)
{
    FlagStatus reval = RESET;

    if(EFUSE_STAT & (uint32_t)(flag & EFUSE_FLAG_MASK)) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      clear EFUSE pending flag (API_ID(0x000EU))
    \param[in]  flag: specifies to clear a flag
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_FLAG_ILLEGAL_ACCESS_ERR_CLR: clear illegal access error flag
      \arg        EFUSE_FLAG_PROGRAM_COMPLETE_CLR: clear programming operation completion flag
      \arg        EFUSE_FLAG_READ_COMPLETE_CLR: clear read operation completion flag
      \arg        EFUSE_FLAG_PROGRAM_VOLTAGE_ERR_CLR: clear program voltage setting error interrupt flag
    \param[out] none
    \retval     none
*/
void efuse_flag_clear(uint32_t flag)
{
    EFUSE_STATC |= (uint32_t)(flag & EFUSE_FLAG_CLEAR_MASK);
}

/*!
    \brief      enable EFUSE interrupt (API_ID(0x000FU))
    \param[in]  interrupt: specifies an interrupt to enbale
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_INT_ILLEGAL_ACCESS_ERR: illegal access error interrupt
      \arg        EFUSE_INT_PROGRAM_COMPLETE: programming operation completion interrupt
      \arg        EFUSE_INT_READ_COMPLETE: read operation completion interrupt
      \arg        EFUSE_INT_PROGRAM_VOLTAGE_ERR: program voltage setting error interrupt
    \param[out] none
    \retval     none
*/
void efuse_interrupt_enable(uint32_t interrupt)
{
    EFUSE_CTL |= (uint32_t)(interrupt & EFUSE_INT_MASK);
}

/*!
    \brief      disable EFUSE interrupt (API_ID(0x0010U))
    \param[in]  interrupt: specifies an interrupt to disbale
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_INT_ILLEGAL_ACCESS_ERR: illegal access error interrupt
      \arg        EFUSE_INT_PROGRAM_COMPLETE: programming operation completion interrupt
      \arg        EFUSE_INT_READ_COMPLETE: read operation completion interrupt
      \arg        EFUSE_INT_PROGRAM_VOLTAGE_ERR: program voltage setting error interrupt
    \param[out] none
    \retval     none
*/
void efuse_interrupt_disable(uint32_t interrupt)
{
    EFUSE_CTL &= ~((uint32_t)(interrupt & EFUSE_INT_MASK));
}

/*!
    \brief      check EFUSE interrupt flag is set or not (API_ID(0x0011U))
    \param[in]  int_flag: specifies to get a flag
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_INT_FLAG_ILLEGAL_ACCESS_ERR: illegal access error interrupt
      \arg        EFUSE_INT_FLAG_PROGRAM_COMPLETE: programming operation completion interrupt
      \arg        EFUSE_INT_FLAG_READ_COMPLETE: read operation completion interrupt
      \arg        EFUSE_INT_FLAG_PROGRAM_VOLTAGE_ERR: program voltage setting error interrupt
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus efuse_interrupt_flag_get(efuse_interrupt_flag_enum int_flag)
{
    FlagStatus reval = RESET;
    uint32_t intenable = 0U, flagstatus = 0U;
    /* get the interrupt enable bit status */
    intenable = (EFUSE_REG_VAL(int_flag) & BIT(EFUSE_BIT_POS(int_flag)));
    /* get the corresponding flag bit status */
    flagstatus = (EFUSE_REG_VAL2(int_flag) & BIT(EFUSE_BIT_POS2(int_flag)));

    if(flagstatus && intenable) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      clear EFUSE pending interrupt flag (API_ID(0x0012U))
    \param[in]  int_flag: specifies to clear a flag
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_INT_FLAG_ILLEGAL_ACCESS_ERR_CLR: clear illegal access error interrupt flag
      \arg        EFUSE_INT_FLAG_PROGRAM_COMPLETE_CLR: clear programming operation completion interrupt flag
      \arg        EFUSE_INT_FLAG_READ_COMPLETE_CLR: clear operation completion interrupt flag
      \arg        EFUSE_INT_FLAG_PROGRAM_VOLTAGE_ERR_CLR: clear program voltage setting error interrupt flag
    \param[out] none
    \retval     none
*/
void efuse_interrupt_flag_clear(uint32_t int_flag)
{
    EFUSE_STATC |= (uint32_t)(int_flag & EFUSE_INT_FLAG_CLEAR_MASK);
}

/*!
    \brief      check whether EFUSE is ready or not (API_ID(0x0013U))
    \note       This function includes timeout exit scenarios.
                Modify according to the user's actual usage scenarios.
    \param[in]  flag:
                only one parameter can be selected which is shown as below:
      \arg        EFUSE_FLAG_ILLEGAL_ACCESS_ERR: illegal access error flag
      \arg        EFUSE_FLAG_PROGRAM_COMPLETE: programming operation completion flag
      \arg        EFUSE_FLAG_READ_COMPLETE: read operation completion flag
      \arg        EFUSE_FLAG_PROGRAM_VOLTAGE_ERR: program voltage setting error flag
    \param[out] none
    \retval     state of EFUSE
      \arg        EFUSE_READY: EFUSE operation has been completed
      \arg        EFUSE_BUSY: EFUSE operation is in progress
      \arg        EFUSE_IAERR: illegal access error
      \arg        EFUSE_PVERR: program voltage setting error
      \arg        EFUSE_TOERR: EFUSE timeout error
*/
static efuse_state_enum efuse_ready_wait(uint32_t efuse_flag, uint32_t timeout)
{
    efuse_state_enum efuse_state = EFUSE_BUSY;

    /* wait for EFUSE ready */
    do {
        /* get EFUSE flag set or not */
        if(EFUSE_STAT & (uint32_t)(efuse_flag & EFUSE_FLAG_MASK)) {
            efuse_state = EFUSE_READY;
        } else if(EFUSE_STAT & EFUSE_STAT_IAERRIF) {
            efuse_state = EFUSE_IAERR;
        } else if(EFUSE_STAT & EFUSE_STAT_PVIF) {
            efuse_state = EFUSE_PVERR;
        } else {
            /* illegal parameters */
        }
        timeout--;
    } while((EFUSE_BUSY == efuse_state) && (0U != timeout));

    if(EFUSE_BUSY == efuse_state) {
        efuse_state = EFUSE_TOERR;
    }

    /* return the EFUSE state */
    return efuse_state;
}

