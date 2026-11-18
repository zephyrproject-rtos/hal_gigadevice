/*!
    \file    gd32h73x_75x_rtdec.c
    \brief   RTDEC driver

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

#include "gd32h73x_75x_rtdec.h"

/* RTDEC firmware version offset macro */
#define ARE_FMVER_OFFSET    ((uint32_t)0x00000010U)

#define RTDEC_LOCK_MASK     ((uint32_t)0x00000006U)    /* RTDEC lock value */
#define RTDEC_FLAG_MASK     ((uint32_t)0x00000007U)    /* RTDEC mask value */
#define RTDEC_INT_MASK      ((uint32_t)0x00000007U)    /* RTDEC interrupt value */

/*!
    \brief      reset RTDEC (API ID: 0x0001U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[out] none
    \retval     none
*/
void rtdec_deinit(uint32_t rtdec_periph)
{
    if(RTDEC0 == rtdec_periph){
        /* reset RTDEC0 */
        rcu_periph_reset_enable(RCU_RTDEC0RST);
        rcu_periph_reset_disable(RCU_RTDEC0RST);
    }
    if(RTDEC1 == rtdec_periph) {
        /* reset RTDEC1 */
        rcu_periph_reset_enable(RCU_RTDEC1RST);
        rcu_periph_reset_disable(RCU_RTDEC1RST);
    }
}

/*!
    \brief      initialize the parameters of RTDEC struct with default values (API ID: 0x0002U)
    \param[in]  none
    \param[out] rtdec_struct: the initialized struct rtdec_parameter_struct pointer
    \retval     none
*/
void rtdec_struct_para_init(rtdec_parameter_struct* rtdec_struct)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_POINTER(rtdec_struct)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0002U), ERR_PARAM_POINTER);
    } else
#endif
    {
        /* configure the structure with default value */
        rtdec_struct->access_mode       = (uint8_t)RTDEC_MODE_DATA_ACCESS;
        rtdec_struct->key_crc           = 0x00U;
        rtdec_struct->fw_version        = 0x0000U;
        rtdec_struct->key               = 0x00000000U;
        rtdec_struct->nonce             = 0x00000000U;
        rtdec_struct->start_addr        = 0x00000000U;
        rtdec_struct->end_addr          = 0x00000000U;
    }
}

/*!
    \brief      initialize RTDEC (API ID: 0x0003U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[in]  rtdec_struct: RTDEC parameter initialization struct members of the structure
                              and the member values are shown as below:
                  access_mode: RTDEC_MODE_CODE_ACCESS, RTDEC_MODE_DATA_ACCESS, RTDEC_MODE_BOTH_ACCESS
                  key_crc: CRC value of area key
                  fw_version: area firmware version
                  key: area key
                  nonce: area nonce
                  start_addr: area start address
                  end_addr: area end address
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus rtdec_init(uint32_t rtdec_periph, uint32_t rtdec_area, rtdec_parameter_struct *rtdec_struct)
{
    ErrStatus reval = SUCCESS;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0003U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_VALID_POINTER(rtdec_struct)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(rtdec_struct->key)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else if(NOT_VALID_POINTER(rtdec_struct->nonce)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0003U), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint8_t key_crc_reg = 0U;
        uint32_t key_nonce_addr = 0U;

        /* write the correct MODE[1:0] value and firmware version in ARExCFG register */
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) &= ~(uint32_t)(RTDEC_MODE | RTDEC_ARE_FMVER);
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) |= ((uint32_t)(rtdec_struct->access_mode & RTDEC_MODE)) | ((uint32_t)(((
                    uint32_t)rtdec_struct->fw_version) << ARE_FMVER_OFFSET));

        /* program ARExKEY registers */
        key_nonce_addr = (uint32_t)rtdec_struct->key;
        RTDEC_ARE_KEY0(rtdec_periph, rtdec_area) = *(uint32_t *)key_nonce_addr;
        key_nonce_addr += 4U;
        RTDEC_ARE_KEY1(rtdec_periph, rtdec_area) = *(uint32_t *)key_nonce_addr;
        key_nonce_addr += 4U;
        RTDEC_ARE_KEY2(rtdec_periph, rtdec_area) = *(uint32_t *)key_nonce_addr;
        key_nonce_addr += 4U;
        RTDEC_ARE_KEY3(rtdec_periph, rtdec_area) = *(uint32_t *)key_nonce_addr;

        /* check the key CRC */
        key_crc_reg = (uint8_t)GET_BITS(RTDEC_ARE_CFG(rtdec_periph, rtdec_area), 8U, 15U);

        if(key_crc_reg != rtdec_struct->key_crc) {
            reval = ERROR;
        } else {
            /* program ARExNONCE registers */
            key_nonce_addr = (uint32_t)rtdec_struct->nonce;
            RTDEC_ARE_NONCE0(rtdec_periph, rtdec_area) = *(uint32_t *)key_nonce_addr;
            key_nonce_addr += 4U;
            RTDEC_ARE_NONCE1(rtdec_periph, rtdec_area) = *(uint32_t *)key_nonce_addr;

            /* write the start address and end address of area */
            RTDEC_ARE_SADDR(rtdec_periph, rtdec_area) = rtdec_struct->start_addr;
            RTDEC_ARE_EADDR(rtdec_periph, rtdec_area) = rtdec_struct->end_addr;
        }
    }
    return reval;
}

/*!
    \brief      configure RTDEC area data attribute (API ID: 0x0004U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[in]  access_mode: allowed access mode of data
                only one parameter can be selected which is shown as below:
      \arg        RTDEC_MODE_CODE_ACCESS: code/instruction access only
      \arg        RTDEC_MODE_DATA_ACCESS: data access only
      \arg        RTDEC_MODE_BOTH_ACCESS: code and data access
    \param[in]  firmware_version: 16-bit number, version of data
    \param[out] none
    \retval     none
*/
void rtdec_config(uint32_t rtdec_periph, uint32_t rtdec_area, uint8_t access_mode, uint16_t firmware_version)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0004U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        /* write the correct MODE[1:0] value and firmware version in ARExCFG register */
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) &= ~(uint32_t)(RTDEC_MODE | RTDEC_ARE_FMVER);
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) |= ((uint32_t)(access_mode & RTDEC_MODE)) | ((uint32_t)(((uint32_t)firmware_version) << ARE_FMVER_OFFSET));
    }
}

/*!
    \brief      configure RTDEC key or register lock (API ID: 0x0005U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[in]  lock_type: key lock or register lock
      \arg:       RTDEC_ARE_CFG_LK: register lock
      \arg:       RTDEC_ARE_K_LK: key lock
    \param[out] none
    \retval     none
*/
void rtdec_lock(uint32_t rtdec_periph, uint32_t rtdec_area, uint32_t lock_type)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0005U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) |= (lock_type & RTDEC_LOCK_MASK);
    }
}

/*!
    \brief      initialize RTDEC area address (API ID: 0x0006U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[in]  saddr: area start address, the 4 MSB bits and the 12 LSB bits are ignored
    \param[in]  eaddr: area end address, the 4 MSB bits and the 12 LSB bits are ignored
    \param[out] none
    \retval     none
*/
void rtdec_addr_init(uint32_t rtdec_periph, uint32_t rtdec_area, uint32_t saddr, uint32_t eaddr)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0006U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        /* write the start address and end address of area */
        RTDEC_ARE_SADDR(rtdec_periph, rtdec_area) = saddr;
        RTDEC_ARE_EADDR(rtdec_periph, rtdec_area) = eaddr;
    }
}

/*!
    \brief      nonce initialization (API ID: 0x0007U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[in]  nonce: an array containing 64-bit nonce data, little endian format
    \param[out] none
    \retval     none
*/
void rtdec_nonce_init(uint32_t rtdec_periph, uint32_t rtdec_area, uint32_t* nonce)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0007U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_VALID_POINTER(nonce)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0007U), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint32_t nonce_addr = (uint32_t)nonce;

        /* program ARExNONCE registers */
        RTDEC_ARE_NONCE0(rtdec_periph, rtdec_area) = *(uint32_t *)(nonce_addr);
        nonce_addr += 4U;
        RTDEC_ARE_NONCE1(rtdec_periph, rtdec_area) = *(uint32_t *)(nonce_addr);
    }
}

/*!
    \brief      initialize RTDEC key, key follows little endian format (API ID: 0x0008U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[in]  key: an array containing 128-bit key data, little endian format
    \param[out] none
    \retval     none
*/
void rtdec_key_init(uint32_t rtdec_periph, uint32_t rtdec_area, uint32_t* key)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0008U), ERR_PARAM_OUT_OF_RANGE);
    } else if(NOT_VALID_POINTER(key)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0008U), ERR_PARAM_POINTER);
    } else
#endif
    {
        uint32_t key_addr = (uint32_t)key;

        /* program ARExKEY registers */
        RTDEC_ARE_KEY0(rtdec_periph, rtdec_area) = *(uint32_t *)(key_addr);
        key_addr += 4U;
        RTDEC_ARE_KEY1(rtdec_periph, rtdec_area) = *(uint32_t *)(key_addr);
        key_addr += 4U;
        RTDEC_ARE_KEY2(rtdec_periph, rtdec_area) = *(uint32_t *)(key_addr);
        key_addr += 4U;
        RTDEC_ARE_KEY3(rtdec_periph, rtdec_area) = *(uint32_t *)(key_addr);
    }
}

/*!
    \brief      get CRC value of RTDEC key data (API ID: 0x0009U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[out] none
    \retval     CRC value
*/
uint8_t rtdec_key_crc_get(uint32_t rtdec_periph, uint32_t rtdec_area)
{
    uint8_t key_crc = 0U;
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x0009U), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        key_crc = (uint8_t)GET_BITS(RTDEC_ARE_CFG(rtdec_periph, rtdec_area), 8U, 15U);
    }
    return key_crc;
}

/*!
    \brief      enable RTDEC area (API ID: 0x000AU)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[out] none
    \retval     none
*/
void rtdec_enable(uint32_t rtdec_periph, uint32_t rtdec_area)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x000AU), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) |= RTDEC_ARE_EN;
    }
}

/*!
    \brief      disable RTDEC area (API ID: 0x000BU)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  rtdec_area: RTDEC_AREAx(x = 0, 1, 2, 3)
    \param[out] none
    \retval     none
*/
void rtdec_disable(uint32_t rtdec_periph, uint32_t rtdec_area)
{
#ifdef FW_DEBUG_ERR_REPORT
    if(NOT_VALID_RTDEC_AREA(rtdec_area)) {
        fw_debug_report_err(RTDEC_MODULE_ID, API_ID(0x000BU), ERR_PARAM_OUT_OF_RANGE);
    } else
#endif
    {
        RTDEC_ARE_CFG(rtdec_periph, rtdec_area) &= ~RTDEC_ARE_EN;
    }
}

/*!
    \brief      get RTDEC error flag (API ID: 0x000CU)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  flag: error flag
                only one parameter can be selected which is shown as below:
      \arg        RTDEC_FLAG_SEC_ERROR:  security error flag
      \arg        RTDEC_FLAG_MODE_ERROR: access mode error flag
      \arg        RTDEC_FLAG_KEY_ERROR:  key error flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rtdec_flag_get(uint32_t rtdec_periph, uint32_t flag)
{
    FlagStatus reval = RESET;
    if(RESET != (RTDEC_INTF(rtdec_periph) & flag)) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear RTDEC error flag (API ID: 0x000DU)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  flag: error flag
                only one parameter can be selected which is shown as below:
      \arg        RTDEC_FLAG_SEC_ERROR:  security error flag
      \arg        RTDEC_FLAG_MODE_ERROR: access mode error flag
      \arg        RTDEC_FLAG_KEY_ERROR:  key error flag
    \param[out] none
    \retval     none
*/
void rtdec_flag_clear(uint32_t rtdec_periph, uint32_t flag)
{
    RTDEC_INTC(rtdec_periph) |= (flag & RTDEC_FLAG_MASK);
}

/*!
    \brief      enable RTDEC interrupt (API ID: 0x000EU)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  interrupt: interrupt type
                one or more parameters can be selected which is shown as below:
      \arg        RTDEC_INT_SEC:  security error interrupt
      \arg        RTDEC_INT_MODE: access mode error interrupt
      \arg        RTDEC_INT_KEY:  key error interrupt
    \param[out] none
    \retval     none
*/
void rtdec_interrupt_enable(uint32_t rtdec_periph, uint32_t interrupt)
{
    RTDEC_INTEN(rtdec_periph) |= (interrupt & RTDEC_INT_MASK);
}

/*!
    \brief      disable RTDEC interrupt (API ID: 0x000FU)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  interrupt: interrupt type
                one or more parameters can be selected which is shown as below:
      \arg        RTDEC_INT_SEC:  security error interrupt
      \arg        RTDEC_INT_MODE: access mode error interrupt
      \arg        RTDEC_INT_KEY:  key error interrupt
    \param[out] none
    \retval     none
*/
void rtdec_interrupt_disable(uint32_t rtdec_periph, uint32_t interrupt)
{
    RTDEC_INTEN(rtdec_periph) &= ~(interrupt & RTDEC_INT_MASK);
}

/*!
    \brief      get RTDEC interrupt flag (API ID: 0x0010U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  int_flag: interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        RTDEC_INT_FLAG_SEC_ERROR:  security error interrupt flag
      \arg        RTDEC_INT_FLAG_MODE_ERROR: access mode error interrupt flag
      \arg        RTDEC_INT_FLAG_KEY_ERROR:  key error interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rtdec_interrupt_flag_get(uint32_t rtdec_periph, uint32_t int_flag)
{
    uint32_t interrupt_enable = 0U,  interrupt_flag = 0U;
    FlagStatus reval = RESET;

    switch(int_flag) {
    /* RTDEC security error interrupt */
    case RTDEC_INT_FLAG_SEC_ERROR:
        interrupt_flag = RTDEC_INTF(rtdec_periph) & int_flag;
        interrupt_enable = RTDEC_INTEN(rtdec_periph) & RTDEC_INT_SEC;
        break;
    /* RTDEC execute-only or execute-never error interrupt */
    case RTDEC_INT_FLAG_MODE_ERROR:
        interrupt_flag = RTDEC_INTF(rtdec_periph) & int_flag;
        interrupt_enable = RTDEC_INTEN(rtdec_periph) & RTDEC_INT_MODE;
        break;
    /* RTDEC key error interrupt */
    case RTDEC_INT_FLAG_KEY_ERROR:
        interrupt_flag = RTDEC_INTF(rtdec_periph) & int_flag;
        interrupt_enable = RTDEC_INTEN(rtdec_periph) & RTDEC_INT_KEY;
        break;
    default:
        break;
    }
    /* get RTDEC interrupt flag status */
    if((0U != interrupt_flag) && (0U != interrupt_enable)) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      clear RTDEC interrupt flag (API ID: 0x0011U)
    \param[in]  rtdec_periph: RTDECx(x = 0, 1)
    \param[in]  int_flag: interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        RTDEC_INT_FLAG_SEC_ERROR:  security error interrupt flag
      \arg        RTDEC_INT_FLAG_MODE_ERROR: access mode error interrupt flag
      \arg        RTDEC_INT_FLAG_KEY_ERROR:  key error interrupt flag
    \param[out] none
    \retval     none
*/
void rtdec_interrupt_flag_clear(uint32_t rtdec_periph, uint32_t int_flag)
{
    RTDEC_INTC(rtdec_periph) |= (int_flag & RTDEC_FLAG_MASK);
}
