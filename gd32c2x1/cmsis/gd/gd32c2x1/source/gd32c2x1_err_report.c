/*!
    \file    gd32xxx_report_err.c
    \brief   Reporting Error driver

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

#include "gd32c2x1_err_report.h"

#define ERROR_HANDLE(s)    do{}while(1)

__WEAK void fw_error_notification(void);

/* initialize the error report buffer and index */
err_report_struct err_report_buffer[ERR_REPORT_BUFFER_SIZE];

uint8_t err_report_buff_index = 0x00U;

/*!
    \brief      reporting error in debug mode
    \param[in]  moduleid: module ID where the error occurred
    \param[in]  apiid: API ID associated with the error
    \param[in]  errid: error ID indicating the specific error type
    \param[out] err_report_buffer: stores the id of the error type
    \retval     none
*/
void fw_debug_report_err(uint16_t moduleid, uint16_t apiid, uint8_t errid)
{
    err_report_struct *debug_report_err_buffer;

    if(err_report_buff_index < ERR_REPORT_BUFFER_SIZE) {
        debug_report_err_buffer = &err_report_buffer[err_report_buff_index++];
        debug_report_err_buffer->moduleid = moduleid;
        debug_report_err_buffer->apiid = apiid;
        debug_report_err_buffer->errid = errid;
    } else {
        /* illegal parameters */
    }
    fw_error_notification();
}

/*!
    \brief      the notification of parameter check
    \param[in]  none
    \param[out] none
    \retval     none
*/
__WEAK void fw_error_notification(void){
    ERROR_HANDLE("Parameter Check Error!");
}
