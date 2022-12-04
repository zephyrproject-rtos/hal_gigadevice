/*!
    \file    gd32a50x_mfcom.c
    \brief   MFCOM driver

    \version 2022-01-30, V1.0.0, firmware for GD32A50X
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

#include "gd32a50x_mfcom.h"

/*!
    \brief      reset MFCOM
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mfcom_deinit(void)
{
    /* reset MFCOM */
    rcu_periph_reset_enable(RCU_MFCOMRST);
    rcu_periph_reset_disable(RCU_MFCOMRST);
}

/*!
    \brief      software reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mfcom_software_reset(void)
{
    MFCOM_CTL |= MFCOM_CTL_SWRSTEN;
    MFCOM_CTL &= ~MFCOM_CTL_SWRSTEN;
}


/*!
    \brief      enable MFCOM function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mfcom_enable(void)
{
    MFCOM_CTL |= MFCOM_CTL_MFCOMEN;
}

/*!
    \brief      disable MFCOM function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void mfcom_disable(void)
{
    MFCOM_CTL &= ~MFCOM_CTL_MFCOMEN;
}

/*!
    \brief      initialize mfcom_timer_parameter_struct with the default values
    \param[in]  init_struct: the initialization timer parameter need
    \param[out] none
    \retval     none
*/
void mfcom_timer_struct_para_init(mfcom_timer_parameter_struct* init_struct)
{
    /* set the mfcom_timer_parameter_struct with the default values */
    init_struct->trigger_select = MFCOM_TIMER_TRGSEL_PIN0;
    init_struct->trigger_polarity = MFCOM_TIMER_TRGPOL_ACTIVE_HIGH;
    init_struct->pin_config = MFCOM_TIMER_PINCFG_INPUT;
    init_struct->pin_select = MFCOM_TIMER_PINSEL_PIN0;
    init_struct->pin_polarity = MFCOM_TIMER_PINPOL_ACTIVE_HIGH;
    init_struct->mode = MFCOM_TIMER_DISABLE;
    init_struct->output = MFCOM_TIMER_OUT_HIGH_EN;
    init_struct->decrement = MFCOM_TIMER_DEC_CLK_SHIFT_OUT;
    init_struct->reset = MFCOM_TIMER_RESET_NEVER;
    init_struct->disable = MFCOM_TIMER_DISMODE_NEVER;
    init_struct->enable = MFCOM_TIMER_ENMODE_ALWAYS;
    init_struct->stopbit = MFCOM_TIMER_STOPBIT_DISABLE;
    init_struct->startbit = MFCOM_TIMER_STARTBIT_DISABLE;
    init_struct->compare = 0U;
}

/*!
    \brief      initialize mfcom_shifter_parameter_struct with the default values
    \param[in]  init_struct: the initialization shifter parameter need
    \param[out] none
    \retval     none
*/
void mfcom_shifter_struct_para_init(mfcom_shifter_parameter_struct* init_struct)
{
    /* set the mfcom_shifter_parameter_struct with the default values */
    init_struct->timer_select = MFCOM_SHIFTER_TIMER0;
    init_struct->timer_polarity = MFCOM_SHIFTER_TIMPOL_ACTIVE_HIGH;
    init_struct->pin_config = MFCOM_SHIFTER_PINCFG_INPUT;
    init_struct->pin_select = MFCOM_SHIFTER_PINSEL_PIN0;
    init_struct->pin_polarity = MFCOM_SHIFTER_PINPOL_ACTIVE_HIGH;
    init_struct->mode = MFCOM_SHIFTER_DISABLE;
    init_struct->input_source = MFCOM_SHIFTER_INSRC_PIN;
    init_struct->stopbit = MFCOM_SHIFTER_STOPBIT_DISABLE;
    init_struct->startbit = MFCOM_SHIFTER_STARTBIT_DISABLE;
}

/*!
    \brief      initialize MFCOM timer parameter
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
	\param[in]  init_struct: the initialization timer parameter need
      \arg         trigger_select: MFCOM_TIMER_TRGSEL_PIN0       MFCOM_TIMER_TRGSEL_SHIFTER0 
                                MFCOM_TIMER_TRGSEL_PIN1          MFCOM_TIMER_TRGSEL_TIMER0   
                                MFCOM_TIMER_TRGSEL_PIN2          MFCOM_TIMER_TRGSEL_SHIFTER1 
                                MFCOM_TIMER_TRGSEL_PIN3          MFCOM_TIMER_TRGSEL_TIMER1   
                                MFCOM_TIMER_TRGSEL_PIN4          MFCOM_TIMER_TRGSEL_SHIFTER2 
                                MFCOM_TIMER_TRGSEL_PIN5          MFCOM_TIMER_TRGSEL_TIMER2   
                                MFCOM_TIMER_TRGSEL_PIN6          MFCOM_TIMER_TRGSEL_SHIFTER3 
                                MFCOM_TIMER_TRGSEL_PIN7          MFCOM_TIMER_TRGSEL_TIMER3   
                                MFCOM_TIMER_TRGSEL_EXTERNAL0     MFCOM_TIMER_TRGSEL_EXTERNAL1
                                MFCOM_TIMER_TRGSEL_EXTERNAL2     MFCOM_TIMER_TRGSEL_EXTERNAL3

                trigger_polarity: MFCOM_TIMER_TRGPOL_ACTIVE_HIGH
                                  MFCOM_TIMER_TRGPOL_ACTIVE_LOW 
                 
                pin_config: MFCOM_TIMER_PINCFG_INPUT    MFCOM_TIMER_PINCFG_OPENDRAIN
                            MFCOM_TIMER_PINCFG_BIDI     MFCOM_TIMER_PINCFG_OUTPUT

                pin_select: MFCOM_TIMER_PINSEL_PIN0      MFCOM_TIMER_PINSEL_PIN1
                            MFCOM_TIMER_PINSEL_PIN2      MFCOM_TIMER_PINSEL_PIN3
                            MFCOM_TIMER_PINSEL_PIN4      MFCOM_TIMER_PINSEL_PIN5
                            MFCOM_TIMER_PINSEL_PIN6      MFCOM_TIMER_PINSEL_PIN7

                pin_polarity: MFCOM_TIMER_PINPOL_ACTIVE_HIGH
                              MFCOM_TIMER_PINPOL_ACTIVE_LOW     
                   
                mode: MFCOM_TIMER_DISABLE      MFCOM_TIMER_BAUDMODE    
                      MFCOM_TIMER_PWMMODE      MFCOM_TIMER_16BITCOUNTER

                output: MFCOM_TIMER_OUT_HIGH_EN          MFCOM_TIMER_OUT_LOW_EN       
                        MFCOM_TIMER_OUT_HIGH_EN_RESET    MFCOM_TIMER_OUT_LOW_EN_RESET 
                                                            
                decrement: MFCOM_TIMER_DEC_CLK_SHIFT_OUT    MFCOM_TIMER_DEC_TRIG_SHIFT_OUT 
                           MFCOM_TIMER_DEC_PIN_SHIFT_PIN    MFCOM_TIMER_DEC_TRIG_SHIFT_TRIG

                reset: MFCOM_TIMER_RESET_NEVER              MFCOM_TIMER_RESET_PIN_TIMOUT 
                       MFCOM_TIMER_RESET_TRIG_TIMOUT        MFCOM_TIMER_RESET_PIN_RISING 
                       MFCOM_TIMER_RESET_TRIG_RISING        MFCOM_TIMER_RESET_TRIG_BOTH  
                                                                                                                                 
                disable: MFCOM_TIMER_DISMODE_NEVER           MFCOM_TIMER_DISMODE_PRE_TIMDIS      
                         MFCOM_TIMER_DISMODE_COMPARE         MFCOM_TIMER_DISMODE_COMPARE_TRIGLOW 
                         MFCOM_TIMER_DISMODE_PINBOTH         MFCOM_TIMER_DISMODE_PINBOTH_TRIGHIGH
                         MFCOM_TIMER_DISMODE_TRIGFALLING                                    
                                                           
                enable: MFCOM_TIMER_ENMODE_ALWAYS            MFCOM_TIMER_ENMODE_PRE_TIMEN         
                        MFCOM_TIMER_ENMODE_TRIGHIGH          MFCOM_TIMER_ENMODE_TRIGHIGH_PINHIGH  
                        MFCOM_TIMER_ENMODE_PINRISING         MFCOM_TIMER_ENMODE_PINRISING_TRIGHIGH
                        MFCOM_TIMER_ENMODE_TRIGRISING        MFCOM_TIMER_ENMODE_TRIGBOTH  
                                              
                stopbit: MFCOM_TIMER_STOPBIT_DISABLE        MFCOM_TIMER_STOPBIT_TIMCMP       
                         MFCOM_TIMER_STOPBIT_TIMDIS         MFCOM_TIMER_STOPBIT_TIMCMP_TIMDIS
                                                    
                startbit: MFCOM_TIMER_STARTBIT_DISABLE
                          MFCOM_TIMER_STARTBIT_ENABLE 
                                                     
                compare: compare value 
    \param[out] none
    \retval     none
*/
void mfcom_timer_init(uint32_t timer, mfcom_timer_parameter_struct* init_struct)
{  
    /* configure compare value */
    MFCOM_TMCMP(timer) = (uint32_t)init_struct->compare;
    /* configure timer output/decrement/reset/disable/enable/stopbit/startbit */
    MFCOM_TMCFG(timer) = init_struct->output | init_struct->decrement | init_struct->reset | init_struct->disable | init_struct->enable | init_struct->stopbit | init_struct->startbit;

    /* configure pin trigger_select/trigger_polarity/pin_config/pin_select/pin_polarity/mode */
    MFCOM_TMCTL(timer) = init_struct->trigger_select | init_struct->trigger_polarity | init_struct->pin_config | init_struct->pin_select | init_struct->pin_polarity | init_struct->mode;
}

/*!
    \brief      initialize MFCOM shifter parameter
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[in]  init_struct: the initialization shifter parameter need
      \arg         timer_select: MFCOM_SHIFTER_TIMER0   MFCOM_SHIFTER_TIMER1
                              MFCOM_SHIFTER_TIMER2      MFCOM_SHIFTER_TIMER3

                timer_polarity: MFCOM_SHIFTER_TIMPOL_ACTIVE_HIGH
                                MFCOM_SHIFTER_TIMPOL_ACTIVE_LOW 

                pin_config: MFCOM_SHIFTER_PINCFG_INPUT    MFCOM_SHIFTER_PINCFG_OPENDRAIN
                            MFCOM_SHIFTER_PINCFG_BIDI     MFCOM_SHIFTER_PINCFG_OUTPUT  
    
                pin_select: MFCOM_SHIFTER_PINSEL_PIN0       MFCOM_SHIFTER_PINSEL_PIN1
                            MFCOM_SHIFTER_PINSEL_PIN2       MFCOM_SHIFTER_PINSEL_PIN3
                            MFCOM_SHIFTER_PINSEL_PIN4       MFCOM_SHIFTER_PINSEL_PIN5
                            MFCOM_SHIFTER_PINSEL_PIN6       MFCOM_SHIFTER_PINSEL_PIN7

                pin_polarity: MFCOM_SHIFTER_PINPOL_ACTIVE_HIGH
                              MFCOM_SHIFTER_PINPOL_ACTIVE_LOW 

                mode: MFCOM_SHIFTER_DISABLE         MFCOM_SHIFTER_RECEIVE         
                      MFCOM_SHIFTER_TRANSMIT        MFCOM_SHIFTER_MATCH_STORE     
                      MFCOM_SHIFTER_MATCH_CONTINUOUS  

                input_source: MFCOM_SHIFTER_INSRC_PIN        
                              MFCOM_SHIFTER_INSRC_NEXTSHIFTER

                stopbit: MFCOM_SHIFTER_STOPBIT_DISABLE    MFCOM_SHIFTER_STOPBIT_LOW    
                         MFCOM_SHIFTER_STOPBIT_HIGH   

                startbit: MFCOM_SHIFTER_STARTBIT_DISABLE     MFCOM_SHIFTER_STARTBIT_DISABLE_TXEN
                          MFCOM_SHIFTER_STARTBIT_LOW         MFCOM_SHIFTER_STARTBIT_HIGH        
    \param[out] none
    \retval     none
*/
void mfcom_shifter_init(uint32_t shifter, mfcom_shifter_parameter_struct* init_struct)
{
    /* configure shifter input_source/stopbit/startbit */
    MFCOM_SCFG(shifter) = init_struct->input_source | init_struct->stopbit | init_struct->startbit;

    /* configure shifter timer_select/timer_polarity/pin_config/pin_select/pin_polarity/mode */
    MFCOM_SCTL(shifter) = init_struct->timer_select | init_struct->timer_polarity | init_struct->pin_config | init_struct->pin_select | init_struct->pin_polarity | init_struct->mode;
}

/*!
    \brief      configure timer pin mode
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[in]  outputmode:
      \arg         MFCOM_TIMER_PINCFG_INPUT:      pin input
      \arg         MFCOM_TIMER_PINCFG_OPENDRAIN:  pin open drain or bidirectional output enable
      \arg         MFCOM_TIMER_PINCFG_BIDI:       pin bidirectional output data
      \arg         MFCOM_TIMER_PINCFG_OUTPUT:     pin output
    \param[out] none
    \retval     none
*/
void mfcom_timer_pin_config(uint32_t timer, uint32_t mode)
{
    uint32_t temp;
    temp = MFCOM_TMCTL(timer);

    /* clear timer pin configuration bits */
    temp &= ~MFCOM_TMCTL_TMPCFG;

    /* set timer pin outputmode */
    temp |= mode;
    MFCOM_TMCTL(timer) = temp;
}

/*!
    \brief      configure shifter pin mode
    \param[in]  shifter:  MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[in]  outputmode:
      \arg         MFCOM_SHIFTER_PINCFG_INPUT: pin input
      \arg         MFCOM_SHIFTER_PINCFG_OPENDRAIN: pin open drain or bidirectional output enable 
      \arg         MFCOM_SHIFTER_PINCFG_BIDI: pin bidirectional output data 
      \arg         MFCOM_SHIFTER_PINCFG_OUTPUT: pin output 
    \param[out] none
    \retval     none
*/
void mfcom_shifter_pin_config(uint32_t shifter, uint32_t mode)
{
    uint32_t temp;
    temp = MFCOM_SCTL(shifter);

    /* clear shifter pin configuration bits */
    temp &= ~MFCOM_SCTL_SPCFG;

    /* set shifter pin outputmode */
    temp |= mode;
    MFCOM_SCTL(shifter) = temp;
}

/*!
    \brief      enable MFCOM timer in specific mode
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[in]  timermode:
      \arg         MFCOM_TIMER_DISABLE: timer disabled
      \arg         MFCOM_TIMER_BAUDMODE: dual 8-bit counters baud/bit mode
      \arg         MFCOM_TIMER_PWMMODE: dual 8-bit counters PWM mode
      \arg         MFCOM_TIMER_16BITCOUNTER: single 16-bit counter mode
    \param[out] none
    \retval     none
*/
void mfcom_timer_enable(uint32_t timer, uint32_t timermode)
{
    uint32_t temp;
    temp = MFCOM_TMCTL(timer);

    /* clear timer mode bits */
    temp &= ~MFCOM_TMCTL_TMMOD;

    /* set timer mode */
    temp |= timermode;
    MFCOM_TMCTL(timer) = temp;
}

/*!
    \brief      enable MFCOM shifter in specific mode
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[in]  shiftermode:
      \arg         MFCOM_SHIFTER_DISABLE: shifter is disabled
      \arg         MFCOM_SHIFTER_RECEIVE: receive mode
      \arg         MFCOM_SHIFTER_TRANSMIT: transmit mode
      \arg         MFCOM_SHIFTER_MATCH_STORE: match store mode
      \arg         MFCOM_SHIFTER_MATCH_CONTINUOUS: match continuous mode
    \param[out] none
    \retval     none
*/
void mfcom_shifter_enable(uint32_t shifter, uint32_t shiftermode)
{
    uint32_t temp;
    temp = MFCOM_SCTL(shifter);

    /* clear shifter mode bits */
    temp &= ~MFCOM_SCTL_SMOD;

    /* set shifter mode */
    temp |= shiftermode;
    MFCOM_SCTL(shifter) = temp;
}

/*!
    \brief      disable MFCOM timer
    \param[in]  timer:  MFCOM timer number
      \arg        MFCOM_TIMER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_timer_disable(uint32_t timer)
{
    MFCOM_TMCTL(timer) &= ~MFCOM_TMCTL_TMMOD;
}

/*!
    \brief      disable MFCOM shifter
    \param[in]  shifter:  MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_disable(uint32_t shifter)
{
    MFCOM_SCTL(shifter) &= ~MFCOM_SCTL_SMOD;
}

/*!
    \brief      set MFCOM timer compare value
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[in]  compare: timer compare value
    \param[out] none
    \retval     none
*/
void mfcom_timer_cmpvalue_set(uint32_t timer, uint32_t compare)
{
    MFCOM_TMCMP(timer) = compare;
}

/*!
    \brief      get MFCOM timer compare value
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[out] timer compare value
    \retval     none
*/
uint32_t mfcom_timer_cmpvalue_get(uint32_t timer)
{
    return MFCOM_TMCMP(timer);
}

/*!
    \brief      set MFCOM timer disable mode
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[in]  dismode: configure conditions that can disable timers and stop decrement.
      \arg         MFCOM_TIMER_DISMODE_NEVER:                timer never disabled
      \arg         MFCOM_TIMER_DISMODE_PRE_TIMDIS:           timer disabled on timer x-1 disable
      \arg         MFCOM_TIMER_DISMODE_COMPARE:              timer disabled on timer compare
      \arg         MFCOM_TIMER_DISMODE_COMPARE_TRIGLOW:      timer disabled on timer compare and trigger Low
      \arg         MFCOM_TIMER_DISMODE_PINBOTH:              timer disabled on pin rising or falling edge
      \arg         MFCOM_TIMER_DISMODE_PINBOTH_TRIGHIGH      timer disabled on pin rising or falling edge provided trigger is high
      \arg         MFCOM_TIMER_DISMODE_TRIGFALLING           timer disabled on trigger falling edge
    \param[out] none
    \retval     none
*/
void mfcom_timer_dismode_set(uint32_t timer, uint32_t dismode)
{
    uint32_t temp;
    temp = MFCOM_TMCFG(timer);

    /* clear timer disable bits */
    temp &= ~MFCOM_TMCFG_TMDIS;

    /* set timer disable mode */
    temp |= dismode;
    MFCOM_TMCFG(timer) = temp;
}

/*!
    \brief      set MFCOM shifter stopbit
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[in]  stopbit: shifter stop bit
      \arg         MFCOM_SHIFTER_STOPBIT_DISABLE: disable shifter stop bit
      \arg         MFCOM_SHIFTER_STOPBIT_LOW: set shifter stop bit to logic low level
      \arg         MFCOM_SHIFTER_STOPBIT_HIGH: set shifter stop bit to logic high level
    \param[out] none
    \retval     none
*/
void mfcom_shifter_stopbit_set(uint32_t shifter, uint32_t stopbit)
{
    uint32_t temp;
    temp = MFCOM_SCFG(shifter);

    /* clear shifter stop bit */
    temp &= ~MFCOM_SCFG_SSTOP;

    /* set shifter stop bit */
    temp |= stopbit;
    MFCOM_SCFG(shifter) = temp;
}

/*!
    \brief      write MFCOM shifter buffer 
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[in]  data: 32-bit data
    \param[in]  rwmode: MFCOM read write mode
      \arg         MFCOM_RWMODE_NORMAL: read and write in normal mode
      \arg         MFCOM_RWMODE_BITSWAP: read and write in bit swapped mode
      \arg         MFCOM_RWMODE_BYTESWAP: read and write in byte swapped mode
      \arg         MFCOM_RWMODE_BITBYTESWAP: read and write in bit byte swapped mode
    \param[out] none
    \retval     none
*/
void mfcom_buffer_write(uint32_t shifter, uint32_t data, uint32_t rwmode)
{
    switch (rwmode){
    case MFCOM_RWMODE_NORMAL:
        /* write MFCOM shift buffer in MFCOM_RWMODE_NORMAL mode */
        MFCOM_SBUF(shifter) = data;
        break;
    case MFCOM_RWMODE_BITSWAP:
        /* write MFCOM shift buffer in MFCOM_RWMODE_BITSWAP mode */
        MFCOM_SBUFBIS(shifter) = data;
        break;
    case MFCOM_RWMODE_BYTESWAP:
        /* write MFCOM shift buffer in MFCOM_RWMODE_BYTESWAP mode */
        MFCOM_SBUFBYS(shifter) = data;
        break;
    case MFCOM_RWMODE_BITBYTESWAP:
        /* write MFCOM shift buffer in MFCOM_RWMODE_BITBYTESWAP mode */
        MFCOM_SBUFBBS(shifter) = data;
        break;
    default:
        break;
    }
}

/*!
    \brief      read MFCOM shifter buffer 
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[in]  rwmode: MFCOM read write mode
      \arg         MFCOM_RWMODE_NORMAL: read and write in normal mode
      \arg         MFCOM_RWMODE_BITSWAP: read and write in bit swapped mode
      \arg         MFCOM_RWMODE_BYTESWAP: read and write in byte swapped mode
      \arg         MFCOM_RWMODE_BITBYTESWAP: read and write in bit byte swapped mode
    \param[out] none
    \retval     data: 32-bit data
*/
uint32_t mfcom_buffer_read(uint32_t shifter, uint32_t rwmode)
{
    uint32_t data;
    switch (rwmode){
    case MFCOM_RWMODE_NORMAL:
        /* read MFCOM shift buffer in MFCOM_RWMODE_NORMAL mode */
        data = MFCOM_SBUF(shifter);
        break;
    case MFCOM_RWMODE_BITSWAP:
        /* read MFCOM shift buffer in MFCOM_RWMODE_BITSWAP mode */
        data = MFCOM_SBUFBIS(shifter);
        break;
    case MFCOM_RWMODE_BYTESWAP:
        /* read MFCOM shift buffer in MFCOM_RWMODE_BYTESWAP mode */
        data = MFCOM_SBUFBYS(shifter);
        break;
    case MFCOM_RWMODE_BITBYTESWAP:
        /* read MFCOM shift buffer in MFCOM_RWMODE_BITBYTESWAP mode */
        data = MFCOM_SBUFBBS(shifter);
        break;
    default:
        data = 0U;
        break;
    }
    return data;
}

/*!
    \brief      get MFCOM shifter flag
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mfcom_shifter_flag_get(uint32_t shifter)
{
    if(0U != (MFCOM_SSTAT & ((uint32_t)1U << shifter))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get MFCOM shifter error flag
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mfcom_shifter_error_flag_get(uint32_t shifter)
{
    if(0U != (MFCOM_SERR & ((uint32_t)1U << shifter))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get MFCOM timer flag
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mfcom_timer_flag_get(uint32_t timer)
{
    if(0U != (MFCOM_TMSTAT & ((uint32_t)1U << timer))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get MFCOM shifter interrupt flag
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mfcom_shifter_interrupt_flag_get(uint32_t shifter)
{
    uint32_t interrupt_flag, interrupt_enable;
    
    interrupt_flag = MFCOM_SSTAT & ((uint32_t)1U << shifter);
    interrupt_enable = MFCOM_SSIEN & ((uint32_t)1U << shifter);
    
    /* judge shifter interrupt flag state */
    if(interrupt_flag & interrupt_enable){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get MFCOM shifter error interrupt flag
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mfcom_shifter_error_interrupt_flag_get(uint32_t shifter)
{
    uint32_t interrupt_flag, interrupt_enable;
    
    interrupt_flag = MFCOM_SERR & ((uint32_t)1U << shifter);
    interrupt_enable = MFCOM_SEIEN & ((uint32_t)1U << shifter);
    
    /* judge shifter error interrupt flag state */
    if(interrupt_flag & interrupt_enable){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get MFCOM timer interrupt flag
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus mfcom_timer_interrupt_flag_get(uint32_t timer)
{
    uint32_t interrupt_flag, interrupt_enable;
    
    interrupt_flag = MFCOM_TMSTAT & ((uint32_t)1U << timer);
    interrupt_enable = MFCOM_TMSIEN & ((uint32_t)1U << timer);
    
    /* judge timer interrupt flag state */
    if(interrupt_flag & interrupt_enable){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear MFCOM shifter flag
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_flag_clear(uint32_t shifter)
{
    MFCOM_SSTAT = ((uint32_t)1U << shifter);
}

/*!
    \brief    clear MFCOM shifter error flag
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_error_flag_clear(uint32_t shifter)
{
    MFCOM_SERR = ((uint32_t)1U << shifter);
}

/*!
    \brief      clear MFCOM timer flag
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_timer_flag_clear(uint32_t timer)
{
    MFCOM_TMSTAT = ((uint32_t)1U << timer);
}

/*!
    \brief      enable MFCOM shifter interrupt
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_interrupt_enable(uint32_t shifter)
{
    MFCOM_SSIEN |= ((uint32_t)1U << shifter);
}

/*!
    \brief      enable MFCOM shifter error interrupt
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_error_interrupt_enable(uint32_t shifter)
{
    MFCOM_SEIEN |= ((uint32_t)1U << shifter);
}

/*!
    \brief      enable MFCOM timer interrupt
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_timer_interrupt_enable(uint32_t timer)
{
    MFCOM_TMSIEN |= ((uint32_t)1U << timer);
}

/*!
    \brief      enable MFCOM shifter dma
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_dma_enable(uint32_t shifter)
{
    MFCOM_SSDMAEN |= ((uint32_t)1U << shifter);
}

/*!
    \brief      disable MFCOM shifter interrupt
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_interrupt_disable(uint32_t shifter)
{
    MFCOM_SSIEN &= ~((uint32_t)1U << shifter);
}

/*!
    \brief      disable MFCOM shifter error interrupt
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_error_interrupt_disable(uint32_t shifter)
{
    MFCOM_SEIEN &= ~((uint32_t)1U << shifter);
}

/*!
    \brief      disable MFCOM timer interrupt
    \param[in]  timer: MFCOM timer number
      \arg         MFCOM_TIMER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_timer_interrupt_disable(uint32_t timer)
{
    MFCOM_TMSIEN &= ~((uint32_t)1U << timer);
}

/*!
    \brief      disable MFCOM shifter dma
    \param[in]  shifter: MFCOM shifter number
      \arg         MFCOM_SHIFTER_x(x=0..3)
    \param[out] none
    \retval     none
*/
void mfcom_shifter_dma_disable(uint32_t shifter)
{
    MFCOM_SSDMAEN &= ~((uint32_t)1U << shifter);
}
