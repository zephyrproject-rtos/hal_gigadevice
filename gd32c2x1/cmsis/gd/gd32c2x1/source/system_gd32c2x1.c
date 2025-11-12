/*!
    \file  system_gd32c2x1.c
    \brief CMSIS Cortex-M23 Device Peripheral Access Layer Source File for
           gd32c2x1 Device Series
*/

/* Copyright (c) 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/* This file refers the CMSIS standard, some adjustments are made according to GigaDevice chips */

#include "gd32c2x1.h"

/* system frequency define */
#define __IRC48M            (IRC48M_VALUE)            /* internal 48 MHz RC oscillator frequency */
#define __HXTAL             (HXTAL_VALUE)             /* high speed crystal oscillator frequency */
#define __LXTAL             (LXTAL_VALUE)             /* low speed crystal oscillator frequency */
#define __IRC32K            (IRC32K_VALUE)            /* internal 32 KHz RC oscillator frequency */
#define __SYS_OSC_CLK       (__IRC48M)                /* main oscillator frequency */

#define VECT_TAB_OFFSET  (uint32_t)0x00000000U        /* vector table base offset */

/* select a system clock by uncommenting the following line */
#define __SYSTEM_CLOCK_IRC48M                (__IRC48M)
//#define __SYSTEM_CLOCK_HXTAL                 (__HXTAL)

//#define __SYSTEM_CLOCK_LXTAL                 (__LXTAL)
//#define __SYSTEM_CLOCK_IRC32K                (__IRC32K)

#define SEL_IRC48MDIV   0x00
#define SEL_HXTAL       0x01
#define SEL_IRC32K      0x02
#define SEL_LXTAL       0x03
#define SEL_HXTALBPS    0x04

/* set the system clock frequency and declare the system clock configuration function */
#ifdef __SYSTEM_CLOCK_HXTAL
uint32_t SystemCoreClock = __SYSTEM_CLOCK_HXTAL;
static void system_clock_hxtal(void);

#elif defined (__SYSTEM_CLOCK_IRC48M)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_IRC48M;
static void system_clock_irc48m(void);

#elif defined (__SYSTEM_CLOCK_LXTAL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_LXTAL;
static void system_clock_lxtal(void);

#elif defined (__SYSTEM_CLOCK_IRC32K)
uint32_t SystemCoreClock = __SYSTEM_CLOCK_IRC32K;
static void system_clock_IRC32K(void);
#endif /* __SYSTEM_CLOCK_HXTAL */

/* configure the system clock */
static void system_clock_config(void);

/*!
    \brief      setup the microcontroller system, initialize the system
    \param[in]  none
    \param[out] none
    \retval     none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
*/
void SystemInit(void)
{
    /* enable IRC48M */
    RCU_CTL0 |= RCU_CTL0_IRC48MEN;
    while(0U == (RCU_CTL0 & RCU_CTL0_IRC48MSTB)) {
    }
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    /* reset CTL register */
    RCU_CTL0 &= ~(RCU_CTL0_HXTALEN | RCU_CTL0_CKMEN  | RCU_CTL0_HXTALBPS );
    /* reset RCU */
    RCU_CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC  | RCU_CFG0_APBPSC | \
                   RCU_CFG0_CKOUT0SEL | RCU_CFG0_CKOUT0DIV );


    RCU_CFG1 &= ~(RCU_CFG1_ADCPSC | RCU_CFG1_USART0SEL | RCU_CFG1_ADCSEL);

    RCU_INT = 0x00000000U;

    /* configure system clock */
    system_clock_config();
}

/*!
    \brief      configure the system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_config(void)
{
#ifdef __SYSTEM_CLOCK_HXTAL
    system_clock_hxtal();
#elif defined (__SYSTEM_CLOCK_IRC48M)
    system_clock_irc48m();
#elif defined (__SYSTEM_CLOCK_LXTAL)
    system_clock_lxtal();
#elif defined (__SYSTEM_CLOCK_IRC32K)
    system_clock_IRC32K();
#endif /* __SYSTEM_CLOCK_8M_HXTAL */
}

#ifdef __SYSTEM_CLOCK_HXTAL
/*!
    \brief      configure the system clock to 8M by HXTAL
    \param[in]  none
    \param[out] none
    \retval     none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
*/
static void system_clock_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    if((HXTAL_VALUE > 24000000U) && (HXTAL_VALUE <= 48000000U)) {
        FMC_WS =(FMC_WS & (~FMC_WS_WSCNT)) | FMC_WAIT_STATE_1;
    }

    /* enable HXTAL */
    RCU_CTL0 |= RCU_CTL0_HXTALEN;

    /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL0 & RCU_CTL0_HXTALSTB);
    } while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));
    /* if fail */
    if(0U == (RCU_CTL0 & RCU_CTL0_HXTALSTB)) {
        while(1) {
        }
    }

    /* HXTAL is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB = AHB */
    RCU_CFG0 |= RCU_APB_CKAHB_DIV1;

    /* select HXTAL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_HXTAL;

    /* wait until HXTAL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_HXTAL) {
    }
}

#elif defined (__SYSTEM_CLOCK_IRC48M)
/*!
    \brief      configure the system clock to IRC48M
    \param[in]  none
    \param[out] none
    \retval     none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
*/
static void system_clock_irc48m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    FMC_WS =(FMC_WS & (~FMC_WS_WSCNT)) | FMC_WAIT_STATE_1;

    /* enable IRC48M */
    RCU_CTL0 |= RCU_CTL0_IRC48MEN;
    /* IRC48M divide by 1 */
    rcu_irc48mdiv_sys_clock_config(RCU_IRC48MDIV_SYS_1);

    /* wait until IRC48M is stable or the startup time is longer than IRC48M_STARTUP_TIMEOUT */
    do {
        timeout++;
        stab_flag = (RCU_CTL0 & RCU_CTL0_IRC48MSTB);
    } while((0U == stab_flag) && (IRC48M_STARTUP_TIMEOUT != timeout));
    /* if fail */
    if(0U == (RCU_CTL0 & RCU_CTL0_IRC48MSTB)) {
        while(1) {
        }
    }

    /* IRC48M is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB = AHB */
    RCU_CFG0 |= RCU_APB_CKAHB_DIV1;

    /* select IRC48M as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_IRC48MDIV_SYS;

    /* wait until IRC48M is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_IRC48MDIV) {
    }
}

#elif defined (__SYSTEM_CLOCK_LXTAL)
/*!
    \brief      configure the system clock to LXTAL
    \param[in]  none
    \param[out] none
    \retval     none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
*/
static void system_clock_lxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;
    
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();

    /* enable LXTAL */
    RCU_CTL1 |= RCU_CTL1_LXTALEN;

    /* if fail */
    while(0U == (RCU_CTL1 & RCU_CTL1_LXTALSTB)) {
    }

    /* LXTAL is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB = AHB */
    RCU_CFG0 |= RCU_APB_CKAHB_DIV1;

    /* select LXTAL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_LXTAL;

    /* wait until LXTAL is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_LXTAL) {
    }
}

#else
/*!
    \brief      configure the system clock to IRC32K
    \param[in]  none
    \param[out] none
    \retval     none
    \note       This function may contain scenarios leading to an infinite loop.
                Modify it according to the actual usage requirements.
*/
static void system_clock_IRC32K(void)
{
    
    /* enable IRC32K */
    RCU_RSTSCK |= RCU_RSTSCK_IRC32KEN;

    /* if fail */
    while(0U == (RCU_RSTSCK & RCU_RSTSCK_IRC32KSTB)) {
    }
    
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB = AHB */
    RCU_CFG0 |= RCU_APB_CKAHB_DIV1;


    /* select IRC32K as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_IRC32K;

    /* wait until IRC48M is selected as system clock */
    while((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_IRC32K) {
    }
}

#endif /* __SYSTEM_CLOCK_8M_HXTAL */

/*!
    \brief      update the SystemCoreClock with current core clock retrieved from cpu registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemCoreClockUpdate(void)
{
    uint32_t sws = 0U;
    uint32_t idx = 0U, clk_exp = 0U;
    uint32_t irc48mdiv_sys = 0U;
    /* exponent of AHB clock divider */
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch(sws) {
    /* IRC48M is selected as CK_SYS */
    case SEL_IRC48MDIV:
        irc48mdiv_sys = GET_BITS(RCU_CTL0, 29, 31);
        SystemCoreClock = IRC48M_VALUE / (1 << irc48mdiv_sys);
        break;
    /* HXTAL is selected as CK_SYS */
    case SEL_HXTAL:
        SystemCoreClock = HXTAL_VALUE;
        break;
    /* IRC32K is selected as CK_SYS */
    case SEL_IRC32K:
        SystemCoreClock = IRC32K_VALUE;
        break;
    /* IRC32K is selected as CK_SYS */
    case SEL_LXTAL:
        SystemCoreClock = LXTAL_VALUE;
        break;
    /* IRC48M is selected as CK_SYS */
    default:
        SystemCoreClock = IRC48M_VALUE/4;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    SystemCoreClock >>= clk_exp;
}

#ifdef __FIRMWARE_VERSION_DEFINE
/*!
    \brief      get firmware version
    \param[in]  none
    \param[out] none
    \retval     firmware version
*/
uint32_t gd32c2x1_firmware_version_get(void)
{
    return __GD32C2X1_STDPERIPH_VERSION;
}
#endif /* __FIRMWARE_VERSION_DEFINE */
