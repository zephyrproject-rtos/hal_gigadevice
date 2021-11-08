/* See LICENSE file for licence details */

#ifndef N200_TIMER_H
#define N200_TIMER_H

#include <gd32vf103.h>
#include <stddef.h>

#define TIMER_MSIP offsetof(SysTimer_Type, MSIP)
#define TIMER_MSIP_size sizeof(((SysTimer_Type*)0)->MSIP)
#define TIMER_MTIMECMP offsetof(SysTimer_Type, MTIMERCMP)
#define TIMER_MTIMECMP_size sizeof(((SysTimer_Type*)0)->MTIMERCMP)
#define TIMER_MTIME offsetof(SysTimer_Type, MTIMER)
#define TIMER_MTIME_size sizeof(((SysTimer_Type*)0)->MTIMER)

#define TIMER_CTRL_ADDR           __SYSTIMER_BASEADDR
#define TIMER_REG(offset)         _REG32(TIMER_CTRL_ADDR, offset)
#define TIMER_FREQ                ((uint32_t)SystemCoreClock/4)

#endif

