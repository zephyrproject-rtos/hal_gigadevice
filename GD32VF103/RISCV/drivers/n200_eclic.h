/* See LICENSE file for licence details */

#ifndef N200_ECLIC_H
#define N200_ECLIC_H

#include <gd32vf103.h>
#include <stddef.h>

#define ECLICINTCTLBITS __ECLIC_INTCTLBITS

/*ECLIC memory map */
/* Offset */
/* 0x0000       1B          RW        ecliccfg */
#define ECLIC_CFG_OFFSET  offsetof(CLIC_Type, CFG)
/*  0x0004       4B          R         eclicinfo */
#define ECLIC_INFO_OFFSET offsetof(CLIC_Type, INFO)
/*  0x000B       1B          RW        mintthresh */
#define ECLIC_MTH_OFFSET  offsetof(CLIC_Type, MTH)

/* 0x1000+4*i   1B/input    RW        eclicintip[i] */
#define ECLIC_INT_IP_OFFSET   (offsetof(CLIC_Type, CTRL) + offsetof(CLIC_CTRL_Type, INTIP) )
/* 0x1001+4*i   1B/input    RW        eclicintie[i] */
#define ECLIC_INT_IE_OFFSET   (offsetof(CLIC_Type, CTRL) + offsetof(CLIC_CTRL_Type, INTIE) )
/* 0x1002+4*i   1B/input    RW        eclicintattr[i]*/
#define ECLIC_INT_ATTR_OFFSET (offsetof(CLIC_Type, CTRL) + offsetof(CLIC_CTRL_Type, INTATTR) )

#define ECLIC_INT_ATTR_SHV              0x01
#define ECLIC_INT_ATTR_TRIG_LEVEL       0x00
#define ECLIC_INT_ATTR_TRIG_EDGE        0x02
#define ECLIC_INT_ATTR_TRIG_POS         0x00
#define ECLIC_INT_ATTR_TRIG_NEG         0x04

/* 0x1003+4*i   1B/input    RW        eclicintctl[i] */
#define ECLIC_INT_CTRL_OFFSET (offsetof(CLIC_Type, CTRL) + offsetof(CLIC_CTRL_Type, INTCTRL) )

#define ECLIC_ADDR_BASE __ECLIC_BASEADDR

#define ECLIC_CFG_NLBITS_MASK CLIC_CLICCFG_NLBIT_Msk
#define ECLIC_CFG_NLBITS_LSB  CLIC_CLICCFG_NLBIT_Pos

#define MSIP_HANDLER    eclic_msip_handler
#define MTIME_HANDLER   eclic_mtip_handler
#define BWEI_HANDLER    eclic_bwei_handler
#define PMOVI_HANDLER   eclic_pmovi_handler

#endif
