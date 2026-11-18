/*!
    \file    usb_conf.h
    \brief   USB core driver basic configuration for Zephyr GD32H73x_75x

    This is a Zephyr-adapted version of the GD32H73x_75x USBHS library
    usb_conf.h. It selects USBHS0 in device mode with the on-chip
    full-speed (FS) PHY. The original example version pulls in a board
    BSP header; here we only include the CMSIS device header shipped
    with the hal_gigadevice module.

    The GD32H73X_75X/GD32H75E device select macro is supplied by
    zephyr/modules/hal_gigadevice/CMakeLists.txt (it picks whichever one
    matches CONFIG_SOC), so it is intentionally not redefined here.

    SPDX-License-Identifier: Apache-2.0
*/

#ifndef USB_CONF_H
#define USB_CONF_H

#include "gd32h73x_75x.h"
/*
 * The USB library and the shim driver call the rcu_ and pmu_ standard
 * peripheral functions (e.g. pmu_to_deepsleepmode(), rcu_periph_clock_enable()).
 * Pull in their declarations here so every vendored library translation unit
 * sees them.
 */
#include "gd32h73x_75x_rcu.h"
#include "gd32h73x_75x_pmu.h"

/*
 * ================= USB controller and speed selection =================
 * This is the SINGLE place to switch the USB controller and the speed.
 * Pick exactly ONE controller and exactly ONE speed, then rebuild.
 *
 *   Controller:  USE_USBHS0  -> USBHS0 (base 0x40040000, IRQ 77)
 *                USE_USBHS1  -> USBHS1 (base 0x40080000, IRQ 175)
 *
 *   Speed:       USE_USB_FS  -> on-chip full-speed  PHY (12 Mbps)
 *                USE_USB_HS  -> on-chip high-speed  PHY (480 Mbps)
 *
 * The shim driver (usb_dc_gd32.c) derives every hardware constant
 * (base address, RCU clock/index, IRQ number, core speed and the USB
 * clock path) from these two macros, so nothing else needs editing to
 * switch between USBHS0/USBHS1 and full-/high-speed.
 * ========================================================================
 */

#define USE_USB_FS
/* #define USE_USB_HS */

/* USB 48 MHz reference clock source. */
/* #define USE_IRC48M */

#define USE_DEVICE_MODE

/* USB Core and PHY interface configuration */

/* on-chip full-speed USB PHY */
#ifdef USE_USB_FS
    #define OC_FS_PHY
#endif

/* on-chip high-speed USB PHY */
#ifdef USE_USB_HS
    #define OC_HS_PHY
#endif /* USE_USB_HS */

/* USB FIFO size config (in 32-bit words, total must fit the 1024-word / 4 KB
 * USBHS dedicated RAM). CDC ACM needs one interrupt IN endpoint and one bulk
 * IN endpoint in addition to control EP0, so TX0..TX3 are all given a non-zero
 * dedicated TX FIFO. RX(512) + TX0(128) + TX1(128) + TX2(128) + TX3(128) = 1024. */
#define RX_FIFO_SIZE                          512U
#define TX0_FIFO_SIZE                         128U
#define TX1_FIFO_SIZE                         128U
#define TX2_FIFO_SIZE                         128U
#define TX3_FIFO_SIZE                         128U
#define TX4_FIFO_SIZE                         0U
#define TX5_FIFO_SIZE                         0U
#define TX6_FIFO_SIZE                         0U
#define TX7_FIFO_SIZE                         0U

#ifdef USE_ULPI_PHY
    #define USB_EXTERNAL_ULPI_PHY_ENABLED
#else
    #ifdef OC_FS_PHY
         #define USB_EMBEDDED_FS_PHY_ENABLED
    #elif defined(OC_HS_PHY)
         #define USB_EMBEDDED_HS_PHY_ENABLED
    #else
         #error "PHY is not selected"
    #endif /* OC_FS_PHY */
#endif /* USE_ULPI_PHY */

/* internal DMA and dedicated EP1 are not used in this Zephyr FS port */
/* #define USB_INTERNAL_DMA_ENABLED */
/* #define USB_DEDICATED_EP1_ENABLED */

#define USB_SOF_OUTPUT                        0U
#define USB_LOW_POWER                         0U

/* VBUS sensing disabled (board USB device port is self-powered for bring-up) */
/* #define VBUS_SENSING_ENABLED */

#define USE_DEVICE_MODE
/* #define USE_HOST_MODE */
/* #define USE_OTG_MODE */

#ifndef OC_FS_PHY
    #ifndef OC_HS_PHY
        #error  "OC_FS_PHY or OC_HS_PHY should be defined!"
    #endif
#endif /* OC_FS_PHY */

#ifndef USE_DEVICE_MODE
    #ifndef USE_HOST_MODE
        #error  "USE_DEVICE_MODE or USE_HOST_MODE should be defined!"
    #endif
#endif /* USE_DEVICE_MODE */

#ifndef USE_USB_HS
    #ifndef USE_USB_FS
        #error  "USE_USB_HS or USE_USB_FS should be defined!"
    #endif
#endif /* USE_USB_HS */

/* all variables and data structures during the transaction process should be
   4-bytes aligned */
#if defined (__GNUC__)         /* GNU Compiler */
    #define __ALIGN_END __attribute__ ((aligned (4)))
    #define __ALIGN_BEGIN
#else
    #define __ALIGN_END

    #if defined (__CC_ARM)     /* ARM Compiler */
        #define __ALIGN_BEGIN __align(4)
    #elif defined (__ICCARM__) /* IAR Compiler */
        #define __ALIGN_BEGIN
    #elif defined (__TASKING__)/* TASKING Compiler */
        #define __ALIGN_BEGIN __align(4)
    #endif /* __CC_ARM */
#endif /* __GNUC__ */

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__GNUC__)       /* GNU Compiler */
    #ifndef __packed
        #define __packed __attribute__ ((__packed__))
    #endif
#elif defined (__TASKING__)    /* TASKING Compiler */
    #define __packed __unaligned
#endif /* __GNUC__ */

#endif /* USB_CONF_H */
