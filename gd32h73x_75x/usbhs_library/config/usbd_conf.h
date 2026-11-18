/*!
    \file    usbd_conf.h
    \brief   USB device core configuration for the Zephyr GD32 USBHS shim

    Minimal Zephyr-adapted version of the GD32H73x_75x USBHS library
    usbd_conf.h. Only the device-core sizing macros used by the vendored
    driver (drv_usb_dev.*) are kept; the GD32 CDC class endpoint macros are
    intentionally omitted because the CDC ACM class is provided by the Zephyr
    legacy USB stack, not by the GD32 firmware library.

    SPDX-License-Identifier: Apache-2.0
*/

#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "usb_conf.h"

#define USBD_CFG_MAX_NUM                    1U
#define USBD_ITF_MAX_NUM                    1U

#define USB_STR_DESC_MAX_SIZE              255U

#define USB_STRING_COUNT                    4U

#endif /* USBD_CONF_H */
