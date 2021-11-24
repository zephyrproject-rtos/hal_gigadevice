/*
 * Copyright (c) 2021 YuLong Yao <feilongphone@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GD32E103XX_AFIO_H_
#define GD32E103XX_AFIO_H_

#include "gd32-afio.h"

/** SPI0 (no remap) */
#define GD32_SPI0_NORMP		GD32_REMAP(0U, 0U, 0x1U, 0U)
/** SPI0 (remap) */
#define GD32_SPI0_RMP		GD32_REMAP(0U, 0U, 0x1U, 1U)

/** I2C0 (no remap) */
#define GD32_I2C0_NORMP		GD32_REMAP(0U, 1U, 0x1U, 0U)
/** I2C0 (remap) */
#define GD32_I2C0_RMP		GD32_REMAP(0U, 1U, 0x1U, 1U)

/** USART0 (no remap) */
#define GD32_USART0_NORMP	GD32_REMAP(0U, 2U, 0x1U, 0U)
/** USART0 (remap) */
#define GD32_USART0_RMP		GD32_REMAP(0U, 2U, 0x1U, 1U)

/** USART1 (no remap) */
#define GD32_USART1_NORMP	GD32_REMAP(0U, 3U, 0x1U, 0U)
/** USART1 (remap) */
#define GD32_USART1_RMP		GD32_REMAP(0U, 3U, 0x1U, 1U)

/** USART2 (no remap) */
#define GD32_USART2_NORMP	GD32_REMAP(0U, 4U, 0x3U, 0U)
/** USART2 (partial remap) */
#define GD32_USART2_PRMP	GD32_REMAP(0U, 4U, 0x3U, 1U)
/** USART2 (full remap) */
#define GD32_USART2_FRMP	GD32_REMAP(0U, 4U, 0x3U, 3U)

/** TIMER0 (no remap) */
#define GD32_TIMER0_NORMP	GD32_REMAP(0U, 6U, 0x3U, 0U)
/** TIMER0 (partial remap) */
#define GD32_TIMER0_PRMP	GD32_REMAP(0U, 6U, 0x3U, 1U)
/** TIMER0 (full remap) */
#define GD32_TIMER0_FRMP	GD32_REMAP(0U, 6U, 0x3U, 3U)

/** TIMER1 (no remap) */
#define GD32_TIMER1_NORMP	GD32_REMAP(0U, 8U, 0x3U, 0U)
/** TIMER1 (partial remap 1) */
#define GD32_TIMER1_PRMP1	GD32_REMAP(0U, 8U, 0x3U, 1U)
/** TIMER1 (partial remap 2) */
#define GD32_TIMER1_PRMP2	GD32_REMAP(0U, 8U, 0x3U, 2U)
/** TIMER1 (full remap) */
#define GD32_TIMER1_FRMP	GD32_REMAP(0U, 8U, 0x3U, 3U)

/** TIMER2 (no remap) */
#define GD32_TIMER2_NORMP	GD32_REMAP(0U, 10U, 0x3U, 0U)
/** TIMER2 (partial remap) */
#define GD32_TIMER2_PRMP	GD32_REMAP(0U, 10U, 0x3U, 2U)
/** TIMER2 (full remap) */
#define GD32_TIMER2_FRMP	GD32_REMAP(0U, 10U, 0x3U, 3U)

/** TIMER3 (no remap) */
#define GD32_TIMER3_NORMP	GD32_REMAP(0U, 12U, 0x1U, 0U)
/** TIMER3 (remap) */
#define GD32_TIMER3_RMP		GD32_REMAP(0U, 12U, 0x1U, 1U)

/** TIMER4CH3 (no remap) */
#define GD32_TIMER4CH3_NORMP   GD32_REMAP(0U, 16U, 0x1U, 0U)
/** TIMER4CH3 (remap) */
#define GD32_TIMER4CH3_RMP     GD32_REMAP(0U, 16U, 0x1U, 1U)

/** SPI2 (no remap) */
#define GD32_SPI2_NORMP		GD32_REMAP(0U, 28U, 0x1U, 0U)
/** SPI2 (remap) */
#define GD32_SPI2_RMP		GD32_REMAP(0U, 28U, 0x1U, 1U)

/** TIMER1_ITR0 (no remap) */
#define GD32_TIMER1ITR0_NORMP GD32_REMAP(0U, 29U, 0x1U, 0U)
/** TIMER1_ITR0 (remap) */
#define GD32_TIMER1ITR0_RMP   GD32_REMAP(0U, 29U, 0x1U, 1U)

/** TIMER8 (no remap) */
#define GD32_TIMER8_NORMP	GD32_REMAP(1U, 5U, 0x1U, 0U)
/** TIMER8 (remap) */
#define GD32_TIMER8_RMP		GD32_REMAP(1U, 5U, 0x1U, 1U)

/** CTC (no remap) */
#define GD32_CTC_NORMP		GD32_REMAP(1U, 11U, 0x3U, 0U)
/** CTC (remap) */
#define GD32_CTC_PRMP		GD32_REMAP(1U, 11U, 0x3U, 1U)

#endif /* GD32E103XX_AFIO_H_ */
