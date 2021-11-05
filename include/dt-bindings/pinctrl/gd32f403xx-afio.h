/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GD32F403XX_AFIO_H_
#define GD32F403XX_AFIO_H_

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

/** CAN0 (no remap) */
#define GD32_CAN0_NORMP		GD32_REMAP(0U, 13U, 0x3U, 0U)
/** CAN0 (partial remap) */
#define GD32_CAN0_PRMP		GD32_REMAP(0U, 13U, 0x3U, 2U)
/** CAN0 (full remap) */
#define GD32_CAN0_FRMP		GD32_REMAP(0U, 13U, 0x3U, 3U)

/** CAN1 (no remap) */
#define GD32_CAN1_NORMP		GD32_REMAP(0U, 22U, 0x1U, 0U)
/** CAN1 (remap) */
#define GD32_CAN1_RMP		GD32_REMAP(0U, 22U, 0x1U, 1U)

/** SPI2 (no remap) */
#define GD32_SPI2_NORMP		GD32_REMAP(0U, 28U, 0x1U, 0U)
/** SPI2 (remap) */
#define GD32_SPI2_RMP		GD32_REMAP(0U, 28U, 0x1U, 1U)

/** TIMER8 (no remap) */
#define GD32_TIMER8_NORMP	GD32_REMAP(1U, 5U, 0x1U, 0U)
/** TIMER8 (remap) */
#define GD32_TIMER8_RMP		GD32_REMAP(1U, 5U, 0x1U, 1U)

/** TIMER9 (no remap) */
#define GD32_TIMER9_NORMP	GD32_REMAP(1U, 6U, 0x1U, 0U)
/** TIMER9 (remap) */
#define GD32_TIMER9_RMP		GD32_REMAP(1U, 6U, 0x1U, 1U)

/** TIMER10 (no remap) */
#define GD32_TIMER10_NORMP	GD32_REMAP(1U, 7U, 0x1U, 0U)
/** TIMER10 (remap) */
#define GD32_TIMER10_RMP	GD32_REMAP(1U, 7U, 0x1U, 1U)

/** TIMER12 (no remap) */
#define GD32_TIMER12_NORMP	GD32_REMAP(1U, 8U, 0x1U, 0U)
/** TIMER12 (remap) */
#define GD32_TIMER12_RMP	GD32_REMAP(1U, 8U, 0x1U, 1U)

/** TIMER13 (no remap) */
#define GD32_TIMER13_NORMP	GD32_REMAP(1U, 9U, 0x1U, 0U)
/** TIMER13 (remap) */
#define GD32_TIMER13_RMP	GD32_REMAP(1U, 9U, 0x1U, 1U)

/** CTC (no remap) */
#define GD32_CTC_NORMP		GD32_REMAP(1U, 11U, 0x3U, 0U)
/** CTC (partial remap) */
#define GD32_CTC_PRMP		GD32_REMAP(1U, 11U, 0x3U, 1U)
/** CTC (full remap) */
#define GD32_CTC_FRMP		GD32_REMAP(1U, 11U, 0x3U, 2U)

#endif /* GD32F403XX_AFIO_H_ */
