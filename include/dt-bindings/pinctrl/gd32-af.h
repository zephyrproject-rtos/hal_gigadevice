/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DT_BINDINGS_PINCTRL_GD32_AF_H_
#define DT_BINDINGS_PINCTRL_GD32_AF_H_

/**
 * @name GD32 AFs
 * @{
 */

/** AF0 */
#define GD32_AF0 0U
/** AF1 */
#define GD32_AF1 1U
/** AF2 */
#define GD32_AF2 2U
/** AF3 */
#define GD32_AF3 3U
/** AF4 */
#define GD32_AF4 4U
/** AF5 */
#define GD32_AF5 5U
/** AF6 */
#define GD32_AF6 6U
/** AF7 */
#define GD32_AF7 7U
/** AF8 */
#define GD32_AF8 8U
/** AF9 */
#define GD32_AF9 9U
/** AF10 */
#define GD32_AF10 10U
/** AF11 */
#define GD32_AF11 11U
/** AF12 */
#define GD32_AF12 12U
/** AF13 */
#define GD32_AF13 13U
/** AF14 */
#define GD32_AF14 14U
/** AF15 */
#define GD32_AF15 15U
/** ANALOG */
#define GD32_ANALOG 16U

/** @} */

/**
 * @name GD32 pinmux bit field mask and positions.
 * @{
 */

/** Port field mask. */
#define GD32_PORT_MSK 0xFU
/** Port field position. */
#define GD32_PORT_POS 0U
/** Pin field mask. */
#define GD32_PIN_MSK 0xFU
/** Pin field position. */
#define GD32_PIN_POS 4U
/** AF field mask. */
#define GD32_AF_MSK 0x1FU
/** AF field position. */
#define GD32_AF_POS 8U

/** @} */

/**
 * Obtain port field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define GD32_PORT_GET(pinmux) \
	(((pinmux) >> GD32_PORT_POS) & GD32_PORT_MSK)

/**
 * Obtain pin field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define GD32_PIN_GET(pinmux) \
	(((pinmux) >> GD32_PIN_POS) & GD32_PIN_MSK)

/**
 * Obtain AF field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define GD32_AF_GET(pinmux) \
	(((pinmux) >> GD32_AF_POS) & GD32_AF_MSK)

/**
 * @brief Remap configuration bit field.
 * 
 * Fields:
 *
 * - 0..3: port
 * - 4..7: pin
 * - 8..12: af
 * 
 * @param port Port ('A'..'P')
 * @param pin Pin (0..15)
 * @param af Alternate function (ANALOG, AFx, x=0..15).
 */
#define GD32_PINMUX_AF(port, pin, af)					\
	(((((port) - 'A') & GD32_PORT_MSK) << GD32_PORT_POS) |		\
	 (((pin) & GD32_PIN_MSK) << GD32_PIN_POS) |			\
	 (((GD32_ ## af) & GD32_AF_MSK) << GD32_AF_POS))

#endif /* DT_BINDINGS_PINCTRL_GD32_AF_H_ */
