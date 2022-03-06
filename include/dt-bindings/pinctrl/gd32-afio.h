/*
 * Copyright (c) 2021 Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DT_BINDINGS_PINCTRL_GD32_AFIO_H_
#define DT_BINDINGS_PINCTRL_GD32_AFIO_H_

/**
 * @name GD32 pin modes
 * @{
 */

/** Analog mode */
#define GD32_MODE_ANALOG 0U
/** GPIO input */
#define GD32_MODE_GPIO_IN 1U
/** Alternate function */
#define GD32_MODE_ALTERNATE 2U

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
/** Mode field mask. */
#define GD32_MODE_MSK 0x3U
/** Mode field position. */
#define GD32_MODE_POS 8U
/** Remap field mask. */
#define GD32_REMAP_MSK 0x3FFU
/** Remap field position. */
#define GD32_REMAP_POS 10U

/** @} */

/** No remap available */
#define GD32_NORMP 0U

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
 * Obtain mode field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define GD32_MODE_GET(pinmux) \
	(((pinmux) >> GD32_MODE_POS) & GD32_MODE_MSK)

/**
 * Obtain pinmux field from pinmux configuration.
 * 
 * @param pinmux Pinmux bit field value.
 */
#define GD32_REMAP_GET(pinmux) \
	(((pinmux) >> GD32_REMAP_POS) & GD32_REMAP_MSK)

/**
 * @brief Remap configuration bit field.
 * 
 * Fields:
 *
 * - 0..3: port
 * - 4..7: pin
 * - 8..9: mode
 * - 10..19: remap
 * 
 * @param port Port ('A'..'P')
 * @param pin Pin (0..15)
 * @param mode Mode (ANALOG, GPIO_IN, ALTERNATE).
 * @param remap Remap value, see #GD32_REMAP.
 */
#define GD32_PINMUX_AFIO(port, pin, mode, remap)			\
	(((((port) - 'A') & GD32_PORT_MSK) << GD32_PORT_POS) |		\
	 (((pin) & GD32_PIN_MSK) << GD32_PIN_POS) |			\
	 (((GD32_MODE_ ## mode) & GD32_MODE_MSK) << GD32_MODE_POS) |	\
	 (((GD32_ ## remap) & GD32_REMAP_MSK) << GD32_REMAP_POS))

/**
 * @name Remap bit field mask and positions.
 * @{
 */

/** Register field mask. */
#define GD32_REMAP_REG_MSK 0x1U
/** Register field position. */
#define GD32_REMAP_REG_POS 0U
/** Position field mask. */
#define GD32_REMAP_POS_MSK 0x1FU
/** Position field position. */
#define GD32_REMAP_POS_POS 1U
/** Mask field mask. */
#define GD32_REMAP_MSK_MSK 0x3U
/** Mask field position. */
#define GD32_REMAP_MSK_POS 6U
/** Value field mask. */
#define GD32_REMAP_VAL_MSK 0x3U
/** Value field position. */
#define GD32_REMAP_VAL_POS 8U

/** @} */

/**
 * Obtain register field from remap configuration.
 * 
 * @param remap Remap bit field value.
 */
#define GD32_REMAP_REG_GET(remap) \
	(((remap) >> GD32_REMAP_REG_POS) & GD32_REMAP_REG_MSK)

/**
 * Obtain position field from remap configuration.
 * 
 * @param remap Remap bit field value.
 */
#define GD32_REMAP_POS_GET(remap) \
	(((remap) >> GD32_REMAP_POS_POS) & GD32_REMAP_POS_MSK)

/**
 * Obtain mask field from remap configuration.
 * 
 * @param remap Remap bit field value.
 */
#define GD32_REMAP_MSK_GET(remap) \
	(((remap) >> GD32_REMAP_MSK_POS) & GD32_REMAP_MSK_MSK)

/**
 * Obtain value field from remap configuration.
 * 
 * @param remap Remap bit field value.
 */
#define GD32_REMAP_VAL_GET(remap) \
	(((remap) >> GD32_REMAP_VAL_POS) & GD32_REMAP_VAL_MSK)

/**
 * @brief Remap configuration bit field.
 * 
 * - 0:    reg (0 or 1).
 * - 1..5: pos (0..31).
 * - 6..7: msk (0x1, 0x3).
 * - 8..9: val (0..3).
 * 
 * @param reg AFIO_PCFx register (0, 1).
 * @param pos Position within AFIO_PCx.
 * @param msk Mask for the AFIO_PCx field.
 * @param val Remap value (0, 1, 2 or 3).
 */
#define GD32_REMAP(reg, pos, msk, val)					\
	((((reg) & GD32_REMAP_REG_MSK) << GD32_REMAP_REG_POS) |		\
	 (((pos) & GD32_REMAP_POS_MSK) << GD32_REMAP_POS_POS) |		\
	 (((msk) & GD32_REMAP_MSK_MSK) << GD32_REMAP_MSK_POS) |		\
	 (((val) & GD32_REMAP_VAL_MSK) << GD32_REMAP_VAL_POS))

#endif /* DT_BINDINGS_PINCTRL_GD32_AFIO_H_ */
