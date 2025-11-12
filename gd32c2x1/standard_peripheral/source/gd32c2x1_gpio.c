/*!
    \file    gd32c2x1_gpio.c
    \brief   GPIO driver

    \version 2025-08-08, V1.1.0, firmware for gd32c2x1
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

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

#include "gd32c2x1_gpio.h"

/*!
    \brief      reset GPIO port (API_ID: (0x0001U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[out] none
    \retval     none
*/
void gpio_deinit(uint32_t gpio_periph)
{
    switch(gpio_periph) {
    case GPIOA:
        /* reset GPIOA */
        rcu_periph_reset_enable(RCU_GPIOARST);
        rcu_periph_reset_disable(RCU_GPIOARST);
        break;
    case GPIOB:
        /* reset GPIOB */
        rcu_periph_reset_enable(RCU_GPIOBRST);
        rcu_periph_reset_disable(RCU_GPIOBRST);
        break;
    case GPIOC:
        /* reset GPIOC */
        rcu_periph_reset_enable(RCU_GPIOCRST);
        rcu_periph_reset_disable(RCU_GPIOCRST);
        break;
    case GPIOD:
        /* reset GPIOD */
        rcu_periph_reset_enable(RCU_GPIODRST);
        rcu_periph_reset_disable(RCU_GPIODRST);
        break;
    case GPIOF:
        /* reset GPIOF */
        rcu_periph_reset_enable(RCU_GPIOFRST);
        rcu_periph_reset_disable(RCU_GPIOFRST);
        break;
    default:
        break;
    }
}

/*!
    \brief      set GPIO mode (API_ID: (0x0002U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  mode: gpio pin mode
                only one parameter can be selected which is shown as below:
      \arg        GPIO_MODE_INPUT: input mode
      \arg        GPIO_MODE_OUTPUT: output mode
      \arg        GPIO_MODE_AF: alternate function mode
      \arg        GPIO_MODE_ANALOG: analog mode
    \param[in]  pull_up_down: gpio pin with pull-up or pull-down resistor
                only one parameter can be selected which is shown as below:
      \arg        GPIO_PUPD_NONE: floating mode, no pull-up and pull-down resistors
      \arg        GPIO_PUPD_PULLUP: with pull-up resistor
      \arg        GPIO_PUPD_PULLDOWN:with pull-down resistor
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_mode_set(uint32_t gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin)
{
    uint16_t i;
    uint32_t ctl, pupd;

    ctl = GPIO_CTL(gpio_periph);
    pupd = GPIO_PUD(gpio_periph);

    for(i = 0U; i < 16U; i++) {
        if(((((uint32_t)1U << i)) & (pin & BITS(0,15))) != (uint32_t)0U) {
            /* clear the specified pin mode bits */
            ctl &= ~GPIO_MODE_MASK(i);
            /* set the specified pin mode bits */
            ctl |= GPIO_MODE_SET(i, mode & BITS(0,1));

            /* clear the specified pin pupd bits */
            pupd &= ~GPIO_PUPD_MASK(i);
            /* set the specified pin pupd bits */
            pupd |= GPIO_PUPD_SET(i, pull_up_down & BITS(0,1));
        }
    }

    GPIO_CTL(gpio_periph) = ctl;
    GPIO_PUD(gpio_periph) = pupd;
}

/*!
    \brief      set GPIO output type and speed (API_ID: (0x0003U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  otype: gpio pin output mode
                only one parameter can be selected which is shown as below:
      \arg        GPIO_OTYPE_PP: push pull mode
      \arg        GPIO_OTYPE_OD: open drain mode
    \param[in]  speed: gpio pin output max speed
                only one parameter can be selected which is shown as below:
      \arg        GPIO_OSPEED_LEVEL_0: output speed level 0
      \arg        GPIO_OSPEED_LEVEL_1: output speed level 1
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_output_options_set(uint32_t gpio_periph, uint8_t otype, uint32_t speed, uint32_t pin)
{
    uint16_t i;
    uint32_t ospeed;

    if(GPIO_OTYPE_OD == (otype & (uint8_t)BIT(0))) {
        GPIO_OMODE(gpio_periph) |= (uint32_t)(pin & BITS(0,15));
    } else {
        GPIO_OMODE(gpio_periph) &= (uint32_t)(~(pin & BITS(0,15)));
    }

    /* get the specified pin output speed bits value */
    ospeed = GPIO_OSPD(gpio_periph);

    for(i = 0U; i < 16U; i++) {
        if((((uint32_t)1U << i) & (pin & BITS(0,15))) != (uint32_t)0U) {
            /* clear the specified pin output speed bits */
            ospeed &= ~GPIO_OSPEED_MASK(i);
            /* set the specified pin output speed bits */
            ospeed |= GPIO_OSPEED_SET(i, speed & BIT(0));
        }
    }
    GPIO_OSPD(gpio_periph) = ospeed;
}

/*!
    \brief      set GPIO pin bit (API_ID: (0x0004U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_bit_set(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_BOP(gpio_periph) = (uint32_t)(pin & BITS(0,15));
}

/*!
    \brief      reset GPIO pin bit (API_ID: (0x0005U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_bit_reset(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_BC(gpio_periph) = (uint32_t)(pin & BITS(0,15));
}

/*!
    \brief      write data to the specified GPIO pin (API_ID: (0x0006U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[in]  bit_value: SET or RESET
                only one parameter can be selected which is shown as below:
      \arg        RESET: clear the port pin
      \arg        SET: set the port pin
    \param[out] none
    \retval     none
*/
void gpio_bit_write(uint32_t gpio_periph, uint32_t pin, bit_status bit_value)
{
    if(RESET != bit_value) {
        GPIO_BOP(gpio_periph) = (uint32_t)(pin & BITS(0,15));
    } else {
        GPIO_BC(gpio_periph) = (uint32_t)(pin & BITS(0,15));
    }
}

/*!
    \brief      write data to the specified GPIO port (API_ID: (0x0007U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  data: specify the value to be written to the port output control register
    \param[out] none
    \retval     none
*/
void gpio_port_write(uint32_t gpio_periph, uint16_t data)
{
    GPIO_OCTL(gpio_periph) = (uint32_t)data;
}

/*!
    \brief      get GPIO pin input status (API_ID: (0x0008U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     SET or RESET
*/
FlagStatus gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin)
{
    FlagStatus reval;

    if((uint32_t)RESET != (GPIO_ISTAT(gpio_periph) & ((pin & BITS(0,15))))) {
        reval = SET;
    } else {
        reval = RESET;
    }

    return reval;
}

/*!
    \brief      get GPIO port input status (API_ID: (0x0009U))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[out] none
    \retval     state of GPIO all pins
*/
uint16_t gpio_input_port_get(uint32_t gpio_periph)
{
    return ((uint16_t)GPIO_ISTAT(gpio_periph));
}

/*!
    \brief      get GPIO pin output status (API_ID: (0x000AU))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     SET or RESET
*/
FlagStatus gpio_output_bit_get(uint32_t gpio_periph, uint32_t pin)
{
    FlagStatus reval;

    if((uint32_t)RESET != (GPIO_OCTL(gpio_periph) & ((pin & BITS(0,15))))) {
        reval = SET;
    } else {
        reval = RESET;
    }
    return reval;
}

/*!
    \brief      get GPIO port output status (API_ID: (0x000BU))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[out] none
    \retval     state of GPIO all pins
*/
uint16_t gpio_output_port_get(uint32_t gpio_periph)
{
    return ((uint16_t)GPIO_OCTL(gpio_periph));
}

/*!
    \brief      set GPIO alternate function (API_ID: (0x000CU))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  alt_func_num: GPIO pin af function, please refer to specific device datasheet
                only one parameter can be selected which is shown as below:
      \arg        GPIO_AF_0: RTC_OUT, CK_OUT0, SWDIO, SWCLK, SPI0, SPI1, USART0, EVENTOUT
      \arg        GPIO_AF_1: USART0, USART1, USART2, TIMER0, TIMER2, SPI1
      \arg        GPIO_AF_2: TIMER0, TIMER13, TIMER15, TIMER16
      \arg        GPIO_AF_3: SPI1, TIMER2, CK_OUT1, USART2
      \arg        GPIO_AF_4: USART0, USART1, SPI1, TIMER13
      \arg        GPIO_AF_5: TIMER0, TIMER16, I2S, USART0, SPI1
      \arg        GPIO_AF_6: I2C0, I2C1, USART1, USART2,
      \arg        GPIO_AF_7: CMP0, CMP1, EVENTOUT, I2C0
      \arg        GPIO_AF_8: SPI0
      \arg        GPIO_AF_9: TIMER0, USART1
      \arg        GPIO_AF_10: SPI0, TIMER0, TIMER15, TIMER16
      \arg        GPIO_AF_11: CK_OUT1, TIMER0, TIMER2
      \arg        GPIO_AF_12: USART0, TIMER0, TIMER2
      \arg        GPIO_AF_13: TIMER2, TIMER13
      \arg        GPIO_AF_14: USART0, TIMER15, I2C0
      \arg        GPIO_AF_15: CK_OUT1, TIMER16, EVENTOUT
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_af_set(uint32_t gpio_periph, uint32_t alt_func_num, uint32_t pin)
{
    uint16_t i;
    uint32_t afrl, afrh;

    afrl = GPIO_AFSEL0(gpio_periph);
    afrh = GPIO_AFSEL1(gpio_periph);

    for(i = 0U; i < 8U; i++) {
        if((((uint32_t)1U << i) & (pin & BITS(0,15))) != (uint32_t)0U) {
            /* clear the specified pin alternate function bits */
            afrl &= ~GPIO_AFR_MASK(i);
            afrl |= GPIO_AFR_SET(i, alt_func_num & BITS(0,3));
        }
    }

    for(i = 8U; i < 16U; i++) {
        if((((uint32_t)1U << i) & (pin & BITS(0,15))) != (uint32_t)0U) {
            /* clear the specified pin alternate function bits */
            afrh &= ~GPIO_AFR_MASK(i - 8U);
            afrh |= GPIO_AFR_SET(i - 8U, alt_func_num & BITS(0,3));
        }
    }

    GPIO_AFSEL0(gpio_periph) = afrl;
    GPIO_AFSEL1(gpio_periph) = afrh;
}

/*!
    \brief      lock GPIO pin bit (API_ID: (0x000DU))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_pin_lock(uint32_t gpio_periph, uint32_t pin)
{
    uint32_t lock = 0x00010000U;
    lock |= pin;

    /* lock key writing sequence: write 1->write 0->write 1->read 0->read 1 */
    GPIO_LOCK(gpio_periph) = (uint32_t)lock;
    GPIO_LOCK(gpio_periph) = (uint32_t)(pin & BITS(0,15));
    GPIO_LOCK(gpio_periph) = (uint32_t)lock;
    GPIO_LOCK(gpio_periph);
    GPIO_LOCK(gpio_periph);
}

/*!
    \brief      toggle GPIO pin status (API_ID: (0x000EU))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
void gpio_bit_toggle(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_TG(gpio_periph) = (uint32_t)(pin & BITS(0,15));
}

/*!
    \brief      toggle GPIO port status (API_ID: (0x000FU))
    \param[in]  gpio_periph: GPIOx(x = A,B,C,D,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,D,F)
    \param[out] none
    \retval     none
*/
void gpio_port_toggle(uint32_t gpio_periph)
{
    GPIO_TG(gpio_periph) = 0x0000FFFFU;
}
