# Copyright (c) 2021, ATL Electronics
# SPDX-License-Identifier: Apache-2.0

string(SUBSTRING ${GD32_SOC_SERIES} 0 8 GD32_SOC)

zephyr_compile_definitions(
	${GD32_SOC}
)

zephyr_include_directories(CMSIS/GD/GD32E10x/Include)
zephyr_include_directories(standard_peripheral/Include)

zephyr_library_sources(CMSIS/GD/GD32E10x/Source/system_gd32e10x.c)

zephyr_library_sources_ifdef(CONFIG_USE_GD32_ADC   standard_peripheral/Source/gd32e10x_adc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_BKP   standard_peripheral/Source/gd32e10x_bkp.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_CRC   standard_peripheral/Source/gd32e10x_crc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_CTC   standard_peripheral/Source/gd32e10x_ctc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DAC   standard_peripheral/Source/gd32e10x_dac.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DBG   standard_peripheral/Source/gd32e10x_dbg.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_DMA   standard_peripheral/Source/gd32e10x_dma.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_EXMC  standard_peripheral/Source/gd32e10x_exmc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_EXTI  standard_peripheral/Source/gd32e10x_exti.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_FMC   standard_peripheral/Source/gd32e10x_fmc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_FWDGT standard_peripheral/Source/gd32e10x_fwdgt.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_GPIO  standard_peripheral/Source/gd32e10x_gpio.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_I2C   standard_peripheral/Source/gd32e10x_i2c.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_MISC  standard_peripheral/Source/gd32e10x_misc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_PMU   standard_peripheral/Source/gd32e10x_pmu.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_RCU   standard_peripheral/Source/gd32e10x_rcu.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_RTC   standard_peripheral/Source/gd32e10x_rtc.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_SPI   standard_peripheral/Source/gd32e10x_spi.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_TIMER standard_peripheral/Source/gd32e10x_timer.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_USART standard_peripheral/Source/gd32e10x_usart.c)
zephyr_library_sources_ifdef(CONFIG_USE_GD32_WWDGT standard_peripheral/Source/gd32e10x_wwdgt.c)
