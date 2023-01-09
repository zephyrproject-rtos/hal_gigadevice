# Introduction

The **hal_gigadevice** is a set of both standard firmware library and ARM CMSIS
configurations for GigaDevice MCUs. The HAL is organized following the
directory structure detailed below.

## Directory Structure

The directory is composed by three parts:

 - SoC specific libraries.
 - ZephyrRTOS module directory (`zephyr`).
 - This README file.

### ARM

Each ARM firmware library is organized in the following structure:

```
.
└── gd32xxx
    ├── cmsis
    │   └── gd
    │       └── gd32xxx
    ├── standard_peripheral
    │   ├── include
    │   └── source
    └── support (optional)
        └── GigaDevice.GD32xxx_DFP.1.0.0.pack (Only for packs not in [Keil MDK5 Software Packs](https://www.keil.com/dd2/pack/))
```

### RISC-V

The RISC-V structure is not defined yet. The riscv sub-directory will change
based on how [NMSIS library](https://github.com/Nuclei-Software/NMSIS/) will
be integrated in Zephyr. At present date, the structure only reflects current
manufacturer version.

```
└── gd32bf103
    ├── riscv
    │   ├── drivers
    │   ├── include
    │   ├── source
    │   └── stubs
    └── standard_peripheral
        ├── include
        └── source
```

 Any contribution should follow the `How to submit code` using as premisses the
 [Zephyr Contribution Guidelines](https://docs.zephyrproject.org/latest/contribute/index.html).

# How to submit code

 - When submiting an updated firmware library version, it is important to make
   sure that the last firmware version will be merged. The library version is
   used to be checked at [GigaDevice MCU Site](http://www.gd32mcu.com/en/download/).
 - This repository is managed mainly on Linux. This means that changes should
   be submited using Linux LF format. Usually running a similar command to
   `find * type f -print0 | xargs -0 dos2unix -k` should be enough.
 - Make sure directory structure is respected.
 - Directory names should be converted to lowercase.
 - Exceptions should be discussed at review phase.

## The gd32 standard peripheral API

The gd32 standard peripheral API define all information to access GigaDevice
peripherals. The firmware library does not have namespaces and prefixes which
easily results in name collision with ZephyrRTOS core and libraries. To avoid
that situation, macros, enum values and function names should follow two
general rules:

 - Public API funtions must be prefixed with `gd32_`.
 - Public defines and enum values must be uppercase and prefixed with `GD32_`.

The cmsis and riscv contents are not part of standard peripheral library and
changes are treated as exceptions. This means that `<firmware_library>.h>` and
`system_<firmware_library>.h>` files should reside on their own respective
places instead in standard peripheral library directory.

### Grace Period

Rework gd32 standard peripheral API is a huge step. To allow developers move
forward a grace period was created. The grace period is available until
`gd32_api_convert` script be available. When the script be available any new
firmware library must be converted. The main objective of the script is to
automate conversion process of all gd32 standard peripheral API.

Besides there is a grace period developers are obligated to solve any existent
conflict. The conflict should follow gd32 standard peripheral API rules.

### Current known conflict list

The conflicts at gd32 standard peripheral API are listed by the afected file(s).
In general, a name is the conflict itself and that whould be reworked following
the gd32 standard peripheral API rules.

 - `<firmware_library>_timer.h/c`: `timer_init()` should be prefixed with
   `gd32_` and become `gd32_timer_init()`.
 - `<firmware_library>_can.h/c`: all `CAN_` macros that collide with any
   macro defined at `/include/drivers/can.h` should be prefixed with `GD32_`
   and become `GD32_CAN_*`.

## Exceptions

The exception list define macros and enum values that may have a different
conflict resolution. See below list with the proposed solution:

 - The `BIT` macro is already define in Zephyr. Fix using `#ifndef`:
```review
+#ifndef BIT
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
+#endif
```
- i2c-gd32 driver needs to know the clock frequency boundary. Fix it with
  two step work. First move I2CCLK_MAX and I2CCLK_MIN marco from i2c source
  file to header file. Then split the I2CCLK_MIN for each supported transfer
  mode.

- gd32l23x

  - `SystemInit` function in `system_gd32l23x.c` contain an invalid usage of
    `nvic_vector_table_set` function, just remove it.

  - `gd32l23x_libopt.h` file not exist, add it manually.

  - i2c have different implement than current gd32 i2c driver. no need to patch
    upper i2c speed requirement.

- gd32a50x

  - `SystemCoreClockUpdate` function contain an `pllmf` calculate error.  
    Fix by change `0xFU` to `RCU_CFG0_PLLMF`:
    ``` diff
      /* PLL multiplication factor */
      pllmf = GET_BITS(RCU_CFG0, 18, 21);
      pllmf += ((RCU_CFG0 & RCU_CFG0_PLLMF_4) ? 15U : 0U);
    - pllmf += ((0xFU == (RCU_CFG0 & RCU_CFG0_PLLMF)) ? 1U : 2U);
    + pllmf += ((RCU_CFG0_PLLMF == (RCU_CFG0 & RCU_CFG0_PLLMF)) ? 1U : 2U);
    ```
  - For debug this board, `pyocd` need a pack file from Gigadevice,
    But Gigadevice don't upload pack file to [keil packs repo](https://www.keil.com/dd2/pack/).
    Fix: Storage pack as `gd32a50x/support/GigaDevice.GD32A50x_DFP.1.0.0.pack`
    (Download from https://gd32mcu.com/cn/download/7?kw=GD32A5 -> GD32A50x AddOn)
