# GD32 pin configurations

This directory contains a set of files describing valid pin configurations for
multiple GD32 devices, for example, that `USART0_TX` signal can only be mapped
to `PA0` and `PC8`. These configurations can be used to generate valid pin
mappings.

GD32 devices use two distinct models to configure signal multiplexing. In
some devices, like GD32F103XX, each pin has one or more associated alternate
function. These alternate functions can be sometimes changed using the remap
functionality, which allows routing all signals from a peripheral to a
different set of pins. For this reason we will name the model used by such
devices *AFIO model*. Some other devices, like GD32F450XX, have a simpler
and more advanced model where each pin has a fixed list of alternate functions.
Similarly, we will name the model used by such devices *AF model*. The datasheet
(not reference manual) can be used to determine which model is used by a device.
If the pin definition table mentions *remaps*, it means the device uses the
*AFIO model*. Alternate function tables (AF0..15) will only be available on
devices using the *AF model*.

In the following sections the expected file format is described.

## Common configuration

Some fields are common independently of the pin configuration model.
The commented example below describes common fields in detail.

- `model` (required): Choose between `afio` or `af`
- `series` (required): Series name, e.g. gd32vf103
- `variants` (required): Each variant has a different set of valid pin
  combinations, either because of a different number of pins or because devices
  with certain memory configurations have less or more peripherals available.

Example:

```yaml
model: afio

series: gd32vf103

variants:
  - pincode: V
    memories: [B, 8]
  - pincode: R
    memories: [B, 8]
  # Same pin code as before but different memories: a variant because it has
  # less peripherals (and so less signals) than B-8.
  - pincode: R
    memories: [6, 4]
```

## AFIO model

The AFIO model supports the following fields in the configuration file:

- `signal-configs` (required): A dictionary of signal configurations. Each
  available signal is required since the mode has to be specified.

  Available configuration fields for each signal are:

  - `modes` (required): A list containing one or more of these modes: `analog`,
    `inp` and `out`.
  - `exclude-memories` (optional): A lit of memories where the signal is not
    available. If not defined, it is assumed that signal is available on all
    memory configurations.
  - `exclude-pincodes` (optional): A list of pincodes where the signal is not
    available. If not defined, it is assumed that signal is available on all
    pin codes.

- `pins` (required): A dictionary of pin configurations.

  Available configuration options for each pin are:

  - `pincodes` (required): A list of pin codes that contain this pin.
  - `afs` (required): The list of valid alternate functions for that pin.
    The list must not include signals that can be remapped.

- `remaps` (required): Available remaps for each remappable signal.

  Available configuration options for each remap are:

  - `pins`: Available pins. The size of the list of pins determines wether
    a signal can be remapped (2 options) or partially or fully remapped
    (4 options). If a signal has a single partial remap, it still needs
    to provide 4 entries, but setting the non-available one to null.

Example:

```yaml
signal-configs:
  # Configuration for 'ADC01_IN0': available on all pincodes/memories, can
  # only operate in 'analog' mode.
  ADC01_IN0:
    modes: [analog]
  # Configuration for 'I2S2_CK': available on all pincodes, all memories except
  # 6, 4 and can operate in both 'inp' and 'out' modes.
  I2S2_CK:
    modes: [inp, out]
    exclude-memories: [6, 4]

pins:
  # Configuration for pin 'PA0'. Supported on V, R, C, T pincodes, valid
  # alternate functions include WKUP, USART1_CTS, ...
  PA0:
    pincodes: [V, R, C, T]
    afs: [WKUP, USART1_CTS, ADC01_IN0, TIMER1_CH0_ETI，TIMER4_CH0]

remaps:
  # Configuration for CAN0_RX signal. It can be remapped to PA11 (no remap),
  # PB8 (partial remap (1)) or PD0 (full remap).
  CAN0_RX:
    pins: [PA11, PB8, None, PD0]
```

## AF model

The AFIO model supports the following fields in the configuration file:

- `signal-configs` (optional): Configuration for signals that require it.

  Available configuration fields are:

  - `exclude-pincodes` (optional): A list of pincodes where the signal is not
    available. If not defined, it is assumed that signal is available on all
    pin codes.

  - `exclude-memories` (optional): A lit of memories where the signal is not
    available. If not available, it is assumed that signal is available on all
    memories.

- `pins` (required): Configuration for all pins.

  Available configuration options for each pin are:

  - `pincodes` (required): A list of pin codes that contain this pin.
  - `afs` (required): A dictionary of signals <> alternate function
    number (or `ANALOG`).

Example:

```yaml
signal-configs:
  # Configuration for 'EXMC_NOE': available on all memories except 6, 4.
  EXMC_NOE:
    exclude-memories: [6, 4]

pins:
  # Configuration for pin 'PA0'. Available on pin codes I, Z, V and has
  # the alternate functions listed in afs.
  PA0:
    pincodes: [I, Z, V]
    afs:
      # ADC012_IN0 available in analog mode
      ADC012_IN0: ANALOG
      # TIMER1_CH0 if AF1 is selected
      TIMER1_CH0: 1
      # USART1_CTS if AF7 is selected
      USART1_CTS: 7
```

## Exception

### GD32F405
For GD32F405Vx series, LQFP100 package have 82 I/O pins, but BGA100 package only have 81 I/O pins. PD8 is lost on BGA100 package.

### GD32F350
For GD32F350xB/8/6 devices, there have some invalid signal mapping for USART0, I2C0 and SPI0.
This issue cause by peripheral number increased on GD32F350xB/8/6, related pins already mapping
to another signals.

Below table show the invalid signal, pins mapping.

| SIGNAL     | PINS      |
| ---------- | --------- |
| USART0_CTS | PA0       |
| USART0_RTS | PA1       |
| USART0_TX  | PA2, PA14 |
| USART0_RX  | PA3, PA15 |
| USART0_CK  | PA4       |
| I2C0_SCL   | PB10, PF6 |
| I2C0_SDA   | PB11, PF7 |
| SPI0_NSS   | PB12      |
| SPI0_SCK   | PB13      |
| SPI0_MISO  | PB14      |
| SPI0_MOSI  | PB15      |

#### GD32L233

GD32L233Kx-QFN32 and GD32L233Kx-LQFP32 have same pincode 'K'. To avoid pincode conflict, change GD32L233Kx-QFN32 pincode to 'Q'.
