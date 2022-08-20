# hal_gigadevice lib refactor

This folder contains `hal_gigadevice` lib refactor scripts.

## Usage

1. Download `GD32xxxx Firmware Library` from [gigadevice](http://www.gd32mcu.com/en/download/7)
2. make folders name like `gd32c10x` at root of `hal_gigadevice` and decompress file to this folder.

files in folder should like this:
``` shell
gd32c10x/
├── Examples
├── Firmware
├── GD32C10x_Firmware_Library_V1.0.2
├── Template
└── Utilities
```

3. use folder as parameter to execute `scripts/lib_refactor/make_new_hal.sh`

## Example

``` shell
wget -O /tmp/gd32c10x.rar http://www.gd32mcu.com/download/down/document_id/216/path_type/1
mkdir gd32c10x
unrar x /tmp/gd32c10x.rar gd32c10x
mv gd32c10x/*/* gd32c10x/
scripts/lib_refactor/make_new_hal.sh gd32c10x
```
