MicroChip AT21CS01 Library for Raspberry Pi PICO

This project contains a library of functions to read and write to a MicroChip AT21CS01 EPROM chip. It uses CMake and contains configuration files to using Microsoft Visual Code toolchain and the Raspberry PI PICO-SDK.

A sample main() function is included to show how to include functions within an existing project. The eight-byte data banks are accessed using the 8-byte offset values contained in the DATA_PAGE array. The factory burned Serial Number can also be read if the application needs to have an immutable serial number.

This work is based on an original Github repository by KOTzulla. It is based on STM32F4 and can be located at: https://github.com/KOTzulla/stm32_at21cs01
Thanks to KOTzulla for his help in teaching me the nature of open drain GPIO!

Some of functions are set with the attribute "__not_in_flash_func" to move a function to SRAM prior to execution. This is needed due to the very tight timing requirements of the AT21CS01 device. In testing, the first bit of the first byte read was dropped when the ReadByte function ran from Flash. Moving to SRAM resolved this issue.

