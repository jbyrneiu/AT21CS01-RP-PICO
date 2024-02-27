
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define GPIO_EPROM 17
#define LOW 0
#define HIGH 1
#define OUTPUT true
#define INPUT false


#define AT21CS01_SPEED_SLOW 0xD0
#define AT21CS01_SPEED_FAST 0xE0

#define AT21CS01_CMD_EEPROMREADWRITE 0xA0
#define AT21CS01_CMD_EEPROMREADSECURITY 0xB0

void at21cs01_Connect(void);
void at21cs01_FillWholeMemory(uint8_t byte);
void at21cs01_ReadFromAddress(uint8_t address, uint8_t readData[], uint8_t size);
uint8_t at21cs01_WriteToAddress(uint8_t address, uint8_t Data[], uint8_t size);
void Error_Handler(void);
