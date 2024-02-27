/*
 * at21cs01.c for Raspberry Pi PICO
 *
 *  Created on: February 24, 2024
 *      Author: jbyrneiu
 *  Based on: https://github.com/KOTzulla/stm32_at21cs01
 *	(Originally for STM32F4 MCU)
 */

#include "at21cs01.h"

uint8_t at21cs01_slaveAddress = 0;
uint8_t DATA_PAGE[] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120};

uint8_t CommSpeed = 0;
uint8_t res = 0;

uint8_t calc_crc8(uint8_t *data, uint8_t len)
{
  uint8_t i;
  uint8_t j;
  uint8_t temp;
  uint8_t databyte;
  uint8_t crc = 0;
  for (i = 0; i < len; i++)
  {
    databyte = data[i];
    for (j = 0; j < 8; j++)
    {
      temp = (crc ^ databyte) & 0x01;
      crc >>= 1;
      if (temp)
        crc ^= 0x8C;
      databyte >>= 1;
    }
  }
  return crc;
}

void __not_in_flash_func(at21cs01_delay_us)(uint16_t us)
{
  sleep_us(us);
}
void __not_in_flash_func(at21cs01_delay_ms)(uint16_t ms)
{
  sleep_ms(ms);
}

void __not_in_flash_func(at21cs01_SIO_SetHigh)()
{
  gpio_set_dir(GPIO_EPROM, GPIO_IN);
}

void __not_in_flash_func(at21cs01_SIO_SetLow)()
{
  gpio_set_dir(GPIO_EPROM, GPIO_OUT);
}

uint8_t __not_in_flash_func(at21cs01_SIO_GetValue)()
{
  res = 0;

  if (gpio_get(GPIO_EPROM))
  {
    res = 1;
  }
  return res;
}

uint8_t __not_in_flash_func(at21cs01_ackNack)()
{

  uint8_t temp;

  if (CommSpeed == 0)
  {
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(1);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(1);
    if (at21cs01_SIO_GetValue() == 0)
    {
      temp = 0x00;
    }
    else
    {
      temp = 0xFF;
    }
    at21cs01_delay_us(9);
    at21cs01_SIO_SetHigh();
  }
  else
  {
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(4);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(2);
    if (at21cs01_SIO_GetValue() == 0)
    {
      temp = 0x00;
    }
    else
    {
      temp = 0xFF;
    }
    at21cs01_delay_us(34);
    at21cs01_SIO_SetHigh();
  }

  return temp;
}

uint8_t __not_in_flash_func(at21cs01_discoveryResponse)()
{
  uint8_t temp;

  at21cs01_SIO_SetHigh();
  at21cs01_delay_us(200);
  at21cs01_SIO_SetLow();
  at21cs01_delay_us(500);
  at21cs01_SIO_SetHigh();
  at21cs01_delay_us(20);

  at21cs01_SIO_SetLow();
  at21cs01_delay_us(1);
  at21cs01_SIO_SetHigh();
  at21cs01_delay_us(2);
  if (at21cs01_SIO_GetValue() == 0)
  {
    temp = 0x00;
  }
  else
  {
    temp = 0xFF;
  }
  at21cs01_delay_us(21);
  at21cs01_SIO_SetHigh();

  CommSpeed = 0;
  return temp;
}

void __not_in_flash_func(at21cs01_startHS)()
{
  at21cs01_SIO_SetHigh();
  if (CommSpeed == 0)
  {
    at21cs01_delay_us(200);
  }
  else
  {
    at21cs01_delay_us(650);
  }
}

void __not_in_flash_func(at21cs01_tx1)()
{
  if (CommSpeed == 0)
  {
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(1);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(14);
  }
  else
  {
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(4);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(41);
  }
}

void __not_in_flash_func(at21cs01_tx0)()
{
  if (CommSpeed == 0)
  {
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(10);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(5);
  }
  else
  {
    at21cs01_SIO_SetLow();
    at21cs01_delay_us(24);
    at21cs01_SIO_SetHigh();
    at21cs01_delay_us(21);
  }
}

uint8_t at21cs01_txByte(uint8_t dataByte)
{
  uint8_t temp;

  for (uint8_t ii = 0; ii < 8; ii++)
  {
    if (0x80 & dataByte)
    {
      at21cs01_tx1();
    }
    else
    {
      at21cs01_tx0();
    }
    dataByte <<= 1;
  }
  temp = at21cs01_ackNack();
  return temp;
}


uint8_t __not_in_flash_func(at21cs01_readByte)()
{
  uint8_t temp;
  uint8_t dataByte = 0x00;

  if (CommSpeed == 0)
  {
    for (int8_t ii = 0; ii < 8; ii++)
    {
      at21cs01_SIO_SetLow();
      at21cs01_delay_us(1);
      at21cs01_SIO_SetHigh();
      at21cs01_delay_us(1);
      temp = at21cs01_SIO_GetValue();
      temp &= 0x01;
      dataByte = (uint8_t)((dataByte << 1) | temp);
      at21cs01_delay_us(9);
    }
  }
  else
  {
    for (int8_t ii = 0; ii < 8; ii++)
    {
      at21cs01_SIO_SetLow();
      at21cs01_delay_us(4);
      at21cs01_SIO_SetHigh();
      at21cs01_delay_us(2);
      temp = at21cs01_SIO_GetValue();
      temp &= 0x01;
      dataByte = (uint8_t)((dataByte << 1) | temp);
      at21cs01_delay_us(34);
    }
  }
  return dataByte;
}

uint8_t at21cs01_scanDeviceAddress()
{
  uint8_t temp;
  uint8_t address = 0;

  for (uint8_t ii = 0; ii < 8; ii++)
  {
    if (at21cs01_discoveryResponse() == 0)
    {
      at21cs01_startHS();
      temp = ii;
      temp <<= 1;
      temp |= 0xA0;
      if (at21cs01_txByte(temp) == 0x00)
      {
        address = ii;
        address <<= 1;
        at21cs01_startHS();
        break;
      }
    }
  }
  at21cs01_slaveAddress = address;
  return address;
}

uint8_t at21cs01_eepromwrite(uint8_t cmd, uint8_t address, uint8_t Data[], uint8_t size)
{

  if (size > 8)
    return 1;
  cmd |= at21cs01_slaveAddress;

  at21cs01_startHS();
  at21cs01_txByte(cmd);
  at21cs01_txByte(address);
  for (uint8_t ii = 0; ii < size; ii++)
  {
    at21cs01_txByte(Data[ii]);
  }
  at21cs01_startHS();
  at21cs01_delay_ms(2);
  return 0;
}

uint8_t at21cs01_WriteToAddress(uint8_t address, uint8_t Data[], uint8_t size)
{
  return at21cs01_eepromwrite(AT21CS01_CMD_EEPROMREADWRITE, address, Data, size);
}

void at21cs01_ReadFromAddress(uint8_t address, uint8_t readData[], uint8_t size)
{
  uint8_t cmd = AT21CS01_CMD_EEPROMREADWRITE;
  cmd |= at21cs01_slaveAddress;

  at21cs01_startHS();
  at21cs01_txByte(cmd);
  at21cs01_txByte(address);
  at21cs01_startHS();
  at21cs01_txByte((uint8_t)(cmd | 0x01));
  for (uint8_t ii = 0; ii < size; ii++)
  {
    readData[ii] = at21cs01_readByte();
    if (ii < (size - 1))
    {
      at21cs01_tx0();
    }
  }
  at21cs01_tx1();
  at21cs01_startHS();
}

void at21cs01_ReadSerialNumber(uint8_t address, uint8_t *readData, uint8_t size)
{
  uint8_t cmd = AT21CS01_CMD_EEPROMREADSECURITY;
  cmd |= at21cs01_slaveAddress;

  at21cs01_startHS();
  at21cs01_txByte(cmd);
  at21cs01_txByte(address);
  at21cs01_startHS();
  at21cs01_txByte((uint8_t)(cmd | 0x01));
  for (uint8_t ii = 0; ii < size; ii++)
  {
    readData[ii] = at21cs01_readByte();
    if (ii < (size - 1))
    {
      at21cs01_tx0();
    }
  }
  at21cs01_tx1();
  at21cs01_startHS();
}

void at21cs01_currentRead(uint8_t cmd, uint8_t *readData, uint8_t size)
{
  cmd |= at21cs01_slaveAddress;

  at21cs01_startHS();
  at21cs01_txByte((uint8_t)(cmd | 0x01));
  for (uint8_t ii = 0; ii < size; ii++)
  {
    readData[ii] = at21cs01_readByte();
    if (ii < (size - 1))
    {
      at21cs01_tx0();
    }
  }
  at21cs01_tx1();
  at21cs01_startHS();
}

uint8_t at21cs01_setCommuncationSpeed(uint8_t speed)
{
  uint8_t temp;
  speed |= at21cs01_slaveAddress;

  at21cs01_startHS();
  temp = at21cs01_txByte((uint8_t)(speed));
  at21cs01_startHS();
  if (((speed & 0xF0) == AT21CS01_SPEED_SLOW) & (temp == 0x00))
  {
    CommSpeed = 1;
  }
  else
  {
    CommSpeed = 0;
  }
  return temp;
}

uint8_t at21cs01_checkCommuncationSpeed(uint8_t speed)
{
  uint8_t temp;
  speed |= at21cs01_slaveAddress;

  at21cs01_startHS();
  temp = at21cs01_txByte((uint8_t)(speed | 0x01));
  at21cs01_startHS();

  if (temp == 0x00)
  {
    if ((speed & 0xF0) == AT21CS01_SPEED_SLOW)
    {
      printf("    Standard Speed ACK\n");
    }
    if ((speed & 0xF0) == AT21CS01_SPEED_FAST)
    {
      printf("    High-Speed ACK\n");
    }
  }
  else
  {
    if ((speed & 0xF0) == AT21CS01_SPEED_SLOW)
    {
      printf("    Standard Speed NACK\n");
    }
    if ((speed & 0xF0) == AT21CS01_SPEED_FAST)
    {
      printf("    High-Speed NACK\n");
    }
  }
  return temp;
}

void at21cs01_FillWholeMemory(uint8_t byte)
{
  uint8_t data[128];
  memset(data, byte, 128);
  uint8_t addr = 0x00;
  for (uint8_t z = 0; z < 16; z++)
  {
    at21cs01_eepromwrite(AT21CS01_CMD_EEPROMREADWRITE, addr, data, 8);
    addr += 8;
  }
}

void at21cs01_Connect()
{
  at21cs01_slaveAddress = at21cs01_scanDeviceAddress();
  at21cs01_setCommuncationSpeed(AT21CS01_SPEED_FAST);
}

void Error_Handler()
{
  while (1)
  {
  }
}

//
//  MAIN PROGRAM
//

int main(void)
{
  uint8_t DataOut[32];
  uint8_t DataIn1[32];
  uint8_t DataIn2[32];
  uint8_t SerialNumber[8];

  gpio_init(GPIO_EPROM);
  gpio_disable_pulls(GPIO_EPROM);
  gpio_put(GPIO_EPROM, LOW);
  gpio_set_dir(GPIO_EPROM, GPIO_OUT);

  strcpy(DataOut, "TESTING");
  memset(DataIn1, 0, 32);
  memset(SerialNumber, 0, 8);

  at21cs01_Connect();

  at21cs01_WriteToAddress(DATA_PAGE[0], DataOut, 8);
  at21cs01_ReadFromAddress(DATA_PAGE[0], DataIn1, 8);

  strcpy(DataOut, "TEST#2..");
  memset(DataIn2, 0, 32);
  memset(SerialNumber, 0, 8);

  at21cs01_WriteToAddress(DATA_PAGE[3], DataOut, 8);
  at21cs01_ReadFromAddress(DATA_PAGE[3], DataIn2, 8);

  at21cs01_ReadSerialNumber(DATA_PAGE[0], SerialNumber, 8);
  uint8_t crc8_check = calc_crc8(SerialNumber, 7) - SerialNumber[7];

  printf("CRC8: %0x\n", crc8_check);

  /* Infinite loop */
  while (1)
  {
  }
}
