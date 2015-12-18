/*!
 * \author no1wudi
 * \file LIS3DSH_HW.h
 */

#ifndef __LIS3DSH_HW_H
#define __LIS3DSH_HW_H

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"

#define CS_H GPIO_SetBits(GPIOE,GPIO_Pin_3)
#define CS_L GPIO_ResetBits(GPIOE,GPIO_Pin_3)

#define LIS3DSH_Write_Mask 0x00

#define LIS3DSH_Read_Mask 0x80

#define LIS3DSH_Sense_2g 0.06

#define LIS3DSH_Sense_4g 0.12

#define LIS3DSH_Sense_6g 0.18

#define LIS3DSH_Sense_8g 0.24

#define LIS3DSH_Sense_16g 0.73


void LIS3DSH_Init_SPI_Bus(void);

unsigned char LIS3DSH_SPI_Read_Write(unsigned char LIS3DSH_Reg);

#endif
