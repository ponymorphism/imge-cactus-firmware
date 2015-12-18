#ifndef __STM32F4_DISCOVERY_DEMO_H
#define __STM32F4_DISCOVERY_DEMO_H

#include "stm32f4_discovery.h"
#include <stdio.h>

#include "pdm_filter.h"

extern volatile unsigned long sysTime;
extern const unsigned char* versionString;
extern const unsigned char* hashString;
extern unsigned char buttonIntEnabled;
extern uint16_t adc_data[4];

extern uint8_t commandBuffer[];
extern uint8_t extendCommand;
extern uint8_t extendCommandReady;

extern uint16_t pdmSampleBuffer[];
extern uint8_t  pdmSampleBufferPos;

extern uint16_t pcmSampleBuffer0[];
extern uint16_t pcmSampleBuffer1[];
extern uint8_t  pcmSampleBuffer0Pos;
extern uint8_t  pcmSampleBuffer1Pos;
extern uint8_t  DataStatus;

extern PDMFilter_InitStruct Filter;



/* TIM2 Autoreload and Capture Compare register values */
#define TIM_ARR                          (uint16_t)1999
#define TIM_CCR                          (uint16_t)1000

/* MEMS Microphone SPI Interface */
#define SPI_SCK_PIN                   GPIO_Pin_10
#define SPI_SCK_GPIO_PORT             GPIOB
#define SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                GPIO_PinSource10
#define SPI_SCK_AF                    GPIO_AF_SPI2

#define SPI_MOSI_PIN                  GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT            GPIOC
#define SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE               GPIO_PinSource3
#define SPI_MOSI_AF                   GPIO_AF_SPI2

#define ABS(x)         (x < 0) ? (-x) : x
#define MAX(a,b)       (a < b) ? (b) : a

#define INTERNAL_BUFF_SIZE      128


void TimingDelay_Decrement(void);

extern void enableButonInt();

void Delay(__IO uint32_t nTime);
void Fail_Handler(void);

extern uint32_t LIS302DL_TIMEOUT_UserCallback(void);

#endif
