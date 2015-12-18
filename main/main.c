#include "main.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#include "event_queue.h"

#include "math.h"

#include "stm32f4xx_exti.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_crc.h"

#include "stm32f4_discovery_lis302dl.h"
#include "LIS3DSH.h"


__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

volatile unsigned long sysTime;
uint16_t adc_data[4];

const unsigned char* versionString = "IMGe Device Version T8.4r2 AxisFix FPU DSH Hash: ";
const unsigned char* hashString = GIT_HASH;

const unsigned char* idString = "IMGe Device ID: ";

unsigned char buttonIntEnabled = 0;

unsigned char accelerometerType = 0;

uint8_t LIS_Buffer[6];

// The buffer to communicate all the commands
uint8_t commandBuffer[128];
uint8_t extendCommand = 0;
uint8_t extendCommandReady = 0;

uint16_t slen;
uint8_t sbuf[64];

// ----------------------------------------------------------------------------
// Audio stuff
#define SPI_SCK_PIN                       GPIO_Pin_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_SOURCE                    GPIO_PinSource10
#define SPI_SCK_AF                        GPIO_AF_SPI2

#define SPI_MOSI_PIN                      GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_SOURCE                   GPIO_PinSource3
#define SPI_MOSI_AF                       GPIO_AF_SPI2

/* Audio recording frequency in Hz */
#define REC_FREQ                          8000  

/* PDM buffer input size */
// #define INTERNAL_BUFF_SIZE      128

/* PCM buffer output size */
#define PCM_OUT_SIZE            32

// copy register to pdmSampleBuffer on I2S interrupt
// 128 bits = 64 register contents = 16 samples @ decimation 64 = 1 ms @ 16000sps
uint16_t pdmSampleBuffer[INTERNAL_BUFF_SIZE];
uint8_t  pdmSampleBufferPos = 0;

uint16_t pcmSampleBuffer0[PCM_OUT_SIZE];
uint16_t pcmSampleBuffer1[PCM_OUT_SIZE];
uint8_t  pcmSampleBuffer0Pos = 0;
uint8_t  pcmSampleBuffer1Pos = 0;
uint8_t  DataStatus = 0;

PDMFilter_InitStruct Filter;

// small helper function, since sscanf, atoi and strtoi all seem to crash.. :-(
static uint8_t* myatoi (const uint8_t* strptr, uint32_t* res) {
  *res = 0;

  if (!(*strptr))
    return strptr;

  // skip spaces at the beginning
  while (*strptr && isspace(*strptr))
    strptr++;

  while (*strptr && isdigit(*strptr))
    *res = (*res<<3)+(*res<<1)+(*strptr++ - '0');

  return strptr;
}



void enableButonInt() {

  // enable interrupts for buttons

  buttonIntEnabled = 1;

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource7);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource8);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource9);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);

  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_InitStructure.EXTI_Line = EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9 | EXTI_Line10 | EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);  

  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void initAcclerometerLowLevel() {
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(LIS302DL_SPI_CLK, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(LIS302DL_SPI_SCK_GPIO_CLK | LIS302DL_SPI_MISO_GPIO_CLK | LIS302DL_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHB1PeriphClockCmd(LIS302DL_SPI_CS_GPIO_CLK, ENABLE);

  /* Enable INT1 GPIO clock */
  RCC_AHB1PeriphClockCmd(LIS302DL_SPI_INT1_GPIO_CLK, ENABLE);

  /* Enable INT2 GPIO clock */
  RCC_AHB1PeriphClockCmd(LIS302DL_SPI_INT2_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(LIS302DL_SPI_SCK_GPIO_PORT, LIS302DL_SPI_SCK_SOURCE, LIS302DL_SPI_SCK_AF);
  GPIO_PinAFConfig(LIS302DL_SPI_MISO_GPIO_PORT, LIS302DL_SPI_MISO_SOURCE, LIS302DL_SPI_MISO_AF);
  GPIO_PinAFConfig(LIS302DL_SPI_MOSI_GPIO_PORT, LIS302DL_SPI_MOSI_SOURCE, LIS302DL_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_SCK_PIN;
  GPIO_Init(LIS302DL_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  LIS302DL_SPI_MOSI_PIN;
  GPIO_Init(LIS302DL_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_MISO_PIN;
  GPIO_Init(LIS302DL_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(LIS302DL_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(LIS302DL_SPI, &SPI_InitStructure);

  /* Enable SPI1  */
  SPI_Cmd(LIS302DL_SPI, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LIS302DL_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(LIS302DL_SPI_CS_GPIO_PORT, LIS302DL_SPI_CS_PIN);

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LIS302DL_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LIS302DL_SPI_INT2_PIN;
  GPIO_Init(LIS302DL_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}


void initLIS302DL() {
  LIS302DL_InitTypeDef LIS302DL_InitStruct;
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;

  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);

  // LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
}

void initLIS3DSH() {
  LIS3DSH_Init();
  // LIS3DSH_Write(LIS3DSH_Reg_Ctrl_3, 0x00);
  LIS3DSH_Write(LIS3DSH_Reg_Ctrl_4, 0x67); // 100Hz, BDU=0, *EN=1
  // LIS3DSH_Write(LIS3DSH_Reg_Ctrl_5, 0x80); // 200Hz filter, 2g Full scale, no self test, 4wire SPI
  // LIS3DSH_Write(LIS3DSH_Reg_Ctrl_6, 0x00);
}

void initAcclerometer() {
  // Init basic pins and SPI (same for LIS302DL and LIS3DSH)
  initAcclerometerLowLevel();

  unsigned char accel_id;
  LIS302DL_Read(&accel_id, 0x0f, 1);

  if (accel_id == 0x3B) {
    // MB997B board, old LIS302DL accelerometer
    accelerometerType = 1;
    initLIS302DL();
  } else if (accel_id == 0x3F) {
    // MB997C board, new LIS3DSH accelerometer
    accelerometerType = 2;
    initLIS3DSH();
  } else {
    // unknown type
    accelerometerType = 0xff;
  }
}

void initMic(void) {
  CRC_ResetDR();
  Filter.LP_HZ = 8000.0f;
  Filter.HP_HZ = 10.0f;
  Filter.Fs = 16000;
  Filter.Out_MicChannels = 1;
  Filter.In_MicChannels = 1;

  PDM_Filter_Init(&Filter);
 
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* SPI SCK pin configuration */
  /* Connect SPI pins to AF5 */    
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);  
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  
  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);
  
  I2S_InitTypeDef I2S_InitStructure;
  
  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = 32000;
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);
  
  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
  I2S_Cmd(SPI2, ENABLE);
}
 
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  // Enable FPU
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */

  RCC_ClocksTypeDef RCC_Clocks;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

  /* Enable RNG Clock */
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

  /* Enable Syscfg clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Enable ADC3 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  /* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  /* Enable TIM3 clock for PWM */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable CRC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
  /* Enable SPI2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

  /* Init PD6 ... PD11 as input */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* configure PB1 as PWM (motor) */ 
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // connect PB1 to TIM3 Channel 4
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

  /* configure PE9, PE11, PE13, PE14 as out (LEDs) */ 
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  // Init PA1, PA2, PA3 and PC1 as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0f;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  DMA_DeInit(DMA2_Stream0);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_data;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, DISABLE);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_DeInit();
  ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // ??
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 4;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_Init(ADC3, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  1, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_2,  2, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_3,  3, ADC_SampleTime_28Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 4, ADC_SampleTime_28Cycles);

  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_Cmd(ADC3, ENABLE);

  ADC_SoftwareStartConv(ADC3);



  /* Configure TIM3 for PWM */

  uint16_t PrescalerValue = (uint16_t)((SystemCoreClock/2)/28000000)-1;

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  // TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Period = 1000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);

  // Init RNG
  RNG_Cmd(ENABLE);


  // Init Accelerometer
  initAcclerometer();

  // Init Microphone
  initMic();


  sysTime = 0;

  /* SysTick end of count event each .1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  // SysTick_Config(RCC_Clocks.HCLK_Frequency / 10000);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  /* Turn OFF all LEDs */
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED3);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED6);

  /* USB configuration */
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);

  unsigned long lastTime = sysTime;
  while (1)
  {
    if (commandBuffer[0] != 0x00) {
      // we got some stuff to do..
      handleCommand();
    }
  }
}

void handleGPIO() {
 slen = snprintf(sbuf, 64, "%08X\r\n", GPIOD->IDR);
  sendString(sbuf, slen);
}

void handleInterrupt(uint8_t sendTimestamp) {
  if (!buttonIntEnabled)
    enableButonInt();

  if (event_queue_is_empty()) {
    sendString("None\r\n",6);
  } else {
    struct event_entry* e;
    e = event_queue_pop_first();
    char c = e->key;
    unsigned long ts = e->timestamp;
    event_queue_free_event(e);

    if (sendTimestamp) {
      slen = snprintf(sbuf, 64, "E: %c %08lX\r\n", c, ts);
    } else {
      slen = snprintf(sbuf, 64, "E: %c\r\n", c);
    }
    sendString(sbuf, slen);
  }
}

void handleADC() {
  slen = snprintf(sbuf, 64, "A: %04X %04X %04X %04X\r\n", adc_data[0], adc_data[1], adc_data[2], adc_data[3]);
  sendString(sbuf, slen);
}

void handleOutputTest() {
  TIM3->CCR4 = 1000 - TIM3->CCR4;
  GPIOE->ODR ^= GPIO_Pin_9;
  GPIOE->ODR ^= GPIO_Pin_11;
  GPIOE->ODR ^= GPIO_Pin_13;
  GPIOE->ODR ^= GPIO_Pin_14;
  sendString("Ok\r\n",4);
}

void handleVersion() {
  sendString(versionString,strlen(versionString));
  sendString(hashString,strlen(hashString));
  sendString("\r\n", 2);
}

void handleID() {
  if (accelerometerType == 0xff) {
    initAcclerometer();
  }

  uint32_t id1, id2, id3;

  id1 = *((uint32_t*)(0x1fff7a10+0x00));
  id2 = *((uint32_t*)(0x1fff7a10+0x04));
  id3 = *((uint32_t*)(0x1fff7a10+0x08));

  sendString(idString,strlen(idString));
  slen = snprintf(sbuf, 64, "%04X%04X%04X Type: %02X\r\n", id1, id2, id3, accelerometerType);
  sendString(sbuf, slen);

  /*
  slen = snprintf(sbuf, 64, "X: %f\r\n", LIS3DSH_Get_X_Out(1));
  sendString(sbuf, slen);

  unsigned char accel_id;
  LIS302DL_Read(&accel_id, 0x0f, 1);
  slen = snprintf(sbuf, 64, "Type2: %02X\r\n", accel_id);
  sendString(sbuf, slen);
  */
}


void handleMotor(uint32_t speed) {
  TIM3->CCR4 = speed;
  sendString("Done.\r\n",7);
}

void handleAccel() {
  if (accelerometerType == 0xff)
    initAcclerometer();

  if (accelerometerType == 1) {
    LIS302DL_Read(LIS_Buffer, LIS3DSH_Reg_X_Out_L, 6);
    slen = snprintf(sbuf, 64, "Accel: %02X %02X %02X\r\n",LIS_Buffer[1], LIS_Buffer[3], LIS_Buffer[5]);
    sendString(sbuf, slen);
  } else if (accelerometerType == 2) {
    // for full 16bit output
    /*
    unsigned short t_x, t_y, t_z;
    t_x = LIS3DSH_Read(LIS3DSH_Reg_X_Out_H);
    t_x = t_x << 8;
    t_x = t_x + LIS3DSH_Read(LIS3DSH_Reg_X_Out_L);
    t_y = LIS3DSH_Read(LIS3DSH_Reg_Y_Out_H);
    t_y = t_y << 8;
    t_y = t_y + LIS3DSH_Read(LIS3DSH_Reg_Y_Out_L);
    t_z = LIS3DSH_Read(LIS3DSH_Reg_Z_Out_H);
    t_z = t_z << 8;
    t_z = t_z + LIS3DSH_Read(LIS3DSH_Reg_Z_Out_L);

    slen = snprintf(sbuf, 64, "Accel: %04X %04X %04X\r\n", t_x, t_y, t_z);
    sendString(sbuf, slen);
    */

    unsigned char t_x, t_y, t_z;
    t_x = LIS3DSH_Read(LIS3DSH_Reg_X_Out_H);
    // t_x = t_x << 8;
    // t_x = t_x + LIS3DSH_Read(LIS3DSH_Reg_X_Out_L);
    t_y = LIS3DSH_Read(LIS3DSH_Reg_Y_Out_H);
    // t_y = t_y << 8;
    // t_y = t_y + LIS3DSH_Read(LIS3DSH_Reg_Y_Out_L);
    t_z = LIS3DSH_Read(LIS3DSH_Reg_Z_Out_H);
    // t_z = t_z << 8;
    // t_z = t_z + LIS3DSH_Read(LIS3DSH_Reg_Z_Out_L);

    slen = snprintf(sbuf, 64, "Accel: %02X %02X %02X\r\n", (char)(-t_y), (char)(t_x), (char)(t_z));
    sendString(sbuf, slen);

  } else {
    slen = snprintf(sbuf, 64, "Accel: type %02X not supported\r\n", accelerometerType);
    sendString(sbuf, slen);
  }
}

void handleMic() {
//  for (int i=0; i<PCM_OUT_SIZE; i++) {
//    slen = snprintf(sbuf, 64, "Sound: %03X: %04X\r\n", i, pcmSampleBuffer0[i]);
//    sendString(sbuf, slen);
//  }

  float res = 0.0;
  for (int i=0; i<PCM_OUT_SIZE; i++) {
    float tmp = (float)((int16_t)pcmSampleBuffer0[i]);
    tmp = tmp * tmp;
    res += tmp;
  }
  res /= (float)PCM_OUT_SIZE;
  if (res < 0.0) {
    res = 0.0;
  }
  res = sqrt(res);
  slen = snprintf(sbuf, 64, "RMS: %f\r\n", res);
  sendString(sbuf,slen);
}


void handleLeds(uint8_t ledNr, uint8_t ledCmd) {

  static uint8_t LEDS[4] = { 14, 13, 11, 9 };

  if (ledNr<=3) {
    switch (ledCmd) {
      case 0:
        GPIOE->BSRRH = (1<<LEDS[ledNr]);
        break;
      case 1:
        GPIOE->BSRRL = (1<<LEDS[ledNr]);
        break;
      case 2:
        GPIOE->ODR ^= (1<<LEDS[ledNr]);
        break;
      default: 
        break;
    }
  }
  sendString("Done.\r\n",7);
}

void handleCommand() {
  if (extendCommand && !extendCommandReady)
    return;

  if (commandBuffer[0] == '1') {
    handleGPIO();
  } else if (commandBuffer[0] == '2') {
    handleInterrupt(0);
  } else if (commandBuffer[0] == '3') {
    handleInterrupt(1);
  } else if (commandBuffer[0] == '4') {    
    handleADC();
  } else if (commandBuffer[0] == '5') {
    handleOutputTest();
  } else if (commandBuffer[0] == 'a') {
    handleAccel();
  } else if (commandBuffer[0] == 's') {
    handleMic();
  } else if (commandBuffer[0] == 'v') {
    handleVersion();
  } else if (commandBuffer[0] == 'i') {
    handleID();
  } else if (commandBuffer[0] == 'm') {
    static uint32_t speed;
    myatoi(commandBuffer+1, &speed);
    handleMotor(speed);
  } else if (commandBuffer[0] == 'l') {
    static uint32_t ledNr;
    static uint32_t ledCmd;
    uint8_t* commandBufferPtr = commandBuffer+1;
    commandBufferPtr = myatoi(commandBufferPtr, &ledNr);
    commandBufferPtr = myatoi(commandBufferPtr, &ledCmd);
    handleLeds(ledNr, ledCmd);
  }

  if (extendCommandReady) {
    extendCommand = 0;
    extendCommandReady = 0;
  }

  commandBuffer[0] = 0x00;
}

uint32_t LIS302DL_TIMEOUT_UserCallback(void) 
{ 
  while (1)
    ;
  return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

