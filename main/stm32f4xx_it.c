#include "stm32f4xx_it.h"
#include "main.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc.h"

#include "event_queue.h"

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  // advance system time
  sysTime += 1;   
}

void EXTI9_5_IRQHandler(void)
{
  int clearLines = 0;
  unsigned int reg = GPIOD->IDR;

  unsigned long ts = sysTime;

  if (EXTI_GetITStatus(EXTI_Line6) == SET) {
    struct event_entry* e = event_queue_new_event();
    if ((reg & GPIO_Pin_6) != 0) {
      e->key = 'A';
    } else {
      e->key = 'a';
    }
    e->timestamp = ts;
    event_queue_insert(e);
    clearLines |= EXTI_Line6;
  }
  if (EXTI_GetITStatus(EXTI_Line7) == SET) {
    struct event_entry* e = event_queue_new_event();
    if ((reg & GPIO_Pin_7) != 0) {
      e->key = 'B';
    } else {
      e->key = 'b';
    }
    e->timestamp = ts;
    event_queue_insert(e);
    clearLines |= EXTI_Line7;
  }
  if (EXTI_GetITStatus(EXTI_Line8) == SET) {
    struct event_entry* e = event_queue_new_event();
    if ((reg & GPIO_Pin_8) != 0) {
      e->key = 'C';
    } else {
      e->key = 'c';
    }
    e->timestamp = ts;
    event_queue_insert(e);
    clearLines |= EXTI_Line8;
  }
  if (EXTI_GetITStatus(EXTI_Line9) == SET) {
    struct event_entry* e = event_queue_new_event();
    if ((reg & GPIO_Pin_9) != 0) {
      e->key = 'D';
    } else {
      e->key = 'd';
    }
    e->timestamp = ts;
    event_queue_insert(e);
    clearLines |= EXTI_Line9;
  }
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(clearLines);
}


void EXTI15_10_IRQHandler(void)
{
  int clearLines = 0;
  unsigned int reg = GPIOD->IDR;

  unsigned long ts = sysTime;

  if (EXTI_GetITStatus(EXTI_Line10) == SET) {
    struct event_entry* e = event_queue_new_event();
    if ((reg & GPIO_Pin_10) != 0) {
      e->key = 'E';
    } else {
      e->key = 'e';
    }
    e->timestamp = ts;
    event_queue_insert(e);
    clearLines |= EXTI_Line10;
  }
  if (EXTI_GetITStatus(EXTI_Line11) == SET) {
    struct event_entry* e = event_queue_new_event();
    if ((reg & GPIO_Pin_11) != 0) {
      e->key = 'F';
    } else {
      e->key = 'f';
    }
    e->timestamp = ts;
    event_queue_insert(e);
    clearLines |= EXTI_Line11;
  }
  /* Clear the EXTI line pending bit */
  EXTI_ClearITPendingBit(clearLines);
}


void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    /* Reset SLEEPDEEP and SLEEPONEXIT bits */
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk)); 
    /* After wake-up from sleep mode, reconfigure the system clock */
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void SPI2_IRQHandler(void)
{
  uint16_t volume;
  uint16_t app;

  /* Check if data are available in SPI Data register */
  if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
  {
    app = SPI_I2S_ReceiveData(SPI2);
    pdmSampleBuffer[pdmSampleBufferPos++] = HTONS(app);

    // Check to prevent overflow condition 
    if (pdmSampleBufferPos >= INTERNAL_BUFF_SIZE)
    {
      pdmSampleBufferPos = 0;
      volume = 20;
      PDM_Filter_64_LSB((uint8_t *)pdmSampleBuffer, (uint16_t *)pcmSampleBuffer0, volume , (PDMFilter_InitStruct *)&Filter);
      PDM_Filter_64_LSB((uint8_t *)pdmSampleBuffer+64, (uint16_t *)pcmSampleBuffer0+16, volume, (PDMFilter_InitStruct *)&Filter);
      DataStatus = 1;       
    }
  }
}
