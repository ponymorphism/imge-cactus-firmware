BINARY = main
LDSCRIPT = ../stm32f4-discovery.ld

PREFIX	?= arm-none-eabi

CC		= $(PREFIX)-gcc
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
GDB		= $(PREFIX)-gdb

STDLIB_INCLUDES = -I../CMSIS/Device/ST/STM32F4xx/Include/ -I../CMSIS/Include/ -I../STM32F4xx_StdPeriph_Driver/inc/
STDLIB_SRCDIR   = ../STM32F4xx_StdPeriph_Driver/src
STDLIB_OBJS     = $(STDLIB_SRCDIR)/stm32f4xx_gpio.o $(STDLIB_SRCDIR)/stm32f4xx_rcc.o $(STDLIB_SRCDIR)/stm32f4xx_exti.o $(STDLIB_SRCDIR)/stm32f4xx_rng.o $(STDLIB_SRCDIR)/misc.o \
                  $(STDLIB_SRCDIR)/stm32f4xx_syscfg.o $(STDLIB_SRCDIR)/stm32f4xx_adc.o $(STDLIB_SRCDIR)/stm32f4xx_dma.o $(STDLIB_SRCDIR)/stm32f4xx_tim.o $(STDLIB_SRCDIR)/stm32f4xx_spi.o \
                  $(STDLIB_SRCDIR)/stm32f4xx_crc.o

USB_DRIVER_INC  = -I../STM32_USB_Device_Library/Core/inc/
USB_DRIVER_SRC  = ../STM32_USB_Device_Library/Core/src
USB_DRIVER_OBJS = $(USB_DRIVER_SRC)/usbd_core.o $(USB_DRIVER_SRC)/usbd_req.o $(USB_DRIVER_SRC)/usbd_ioreq.o

USB_CLASS_INC   = -ICDC/inc
USB_CLASS_SRC   = CDC/src
USB_CLASS_OBJS  = $(USB_CLASS_SRC)/usbd_cdc_core.o

USB_OTG_INC     = -I../STM32_USB_OTG_Driver/inc/
USB_OTG_SRC     = ../STM32_USB_OTG_Driver/src
USB_OTG_OBJS    = $(USB_OTG_SRC)/usb_dcd.o $(USB_OTG_SRC)/usb_core.o $(USB_OTG_SRC)/usb_dcd_int.o

UTIL_INCLUDES   = -I../Utilities/STM32F4-Discovery/
UTIL_SRCDIR     = ../Utilities/STM32F4-Discovery/
UTIL_OBJS       = $(UTIL_SRCDIR)/stm32f4_discovery.o $(UTIL_SRCDIR)/stm32f4_discovery_lis302dl.o $(UTIL_SRCDIR)/libPDMFilter_GCC.a

LIS3DSH_INCLUDES = -ILIS3DSH
LIS3DSH_SRC      = LIS3DSH
LIS3DSH_OBJS     = $(LIS3DSH_SRC)/LIS3DSH.o $(LIS3DSH_SRC)/LIS3DSH_HW.o

CFLAGS		+= -Wall -Wextra -I. $(STDLIB_INCLUDES) $(UTIL_INCLUDES) $(USB_CLASS_INC) $(USB_DRIVER_INC) $(USB_OTG_INC) $(LIS3DSH_INCLUDES)\
               -DUSE_STDPERIPH_DRIVER -DUSE_STM32_DISCOVERY -DHSE_VALUE=8000000 -DUSB_OTG_FS_CORE -DUSE_USB_OTG_FS -DSTM32F4XX\
               -fno-common -MD -std=c99 -O0 -mthumb -DGIT_HASH=\"`git rev-parse HEAD`\" \
               -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fno-strict-aliasing -ffunction-sections


LDSCRIPT	?= $(BINARY).ld
LDFLAGS		+= -mfix-cortex-m3-ldrd -Wl,-Map,main.map -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 \
               -lm -lc -lnosys -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections
OBJS		+= $(BINARY).o event_queue.o startup_stm32f4xx.o system_stm32f4xx.o stm32f4xx_it.o usb_bsp.o usbd_usr.o usbd_desc.o usbd_cdc.o\
			   $(UTIL_OBJS) $(STDLIB_OBJS) $(USB_CLASS_OBJS) $(USB_DRIVER_OBJS) $(USB_OTG_OBJS) $(LIS3DSH_OBJS)

.SUFFIXES: .elf .bin .hex .srec .list .images
.SECONDEXPANSION:
.SECONDARY:

all: images

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.hex %.srec %.list
	@#echo "*** $* images generated ***"

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@#printf "  OBJCOPY $(*).srec\n"
	$(Q)$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@#printf "  OBJDUMP $(*).list\n"
	$(Q)$(OBJDUMP) -S $(*).elf > $(*).list

%.elf: $(OBJS) $(LDSCRIPT)
	@#printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) -o $(*).elf $(OBJS) $(LDFLAGS)

%.o: %.c Makefile
	@#printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.S Makefile
	@#printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<


clean:
	$(Q)rm -f *.o
	$(Q)rm -f *.d
	$(Q)rm -f *.elf
	$(Q)rm -f *.bin
	$(Q)rm -f *.hex
	$(Q)rm -f *.srec
	$(Q)rm -f *.list
	$(Q)rm -f *.map

%.flash: %.bin
	st-flash write $< 0x8000000

.PHONY: images clean

#-include $(OBJS:.*=.d)

