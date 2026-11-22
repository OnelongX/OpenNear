CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size
MASTER  ?= 0

CFLAGS  := -std=c11 -Os -Wall -Wextra -ffunction-sections -fdata-sections \
           -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
           -I../../core/include -I../../hal -Iinclude -DOPENNEAR_IS_MASTER=$(MASTER)
LDFLAGS := -T nrf52840.ld -nostartfiles -Wl,--gc-sections -lm -lc -lnosys

SRCS := \
    ../../core/src/on_frame.c \
    ../../core/src/on_sync.c \
    ../../core/src/on_superframe.c \
    ../../core/src/on_sec.c \
    ../../core/src/on_afh.c \
    ../../core/src/on_amc.c \
    ../../core/src/on_sar.c \
    ../../core/src/on_node.c \
    src/startup_nrf52840.c \
    src/hal_nrf52_radio_ppi.c \
    src/main_nrf52.c

OBJS := $(SRCS:.c=.o)

all: opennear_nrf52.hex

opennear_nrf52.elf: $(OBJS) nrf52840.ld
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

opennear_nrf52.hex: opennear_nrf52.elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -f $(OBJS) opennear_nrf52.elf opennear_nrf52.hex
