PRJ_NAME   = main
CC         = arm-none-eabi-gcc
SRCDIR     = src
BUILDDIR   = build
SRC        = $(wildcard $(SRCDIR)/*.c)
ASRC       = $(wildcard $(SRCDIR)/*.s)
OBJ        = $(patsubst $(SRCDIR)/%.c,$(BUILDDIR)/%.o,$(SRC)) $(patsubst $(SRCDIR)/%.s,$(BUILDDIR)/%.o,$(ASRC))
OBJCOPY    = arm-none-eabi-objcopy
OBJDUMP    = arm-none-eabi-objdump

DEVICE     = STM32F103xB
OPT        ?= -Os
LDSCRIPT   = stm32f103rbt6.ld
CFLAGS     = -fdata-sections -ffunction-sections -g3 -Wall -mcpu=cortex-m3 -mlittle-endian -mfloat-abi=soft -mthumb  -I inc/ -D $(DEVICE) $(OPT)
ASFLAGS    = $(CFLAGS)
LDFLAGS    = -T $(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs

.PHONY: all clean flash burn hex bin

all: $(BUILDDIR)/$(PRJ_NAME).elf

$(BUILDDIR)/$(PRJ_NAME).elf: $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS)
	arm-none-eabi-size $@

$(BUILDDIR)/%.o: $(SRCDIR)/%.c $(DEPS)
	$(CC) -MMD -c $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: $(SRCDIR)/%.s $(DEPS)
	$(CC) -MMD -c $(ASFLAGS) $< -o $@

-include $(wildcard $(BUILDDIR)/*.d)

clean:
	rm -f $(OBJ) $(BUILDDIR)/$(PRJ_NAME).elf $(PRJ_NAME).hex $(PRJ_NAME).bin $(wildcard $(BUILDDIR)/*.d)

