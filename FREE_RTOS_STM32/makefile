PRJ_NAME   := main
CC         := arm-none-eabi-gcc
OBJCOPY    := arm-none-eabi-objcopy
OBJDUMP    := arm-none-eabi-objdump
DEVICE     := STM32F103xB
OPT        ?= -Os
LDSCRIPT   := stm32f103rbt6.ld
CFLAGS     := -fdata-sections -ffunction-sections -g3 -Wall -mcpu=cortex-m3 -mlittle-endian -mfloat-abi=soft -mthumb -D$(DEVICE) $(OPT)
ASFLAGS    := $(CFLAGS)
LDFLAGS    := -T$(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs


BUILD_DIR := build
SRC_DIR   := src


RTOS_DIR  := freeRTOS
RTOS_SRC  := $(wildcard $(RTOS_DIR)/Source/*.c)
PORT_SRC  := $(wildcard $(RTOS_DIR)/Source/portable/GCC/ARM_CM3/*.c)
MEM_SRC  := $(wildcard $(RTOS_DIR)/Source/portable/MemMang/*.c)

STARTUP   := $(SRC_DIR)/startup_stm32f103rbtx.s
SYSTEM    := $(SRC_DIR)/system_stm32f1xx.c

ASRC      := $(wildcard $(SRC_DIR)/*.s)
CSRCS     := $(wildcard $(SRC_DIR)/*.c)

CFLAGS += \
    -I./inc \
    -I./$(RTOS_DIR)/Source/include \
    -I./$(RTOS_DIR)/Source/portable/GCC/ARM_CM3 

VPATH := \
    $(SRC_DIR) \
    $(RTOS_DIR)/Source \
    $(RTOS_DIR)/Source/portable/MemMang \
    $(RTOS_DIR)/Source/portable/GCC/ARM_CM3 

OBJS := \
    $(patsubst $(SRC_DIR)/%.s,$(BUILD_DIR)/%.o,$(ASRC)) \
    $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(CSRCS)) \
    $(patsubst $(RTOS_DIR)/Source/%.c,$(BUILD_DIR)/%.o,$(RTOS_SRC)) \
    $(patsubst $(RTOS_DIR)/Source/portable/GCC/ARM_CM3/%.c,$(BUILD_DIR)/%.o,$(PORT_SRC)) \
    $(patsubst $(RTOS_DIR)/Source/portable/MemMang/%.c,$(BUILD_DIR)/%.o,$(MEM_SRC))

all: $(BUILD_DIR)/$(PRJ_NAME).elf

$(BUILD_DIR)/$(PRJ_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)
	arm-none-eabi-size $@

$(BUILD_DIR)/%.o: %.c
	$(CC) -MMD -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s
	$(CC) -MMD -c $(ASFLAGS) $< -o $@

-include $(wildcard $(BUILD_DIR)/*.d)

clean:
	rm -f $(OBJS) $(BUILD_DIR)/$(PRJ_NAME).elf $(wildcard $(BUILD_DIR)/*.d)

