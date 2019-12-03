TARGET = stm32f103_sx126x

################################
# GNU ARM Embedded Toolchain
#################################
CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
AS = $(CROSS_COMPILE)as
CP = $(CROSS_COMPILE)objcopy
OD = $(CROSS_COMPILE)objdump
NM = $(CROSS_COMPILE)nm
SIZE = $(CROSS_COMPILE)size
A2L = $(CROSS_COMPILE)addr2line


################################
# Working directories
################################
TOP = $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
#TOP=$(shell readlink -f "$(dir $(lastword $(MAKEFILE_LIST)))")
APP_DIR = $(TOP)/app
HARDWARE_DIR = $(TOP)/hardware
LIB_DIR = $(TOP)/libs
CMSIS_DIR = $(LIB_DIR)/stdlib/CMSIS
STD_DIR = $(LIB_DIR)/stdlib/STM32F10x_StdPeriph_Driver
RTOS_DIR = $(LIB_DIR)/contiki
BUILD_DIR = $(TOP)/build
OBJ_DIR = $(BUILD_DIR)/obj


###############################
# Source Files
###############################
ASRC = $(wildcard $(TOP)/startup/*.s)
VPATH := $(TOP)/startup

CMSIS_SRC = $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
	    $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c)
VPATH := $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x

STD_SRC = $(wildcard $(STD_DIR)/src/*.c)
VPATH := $(VPATH):$(STD_DIR)/src

RTOS_SRC = $(RTOS_DIR)/core/sys/autostart.c \
	   $(RTOS_DIR)/core/sys/etimer.c \
	   $(RTOS_DIR)/core/sys/process.c \
	   $(RTOS_DIR)/core/sys/timer.c \
	   $(RTOS_DIR)/cpu/clock.c
VPATH := $(VPATH):$(RTOS_DIR)/core/sys:$(RTOS_DIR)/cpu

APP_SRC = $(wildcard $(APP_DIR)/*.c \
	  $(APP_DIR)/lora/*.c)
VPATH := $(VPATH):$(APP_DIR):$(APP_DIR)/lora

HARDWARE_SRC = $(wildcard $(HARDWARE_DIR)/*.c)
VPATH := $(VPATH):$(HARDWARE_DIR)

CSRC = $(CMSIS_SRC) $(STD_SRC) $(HARDWARE_SRC) $(RTOS_SRC) $(APP_SRC)

INCLUDE = . \
	  $(APP_DIR) \
	  $(APP_DIR)/lora \
	  $(HARDWARE_DIR) \
	  $(STD_DIR)/inc\
	  $(CMSIS_DIR)/CM3/CoreSupport \
	  $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \
	  $(RTOS_DIR)/core \
	  $(RTOS_DIR)/core/sys \
	  $(RTOS_DIR)/cpu


###############################
# Object list
###############################
OBJS = $(addsuffix .o, $(addprefix $(OBJ_DIR)/, $(basename $(notdir $(ASRC)))))
OBJS += $(addsuffix .o, $(addprefix $(OBJ_DIR)/, $(basename $(notdir $(CSRC)))))


##############################
# Target Output File
#############################
TARGET_ELF = $(BUILD_DIR)/$(TARGET).elf
TARGET_HEX = $(BUILD_DIR)/$(TARGET).hex
TARGET_BIN = $(BUILD_DIR)/$(TARGET).bin


#############################
# Flags
############################
MCFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
OPTIMIZE = -O1
DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD
CFLAGS = -fdata-sections -ffunction-sections -fmessage-length=0 -MMD -g3 -Wall $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(addprefix -I, $(INCLUDE)) -std=c99 -std=gnu11
ASFLAGS = $(CFLAGS)

LDSCRIPT = $(TOP)/stm32f103c8tx.ld
LDFLAGS = -T $(LDSCRIPT) $(MCFLAGS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -lm -specs=nano.specs -u _printf_float


################################
# Build
################################
all: $(TARGET_HEX) $(TARGET_BIN)

$(TARGET_HEX): $(TARGET_ELF)
	@echo "Create hex file..." $(notdir $@)
	$(CP) -O ihex --set-start 0x08000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	@echo "Generating bin file..." $(notdir $@)
	$(CP) -O binary -S $< $@

$(TARGET_ELF): $(OBJS)
	@echo "Build ELF file..." $(notdir $@)
	$(CC) -o $@ $^  $(LDFLAGS)
	$(SIZE) $@


$(OBJ_DIR)/%.o: %.c
	mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	$(CC) -c -o $@ $(CFLAGS) $<

$(OBJ_DIR)/%.o: %.s
	mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	$(CC) -c -o $@ $(ASFLAGS) $<


.PHONY: all flash probe clean erase
	
flash:
	sudo st-flash --reset write $(TARGET_BIN) 0x08000000

probe:
	sudo st-info --probe

erase:
	sudo st-flash erase

clean:
	rm -f $(OBJS) $(TARGET_ELF) $(TARGET_HEX) $(TARGET_BIN)
