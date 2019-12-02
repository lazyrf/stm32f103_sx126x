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

APP_SRC = $(wildcard $(APP_DIR)/*.c)
VPATH := $(VPATH):$(APP_DIR)

HARDWARE_SRC = $(wildcard $(HARDWARE_DIR)/*.c)
VPATH := $(VPATH):$(HARDWARE_DIR)

CSRC = $(CMSIS_SRC) $(STD_SRC) $(HARDWARE_SRC) $(APP_SRC)

INCLUDE = . \
	  $(APP_DIR) \
	  $(HARDWARE_DIR) \
	  $(STD_DIR)/inc\
	  $(CMSIS_DIR)/CM3/CoreSupport \
	  $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x


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
MCFLAGS = -mcpu=cortex-m3 -mthumb
OPTIMIZE = -Os
DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD
CFLAGS = -fdata-sections -ffunction-sections -g -Wall $(MCFLAGS) $(DEFS) $(OPTIMIZE) $(addprefix -I, $(INCLUDE)) -std=c99 --specs=nano.specs
ASFLAGS = $(CFLAGS)

LDSCRIPT = $(TOP)/stm32f103c8tx.ld
LDFLAGS = -T $(LDSCRIPT) -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs


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
	$(CC) -c -o $@ $(CFLAGS) $<


.PHONY: all flash probe clean
	
flash:
	sudo st-flash --reset write $(TARGET_BIN) 0x08000000

probe:
	sudo st-info --probe

clean:
	rm -f $(OBJS) $(TARGET_ELF) $(TARGET_HEX) $(TARGET_BIN)
