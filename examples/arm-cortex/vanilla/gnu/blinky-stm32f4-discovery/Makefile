##############################################################################
# Product: Makefile for Blinky, EK-LM4F120XL, Vanilla kernel, GNU/Sourcery
# Last Updated for Version: 4.5.04
# Date of the Last Update:  Feb 07, 2013
#
#                    Q u a n t u m     L e a P s
#                    ---------------------------
#                    innovating embedded systems
#
# Copyright (C) 2002-2013 Quantum Leaps, LLC. All rights reserved.
#
# This program is open source software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as published
# by the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# Alternatively, this program may be distributed and modified under the
# terms of Quantum Leaps commercial licenses, which expressly supersede
# the GNU General Public License and are specifically designed for
# licensees interested in retaining the proprietary status of their code.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Contact information:
# Quantum Leaps Web sites: http://www.quantum-leaps.com
#                          http://www.state-machine.com
# e-mail:                  info@quantum-leaps.com
##############################################################################
# examples of invoking this Makefile:
# building configurations: Debug (default), Release, and Spy
# make
# make CONF=rel
# make CONF=spy
#
# cleaning configurations: Debug (default), Release, and Spy
# make clean
# make CONF=rel clean
# make CONF=spy clean

#-----------------------------------------------------------------------------
# GNU ARM compiler
#
ifeq ($(GNU_ARM),)
GNU_ARM = /opt/arm/toolchain
endif

CC    := $(GNU_ARM)/bin/arm-none-eabi-gcc
CPP   := $(GNU_ARM)/bin/arm-none-eabi-g++
AS    := $(GNU_ARM)/bin/arm-none-eabi-as
LINK  := $(GNU_ARM)/bin/arm-none-eabi-gcc
BIN   := $(GNU_ARM)/bin/arm-none-eabi-objcopy
SIZE  := $(GNU_ARM)/bin/arm-none-eabi-size
RM    := rm -rf
MKDIR := mkdir

#-----------------------------------------------------------------------------
# NOTE: the Makefile expects that the QPC environment variable is defined
# and points to the QP/C installation directory
#
ifndef QPC
$(error The QPC environment variable must be defined)
endif

#-----------------------------------------------------------------------------
# directories
#
QP_PORT_DIR := $(QPC)/ports/arm-cortex/vanilla/gnu
STM32F4_LIB := ../../../../../../STM32F4-Discovery_FW_V1.1.0
CMSIS_DIR   := $(STM32F4_LIB)/Libraries/CMSIS
DRIVER_DIR  := $(STM32F4_LIB)/Libraries/STM32F4xx_StdPeriph_Driver
BOARD_DIR   := $(STM32F4_LIB)/Utilities/STM32F4-Discovery
APP_DIR     := .

# source directories
VPATH = $(APP_DIR) \
	$(DRIVER_DIR)/src \
	$(CMSIS_DIR)/ST/STM32F4xx/Source/Templates/gcc_ride7

# Output file basename
OUTPUT    := blinky-vanilla

# include directories
INCLUDES  = -I$(QPC)/include \
	-I$(QP_PORT_DIR) \
	-I. \
        -I$(CMSIS_DIR)/Include \
        -I$(CMSIS_DIR)/ST/STM32F4xx/Include \
	-I$(BOARD_DIR) \
        -I$(DRIVER_DIR)/inc

# defines
DEFINES = -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx

ARM_CORE = cortex-m4
#ARM_FPU  = fpv4-sp-d16
ARM_FPU  = vfp

#-----------------------------------------------------------------------------
# files
#

# assembler source files
ASM_SRCS := startup_stm32f4xx.s

# C source files
C_SRCS := $(wildcard *.c) stm32f4xx_gpio.c stm32f4xx_rcc.c

# C++ source files
CPP_SRCS := $(wildcard *.cpp)

LD_SCRIPT := stm32f4xx_flash.ld
LIBS      := -lqp_$(ARM_CORE)f_cs

#-----------------------------------------------------------------------------
# build options for various configurations
#

### Build flags
#
# -fdata-sections, -ffunction-sections and -Wl,--gc-sections -Wl,-s
# are used for dead code elimination, see:
# http://gcc.gnu.org/ml/gcc-help/2003-08/msg00128.html
#

ifeq (rel, $(CONF))       # Release configuration ............................

BIN_DIR := rel

ASFLAGS = -mcpu=$(ARM_CORE) -defsym=FPU_VFP_V4_SP_D16=1 \
	-mlittle-endian -mthumb -mthumb-interwork

CFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wall -Wextra -Warray-bounds \
	-fdata-sections -ffunction-sections \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG

CPPFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wall -Wextra -Warray-bounds -fno-rtti -fno-exceptions \
	-fdata-sections -ffunction-sections \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG

LINKFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wl,--gc-sections \
	-T$(LD_SCRIPT) -Xlinker -Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR)

else ifeq (spy, $(CONF))  # Spy configuration ................................

BIN_DIR := spy

ASFLAGS = -mcpu=$(ARM_CORE) -defsym=FPU_VFP_V4_SP_D16=1 \
	-mlittle-endian -mthumb -mthumb-interwork

CFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wall -Wextra -Warray-bounds \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES) -DQ_SPY

CPPFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wall -Wextra -Warray-bounds -fno-rtti -fno-exceptions \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES) -DQ_SPY

LINKFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wl,--gc-sections \
	-T$(LD_SCRIPT) -Xlinker -Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR)

else                     # default Debug configuration .......................

BIN_DIR := dbg

ASFLAGS = -mcpu=$(ARM_CORE) -defsym=FPU_VFP_V4_SP_D16=1 \
	-mlittle-endian -mthumb -mthumb-interwork -g

# -mfloat-abi=hard
CFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wall -Wextra -Warray-bounds \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES)

CPPFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wall -Wextra -Warray-bounds -fno-rtti -fno-exceptions \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES)

LINKFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) \
	-mlittle-endian -mthumb -mthumb-interwork \
	-Wl,--gc-sections \
	-T$(LD_SCRIPT) -Xlinker -Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR)

endif


ASM_OBJS     := $(patsubst %.s,%.o,$(ASM_SRCS))
C_OBJS       := $(patsubst %.c,%.o,$(C_SRCS))
CPP_OBJS     := $(patsubst %.cpp,%.o,$(CPP_SRCS))

TARGET_BIN   := $(BIN_DIR)/$(OUTPUT).bin
TARGET_HEX   := $(BIN_DIR)/$(OUTPUT).hex
TARGET_ELF   := $(BIN_DIR)/$(OUTPUT).elf
ASM_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(ASM_OBJS))
C_OBJS_EXT   := $(addprefix $(BIN_DIR)/, $(C_OBJS))
C_DEPS_EXT   := $(patsubst %.o, %.d, $(C_OBJS_EXT))
CPP_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(CPP_OBJS))
CPP_DEPS_EXT := $(patsubst %.o, %.d, $(CPP_OBJS_EXT))


#-----------------------------------------------------------------------------
# rules
#

all: $(BIN_DIR) $(TARGET_BIN) $(TARGET_HEX)

$(BIN_DIR):
	@echo
	mkdir -p $@

$(TARGET_BIN): $(TARGET_ELF)
	$(BIN) -O binary $< $@

$(TARGET_HEX): $(TARGET_ELF)
	$(BIN) -O ihex $< $@

$(TARGET_ELF) : $(ASM_OBJS_EXT) $(C_OBJS_EXT) $(CPP_OBJS_EXT)
	$(LINK) $(LINKFLAGS) -o $@ $^ $(LIBS)

$(BIN_DIR)/%.d : %.c
	$(CC) -MM -MT $(@:.d=.o) $(CFLAGS) $< > $@

$(BIN_DIR)/%.d : %.cpp
	$(CPP) -MM -MT $(@:.d=.o) $(CPPFLAGS) $< > $@

$(BIN_DIR)/%.o : %.s
	$(AS) $(ASFLAGS) $< -o $@

$(BIN_DIR)/%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

-include $(C_DEPS_EXT) $(CPP_DEPS_EXT)


.PHONY : clean
clean:
	-$(RM) $(BIN_DIR)/*.o \
	$(BIN_DIR)/*.d \
	$(BIN_DIR)/*.hex \
	$(BIN_DIR)/*.elf \
	$(BIN_DIR)/*.map
	
show:
	@echo CONF = $(CONF)
	@echo ASM_SRCS = $(ASM_SRCS)
	@echo C_SRCS = $(C_SRCS)
	@echo CPP_SRCS = $(CPP_SRCS)
	@echo ASM_OBJS_EXT = $(ASM_OBJS_EXT)
	@echo C_OBJS_EXT = $(C_OBJS_EXT)
	@echo C_DEPS_EXT = $(C_DEPS_EXT)
	@echo CPP_DEPS_EXT = $(CPP_DEPS_EXT)
	@echo TARGET_ELF = $(TARGET_ELF)

flash:
	st-flash write $(BIN_DIR)/$(OUTPUT).bin 0x08000000

size: $(BIN_DIR)/$(OUTPUT).elf
	$(SIZE) $(BIN_DIR)/$(OUTPUT).elf
