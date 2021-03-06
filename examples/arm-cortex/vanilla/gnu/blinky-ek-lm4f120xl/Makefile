##############################################################################
# Product: Makefile for DPP, EK-LM4F120XL, Vanilla kernel, GNU/Sourcery
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
# NOTE: the Makefile expects that the QPC environment variable is defined
# and points to the QP/C installation directory
#
ifndef QPC
$(error The QPC environment variable must be defined)
endif

#-----------------------------------------------------------------------------
# general utilities
#
RM = del

#-----------------------------------------------------------------------------
# tools
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
# directories
#
QP_PORT_DIR := $(QPC)/ports/arm-cortex/vanilla/gnu
APP_DIR     := .
LM4F_DIR    := ../../../../../../SW-EK-LM4F120XL-9453/
DRIVER_DIR  := $(LM4F_DIR)/driverlib
LIB_DIR     := 

# source directories
VPATH = $(APP_DIR) \
	$(APP_DIR)/runtime

# Output file basename
OUTPUT    := blinky

# include directories
INCLUDES  = -I$(APP_DIR) \
	-I$(DRIVER_DIR) \
	-I$(LM4F_DIR) \
	-I$(QPC)/include \
	-I$(QP_PORT_DIR)

COMPILER = gcc
ARM_CORE = cortex-m4
ARM_FPU = fpv4-sp-d16
MFLAGS = -mcpu=$(ARM_CORE) -mfpu=$(ARM_FPU) -mfloat-abi=softfp \
	-mlittle-endian -mthumb -mthumb-interwork

# defines
DEFINES = -DPART_LM4F120H5QR -D$(COMPILER) -DTARGET_IS_BLIZZARD_RA1 -D__FPU_PRESENT

#-----------------------------------------------------------------------------
# files
#

# assembler source files
ASM_SRCS :=

# C source files
C_SRCS := $(wildcard *.c) \
	startup_lm4f.c

# C++ source files
CPP_SRCS :=	$(wildcard *.cpp)

LD_SCRIPT := $(APP_DIR)/runtime/lm4f120h5qr.ld
LIBS      := -lqp_$(ARM_CORE)-$(ARM_FPU)

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

ASFLAGS = -mcpu=$(ARM_CORE) -defsym=FPU_VFP_V4_SP_D16=1

CFLAGS = $(MFLAGS) -Wall \
	-fdata-sections -ffunction-sections \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG

CPPFLAGS = $(MFLAGS) \
	-fdata-sections -ffunction-sections \
	-Wall -fno-rtti -fno-exceptions \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG

LINKFLAGS = $(MFLAGS) \
	-Wl,--gc-sections \
	-T$(LD_SCRIPT) -nostdlib -Xlinker \
	-Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR)

else ifeq (spy, $(CONF))  # Spy configuration ................................

BIN_DIR := spy

ASFLAGS = -g -mcpu=$(ARM_CORE) -defsym=FPU_VFP_V4_SP_D16=1

CFLAGS = $(MFLAGS) -Wall \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES) -DQ_SPY

CPPFLAGS = $(MFLAGS)b \
	-fdata-sections -ffunction-sections \
	-Wall -fno-rtti -fno-exceptions \
	-g -O $(INCLUDES) $(DEFINES) -DQ_SPY

LINKFLAGS = $(MFLAGS) \
	-Wl,--gc-sections \
	-T$(LD_SCRIPT) -nostdlib -Xlinker \
	-Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR)

else                     # default Debug configuration .......................

BIN_DIR := dbg

ASFLAGS = -g -mcpu=$(ARM_CORE) -defsym=FPU_VFP_V4_SP_D16=1

CFLAGS = $(MFLAGS) -Wall \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES)

CPPFLAGS = $(MFLAGS) \
	-fdata-sections -ffunction-sections \
	-Wall -fno-rtti -fno-exceptions \
	-g -O $(INCLUDES) $(DEFINES)

LINKFLAGS = $(MFLAGS) \
	-Wl,--gc-sections \
	-T$(LD_SCRIPT) -nostdlib -Xlinker \
	-Map=$(BIN_DIR)/$(OUTPUT).map \
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

flash: $(TARGET_BIN)
	lm4flash $(TARGET_BIN)

size: $(TARGET_ELF)
	$(SIZE) $(TARGET_ELF)
