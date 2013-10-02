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
ifeq ($(GNU_MSP430),)
GNU_MSP430 = /opt/msp430
endif

CC    := $(GNU_MSP430)/bin/msp430-gcc
CPP   := $(GNU_MSP430)/bin/msp430-g++
AS    := $(GNU_MSP430)/bin/msp430-as
LINK  := $(GNU_MSP430)/bin/msp430-gcc
BIN   := $(GNU_MSP430)/bin/msp430-objcopy
SIZE  := $(GNU_MSP430)/bin/msp430-size
RM    := rm -rf
MKDIR := mkdir

#-----------------------------------------------------------------------------
# directories
#
QP_PORT_DIR := $(QPC)/ports/msp430/vanilla/gnu
APP_DIR     := .
LIB_DIR     :=

# source directories
VPATH = $(APP_DIR)

# Output file basename
OUTPUT    := blinky-vanilla

# include directories
INCLUDES  = -I$(QPC)/include	\
	-I$(QP_PORT_DIR) \
	-I. \

# defines
DEFINES = 

MSP430_CORE = 430
MSP430_MCU = msp430g2553

#-----------------------------------------------------------------------------
# files
#

# assembler source files
ASM_SRCS :=

# C source files
C_SRCS := $(wildcard *.c)

# C++ source files
CPP_SRCS :=	$(wildcard *.cpp)

LIBS      := -lqp

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

ASFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE)

CFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) -Wall \
	-fdata-sections -ffunction-sections \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG

CPPFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) \
	-Wall -fno-rtti -fno-exceptions \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG

LINKFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) \
	-Wl,--gc-sections \
	-Xlinker -Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR) \
	-L$(GNU_MSP430)/msp430/lib/ldscripts/$(MSP430_MCU)

else ifeq (spy, $(CONF))  # Spy configuration ................................

BIN_DIR := spy

ASFLAGS = -g -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE)

CFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) -Wall \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES) -DQ_SPY

CPPFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) \
	-Wall -fno-rtti -fno-exceptions \
	-g -O $(INCLUDES) $(DEFINES) -DQ_SPY

LINKFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) \
	-Wl,--gc-sections \
	-Xlinker -Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR) \
	-L$(GNU_MSP430)/msp430/lib/ldscripts/$(MSP430_MCU)

else                     # default Debug configuration .......................

BIN_DIR := dbg

ASFLAGS = -g -mcpu=$(MSP430_CORE)

CFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) -Wall \
	-fdata-sections -ffunction-sections \
	-g -O $(INCLUDES) $(DEFINES)

CPPFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) \
	-Wall -fno-rtti -fno-exceptions \
	-g -O $(INCLUDES) $(DEFINES)

LINKFLAGS = -mmcu=$(MSP430_MCU) -mcpu=$(MSP430_CORE) \
	-Wl,--gc-sections \
	-Xlinker -Map=$(BIN_DIR)/$(OUTPUT).map \
	-L$(QP_PORT_DIR)/$(BIN_DIR) \
	-L$(GNU_MSP430)/msp430/lib/ldscripts/$(MSP430_MCU)

endif


ASM_OBJS     := $(patsubst %.s,%.o,$(ASM_SRCS))
C_OBJS       := $(patsubst %.c,%.o,$(C_SRCS))
CPP_OBJS     := $(patsubst %.cpp,%.o,$(CPP_SRCS))

TARGET_BIN   := $(BIN_DIR)/$(OUTPUT).hex
TARGET_ELF   := $(BIN_DIR)/$(OUTPUT).elf
ASM_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(ASM_OBJS))
C_OBJS_EXT   := $(addprefix $(BIN_DIR)/, $(C_OBJS))
C_DEPS_EXT   := $(patsubst %.o, %.d, $(C_OBJS_EXT))
CPP_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(CPP_OBJS))
CPP_DEPS_EXT := $(patsubst %.o, %.d, $(CPP_OBJS_EXT))


#-----------------------------------------------------------------------------
# rules
#

all: $(BIN_DIR) $(TARGET_BIN)

$(BIN_DIR):
	@echo
	mkdir -p $@

$(TARGET_BIN): $(TARGET_ELF)
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
	mspdebug rf2500 "prog $(BIN_DIR)/$(OUTPUT).elf"

size: $(BIN_DIR)/$(OUTPUT).elf
	$(SIZE) $(BIN_DIR)/$(OUTPUT).elf
