#******************************************************************************
#
# Makefile - Rules for building the driver library and examples.
#
# Copyright (c) 2005,2006 Luminary Micro, Inc.  All rights reserved.
#
# Software License Agreement
#
# Luminary Micro, Inc. (LMI) is supplying this software for use solely and
# exclusively on LMI's Stellaris Family of microcontroller products.
#
# The software is owned by LMI and/or its suppliers, and is protected under
# applicable copyright laws.  All rights are reserved.  Any use in violation
# of the foregoing restrictions may subject the user to criminal sanctions
# under applicable laws, as well as to civil liability for the breach of the
# terms and conditions of this license.
#
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
# OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
# LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
#
#******************************************************************************

include makedefs

RTOS_SOURCE_DIR=Source

CFLAGS+=-I hw_include -I . -I ${RTOS_SOURCE_DIR}/include -I ${RTOS_SOURCE_DIR}/portable/GCC/ARM_CM3 -I ../Common/include -D GCC_ARMCM3_LM3S102 -D inline=

VPATH=${RTOS_SOURCE_DIR}:${RTOS_SOURCE_DIR}/portable/MemMang:${RTOS_SOURCE_DIR}/portable/GCC/ARM_CM3:init:hw_include

OBJS=${COMPILER}/lab7.o	\
	  ${COMPILER}/list.o    \
      ${COMPILER}/queue.o   \
      ${COMPILER}/tasks.o   \
      ${COMPILER}/port.o    \
      ${COMPILER}/heap_1.o  \
	  ${COMPILER}/osram96x16.o

INIT_OBJS= ${COMPILER}/startup.o

LIBS= hw_include/libdriver.a


#
# The default rule, which causes init to be built.
#
all: ${COMPILER}           \
     ${COMPILER}/LAB7.axf \
	 
#
# The rule to clean out all the build products
#

clean:
	@rm -rf ${COMPILER} ${wildcard *.bin} LAB7.axf
	
#
# The rule to create the target directory
#
${COMPILER}:
	@mkdir ${COMPILER}

${COMPILER}/LAB7.axf: ${INIT_OBJS} ${OBJS} ${LIBS}
SCATTER_LAB7=standalone.ld
ENTRY_LAB7=ResetISR

#
#
# Include the automatically generated dependency files.
#
-include ${wildcard ${COMPILER}/*.d} __dummy__


	 



