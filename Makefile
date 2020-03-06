##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16 -std=gnu99 -D_GNU_SOURCE
  USE_OPT += -DBOARD_OTG_NOVBUSSENS $(build_args)
  USE_OPT += -fsingle-precision-constant -Wdouble-promotion
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x200
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = BLDC_4_ChibiOS

# Imported source files and paths
CHIBIOS = ChibiOS
ST_LL_DRIVER = st_drivers/STM32F4xx_LL_Driver
# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files
include $(CHIBIOS)/os/nil/nil.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include hwconf/hwconf.mk
include applications/applications.mk
include st_drivers/STM32F4xx_LL_Driver/stm32ll.mk
include Legacy/stlegacy.mk

# Define linker script file here
LDSCRIPT= ld_eeprom_emu.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       board.c \
       main.c \
       irq_handlers.c \
       buffer.c \
       crc.c \
       digital_filter.c \
       ledpwm.c \
       mcpwm.c \
       utils.c \
       packet.c \
       terminal.c \
       conf_general.c \
       eeprom.c \
       commands.c \
       timeout.c \
       encoder.c \
       flash_helper.c \
       mc_interface.c \
       mcpwm_foc.c \
       confgenerator.c \
       timer.c \
       stdperiph_stm32f4/src/stm32f4xx_flash.c \
       $(HWSRC) \
       $(APPSRC) \
       $(STM32LLSRC) \
       $(STLEGACYSRC)

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# List ASM source files here
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(ALLINC) \
         $(CHIBIOS)/os/various \
         mcconf \
         appconf \
         stdperiph_stm32f4/inc \
         $(HWINC) \
         $(APPINC) \
         $(NRFINC) \
         $(CANARDINC) \
         $(IMUINC) \
         $(COMPRESSIONINC) \
         $(BLACKMAGICINC) \
         $(STM32LLINC) \
         $(STLEGACYINC)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes -Wshadow

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DUSE_FULL_LL_DRIVER -DSTM32F405xx

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################

build/$(PROJECT).bin: build/$(PROJECT).elf 
	$(BIN) build/$(PROJECT).elf build/$(PROJECT).bin --gap-fill 0xFF

# Program
upload: build/$(PROJECT).bin
#	qstlink2 --cli --erase --write build/$(PROJECT).bin
#	openocd -f interface/stlink-v2.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x_stlink.cfg -c "program build/$(PROJECT).elf verify reset" # Older openocd
	openocd -f board/stm32f4discovery.cfg -c "reset_config trst_only combined" -c "program build/$(PROJECT).elf verify reset exit" # For openocd 0.9

clear_option_bytes:
	openocd -f board/stm32f4discovery.cfg -c "init" -c "stm32f2x unlock 0" -c "mww 0x40023C08 0x08192A3B; mww 0x40023C08 0x4C5D6E7F; mww 0x40023C14 0x0fffaaed" -c "exit"

#program with olimex arm-usb-tiny-h and jtag-swd adapter board. needs openocd>=0.9
upload-olimex: build/$(PROJECT).bin
	openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/olimex-arm-jtag-swd.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x.cfg -c "program build/$(PROJECT).elf verify reset"

upload-pi: build/$(PROJECT).bin
	openocd -f pi_stm32.cfg -c "reset_config trst_only combined" -c "program build/$(PROJECT).elf verify reset exit"

upload-pi-remote: build/$(PROJECT).elf
	./upload_remote_pi build/$(PROJECT).elf ted 10.42.0.199 22

debug-start:
	openocd -f stm32-bv_openocd.cfg

