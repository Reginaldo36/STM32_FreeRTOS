TOOLCHAIN    = arm-none-eabi-
CC           = $(TOOLCHAIN)gcc
CP           = $(TOOLCHAIN)objcopy
AS           = $(TOOLCHAIN)gcc -x assembler-with-cpp
HEX          = $(CP) -O ihex
BIN          = $(CP) -O binary -S

# define mcu, specify the target processor
MCU          = cortex-m3

# TARGET=OUT

# specify define
DDEFS       =

# define root dir
# ROOT_DIR     = .

# define include dir
INCLUDE_DIRS = $(ROOT_DIR)/user/

# define stm32f10x lib dir
STM32F10x_LIB_DIR      = $(ROOT_DIR)/stm32f10x_lib

# define freertos dir
FREERTOS_DIR = $(ROOT_DIR)/freertos

# define user dir
# USER_DIR     = $(ROOT_DIR)/user

# link file
LINK_SCRIPT  = $(ROOT_DIR)/stm32_flash.ld

# user specific
SRC       =
ASM_SRC   =
SRC      += $(shell ls $(USER_DIR)/*.c)

# user include
INCLUDE_DIRS  = $(USER_DIR)

# include sub makefiles
include $(ROOT_DIR)/makefile_std_lib.mk   # STM32 Standard Peripheral Library
include $(ROOT_DIR)/makefile_freertos.mk  # freertos source

INC_DIR  = $(patsubst %, -I%, $(INCLUDE_DIRS))

# run from Flash
DEFS	 = $(DDEFS) -DRUN_FROM_FLASH=1

OBJECTS  = $(ASM_SRC:.s=.o) $(SRC:.c=.o)

# Define optimisation level here
OPT = -Os

MC_FLAGS = -mcpu=$(MCU)

AS_FLAGS = $(MC_FLAGS) 
AS_FLAGS += -g 
AS_FLAGS += -gdwarf-2 
AS_FLAGS += -mthumb  
AS_FLAGS += -Wa,-amhls=$(<:.s=.lst)

CP_FLAGS = $(MC_FLAGS) $(OPT) 
CP_FLAGS += -g 
CP_FLAGS += -gdwarf-2 
CP_FLAGS += -mthumb 
CP_FLAGS += -fomit-frame-pointer 
CP_FLAGS += -Wall 
CP_FLAGS += -fverbose-asm 
CP_FLAGS += -Wa,-ahlms=$(<:.c=.lst) $(DEFS)

LD_FLAGS = $(MC_FLAGS) 
LD_FLAGS += -g 
LD_FLAGS += -gdwarf-2 
LD_FLAGS += -mthumb 
LD_FLAGS += -nostartfiles 
LD_FLAGS += -Xlinker 
LD_FLAGS += --gc-sections 
LD_FLAGS += -T$(LINK_SCRIPT) 
LD_FLAGS += -Wl,-Map=$(TARGET).map,--cref,--no-warn-mismatch

# makefile rules

all: $(OBJECTS) $(TARGET).elf  $(TARGET).hex $(TARGET).bin
	$(TOOLCHAIN)size $(TARGET).elf
	@du -sbc *

%.o: %.c
	$(CC) -c $(CP_FLAGS) -I . $(INC_DIR) $< -o $@

%.o: %.s
	$(AS) -c $(AS_FLAGS) $< -o $@

%.elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LD_FLAGS) -o $@

%.hex: %.elf
	$(HEX) $< $@

%.bin: %.elf
	$(BIN)  $< $@

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x8000000

erase:
	st-flash erase

clean:
	-rm -rf $(OBJECTS)
	-rm -rf $(TARGET).elf
	-rm -rf $(TARGET).map
	-rm -rf $(TARGET).hex
	-rm -rf $(TARGET).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(ASM_SRC:.s=.lst)

