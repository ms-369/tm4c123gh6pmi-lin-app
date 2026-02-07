CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
OBJCPY = arm-none-eabi-objcopy

CFLAGS = -g -mcpu=cortex-m4 -mthumb -nostdlib -specs=nosys.specs -I inc
ASFLAGS = -g -mcpu=cortex-m4 -mthumb

SRC_DIR = src
PLATFORM_DIR = platform
OBJ_DIR = obj
BUILD_DIR = build

SRC_FILES = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(PLATFORM_DIR)/*.c)

OBJ = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(filter $(SRC_DIR)/%.c, $(SRC_FILES))) \
      $(patsubst $(PLATFORM_DIR)/%.c, $(OBJ_DIR)/%.o, $(filter $(PLATFORM_DIR)/%.c, $(SRC_FILES)))

STARTUP = $(OBJ_DIR)/startup.o
ELF = $(BUILD_DIR)/final.elf
BIN = $(BUILD_DIR)/final.bin
HEX = $(BUILD_DIR)/final.hex
MAP = $(BUILD_DIR)/final.map

all: clean build flash

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: $(PLATFORM_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

build: $(OBJ) $(STARTUP)
	$(CC) $(CFLAGS) $(OBJ) $(STARTUP) -T platform/linker.ld -Wl,-Map,$(MAP) -o $(ELF)
	$(OBJCPY) -O binary $(ELF) $(BIN)
	$(OBJCPY) -O ihex $(ELF) $(HEX)
	@echo "Successfully built project!"

$(STARTUP): platform/startup.s
	$(AS) $(ASFLAGS) $< -o $@

flash:
	lmflash -e all -v -r $(BIN)

clean:
	rm -rf $(OBJ_DIR)/*.o $(BUILD_DIR)/*

debug:
	arm-none-eabi-gdb $(ELF)

.PHONY: all clean build flash debug
