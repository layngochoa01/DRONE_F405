# Makefile - BLUEBERRY F405 bare-metal
# Ubuntu: sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

TARGET = blueberryf405

BUILD_DIR = build

C_SRCS = \
    src/main.c \
    src/sensors/icm42605.c \
	src/sensors/hc05.c	\
	src/drivers/spi.c \
	src/drivers/uart5.c	\
	src/drivers/timer.c \
	src/drivers/flash_storage.c \
	src/syscalls.c	\
	src/estimation/imu.c \
	src/estimation/filter.c \
	src/estimation/attitude.c

C_INCLUDES = \
    -Iinclude \
    -Isrc/drivers \
	-Isrc/sensors \
	-Isrc/estimation

AS_SRCS = startup.s

OBJS = $(patsubst src/%.c, $(BUILD_DIR)/%.o, $(C_SRCS)) \
       $(patsubst %.s,     $(BUILD_DIR)/%.o, $(AS_SRCS))

CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

C_FLAGS = $(CPU_FLAGS) \
          -std=c11 \
          -Wall \
          -Wextra \
          -O2 \
          $(C_INCLUDES) \
          -ffunction-sections \
          -fdata-sections \
          -ffreestanding \
          -MMD -MP \
          -g

AS_FLAGS = $(CPU_FLAGS) -x assembler-with-cpp

LD_FLAGS = $(CPU_FLAGS) \
           -T stm32_flash.ld \
           -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map \
           -Wl,--print-memory-usage \
           -nostartfiles 

all: $(TARGET).hex $(TARGET).bin
	@echo ""
	@echo "=========================================="
	@echo "  Build xong! Flash len board bang:"
	@echo "  openocd -f interface/stlink.cfg \\"
	@echo "          -f target/stm32f4x.cfg \\"
	@echo "          -c 'program $(TARGET).hex verify reset exit'"
	@echo "=========================================="
	@$(SIZE) $(TARGET).elf

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "[CC]  $<"
	$(CC) $(C_FLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "[AS]  $<"
	$(AS) $(AS_FLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET).elf: $(OBJS)
	@echo "[LD]  $@"
	$(LD) $(LD_FLAGS) $(OBJS) -lm -lgcc -lc -o $@

$(TARGET).hex: $(TARGET).elf
	@echo "[HEX] $@"
	$(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	@echo "[BIN] $@"
	$(OBJCOPY) -O binary $< $@

dump: $(TARGET).elf
	$(OBJDUMP) -d -S $(TARGET).elf | less

map: $(TARGET).map
	@grep -E "^\.(text|data|bss|stack|vectors)" $(TARGET).map

clean:
	@echo "[CLEAN]"
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).map

flash: $(TARGET).hex
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f4x.cfg \
		-c "program $(TARGET).hex verify reset exit"

debug: $(TARGET).elf
	-pkill openocd
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg &
	sleep 1
	arm-none-eabi-gdb $(TARGET).elf \
		-ex "target remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"

dfu: $(TARGET).bin
	dfu-util -a 0 -s 0x08000000:leave -D $(TARGET).bin

-include $(OBJS:.o=.d)


.PHONY: all clean dump map flash debug dfu
