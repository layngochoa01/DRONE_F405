# Makefile - BLUEBERRY F405 bare-metal
# Ubuntu: sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

# =========================================================
# TOOLCHAIN
# =========================================================
CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

# =========================================================
# TARGET
# =========================================================
TARGET = blueberryf405

# =========================================================
# SOURCE FILES
# =========================================================
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
	src/flight/imu.c \
	src/flight/filter.c \
	src/flight/attitude.c

C_INCLUDES = \
    -Iinclude \
    -Isrc/drivers \
	-Isrc/sensors \
	-Isrc/flight

AS_SRCS = startup.s

OBJS = $(patsubst src/%.c, $(BUILD_DIR)/%.o, $(C_SRCS)) \
       $(patsubst %.s,     $(BUILD_DIR)/%.o, $(AS_SRCS))

# =========================================================
# CPU FLAGS (dùng chung cho C, ASM, LD)
# -mcpu=cortex-m4    : Target CPU
# -mthumb            : Thumb-2 instruction set
# -mfpu=fpv4-sp-d16  : Hardware FPU (STM32F405 có FPU)
# -mfloat-abi=hard   : Dùng hardware FPU
# =========================================================
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# =========================================================
# COMPILER FLAGS
# -std=c11           : C11 standard
# -Wall -Wextra      : Warnings
# -O2                : Optimize
# -ffunction-sections: Mỗi hàm 1 section (gc-sections xóa hàm không dùng)
# -fdata-sections    : Tương tự cho data
# -ffreestanding     : Không có stdlib (bare-metal)
# -MMD -MP           : Tự động tạo dependency file (.d) cho header
# -g                 : Debug symbols
# =========================================================
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

# =========================================================
# ASSEMBLER FLAGS
# -x assembler-with-cpp : Cho phép #include trong .s
# =========================================================
AS_FLAGS = $(CPU_FLAGS) -x assembler-with-cpp

# =========================================================
# LINKER FLAGS
# -T stm32_flash.ld      : Linker script
# --gc-sections          : Xóa code/data không dùng
# -Map=...               : Tạo file .map để debug memory
# --print-memory-usage   : In thống kê RAM/Flash sau link
# -nostdlib              : Không link stdlib
# -nostartfiles          : Không dùng crt0 của GCC
# =========================================================
LD_FLAGS = $(CPU_FLAGS) \
           -T stm32_flash.ld \
           -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map \
           -Wl,--print-memory-usage \
           -nostartfiles 

# =========================================================
# BUILD RULES
# =========================================================
all: $(TARGET).hex $(TARGET).bin
	@echo ""
	@echo "=========================================="
	@echo "  Build xong! Flash len board bang:"
	@echo "  openocd -f interface/stlink.cfg \\"
	@echo "          -f target/stm32f4x.cfg \\"
	@echo "          -c 'program $(TARGET).hex verify reset exit'"
	@echo "=========================================="
	@$(SIZE) $(TARGET).elf

# Compile C → .o (tạo subfolder nếu cần)
$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	@echo "[CC]  $<"
	$(CC) $(C_FLAGS) -c $< -o $@

# Assemble .s → .o
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "[AS]  $<"
	$(AS) $(AS_FLAGS) -c $< -o $@

# Tạo thư mục build/
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Link → ELF
$(TARGET).elf: $(OBJS)
	@echo "[LD]  $@"
	$(LD) $(LD_FLAGS) $(OBJS) -lm -lgcc -lc -o $@

# ELF → HEX (Intel HEX, dùng để flash)
$(TARGET).hex: $(TARGET).elf
	@echo "[HEX] $@"
	$(OBJCOPY) -O ihex $< $@

# ELF → BIN (raw binary)
$(TARGET).bin: $(TARGET).elf
	@echo "[BIN] $@"
	$(OBJCOPY) -O binary $< $@


# =========================================================
# UTILITY
# =========================================================

# Disassembly - xem code thật sự được generate
dump: $(TARGET).elf
	$(OBJDUMP) -d -S $(TARGET).elf | less

# Tóm tắt memory map
map: $(TARGET).map
	@grep -E "^\.(text|data|bss|stack|vectors)" $(TARGET).map

# Dọn dẹp
clean:
	@echo "[CLEAN]"
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).map

# Flash với OpenOCD + ST-Link
flash: $(TARGET).hex
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32f4x.cfg \
		-c "program $(TARGET).hex verify reset exit"

# Debug: mở OpenOCD + GDB
debug: $(TARGET).elf
	-pkill openocd
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg &
	sleep 1
	arm-none-eabi-gdb $(TARGET).elf \
		-ex "target remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"

# Nạp firmware qua DFU (USB bootloader)
# Trước khi chạy: giữ BOOT0=HIGH rồi nhấn Reset
dfu: $(TARGET).bin
	dfu-util -a 0 -s 0x08000000:leave -D $(TARGET).bin

# Auto dependency (header thay đổi → recompile)
-include $(OBJS:.o=.d)


.PHONY: all clean dump map flash debug dfu
