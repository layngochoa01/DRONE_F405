/* linker.ld - STM32F405 Linker Script
 * STM32F405 Memory:
 *   Flash: 0x08000000 - 0x080FFFFF (1MB)
 *   RAM:   0x20000000 - 0x2001FFFF (128KB)
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_stack_size = 0x1000;   /* 4KB */

/* Stack cố định ở đỉnh RAM — an toàn hơn, không bị .bss đẩy */
_stack_top = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(reset_handler)

SECTIONS
{
    .vectors :
    {
        . = ALIGN(4);
        KEEP(*(.vectors))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _text_end = .;
    } > FLASH

    .data :
    {
        . = ALIGN(4);
        _data_lma = LOADADDR(.data);   /* địa chỉ nguồn trong Flash */
        _data_start = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    } > RAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _bss_start = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > RAM

    . = ALIGN(4);
    _heap_end = .;
    
    /* Kiểm tra RAM không bị tràn */
    ASSERT(_bss_end + _stack_size <= ORIGIN(RAM) + LENGTH(RAM),
           "ERROR: RAM overflow! Giam stack_size hoac xoa bot bien")
}
