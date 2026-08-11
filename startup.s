/* startup.s - STM32F405 startup code
 * Tham chiếu: ARM Cortex-M4 Technical Reference Manual
 *             STM32F405 Reference Manual RM0090
 */

    .syntax unified
    .cpu cortex-m4
    .thumb

    .extern UART5_IRQHandler
    .extern  DMA2_Stream0_IRQHandler
    .extern  DMA2_Stream3_IRQHandler 
    .extern TIM2_IRQHandler

/* =========================================================
 * VECTOR TABLE - RM0090 Table 61 (STM32F405 interrupt vectors)
 * ========================================================= */
    .section .vectors, "a"
    .align 2

_vector_table:
    .word   _stack_top
    .word   reset_handler
    .word   nmi_handler
    .word   hardfault_handler
    .word   memmanage_handler
    .word   busfault_handler
    .word   usagefault_handler
    .word   0
    .word   0
    .word   0
    .word   0
    .word   svc_handler
    .word   debugmon_handler
    .word   0
    .word   pendsv_handler
    .word   systick_handler

    .word   0                   /* IRQ0:  WWDG               */
    .word   0                   /* IRQ1:  PVD                */
    .word   0                   /* IRQ2:  TAMP_STAMP         */
    .word   0                   /* IRQ3:  RTC_WKUP           */
    .word   0                   /* IRQ4:  FLASH              */
    .word   0                   /* IRQ5:  RCC                */
    .word   0                   /* IRQ6:  EXTI0              */
    .word   0                   /* IRQ7:  EXTI1              */
    .word   0                   /* IRQ8:  EXTI2              */
    .word   0                   /* IRQ9:  EXTI3              */
    .word   0                   /* IRQ10: EXTI4              */
    .word   0                   /* IRQ11: DMA1_Stream0       */
    .word   0                   /* IRQ12: DMA1_Stream1       */
    .word   0                   /* IRQ13: DMA1_Stream2       */
    .word   0                   /* IRQ14: DMA1_Stream3       */
    .word   0                   /* IRQ15: DMA1_Stream4       */
    .word   0                   /* IRQ16: DMA1_Stream5       */
    .word   0                   /* IRQ17: DMA1_Stream6       */
    .word   0                   /* IRQ18: ADC                */
    .word   0                   /* IRQ19: CAN1_TX            */
    .word   0                   /* IRQ20: CAN1_RX0           */
    .word   0                   /* IRQ21: CAN1_RX1           */
    .word   0                   /* IRQ22: CAN1_SCE           */
    .word   0                   /* IRQ23: EXTI9_5            */
    .word   0                   /* IRQ24: TIM1_BRK_TIM9      */
    .word   0                   /* IRQ25: TIM1_UP_TIM10      */
    .word   0                   /* IRQ26: TIM1_TRG_TIM11     */
    .word   0                   /* IRQ27: TIM1_CC            */
    .word   TIM2_IRQHandler                   /* IRQ28: TIM2               */
    .word   0                   /* IRQ29: TIM3               */
    .word   0                   /* IRQ30: TIM4               */
    .word   0                   /* IRQ31: I2C1_EV            */
    .word   0                   /* IRQ32: I2C1_ER            */
    .word   0                   /* IRQ33: I2C2_EV            */
    .word   0                   /* IRQ34: I2C2_ER            */
    .word   0                   /* IRQ35: SPI1               */
    .word   0                   /* IRQ36: SPI2               */
    .word   0                   /* IRQ37: USART1             */
    .word   0                   /* IRQ38: USART2             */
    .word   0                   /* IRQ39: USART3             */
    .word   0                   /* IRQ40: EXTI15_10          */
    .word   0                   /* IRQ41: RTC_Alarm          */
    .word   0                   /* IRQ42: OTG_FS_WKUP        */
    .word   0                   /* IRQ43: TIM8_BRK_TIM12     */
    .word   0                   /* IRQ44: TIM8_UP_TIM13      */
    .word   0                   /* IRQ45: TIM8_TRG_COM_TIM14 */
    .word   0                   /* IRQ46: TIM8_CC            */
    .word   0                   /* IRQ47: DMA1_Stream7       */
    .word   0                   /* IRQ48: FMC                */
    .word   0                   /* IRQ49: SDIO               */
    .word   0                   /* IRQ50: TIM5               */
    .word   0                   /* IRQ51: SPI3               */
    .word   0                   /* IRQ52: UART4              */
    .word   UART5_IRQHandler    /* IRQ53: UART5              */
    .word   0                   /* IRQ54: TIM6_DAC           */
    .word   0                   /* IRQ55: TIM7               */
    .word   DMA2_Stream0_IRQHandler                          /* IRQ56: DMA2_Stream0       */
    .word   0                   /* IRQ57: DMA2_Stream1       */
    .word   0                   /* IRQ58: DMA2_Stream2       */
    .word   DMA2_Stream3_IRQHandler                          /* IRQ59: DMA2_Stream3       */
    .word   0                   /* IRQ60: DMA2_Stream4       */
    .word   0                   /* IRQ61: ETH                */
    .word   0                   /* IRQ62: ETH_WKUP           */
    .word   0                   /* IRQ63: CAN2_TX            */
    .word   0                   /* IRQ64: CAN2_RX0           */
    .word   0                   /* IRQ65: CAN2_RX1           */
    .word   0                   /* IRQ66: CAN2_SCE           */
    .word   0                   /* IRQ67: OTG_FS             */
    .word   0                   /* IRQ68: DMA2_Stream5       */
    .word   0                   /* IRQ69: DMA2_Stream6       */
    .word   0                   /* IRQ70: DMA2_Stream7       */
    .word   0                   /* IRQ71: USART6             */
    .word   0                   /* IRQ72: I2C3_EV            */
    .word   0                   /* IRQ73: I2C3_ER            */
    .word   0                   /* IRQ74: OTG_HS_EP1_OUT     */
    .word   0                   /* IRQ75: OTG_HS_EP1_IN      */
    .word   0                   /* IRQ76: OTG_HS_WKUP        */
    .word   0                   /* IRQ77: OTG_HS             */
    .word   0                   /* IRQ78: DCMI               */
    .word   0                   /* IRQ79: Reserved           */
    .word   0                   /* IRQ80: HASH_RNG           */
    .word   0                   /* IRQ81: FPU                */

/* =========================================================
 * RESET HANDLER
 * ========================================================= */
    .section .text
    .align 2
    .thumb_func
    .global reset_handler
    .type   reset_handler, %function

reset_handler:
    /* --- Enable FPU --- */
    ldr     r0, =0xE000ED88
    ldr     r1, [r0]
    orr     r1, r1, #(0xF << 20)   /* CP10[21:20]=11, CP11[23:22]=11 */
    str     r1, [r0]
    dsb                             /* Data Sync Barrier */
    isb                             /* Instruction Sync Barrier */
    /* Bước 1: Copy .data từ Flash → RAM */
    ldr     r0, =_data_lma
    ldr     r1, =_data_start
    ldr     r2, =_data_end

copy_data_loop:
    cmp     r1, r2
    bge     copy_data_done
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       copy_data_loop

copy_data_done:

    /* Bước 2: Zero-fill .bss */
    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    mov     r2, #0

zero_bss_loop:
    cmp     r0, r1
    bge     zero_bss_done
    str     r2, [r0], #4
    b       zero_bss_loop

zero_bss_done:

    /* Bước 3: Gọi main() */
    bl      main

    /* Bước 4: main() thoát → loop vô tận */
hang:
    b       hang

/* =========================================================
 * DEFAULT EXCEPTION HANDLERS
 * .weak → có thể override từ C bằng cách định nghĩa lại hàm
 * ========================================================= */
    .thumb_func
    .weak   nmi_handler
    .global nmi_handler
nmi_handler:
    b       nmi_handler

    .thumb_func
    .weak   hardfault_handler
    .global hardfault_handler
hardfault_handler:
    b       hardfault_handler

    .thumb_func
    .weak   memmanage_handler
    .global memmanage_handler
memmanage_handler:
    b       memmanage_handler

    .thumb_func
    .weak   busfault_handler
    .global busfault_handler
busfault_handler:
    b       busfault_handler

    .thumb_func
    .weak   usagefault_handler
    .global usagefault_handler
usagefault_handler:
    b       usagefault_handler

    .thumb_func
    .weak   svc_handler
    .global svc_handler
svc_handler:
    b       svc_handler

    .thumb_func
    .weak   debugmon_handler
    .global debugmon_handler
debugmon_handler:
    b       debugmon_handler

    .thumb_func
    .weak   pendsv_handler
    .global pendsv_handler
pendsv_handler:
    b       pendsv_handler

    .thumb_func
    .weak   systick_handler
    .global systick_handler
systick_handler:
    b       systick_handler

    .end
