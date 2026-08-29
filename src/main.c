/*
 * main.c - BLUEBERRY F405 bare-metal
 * Chức năng:
 *   1. Cấu hình clock 168MHz (PLL từ HSI 16MHz)
 *   2. SysTick delay 1ms
 *   3. Blink LED PA13 (xanh dương) + PA14 (đỏ)
 *   4. Đọc ICM42605 qua SPI1
 *   5. Log ra terminal data imu da dps dma+timer2 có LPF + AAF + bỏ AFSR
 *   6. log roll, ptich, yaw 
 *   7. FLASH STORAGE 2slot
 *   8. calib gyro+accel
 *   9. check rc receiver
 *
 * SPI1: PA5(SCK) | PA7(MOSI) | PB4(MISO) | PC14(CS)
 * UART5: 
 */

#include <stdint.h>
#include "uart5.h"
#include "register.h"
#include "icm42605.h"
#include "hc05.h"
#include "timer.h"
#include "attitude.h"
#include "flash_storage.h"
#include "filter.h"

static volatile uint32_t ms_tick = 0;
static ICM_Calibration_t s_cal;   

void systick_handler(void)
{
    ms_tick++;
}

uint32_t get_tick(void)
{
    return ms_tick;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = ms_tick;
    while ((ms_tick - start) < ms) {}
}

static void clock_init(void)
{
    FLASH_ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN
              | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH_ACR & 0x7U) != 5U) {}
 
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
 
    RCC->PLLCFGR = RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN
                 | RCC_PLLCFGR_PLLP_DIV2 | RCC_PLLCFGR_PLLSRC_HSI
                 | RCC_PLLCFGR_PLLQ;
 
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4
              | RCC_CFGR_PPRE2_DIV2;
 
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}
 
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & (3U << 2)) != RCC_CFGR_SWS_PLL) {}
}
 
static void systick_init(void)
{
    SYSTICK->LOAD = 168000U - 1U;
    SYSTICK->VAL  = 0U;
    SYSTICK->CTRL = SYSTICK_CTRL_ENABLE | SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_CLKSRC;
}

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    GPIOA->MODER &= ~((3U << (LED_RED_PIN * 2)) | (3U << (LED_BLUE_PIN * 2)));
    GPIOA->MODER |=  ((GPIO_MODE_OUTPUT << (LED_RED_PIN * 2)) | (GPIO_MODE_OUTPUT << (LED_BLUE_PIN  * 2)));

    GPIOA->BSRR = GPIO_BSRR_SET(LED_RED_PIN) | GPIO_BSRR_SET(LED_BLUE_PIN);
}

static inline void led_on(uint8_t pin)
{
    GPIOA->BSRR = GPIO_BSRR_RESET(pin);
}

static inline void led_off(uint8_t pin)
{
    GPIOA->BSRR = GPIO_BSRR_SET(pin);
}

static inline void led_toggle(uint8_t pin)
{
    if (GPIOA->ODR & (1U << pin)) {
        GPIOA->BSRR = GPIO_BSRR_RESET(pin);
    } else {
        GPIOA->BSRR = GPIO_BSRR_SET(pin);
    }
}

void hardfault_handler(void)
{
    while (1) {
        led_on(LED_RED_PIN);
        for (volatile uint32_t i = 0; i < 200000U; i++) {}
        led_off(LED_RED_PIN);
        for (volatile uint32_t i = 0; i < 200000U; i++) {}
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; 
        ICM42605_TriggerRead();
    }
}
 
static void handle_calib_gyro(void)
{
    UART5_WriteString("CALIB_GYRO: keep board STILL...\r\n");
    led_on(LED_BLUE_PIN);
 
    ICM_CalibStatus_t gs = ICM42605_CalibrateGyro(&s_cal, 200);
 
    if (gs != CALIB_OK) {
        UART5_WriteF("CALIB_GYRO FAILED code=%d\r\n", gs);
        led_off(LED_BLUE_PIN);
        return;
    }
 
    /* Lưu vào Flash */
    CalibData_t flash_data = {
        .gx_offset = s_cal.gx_offset,
        .gy_offset = s_cal.gy_offset,
        .gz_offset = s_cal.gz_offset,
        .ax_offset = s_cal.ax_offset,
        .ay_offset = s_cal.ay_offset,
        .az_offset = s_cal.az_offset,
    };
 
    FlashStatus_t fs = FlashStorage_Save(&flash_data);
    if (fs == FLASH_OK) {
        UART5_WriteString("CALIB_GYRO saved to Flash OK\r\n");

        /* Đọc lại Flash để verify */
        CalibData_t verify;
        FlashStatus_t fv = FlashStorage_Init(&verify);
        if (fv == FLASH_OK) {
            UART5_WriteF("=== VERIFY FLASH ===\r\n");
            UART5_WriteF("SEQ : %lu\r\n",  verify.sequence);
            UART5_WriteF("GX  : %.6f\r\n", verify.gx_offset);
            UART5_WriteF("GY  : %.6f\r\n", verify.gy_offset);
            UART5_WriteF("GZ  : %.6f\r\n", verify.gz_offset);
            UART5_WriteF("CRC : 0x%08lX\r\n", verify.crc);
            UART5_WriteF("====================\r\n");
        } else {
            UART5_WriteString("VERIFY FAILED!\r\n");
        }
    } else {
        UART5_WriteF("CALIB_GYRO Flash save FAILED code=%d\r\n", fs);
    }
 
    led_off(LED_BLUE_PIN);
}
 
static void handle_calib_accel(void)
{
    UART5_WriteString("CALIB_ACCEL: place board FLAT and STILL...\r\n");
    led_on(LED_BLUE_PIN);
 
    ICM_CalibStatus_t as = ICM42605_CalibrateAccel(&s_cal, 200);
 
    if (as != CALIB_OK) {
        UART5_WriteF("CALIB_ACCEL FAILED code=%d\r\n", as);
        led_off(LED_BLUE_PIN);
        return;
    }
 
    CalibData_t flash_data = {
        .gx_offset = s_cal.gx_offset,
        .gy_offset = s_cal.gy_offset,
        .gz_offset = s_cal.gz_offset,
        .ax_offset = s_cal.ax_offset,
        .ay_offset = s_cal.ay_offset,
        .az_offset = s_cal.az_offset,
    };
 
    FlashStatus_t fs = FlashStorage_Save(&flash_data);
    if (fs == FLASH_OK) {
        UART5_WriteString("CALIB_ACCEL saved to Flash OK\r\n");
    } else {
        UART5_WriteF("CALIB_ACCEL Flash save FAILED code=%d\r\n", fs);
    }
 
    led_off(LED_BLUE_PIN);
}
 
static void handle_calib_erase(void)
{
    FlashStatus_t fs = FlashStorage_Erase();
    if (fs == FLASH_OK) {
        UART5_WriteString("CALIB_ERASE OK\r\n");
        s_cal.gx_offset = s_cal.gy_offset = s_cal.gz_offset = 0.0f;
        s_cal.ax_offset = s_cal.ay_offset = s_cal.az_offset = 0.0f;
        s_cal.ax_gain   = s_cal.ay_gain   = s_cal.az_gain   = 1.0f;
        s_cal.gyro_done  = false;
        s_cal.accel_done = false;
    } else {
        UART5_WriteF("CALIB_ERASE FAILED code=%d\r\n", fs);
    }
}

static void handle_calib(void){
    handle_calib_gyro();
    handle_calib_accel();
}

int main(){

    *((volatile uint32_t *)0xE000ED88) |= (0xFU << 20);

    clock_init();
    systick_init();
    led_init();

    ICM_Status imu_status = ICM42605_Init();

    if (imu_status == ICM_OK) {
        for (int i = 0; i < 3; i++) {
            led_on(LED_RED_PIN);
            delay_ms(500);
            led_off(LED_RED_PIN);
            delay_ms(500);
        }
    } else {
        while (1) {
            led_toggle(LED_BLUE_PIN);
            delay_ms(500);
        }
     }

    HC05_Init(42000000U, 115200U);


    CalibData_t flash_data;
    FlashStatus_t fs = FlashStorage_Init(&flash_data);
    
    if (fs == FLASH_OK) {
        s_cal.gx_offset  = flash_data.gx_offset;
        s_cal.gy_offset  = flash_data.gy_offset;
        s_cal.gz_offset  = flash_data.gz_offset;
        s_cal.ax_offset  = flash_data.ax_offset;
        s_cal.ay_offset  = flash_data.ay_offset;
        s_cal.az_offset  = flash_data.az_offset;
        s_cal.ax_gain    = 1.0f;
        s_cal.ay_gain    = 1.0f;
        s_cal.az_gain    = 1.0f;
        s_cal.gyro_done  = true;
        s_cal.accel_done = false;  
        UART5_WriteF("Calib loaded: GX:%.4f GY:%.4f GZ:%.4f\r\n",
                     s_cal.gx_offset, s_cal.gy_offset, s_cal.gz_offset);
    } else {
        s_cal.gx_offset = s_cal.gy_offset = s_cal.gz_offset = 0.0f;
        s_cal.ax_offset = s_cal.ay_offset = s_cal.az_offset = 0.0f;
        s_cal.ax_gain   = s_cal.ay_gain   = s_cal.az_gain   = 1.0f;
        s_cal.gyro_done  = false;
        s_cal.accel_done = false;
        UART5_WriteString("No calib data, using defaults\r\n");
    }

    
    Timer2_InitHz(84000000U, 100U);
    Timer2_Start();

    ICM42605_Data   data;
    IMUFilter_t imu_filter;

    bool data_valid = false;
    uint32_t last_blink = 0;
    uint32_t last_log  = 0;

    IMU_FilterInit(&imu_filter, 1.0f / 100.0f);
    Attitude_Init(1.0f / 100.0f);

    while (1) {
        uint32_t now = get_tick();

        if (ICM42605_IsDataReady()) {
            ICM42605_GetLatestData(&data);
            ICM42605_RemapAxes(&data);
            ICM42605_ApplyCalibration(&data, &s_cal);
            IMU_FilterApply(&imu_filter, &data);
            Attitude_Update(&data, 1.0f / 100.0f);
            data_valid = true;
        }

        HC05_Poll();

        HC05_CalibCmd cmd = HC05_GetCalibCmd();
        if (cmd == HC05_CMD_CALIB) handle_calib();
        if (cmd == HC05_CMD_CALIB_GYRO) handle_calib_gyro();
        if (cmd == HC05_CMD_CALIB_ACCEL) handle_calib_accel();
        if (cmd == HC05_CMD_CALIB_ERASE) handle_calib_erase();

        if (HC05_GetStreamState() == HC05_STREAM_RUN) {
            if (data_valid && ((now - last_log) >= 100U)){
                last_log = now;
                // HC05_SendIMU(&data);

                // const Attitude_t *att = Attitude_Get();
                // UART5_WriteF("R: %.2f P: %.2f Y: %.2f\r\n", att->roll, att->pitch, att->yaw );
                const Quaternion_t *q = Quaternion_Get();
                HC05_LogQuaternion(q);
            }
        }
        else led_on(LED_BLUE_PIN);

        if ((now - last_blink) >= 500U) {
            last_blink = now;
            led_toggle(LED_RED_PIN);
        }
    }
    
    return 0;
}
