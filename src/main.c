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
 *   9. test spl06
 *      log press and temp
 *      
 * check rc receiver
 *    
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
#include "spl06.h"
#include "altitude.h"

#define NOISE_ALT   0.001f
#define NOISE_VEL   0.001f
#define NOISE_BIAS  0.00001f
#define NOISE_BARO  1.0f

static volatile uint32_t ms_tick = 0;
static ICM_Calibration_t s_cal;   
static bool spl06_ready = false;

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

static void save_calib_to_flash(void)
{
    CalibData_t flash_data = {
        .gx_offset  = s_cal.gx_offset,
        .gy_offset  = s_cal.gy_offset,
        .gz_offset  = s_cal.gz_offset,
        .ax_offset  = s_cal.ax_offset,
        .ay_offset  = s_cal.ay_offset,
        .az_offset  = s_cal.az_offset,
        .gyro_done  = s_cal.gyro_done  ? 1U : 0U,
        .accel_done = s_cal.accel_done ? 1U : 0U,
    };

    FlashStatus_t fs = FlashStorage_Save(&flash_data);
    if (fs == FLASH_OK) {
        UART5_WriteString("Calib saved to Flash OK\r\n");
    } else {
        UART5_WriteF("Calib Flash save FAILED code=%d\r\n", fs);
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
 
    save_calib_to_flash();

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
 
    save_calib_to_flash();
 
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

static bool altitude_calibrate_baseline(void)
{
    const int samples = 100;
    float sum = 0.0f;
    int count = 0;

    UART5_WriteString("ALT BASELINE: collecting...\r\n");

    for (int i = 0; i < samples; i++) {
        SPL06_Data_t baro;
        if (SPL06_Update(&baro)) {
            sum += baro.pressPa;
            count++;
        }
        delay_ms(20);
    }

    if (count == 0) {
        UART5_WriteString("ALT BASELINE: FAILED\r\n");
        return false;
    }

    float baseline = sum / (float)count;
    Altitude_SetBaseline(baseline);

    UART5_WriteF("ALT BASELINE: %.2f Pa (%d samples)\r\n", baseline, count);
    return true;
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
    // delay_ms(15000);
    spl06_ready = SPL06_Init();
    AltitudeBaro_Init(101325.0f);

    if (spl06_ready) {
        delay_ms(3000);
        altitude_calibrate_baseline();
    }

    CalibData_t flash_data;
    FlashStatus_t fs = FlashStorage_Init(&flash_data);

    // delay_ms(10000);
    // UART5_WriteF(
    //     "FLASH DATA: AX=%.6f AY=%.6f AZ=%.6f\r\n",
    //     flash_data.ax_offset,
    //     flash_data.ay_offset,
    //     flash_data.az_offset
    // );
    //    UART5_WriteF(
    //     "FLASH DATA: GX=%.6f GY=%.6f GZ=%.6f\r\n",
    //     flash_data.gx_offset,
    //     flash_data.gy_offset,
    //     flash_data.gz_offset
    // );

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
        s_cal.gyro_done  = (flash_data.gyro_done  != 0U);
        s_cal.accel_done = (flash_data.accel_done != 0U);  
        // UART5_WriteF("Calib loaded: GX:%.4f GY:%.4f GZ:%.4f\r\n", s_cal.gx_offset, s_cal.gy_offset, s_cal.gz_offset);
        // UART5_WriteF( "ACCEL OFFSET: X=%.4f Y=%.4f Z=%.4f\r\n", s_cal.ax_offset, s_cal.ay_offset, s_cal.az_offset);
    } else {
        s_cal.gx_offset = s_cal.gy_offset = s_cal.gz_offset = 0.0f;
        s_cal.ax_offset = s_cal.ay_offset = s_cal.az_offset = 0.0f;
        s_cal.ax_gain   = s_cal.ay_gain   = s_cal.az_gain   = 1.0f;
        s_cal.gyro_done  = false;
        s_cal.accel_done = false;
        UART5_WriteString("No calib data, using defaults\r\n");
    }

    AltitudeKF_Init(NOISE_ALT, NOISE_VEL, NOISE_BIAS, NOISE_BARO);
    // delay_ms(10000);
    

    
    Timer2_InitHz(84000000U, 100U);
    Timer2_Start();

    ICM42605_Data   data;
    IMUFilter_t imu_filter;
/////////////////////////////MAIN LOOP////////////////////////////////////
    bool data_valid = false;
    uint32_t last_blink = 0;
    uint32_t last_log  = 0;

    static uint32_t last_baro = 0;
    static uint32_t last_kf_log = 0;

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
            if(data_valid){
                float rMat[3][3];
                Attitude_GetRotationMatrix(rMat);

                float accelZ_world = Altitude_ComputeVerticalAccel(rMat, data.accel_x, data.accel_y, data.accel_z);
                // UART5_WriteF("AZ:[ %8.2f], \r\t",accelZ_world );
                AltitudeKF_Predict(accelZ_world, 1.0f / 100.0f);
            }
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
                const Quaternion_t *q = Quaternion_Get();
                // HC05_LogQuaternion(q);
            }
        }
        else led_on(LED_BLUE_PIN);

        if (spl06_ready && (now - last_baro) >= 125U) {
            last_baro = now;

            SPL06_Data_t baro;
            if (SPL06_Update(&baro)) {
                float baroAlt = Altitude_PressureToMeters(baro.pressPa);
                // UART5_WriteF( "P: %.2f Pa  BAlt: %.3f m\r\n",  baro.pressPa, baroAlt);
                AltitudeKF_UpdateBaro(baroAlt);
            }
        }

        if((now - last_kf_log) >= 200U){
            last_kf_log = now;
            const AltitudeKF_State_t *kf = AltitudeKF_Get();
            // HC05_LogAltitudeFKState(kf);
        }
    }
    
    return 0;
}
