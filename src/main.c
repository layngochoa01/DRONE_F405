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
#include "sbus.h"
#include "hmc5883l.h"
#include "gps.h"
#include "usart1.h"
#include "pwm_out.h"
#include "system.h"
#include "flight_state.h"
#include "flight_control.h"
#include "position.h"

#define NOISE_ALT   0.001f
#define NOISE_VEL   0.001f
#define NOISE_BIAS  0.00001f
#define NOISE_BARO  0.0025f

#define COMPASS_ENABLE 1
#define GPS_ENABLE     1

#define CALIB_SAMPLES  2000U

static volatile uint32_t ms_tick = 0;
static ICM_Calibration_t s_cal;
static bool spl06_ready = false;

static IMUFilter_t s_imuFilter;
static volatile uint32_t s_imuMiss = 0;

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

void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;
        if (ICM42605_TriggerRead() == ICM_ERR_BUSY) s_imuMiss++;
    }
}

static void imu_sample_isr(const ICM42605_Data *sample)
{
    ICM42605_Data d = *sample;
    ICM42605_RemapAxes(&d);
    ICM42605_ApplyCalibration(&d, &s_cal);
    IMU_FilterApply(&s_imuFilter, &d);
    FlightControl_FastUpdate(&d);
}

void USART1_IRQHandler(void)
{
    UART1_IRQ_Handler();
}

void USART2_IRQHandler(void)
{
    SBUS_ISR_Handler();
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

    ICM_CalibStatus_t gs = ICM42605_CalibrateGyro(&s_cal, CALIB_SAMPLES);

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

    ICM_CalibStatus_t as = ICM42605_CalibrateAccel(&s_cal, CALIB_SAMPLES);

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

static void handle_calib_show(void)
{
    uint32_t seq = 0;
    int32_t slot = FlashStorage_GetActiveSlot(&seq);
    UART5_WriteF("FLASH slot:%ld seq:%lu | gyro_done:%d accel_done:%d\r\n",
                 (long)slot, (unsigned long)seq, s_cal.gyro_done, s_cal.accel_done);
    UART5_WriteF("G off: %.4f %.4f %.4f\r\n", s_cal.gx_offset, s_cal.gy_offset, s_cal.gz_offset);
    UART5_WriteF("A off: %.4f %.4f %.4f\r\n", s_cal.ax_offset, s_cal.ay_offset, s_cal.az_offset);
}

static void handle_calib(void){
    handle_calib_gyro();
    handle_calib_accel();
}

static bool altitude_calibrate_baseline(void)
{
    const int samples = 50;
    const int timeout_ms = 10000;
    float sum = 0.0f;
    int count = 0;
    uint32_t start = get_tick();

    UART5_WriteString("ALT BASELINE: collecting...\r\n");

    while (count < samples && (get_tick() - start) < timeout_ms) {
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
    cycle_counter_init();
    led_init();

    Motor_Init(168000000U, 84000000U);
    Motor_StopAll();

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
    SBUS_Init(42000000U);
#if GPS_ENABLE
    GPS_Init();
#endif
    delay_ms(15000);
    spl06_ready = SPL06_Init();
    AltitudeBaro_Init(101325.0f);

    if (spl06_ready) {
        delay_ms(8000);
        altitude_calibrate_baseline();
    }

#if COMPASS_ENABLE
    bool compass_ready = Compass_Init();
#else
    bool compass_ready = false;
#endif
    UART5_WriteF("BARO:%s MAG:%s (%s)\r\n", spl06_ready ? "OK" : "FAIL",
                 compass_ready ? "OK" : "FAIL", Compass_TypeName());

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
        s_cal.gyro_done  = (flash_data.gyro_done  != 0U);
        s_cal.accel_done = (flash_data.accel_done != 0U);
        UART5_WriteString("Calib loaded from Flash:\r\n");
        handle_calib_show();
    } else {
        s_cal.gx_offset = s_cal.gy_offset = s_cal.gz_offset = 0.0f;
        s_cal.ax_offset = s_cal.ay_offset = s_cal.az_offset = 0.0f;
        s_cal.ax_gain   = s_cal.ay_gain   = s_cal.az_gain   = 1.0f;
        s_cal.gyro_done  = false;
        s_cal.accel_done = false;
        UART5_WriteString("No calib data, using defaults\r\n");
    }

    AltitudeKF_Init(NOISE_ALT, NOISE_VEL, NOISE_BIAS, NOISE_BARO);
    FlightControl_Init(spl06_ready, compass_ready);

    IMU_FilterInit(&s_imuFilter, FC_DT);
    ICM42605_SetSampleCallback(imu_sample_isr);

    Timer5_Init(84000000U);
    Timer7_InitHz(84000000U, FC_LOOP_HZ);
    Timer7_Start();

    uint32_t last_log = 0;
    enum { LOG_NORMAL, LOG_MAG, LOG_GPS, LOG_PERF, LOG_GPSCSV } log_mode = LOG_NORMAL;
    uint32_t gps_log_pvt = 0;
    uint32_t gps_stat_ms = 0;

    while (1) {
        uint32_t now = get_tick();

        FlightControl_SlowUpdate();

#if GPS_ENABLE
        GPS_Process();
#endif
        HC05_Poll();

        HC05_CalibCmd cmd = HC05_GetCalibCmd();

        if (cmd == HC05_CMD_LOG_MAG)    { log_mode = LOG_MAG;    UART5_WriteString("LOG MAG\r\n"); }
        if (cmd == HC05_CMD_LOG_GPS)    { log_mode = LOG_GPS;    UART5_WriteString("LOG GPS\r\n"); }
        if (cmd == HC05_CMD_LOG_NORMAL) { log_mode = LOG_NORMAL; UART5_WriteString("LOG NORMAL\r\n"); }
        if (cmd == HC05_CMD_LOG_PERF)   { log_mode = LOG_PERF;   UART5_WriteString("LOG PERF\r\n"); }
        if (cmd == HC05_CMD_LOG_GPSCSV) {
            log_mode    = LOG_GPSCSV;
            gps_log_pvt = GPS_GetStats()->pvt;
            gps_stat_ms = now;
            UART5_WriteString("# GPSCSV G,t_ms,iTOW,fixType,flags,numSV,latE7,lonE7,hMSL_mm,hAcc_mm,"
                              "vAcc_mm,velN_mmps,velE_mmps,velD_mmps,gSpeed_mmps,sAcc_mmps,pDOP\r\n");
            UART5_WriteString("# GPSCSV S,bytes,packets,pvt,badChecksum,resync,fixType,numSV\r\n");
        }
        if (cmd == HC05_CMD_MAGCAL) {
            if (FlightState_Get() == FLIGHT_STATE_DISARMED && Compass_GetType() != COMPASS_NONE) {
                Compass_CalStart();
                UART5_WriteString("MAGCAL: xoay drone moi huong ~30s, xong gui MAGCALEND\r\n");
            } else {
                UART5_WriteString("MAGCAL DENIED (can DISARMED + co compass)\r\n");
            }
        }
        if (cmd == HC05_CMD_MAGCAL_END) {
            float ox, oy, oz;
            if (Compass_CalFinish(&ox, &oy, &oz)) {
                UART5_WriteF("MAGCAL OK offset(chip) X:%.4f Y:%.4f Z:%.4f\r\n", ox, oy, oz);
            } else {
                UART5_WriteString("MAGCAL FAILED: qua it mau, xoay lau hon\r\n");
            }
        }

        if (cmd == HC05_CMD_BENCH_ON) {
            if (FlightState_Get() == FLIGHT_STATE_DISARMED) FlightState_BluetoothBenchArm(true);
            UART5_WriteString(FlightControl_SetBench(true)
                              ? "BENCH ON (thao canh!) - BENCHOFF/DISARM de tat\r\n"
                              : "BENCH DENIED: dang bay hoac chua ARM\r\n");
        }
        if (cmd == HC05_CMD_BENCH_OFF || HC05_GetFlightCmd() == HC05_FLIGHT_CMD_DISARM) {
            FlightControl_SetBench(false);
            if (FlightState_IsBluetoothArmed()) FlightState_BluetoothBenchArm(false);
            UART5_WriteString("BENCH OFF\r\n");
        }
        if (FlightState_Get() == FLIGHT_STATE_DISARMED) {
            if (cmd == HC05_CMD_CALIB)       handle_calib();
            if (cmd == HC05_CMD_CALIB_GYRO)  handle_calib_gyro();
            if (cmd == HC05_CMD_CALIB_ACCEL) handle_calib_accel();
            if (cmd == HC05_CMD_CALIB_ERASE) handle_calib_erase();
            if (cmd == HC05_CMD_CALIB_SHOW)  handle_calib_show();
        }

        HC05_MotorTestCmd mcmd = HC05_GetMotorTestCmd();
        if (mcmd == HC05_MOTOR_STOP) {
            FlightControl_MotorTestStop();
            UART5_WriteString("MOTOR STOP\r\n");
        } else if (mcmd != HC05_MOTOR_NONE) {
            int8_t idx = (mcmd == HC05_MOTOR_ALL) ? -1 : (int8_t)(mcmd - HC05_MOTOR_1);
            if (FlightControl_MotorTest(idx)) {
                UART5_WriteF("MOTOR TEST %s @%uus 3s\r\n",
                             idx < 0 ? "ALL" : (idx == 0 ? "M1" : idx == 1 ? "M2" : idx == 2 ? "M3" : "M4"),
                             FC_MOTOR_TEST_US);
            } else {
                UART5_WriteString("MOTOR TEST DENIED (must be DISARMED)\r\n");
            }
        }

        if (HC05_GetStreamState() == HC05_STREAM_RUN) {
            if (log_mode == LOG_GPSCSV) {
                const GPS_Stats_t *gs = GPS_GetStats();
                if (gs->pvt != gps_log_pvt) {
                    gps_log_pvt = gs->pvt;
                    GPS_Data_t g = *GPS_GetData();
                    UART5_WriteF("G,%lu,%lu,%u,%u,%u,%ld,%ld,",
                                 (unsigned long)now, (unsigned long)g.iTOW, g.fixType, g.flags,
                                 g.satellites, (long)g.latE7, (long)g.lonE7);
                    UART5_WriteF("%ld,%lu,%lu,%ld,%ld,%ld,%ld,%lu,%u\r\n",
                                 (long)g.hMSLmm, (unsigned long)g.hAccMm, (unsigned long)g.vAccMm,
                                 (long)g.velNmms, (long)g.velEmms, (long)g.velDmms,
                                 (long)g.gSpeedMms, (unsigned long)g.sAccMms, g.pDOP);
                }
                if ((now - gps_stat_ms) >= 1000U) {
                    gps_stat_ms = now;
                    const GPS_Data_t *g = GPS_GetData();
                    UART5_WriteF("S,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
                                 (unsigned long)gs->bytes, (unsigned long)gs->packets,
                                 (unsigned long)gs->pvt, (unsigned long)gs->badChecksum,
                                 (unsigned long)gs->resync, g->fixType, g->satellites);
                }
            } else if ((now - last_log) >= 200U) {
                last_log = now;
                const Attitude_t *att = Attitude_Get();
                const FlightControl_Status_t *st = FlightControl_GetStatus();
                if (log_mode == LOG_PERF) {
                    FlightControl_Perf_t pf;
                    FlightControl_GetPerf(&pf, true);
                    UART5_WriteF("PERF exec:%.1f/%.1fus period:%.1f..%.1fus (muc tieu %luus) miss:%lu\r\n",
                                 pf.execUs, pf.execMaxUs, pf.periodMinUs, pf.periodMaxUs,
                                 (unsigned long)(1000000U / FC_LOOP_HZ), (unsigned long)s_imuMiss);
                } else if (log_mode == LOG_MAG) {
                    uint32_t age;
                    const Compass_Data_t *c = FlightControl_GetCompass(&age);
                    float ox, oy, oz;
                    Compass_GetOffsets(&ox, &oy, &oz);
                    UART5_WriteF("MAG %s chip X:%6.3f Y:%6.3f Z:%6.3f | H:%5.1f | age:%lums%s\r\n",
                                 Compass_TypeName(), c->rawX - ox, c->rawY - oy, c->rawZ - oz,
                                 c->headingDeg, (unsigned long)age,
                                 Compass_IsCalibrating() ? " [CAL]" : "");
                } else if (log_mode == LOG_GPS) {
                    const GPS_Stats_t *gs = GPS_GetStats();
                    const GPS_Data_t  *g  = GPS_GetData();
                    UART5_WriteF("GPS bytes:%lu pkt:%lu pvt:%lu badck:%lu resync:%lu drop:%lu\r\n",
                                 (unsigned long)gs->bytes, (unsigned long)gs->packets,
                                 (unsigned long)gs->pvt, (unsigned long)gs->badChecksum,
                                 (unsigned long)gs->resync, (unsigned long)UART1_GetDroppedCount());
                    UART5_WriteF("    fix:%u ok:%u sv:%u hAcc:%.2fm pDOP:%.2f healthy:%d\r\n",
                                 g->fixType, (unsigned)(g->flags & 1U), g->satellites,
                                 g->hAccM, g->hdop, (int)GPS_IsHealthy(now));
                    if (g->fixValid) {
                        UART5_WriteF("    lat:%.7f lon:%.7f alt:%.1f vN:%.2f vE:%.2f valid:%d\r\n",
                                     (double)g->latE7 * 1e-7, (double)g->lonE7 * 1e-7,
                                     g->altitudeMsl, g->velN, g->velE, (int)st->gpsValid);
                    }
                } else if (FlightControl_IsBench()) {
                    UART5_WriteF("R:%5.1f P:%5.1f Y:%5.1f | M1:%4d M2:%4d M3:%4d M4:%4d\r\n",
                                 att->roll, att->pitch, att->yaw,
                                 (int)st->motorUs[0], (int)st->motorUs[1],
                                 (int)st->motorUs[2], (int)st->motorUs[3]);
                } else {
                    UART5_WriteF("%s ALT:%.2f/%.2f R:%.1f P:%.1f Y:%.1f T:%.2f GPS:%d MAG:%d\r\n",
                                 FlightState_Name(st->state), AltitudeKF_Get()->altitude,
                                 FlightState_GetTargetAltitude(),
                                 att->roll, att->pitch, att->yaw, st->throttle,
                                 st->gpsValid, st->magHealthy);
                }
            }
        }
        else led_on(LED_BLUE_PIN);
    }

    return 0;
}
