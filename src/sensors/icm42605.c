#include "icm42605.h"
#include "register.h"
#include "uart5.h"
#include "spi.h"
#include <stdbool.h>

uint32_t get_tick(void);
#include <math.h>

static const uint8_t  ICM_AAF_DELT     = 21;
static const uint16_t ICM_AAF_DELT_SQR = 440;
static const uint8_t  ICM_AAF_BITSHIFT = 6;
static uint8_t s_dma_tx_buf[ICM_DMA_BURST_LEN] __attribute__((aligned(4)));
static uint8_t s_dma_rx_buf[ICM_DMA_BURST_LEN] __attribute__((aligned(4)));
static volatile uint8_t s_dma_busy = 0;
static volatile ICM42605_Data s_latest_data;
static volatile uint8_t       s_data_ready = 0;
static void ICM42605_DMA_Callback(void);

void CS_Low(void)
{
    GPIOC->BSRR = GPIO_BSRR_RESET(ICM42605_CS_PIN);
}

void CS_High(void)
{
    GPIOC->BSRR = GPIO_BSRR_SET(ICM42605_CS_PIN);
}

static void delay_us(volatile uint32_t us)
{
    for (volatile uint32_t i = 0; i < (us * 14); i++) {
        __asm__("nop");
    }
}

static void delay_ms(volatile uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

static ICM_Status ICM_WriteReg(uint8_t reg, uint8_t data)
{
    CS_Low();
    uint8_t r1  = SPI1_TransmitReceive(reg & ~ICM_SPI_READ);
    uint8_t r2 = SPI1_TransmitReceive(data);
    CS_High();
    delay_us(1);

    if (r1 == SPI_ERROR || r2 == SPI_ERROR) return ICM_ERR_CONFIG;
    return ICM_OK;
}

static uint8_t ICM_ReadReg(uint8_t reg)
{
    uint8_t data;
    CS_Low();
    SPI1_TransmitReceive(reg | ICM_SPI_READ);
    data = SPI1_TransmitReceive(0x00);
    CS_High();
    delay_us(1);
    return data;
}

static void ICM_SetBank(uint8_t bank)
{
    ICM_WriteReg(ICM_REG_BANK_SEL, bank);
}

ICM_Status ICM42605_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    GPIOC->MODER   &= ~(0x3U << (ICM42605_CS_PIN * 2));
    GPIOC->MODER   |=  (GPIO_MODE_OUTPUT << (ICM42605_CS_PIN * 2));
    GPIOC->OTYPER  &= ~(1U << ICM42605_CS_PIN);
    GPIOC->OSPEEDR |=  (GPIO_SPEED_VERY_HIGH << (ICM42605_CS_PIN * 2));
    GPIOC->PUPDR   &= ~(0x3U << (ICM42605_CS_PIN * 2));
    CS_High();

    SPI1_Init();
    delay_ms(10);

    ICM_SetBank(ICM_BANK_SELECT0);
    ICM_WriteReg(ICM_REG_DEVICE_CONFIG, ICM_DEVICE_CONFIG_RESET);
    delay_ms(2);

    uint8_t who = ICM_ReadReg(ICM_REG_WHO_AM_I);
    if (who != ICM_WHO_AM_I_VALUE) {
        return ICM_ERR_WHO_AM_I;
    }

    ICM_WriteReg(ICM_REG_PWR_MGMT0, ICM_PWR_GYRO_LN | ICM_PWR_ACCEL_LN);
    delay_ms(15);

    ICM_WriteReg(ICM_REG_GYRO_CONFIG0, ICM_GYRO_FSR_2000DPS | ICM_GYRO_ODR_4KHZ);
    delay_ms(15);
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_FSR_16G | ICM_ACCEL_ODR_4KHZ);
    delay_ms(15);

    ICM_WriteReg(ICM_REG_GYRO_ACCEL_CONFIG0, ICM_FILT_BW_LOW_LATENCY);
    delay_ms(15);

    ICM_SetBank(ICM_BANK_SELECT1);
    ICM_WriteReg(ICM_REG_GYRO_CONFIG_STATIC3, ICM_AAF_DELT);
    ICM_WriteReg(ICM_REG_GYRO_CONFIG_STATIC4, (uint8_t)(ICM_AAF_DELT_SQR & 0xFF));
    ICM_WriteReg(ICM_REG_GYRO_CONFIG_STATIC5, (uint8_t)((ICM_AAF_DELT_SQR >> 8) | (ICM_AAF_BITSHIFT << 4)));

    ICM_SetBank(ICM_BANK_SELECT2);
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG_STATIC2, (uint8_t)(ICM_AAF_DELT << 1));
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG_STATIC3, (uint8_t)(ICM_AAF_DELT_SQR & 0xFF));
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG_STATIC4, (uint8_t)((ICM_AAF_DELT_SQR >> 8) | (ICM_AAF_BITSHIFT << 4)));

    ICM_SetBank(ICM_BANK_SELECT0);
    uint8_t intfConfig1Value = ICM_ReadReg(ICM_REG_INTF_CONFIG1);
    intfConfig1Value &= ~ICM_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM_INTF_CONFIG1_AFSR_DIS;
    ICM_WriteReg(ICM_REG_INTF_CONFIG1, intfConfig1Value);
    delay_ms(15);

    ICM_SetBank(ICM_BANK_SELECT0);
    delay_ms(1);

    s_dma_tx_buf[0] = ICM_REG_TEMP_DATA1 | ICM_SPI_READ;
    for (uint8_t i = 1; i < ICM_DMA_BURST_LEN; i++) {
        s_dma_tx_buf[i] = 0x00;
    }

    SPI1_DMA_Init();

    return ICM_OK;
}

void ICM42605_Convert(const ICM42605_RawData *raw, ICM42605_Data *data)
{
    const float accel_scale = 9.81f / 2048.0f;
    data->accel_x = (float)raw->accel_x * accel_scale;
    data->accel_y = (float)raw->accel_y * accel_scale;
    data->accel_z = (float)raw->accel_z * accel_scale;

    const float gyro_scale = 1.0f / 16.4f;
    data->gyro_x = (float)raw->gyro_x * gyro_scale;
    data->gyro_y = (float)raw->gyro_y * gyro_scale;
    data->gyro_z = (float)raw->gyro_z * gyro_scale;

    data->temp = ((float)raw->temp / 132.48f) + 25.0f;
}

ICM_Status ICM42605_TriggerRead(void)
{
    if (s_dma_busy) {
        return ICM_ERR_BUSY;
    }

    s_dma_busy = 1;

    while (DMA2->S[0].CR & DMA_CR_EN);
    while (DMA2->S[3].CR & DMA_CR_EN);
    SPI1_DMA_Transfer(s_dma_tx_buf, s_dma_rx_buf, ICM_DMA_BURST_LEN, ICM42605_DMA_Callback);

    return ICM_OK;
}

static ICM42605_SampleCallback s_sampleCb = 0;

void ICM42605_SetSampleCallback(ICM42605_SampleCallback cb)
{
    s_sampleCb = cb;
}

static void ICM42605_DMA_Callback(void)
{
    ICM42605_RawData raw;

    raw.temp    = (int16_t)(((uint16_t)s_dma_rx_buf[1]  << 8) | s_dma_rx_buf[2]);
    raw.accel_x = (int16_t)(((uint16_t)s_dma_rx_buf[3]  << 8) | s_dma_rx_buf[4]);
    raw.accel_y = (int16_t)(((uint16_t)s_dma_rx_buf[5]  << 8) | s_dma_rx_buf[6]);
    raw.accel_z = (int16_t)(((uint16_t)s_dma_rx_buf[7]  << 8) | s_dma_rx_buf[8]);
    raw.gyro_x  = (int16_t)(((uint16_t)s_dma_rx_buf[9]  << 8) | s_dma_rx_buf[10]);
    raw.gyro_y  = (int16_t)(((uint16_t)s_dma_rx_buf[11] << 8) | s_dma_rx_buf[12]);
    raw.gyro_z  = (int16_t)(((uint16_t)s_dma_rx_buf[13] << 8) | s_dma_rx_buf[14]);

    ICM42605_Data converted;
    ICM42605_Convert(&raw, &converted);

    s_latest_data = converted;
    s_data_ready  = 1;

    s_dma_busy = 0;

    if (s_sampleCb) s_sampleCb(&converted);
}

uint8_t ICM42605_IsDataReady(void)
{
    return s_data_ready;
}

void ICM42605_GetLatestData(ICM42605_Data *out)
{
    __asm volatile ("cpsid i" ::: "memory");
    *out         = *(ICM42605_Data *)&s_latest_data;
    s_data_ready = 0;
    __asm volatile ("cpsie i" ::: "memory");
}

static bool calibCollect(uint16_t n, bool gyro, double mean[3], double sd[3],
                         float vmin[3], float vmax[3])
{
    double sum[3] = { 0.0, 0.0, 0.0 };
    double sq[3]  = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; i++) { vmin[i] = 1e9f; vmax[i] = -1e9f; }
    ICM42605_Data d;
    uint16_t got = 0;
    uint32_t start = get_tick();

    while (got < n) {
        if ((get_tick() - start) > CALIB_TIMEOUT_MS) return false;
        if (!ICM42605_IsDataReady()) continue;
        ICM42605_GetLatestData(&d);
        ICM42605_RemapAxes(&d);

        double v[3];
        if (gyro) { v[0] = d.gyro_x;  v[1] = d.gyro_y;  v[2] = d.gyro_z;  }
        else      { v[0] = d.accel_x; v[1] = d.accel_y; v[2] = d.accel_z; }
        for (int i = 0; i < 3; i++) {
            sum[i] += v[i];
            sq[i]  += v[i] * v[i];
            if ((float)v[i] < vmin[i]) vmin[i] = (float)v[i];
            if ((float)v[i] > vmax[i]) vmax[i] = (float)v[i];
        }
        got++;
    }

    for (int i = 0; i < 3; i++) {
        mean[i] = sum[i] / n;
        double var = sq[i] / n - mean[i] * mean[i];
        sd[i] = (var > 0.0) ? sqrt(var) : 0.0;
    }
    return true;
}

ICM_CalibStatus_t ICM42605_CalibrateGyro(ICM_Calibration_t *cal, uint16_t samples)
{
    if (!cal || samples < CALIB_MIN_VALID_SAMPLES) {
        return CALIB_ERR_PARAM;
    }

    UART5_WriteF("Gyro calib: keep board STILL...\r\n");

    for (uint8_t attempt = 1; attempt <= CALIB_MAX_ATTEMPTS; attempt++) {
        double mean[3], sd[3];
        float vmin[3], vmax[3];
        if (!calibCollect(samples, true, mean, sd, vmin, vmax)) {
            UART5_WriteF("Gyro calib FAILED: no IMU data\r\n");
            return CALIB_ERR_TIMEOUT;
        }

        UART5_WriteF("  try %u: std X:%.3f Y:%.3f Z:%.3f dps (max %.2f)\r\n",
                     attempt, sd[0], sd[1], sd[2], CALIB_GYRO_STD_MAX);
        UART5_WriteF("         p-p X:%.2f Y:%.2f Z:%.2f dps\r\n",
                     vmax[0] - vmin[0], vmax[1] - vmin[1], vmax[2] - vmin[2]);

        if (sd[0] <= CALIB_GYRO_STD_MAX && sd[1] <= CALIB_GYRO_STD_MAX && sd[2] <= CALIB_GYRO_STD_MAX) {
            cal->gx_offset     = (float)mean[0];
            cal->gy_offset     = (float)mean[1];
            cal->gz_offset     = (float)mean[2];
            cal->valid_samples = samples;
            cal->gyro_done     = true;

            UART5_WriteF("Gyro calib OK (%u samples): GX:%.4f GY:%.4f GZ:%.4f\r\n",
                         samples, cal->gx_offset, cal->gy_offset, cal->gz_offset);
            return CALIB_OK;
        }
        UART5_WriteF("  [!] Motion detected, retrying...\r\n");
    }

    UART5_WriteF("Gyro calib FAILED: board not still\r\n");
    return CALIB_ERR_NOT_STILL;
}

ICM_CalibStatus_t ICM42605_CalibrateAccel(ICM_Calibration_t *cal, uint16_t samples)
{
    if (!cal || samples < CALIB_MIN_VALID_SAMPLES) {
        return CALIB_ERR_PARAM;
    }

    UART5_WriteF("Accel calib: place board FLAT and STILL...\r\n");

    for (uint8_t attempt = 1; attempt <= CALIB_MAX_ATTEMPTS; attempt++) {
        double mean[3], sd[3];
        float vmin[3], vmax[3];
        if (!calibCollect(samples, false, mean, sd, vmin, vmax)) {
            UART5_WriteF("Accel calib FAILED: no IMU data\r\n");
            return CALIB_ERR_TIMEOUT;
        }

        UART5_WriteF("  try %u: std X:%.3f Y:%.3f Z:%.3f m/s2 (max %.2f)\r\n",
                     attempt, sd[0], sd[1], sd[2], CALIB_ACC_STD_MAX);
        UART5_WriteF("         p-p X:%.3f Y:%.3f Z:%.3f m/s2\r\n",
                     vmax[0] - vmin[0], vmax[1] - vmin[1], vmax[2] - vmin[2]);

        if (sd[0] <= CALIB_ACC_STD_MAX && sd[1] <= CALIB_ACC_STD_MAX && sd[2] <= CALIB_ACC_STD_MAX) {
            cal->ax_offset = (float)mean[0];
            cal->ay_offset = (float)mean[1];
            cal->az_offset = (float)mean[2] - 9.81f;
            cal->ax_gain = 1.0f;
            cal->ay_gain = 1.0f;
            cal->az_gain = 1.0f;
            cal->accel_done = true;

            UART5_WriteF("Accel calib OK (%u samples): AX:%.4f AY:%.4f AZ:%.4f\r\n",
                         samples, cal->ax_offset, cal->ay_offset, cal->az_offset);
            return CALIB_OK;
        }
        UART5_WriteF("  [!] Motion detected, retrying...\r\n");
    }

    UART5_WriteF("Accel calib FAILED: board not still\r\n");
    return CALIB_ERR_NOT_STILL;
}

void ICM42605_ApplyCalibration(ICM42605_Data *data, const ICM_Calibration_t *cal)
{
    if (cal->accel_done) {
        data->accel_x = (data->accel_x - cal->ax_offset) * cal->ax_gain;
        data->accel_y = (data->accel_y - cal->ay_offset) * cal->ay_gain;
        data->accel_z = (data->accel_z - cal->az_offset) * cal->az_gain;
    }

    if (cal->gyro_done) {
        data->gyro_x -= cal->gx_offset;
        data->gyro_y -= cal->gy_offset;
        data->gyro_z -= cal->gz_offset;
    }
}

void ICM42605_RemapAxes(ICM42605_Data *data)
{
    float ax = - data->accel_y, ay = - data->accel_x, az = - data->accel_z;
    float gx = - data->gyro_y,  gy = - data->gyro_x,  gz = - data->gyro_z;

    data->accel_x = ax; data->accel_y = ay; data->accel_z = az;
    data->gyro_x = gx; data->gyro_y = gy; data->gyro_z = gz;

}
