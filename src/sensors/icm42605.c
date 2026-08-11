#include "icm42605.h"
#include "register.h"
#include "uart5.h"
#include "spi.h"
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
    /* Mỗi vòng ~1us ở 168MHz, chỉnh hệ số nếu cần */
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
    uint8_t r1  = SPI1_TransmitReceive(reg & ~ICM_SPI_READ); /* bit7 = 0: write */
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
    SPI1_TransmitReceive(reg | ICM_SPI_READ);  /* bit7 = 1: read */
    data = SPI1_TransmitReceive(0x00);         /* dummy byte */
    CS_High();
    delay_us(1);
    return data;
}

static void ICM_SetBank(uint8_t bank)
{
    ICM_WriteReg(ICM_REG_BANK_SEL, bank);
}

static void ICM_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CS_Low();
    SPI1_TransmitReceive(reg | ICM_SPI_READ);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI1_TransmitReceive(0x00);
    }
    CS_High();
    delay_us(1);
}

ICM_Status ICM42605_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    __asm volatile ("nop");
    __asm volatile ("nop");
    
    /* set up PC14 (CS) */
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

    ICM_WriteReg(ICM_REG_GYRO_CONFIG0, ICM_GYRO_FSR_2000DPS | ICM_GYRO_ODR_1KHZ);
    delay_ms(15);
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_FSR_16G | ICM_ACCEL_ODR_1KHZ);
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

uint8_t ICM42605_WhoAmI(void)
{
    return ICM_ReadReg(ICM_REG_WHO_AM_I);
}

void ICM42605_ReadRaw(ICM42605_RawData *data)
{
    /* Burst read 14 bytes: TEMP(2) + ACCEL(6) + GYRO(6)
     * Bắt đầu từ TEMP_DATA1 (0x1D) */
    uint8_t buf[14];
    ICM_ReadBurst(ICM_REG_TEMP_DATA1, buf, 14);

    data->temp    = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    data->accel_x = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    data->accel_y = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    data->accel_z = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    data->gyro_x  = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    data->gyro_y  = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
}

void ICM42605_Convert(const ICM42605_RawData *raw, ICM42605_Data *data)
{
    /* Accel: FSR ±16g */
    const float accel_scale = 9.81f / 2048.0f;
    data->accel_x = (float)raw->accel_x * accel_scale;
    data->accel_y = (float)raw->accel_y * accel_scale;
    data->accel_z = (float)raw->accel_z * accel_scale;

    /* Gyro: FSR ±2000dps → sensitivity = 16.4 LSB/(deg/s)
     * Quy đổi sang deg/s: chia cho 16.4 */
    const float gyro_scale = 1.0f / 16.4f;
    data->gyro_x = (float)raw->gyro_x * gyro_scale;
    data->gyro_y = (float)raw->gyro_y * gyro_scale;
    data->gyro_z = (float)raw->gyro_z * gyro_scale;

    /*T(°C) = (raw / 132.48) + 25 */
    data->temp = ((float)raw->temp / 132.48f) + 25.0f;
}

void ICM42605_ReadAll(ICM42605_Data *data)
{
    ICM42605_RawData raw;
    ICM42605_ReadRaw(&raw);
    ICM42605_Convert(&raw, data);
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
}
 
uint8_t ICM42605_IsDataReady(void)
{
    return s_data_ready;
}
 
void ICM42605_GetLatestData(ICM42605_Data *out)
{
    __asm volatile ("cpsid i" ::: "memory");   /* disable IRQ */
    *out         = *(ICM42605_Data *)&s_latest_data;
    s_data_ready = 0;
    __asm volatile ("cpsie i" ::: "memory");   /* enable IRQ */
}

ICM_CalibStatus_t ICM42605_CalibrateGyro(ICM_Calibration_t *cal, uint16_t samples)
{
    if (!cal || samples < CALIB_MIN_VALID_SAMPLES) {
        return CALIB_ERR_PARAM;
    }

    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double mean_gx = 0, mean_gy = 0, mean_gz = 0;
    uint16_t total     = 0;   /* tổng samples đọc */
    uint16_t valid     = 0;   /* samples hợp lệ   */
    uint16_t max_total = samples * 3; /* cho phép thử 3x */

    ICM42605_Data data;

    UART5_WriteF("Gyro calib: keep board STILL...\r\n");

    while (total < max_total && valid < samples) {

        /* Chờ data mới từ DMA */
        if (!ICM42605_IsDataReady()) continue;
        ICM42605_GetLatestData(&data);
        total++;

        /* Pass 1: sample đầu tiên làm baseline */
        if (valid == 0) {
            mean_gx = data.gyro_x;
            mean_gy = data.gyro_y;
            mean_gz = data.gyro_z;
            sum_gx  = data.gyro_x;
            sum_gy  = data.gyro_y;
            sum_gz  = data.gyro_z;
            valid   = 1;
            continue;
        }

        /* Học INAV MORON_THRESHOLD:
         * So sample với mean hiện tại
         * Nếu lệch quá → board đang rung → reject */
        float dx = fabsf(data.gyro_x - (float)mean_gx);
        float dy = fabsf(data.gyro_y - (float)mean_gy);
        float dz = fabsf(data.gyro_z - (float)mean_gz);

        if (dx > CALIB_GYRO_MORON_THRESHOLD ||
            dy > CALIB_GYRO_MORON_THRESHOLD ||
            dz > CALIB_GYRO_MORON_THRESHOLD)
        {
            /* Sample bị reject - reset để bắt đầu lại */
            sum_gx = 0; sum_gy = 0; sum_gz = 0;
            mean_gx = 0; mean_gy = 0; mean_gz = 0;
            valid = 0;
            UART5_WriteF("  [!] Motion detected, retrying...\r\n");
            continue;
        }

        /* Sample hợp lệ - cộng dồn */
        sum_gx += data.gyro_x;
        sum_gy += data.gyro_y;
        sum_gz += data.gyro_z;
        valid++;

        /* Cập nhật running mean */
        mean_gx = sum_gx / valid;
        mean_gy = sum_gy / valid;
        mean_gz = sum_gz / valid;
    }

    /* Kiểm tra đủ samples không */
    if (valid < CALIB_MIN_VALID_SAMPLES) {
        UART5_WriteF("Gyro calib FAILED: board not still (%d valid)\r\n",
                     valid);
        return CALIB_ERR_NOT_STILL;
    }

    cal->gx_offset    = (float)mean_gx;
    cal->gy_offset    = (float)mean_gy;
    cal->gz_offset    = (float)mean_gz;
    cal->valid_samples = valid;
    cal->gyro_done    = true;

    UART5_WriteF("Gyro calib OK (%d samples): "
                 "GX:%.4f GY:%.4f GZ:%.4f\r\n",
                 valid,
                 cal->gx_offset,
                 cal->gy_offset,
                 cal->gz_offset);

    return CALIB_OK;
}

ICM_CalibStatus_t ICM42605_CalibrateAccel(ICM_Calibration_t *cal, uint16_t samples)
{
    if (!cal || samples < CALIB_MIN_VALID_SAMPLES) {
        return CALIB_ERR_PARAM;
    }

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double mean_ax = 0, mean_ay = 0, mean_az = 0;
    uint16_t total     = 0;
    uint16_t valid     = 0;
    uint16_t max_total = samples * 3;

    ICM42605_Data data;

    UART5_WriteF("Accel calib: place board FLAT and STILL...\r\n");

    while (total < max_total && valid < samples) {

        if (!ICM42605_IsDataReady()) continue;
        ICM42605_GetLatestData(&data);
        total++;

        if (valid == 0) {
            mean_ax = data.accel_x;
            mean_ay = data.accel_y;
            mean_az = data.accel_z;
            sum_ax  = data.accel_x;
            sum_ay  = data.accel_y;
            sum_az  = data.accel_z;
            valid   = 1;
            continue;
        }

        /* MORON_THRESHOLD cho accel */
        float dx = fabsf(data.accel_x - (float)mean_ax);
        float dy = fabsf(data.accel_y - (float)mean_ay);
        float dz = fabsf(data.accel_z - (float)mean_az);

        if (dx > CALIB_ACC_MORON_THRESHOLD ||
            dy > CALIB_ACC_MORON_THRESHOLD ||
            dz > CALIB_ACC_MORON_THRESHOLD)
        {
            sum_ax = 0; sum_ay = 0; sum_az = 0;
            mean_ax = 0; mean_ay = 0; mean_az = 0;
            valid = 0;
            UART5_WriteF("  [!] Motion detected, retrying...\r\n");
            continue;
        }

        sum_ax += data.accel_x;
        sum_ay += data.accel_y;
        sum_az += data.accel_z;
        valid++;

        mean_ax = sum_ax / valid;
        mean_ay = sum_ay / valid;
        mean_az = sum_az / valid;
    }

    if (valid < CALIB_MIN_VALID_SAMPLES) {
        UART5_WriteF("Accel calib FAILED: board not still (%d valid)\r\n",
                     valid);
        return CALIB_ERR_NOT_STILL;
    }


    cal->ax_offset = (float)mean_ax;          /*  0    */
    cal->ay_offset = (float)mean_ay;          /* 0    */
    cal->az_offset = (float)mean_az - 9.81f;  /*  9.81 */

    cal->ax_gain = 1.0f;
    cal->ay_gain = 1.0f;
    cal->az_gain = 1.0f;

    cal->accel_done = true;

    UART5_WriteF("Accel calib OK (%d samples): "
                 "AX:%.4f AY:%.4f AZ:%.4f\r\n",
                 valid,
                 cal->ax_offset,
                 cal->ay_offset,
                 cal->az_offset);

    return CALIB_OK;
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

void ICM42605_RemapAxes(ICM42605_Data *data, BoardAlignment_t align)
{
    float ax = data->accel_x, ay = data->accel_y, az = data->accel_z;
    float gx = data->gyro_x,  gy = data->gyro_y,  gz = data->gyro_z;

    switch (align) {
        case ALIGN_CW0:
            break;
        case ALIGN_CW90:
            data->accel_x =  ay; data->accel_y = -ax;
            data->gyro_x  =  gy; data->gyro_y  = -gx;
            break;
        case ALIGN_CW180:
            data->accel_x = -ax; data->accel_y = -ay;
            data->gyro_x  = -gx; data->gyro_y  = -gy;
            break;
        case ALIGN_CW270:
            data->accel_x = -ay; data->accel_y =  ax;
            data->gyro_x  = -gy; data->gyro_y  =  gx;
            break;
        case ALIGN_CW0_FLIP:
            data->accel_x =  ax;  data->accel_y = -ay;  data->accel_z = -az;
            data->gyro_x  =  gx;  data->gyro_y  = -gy;   data->gyro_z  = -gz;
            break;

        case ALIGN_CW90_FLIP:
            data->accel_x = -ay;  data->accel_y = -ax;  data->accel_z = -az;
            data->gyro_x  = -gy;  data->gyro_y  = -gx;   data->gyro_z  = -gz;
            break;

        case ALIGN_CW180_FLIP:
            data->accel_x = -ax;  data->accel_y =  ay;  data->accel_z = -az;
            data->gyro_x  = -gx;  data->gyro_y  =  gy;   data->gyro_z  = -gz;
            break;

        case ALIGN_CW270_FLIP:
            data->accel_x =  ay;  data->accel_y =  ax;  data->accel_z = -az;
            data->gyro_x  =  gy;  data->gyro_y  =  gx;   data->gyro_z  = -gz;
            break;
        case MY_CASE_NED:
            data->accel_x = -ay;  data->accel_y = -ax;  data->accel_z = az;
            data->gyro_x  = -gy;  data->gyro_y  = -gx;   data->gyro_z  = -gz;
            break;
        default: break;
    }
}


