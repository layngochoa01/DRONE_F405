#ifndef ICM42605_H
#define ICM42605_H

#include <stdint.h>
#include <stdbool.h>

#define ICM_BANK_SELECT0            0x00
#define ICM_BANK_SELECT1            0x01
#define ICM_BANK_SELECT2            0x02

#define ICM_REG_INT_CONFIG0         0x63
#define ICM_REG_INT_CONFIG1         0x64
#define ICM_REG_INT_SOURCE0         0x65

#define ICM_REG_GYRO_CONFIG_STATIC3 0x0C
#define ICM_REG_GYRO_CONFIG_STATIC4 0x0D
#define ICM_REG_GYRO_CONFIG_STATIC5 0x0E

#define ICM_REG_ACCEL_CONFIG_STATIC2 0x03
#define ICM_REG_ACCEL_CONFIG_STATIC3 0x04
#define ICM_REG_ACCEL_CONFIG_STATIC4 0x05

#define ICM_REG_DEVICE_CONFIG       0x11
#define ICM_REG_DRIVE_CONFIG        0x13
#define ICM_REG_INT_CONFIG          0x14
#define ICM_REG_FIFO_CONFIG         0x16

#define ICM_REG_TEMP_DATA1          0x1D
#define ICM_REG_TEMP_DATA0          0x1E
#define ICM_REG_ACCEL_DATA_X1       0x1F
#define ICM_REG_ACCEL_DATA_X0       0x20
#define ICM_REG_ACCEL_DATA_Y1       0x21
#define ICM_REG_ACCEL_DATA_Y0       0x22
#define ICM_REG_ACCEL_DATA_Z1       0x23
#define ICM_REG_ACCEL_DATA_Z0       0x24
#define ICM_REG_GYRO_DATA_X1        0x25
#define ICM_REG_GYRO_DATA_X0        0x26
#define ICM_REG_GYRO_DATA_Y1        0x27
#define ICM_REG_GYRO_DATA_Y0        0x28
#define ICM_REG_GYRO_DATA_Z1        0x29
#define ICM_REG_GYRO_DATA_Z0        0x2A

#define ICM_REG_INT_STATUS          0x2D
#define ICM_REG_FIFO_COUNTH         0x2E
#define ICM_REG_FIFO_COUNTL         0x2F
#define ICM_REG_FIFO_DATA           0x30
#define ICM_REG_SIGNAL_PATH_RESET   0x4B
#define ICM_REG_INTF_CONFIG0        0x4C
#define ICM_REG_INTF_CONFIG1        0x4D
#define ICM_REG_PWR_MGMT0           0x4E
#define ICM_REG_GYRO_CONFIG0        0x4F
#define ICM_REG_ACCEL_CONFIG0       0x50
#define ICM_REG_GYRO_CONFIG1        0x51
#define ICM_REG_GYRO_ACCEL_CONFIG0  0x52
#define ICM_REG_ACCEL_CONFIG1       0x53
#define ICM_REG_WHO_AM_I            0x75
#define ICM_REG_BANK_SEL            0x76

#define ICM_WHO_AM_I_VALUE          0x42U

#define ICM_DEVICE_CONFIG_RESET     (1U << 0)

#define ICM_PWR_TEMP_DIS            (1U << 5)
#define ICM_PWR_IDLE                (1U << 4)
#define ICM_PWR_GYRO_OFF            (0x0U << 2)
#define ICM_PWR_GYRO_STANDBY        (0x1U << 2)
#define ICM_PWR_GYRO_LN             (0x3U << 2)
#define ICM_PWR_ACCEL_OFF           (0x0U << 0)
#define ICM_PWR_ACCEL_LP            (0x2U << 0)
#define ICM_PWR_ACCEL_LN            (0x3U << 0)

#define ICM_GYRO_FSR_2000DPS        (0x0U << 5)
#define ICM_GYRO_FSR_1000DPS        (0x1U << 5)
#define ICM_GYRO_FSR_500DPS         (0x2U << 5)
#define ICM_GYRO_FSR_250DPS         (0x3U << 5)
#define ICM_GYRO_ODR_4KHZ           (0x4U << 0)
#define ICM_GYRO_ODR_1KHZ           (0x6U << 0)
#define ICM_GYRO_ODR_200HZ          (0x7U << 0)

#define ICM_ACCEL_FSR_16G           (0x0U << 5)
#define ICM_ACCEL_FSR_8G            (0x1U << 5)
#define ICM_ACCEL_FSR_4G            (0x2U << 5)
#define ICM_ACCEL_FSR_2G            (0x3U << 5)
#define ICM_ACCEL_ODR_4KHZ          (0x4U << 0)
#define ICM_ACCEL_ODR_1KHZ          (0x6U << 0)
#define ICM_ACCEL_ODR_200HZ         (0x7U << 0)

#define ICM_SPI_READ                (1U << 7)
#define ICM_SPI_WRITE               (0U << 7)

#define ICM_FILT_BW_LOW_LATENCY     0xFF

#define ICM_INT1_MODE_PULSED        (0U << 2)
#define ICM_INT1_DRIVE_PP           (1U << 1)
#define ICM_INT1_POLARITY_HIGH      (1U << 0)

#define ICM_UI_DRDY_CLEAR_ON_SBR    0x00
#define ICM_UI_DRDY_INT1_EN         (1U << 3)

#define ICM_INTF_CONFIG1_AFSR_MASK  0xC0
#define ICM_INTF_CONFIG1_AFSR_DIS   0x40

#define ICM_DMA_BURST_LEN           15U

typedef enum {
    ICM_OK = 0,
    ICM_ERR_SPI_INIT,
    ICM_ERR_DMA_INIT,
    ICM_ERR_WHO_AM_I,
    ICM_ERR_PWR_MGMT,
    ICM_ERR_CONFIG,
    ICM_ERR_DMA_TIMEOUT,
    ICM_ERR_BUSY
} ICM_Status;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} ICM42605_RawData;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} ICM42605_Data;

void ICM42605_RemapAxes(ICM42605_Data *data);

ICM_Status ICM42605_Init(void);

void ICM42605_Convert(const ICM42605_RawData *raw, ICM42605_Data *data);

ICM_Status ICM42605_TriggerRead(void);

uint8_t ICM42605_IsDataReady(void);

void ICM42605_GetLatestData(ICM42605_Data *out);

typedef void (*ICM42605_SampleCallback)(const ICM42605_Data *sample);
void ICM42605_SetSampleCallback(ICM42605_SampleCallback cb);

#define CALIB_GYRO_STD_MAX           1.5f
#define CALIB_ACC_STD_MAX            0.10f
#define CALIB_MAX_ATTEMPTS           3U
#define CALIB_TIMEOUT_MS             5000U
#define CALIB_MIN_VALID_SAMPLES      100U

typedef struct {
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;

    float ax_gain, ay_gain, az_gain;

    bool  gyro_done;
    bool  accel_done;
    uint16_t valid_samples;
} ICM_Calibration_t;

typedef enum {
    CALIB_OK = 0,
    CALIB_ERR_NOT_STILL,
    CALIB_ERR_TIMEOUT,
    CALIB_ERR_PARAM
} ICM_CalibStatus_t;

ICM_CalibStatus_t ICM42605_CalibrateGyro(ICM_Calibration_t *cal, uint16_t samples);

ICM_CalibStatus_t ICM42605_CalibrateAccel(ICM_Calibration_t *cal, uint16_t samples);

void ICM42605_ApplyCalibration(ICM42605_Data *data, const ICM_Calibration_t *cal);

#endif
