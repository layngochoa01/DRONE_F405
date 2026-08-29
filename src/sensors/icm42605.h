#ifndef ICM42605_H
#define ICM42605_H

#include <stdint.h>
#include <stdbool.h> 

/* Bank Select */
#define ICM_BANK_SELECT0            0x00
#define ICM_BANK_SELECT1            0x01
#define ICM_BANK_SELECT2            0x02

/* Interrupt Registers (Bank 0) */
#define ICM_REG_INT_CONFIG0         0x63
#define ICM_REG_INT_CONFIG1         0x64
#define ICM_REG_INT_SOURCE0         0x65

/* AAF Registers - Bank 1 (Gyro) */
#define ICM_REG_GYRO_CONFIG_STATIC3 0x0C
#define ICM_REG_GYRO_CONFIG_STATIC4 0x0D
#define ICM_REG_GYRO_CONFIG_STATIC5 0x0E

/* AAF Registers - Bank 2 (Accel) */
#define ICM_REG_ACCEL_CONFIG_STATIC2 0x03
#define ICM_REG_ACCEL_CONFIG_STATIC3 0x04
#define ICM_REG_ACCEL_CONFIG_STATIC4 0x05

/* Bank 0 registers */
#define ICM_REG_DEVICE_CONFIG       0x11    /* Reset, SPI mode */
#define ICM_REG_DRIVE_CONFIG        0x13    /* Drive config */
#define ICM_REG_INT_CONFIG          0x14    /* Interrupt config */
#define ICM_REG_FIFO_CONFIG         0x16    /* FIFO config */

/* Sensor data registers - Bank 0 */
#define ICM_REG_TEMP_DATA1          0x1D    /* Temp high byte */
#define ICM_REG_TEMP_DATA0          0x1E    /* Temp low byte */
#define ICM_REG_ACCEL_DATA_X1       0x1F    /* Accel X high */
#define ICM_REG_ACCEL_DATA_X0       0x20    /* Accel X low */
#define ICM_REG_ACCEL_DATA_Y1       0x21    /* Accel Y high */
#define ICM_REG_ACCEL_DATA_Y0       0x22    /* Accel Y low */
#define ICM_REG_ACCEL_DATA_Z1       0x23    /* Accel Z high */
#define ICM_REG_ACCEL_DATA_Z0       0x24    /* Accel Z low */
#define ICM_REG_GYRO_DATA_X1        0x25    /* Gyro X high */
#define ICM_REG_GYRO_DATA_X0        0x26    /* Gyro X low */
#define ICM_REG_GYRO_DATA_Y1        0x27    /* Gyro Y high */
#define ICM_REG_GYRO_DATA_Y0        0x28    /* Gyro Y low */
#define ICM_REG_GYRO_DATA_Z1        0x29    /* Gyro Z high */
#define ICM_REG_GYRO_DATA_Z0        0x2A    /* Gyro Z low */

#define ICM_REG_INT_STATUS          0x2D    /* Interrupt status */
#define ICM_REG_FIFO_COUNTH         0x2E    /* FIFO count high */
#define ICM_REG_FIFO_COUNTL         0x2F    /* FIFO count low */
#define ICM_REG_FIFO_DATA           0x30    /* FIFO data */
#define ICM_REG_SIGNAL_PATH_RESET   0x4B    /* Signal path reset */
#define ICM_REG_INTF_CONFIG0        0x4C    /* Interface config */
#define ICM_REG_INTF_CONFIG1        0x4D    /* Interface config 1 */
#define ICM_REG_PWR_MGMT0           0x4E    /* Power management */
#define ICM_REG_GYRO_CONFIG0        0x4F    /* Gyro FSR & ODR */
#define ICM_REG_ACCEL_CONFIG0       0x50    /* Accel FSR & ODR */
#define ICM_REG_GYRO_CONFIG1        0x51    /* Gyro filter config */
#define ICM_REG_GYRO_ACCEL_CONFIG0  0x52    /* Accel filter config */
#define ICM_REG_ACCEL_CONFIG1       0x53    /* Accel filter config 1 */
#define ICM_REG_WHO_AM_I            0x75    /* Device ID = 0x42 */
#define ICM_REG_BANK_SEL            0x76    /* Register bank select */

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

/* GYRO_CONFIG0 - FSR | ODR */
#define ICM_GYRO_FSR_2000DPS        (0x0U << 5) /* ±2000 dps */
#define ICM_GYRO_FSR_1000DPS        (0x1U << 5) /* ±1000 dps */
#define ICM_GYRO_FSR_500DPS         (0x2U << 5) /* ±500  dps */
#define ICM_GYRO_FSR_250DPS         (0x3U << 5) /* ±250  dps */
#define ICM_GYRO_ODR_1KHZ           (0x6U << 0) /* 1kHz */
#define ICM_GYRO_ODR_200HZ          (0x7U << 0) /* 200Hz */

/* ACCEL_CONFIG0 - FSR | ODR */
#define ICM_ACCEL_FSR_16G           (0x0U << 5) /* ±16g */
#define ICM_ACCEL_FSR_8G            (0x1U << 5) /* ±8g  */
#define ICM_ACCEL_FSR_4G            (0x2U << 5) /* ±4g  */
#define ICM_ACCEL_FSR_2G            (0x3U << 5) /* ±2g  */
#define ICM_ACCEL_ODR_1KHZ          (0x6U << 0) /* 1kHz */
#define ICM_ACCEL_ODR_200HZ         (0x7U << 0) /* 200Hz */

/* SPI read/write bit */
#define ICM_SPI_READ                (1U << 7)   /* Bit 7 = 1: đọc */
#define ICM_SPI_WRITE               (0U << 7)   /* Bit 7 = 0: ghi */

/* GYRO_ACCEL_CONFIG0 - Low Latency */
#define ICM_FILT_BW_LOW_LATENCY     0xFF  /* (15 << 4) cho Accel | (15 << 0) cho Gyro */

/* INT_CONFIG - Push-Pull, Active High, Pulsed */
#define ICM_INT1_MODE_PULSED        (0U << 2)
#define ICM_INT1_DRIVE_PP           (1U << 1)
#define ICM_INT1_POLARITY_HIGH      (1U << 0)

/* INT_CONFIG0 & SOURCE0 */
#define ICM_UI_DRDY_CLEAR_ON_SBR    0x00
#define ICM_UI_DRDY_INT1_EN         (1U << 3)

/* INTF_CONFIG1 - AFSR */
#define ICM_INTF_CONFIG1_AFSR_MASK  0xC0
#define ICM_INTF_CONFIG1_AFSR_DIS   0x40

#define ICM_DMA_BURST_LEN           15U   
 
typedef enum {
    ICM_OK = 0,
    ICM_ERR_SPI_INIT,       // Lỗi khởi tạo ngoại vi SPI
    ICM_ERR_DMA_INIT,       // Lỗi cấu hình các kênh DMA
    ICM_ERR_WHO_AM_I,       // Lỗi không đọc được hoặc sai mã WHO_AM_I (Lỗi kết nối phần cứng)
    ICM_ERR_PWR_MGMT,       // Lỗi cấu hình nguồn/vòng bọc xung (Clock source) cho IMU
    ICM_ERR_CONFIG,         // Lỗi ghi các thanh ghi cấu hình (ODR, FSR)
    ICM_ERR_DMA_TIMEOUT,    // Lỗi DMA bị treo không hoàn thành truyền nhận
    ICM_ERR_BUSY            // Cảm biến đang bận xử lý lượt đọc trước
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

void ICM42605_ReadRaw(ICM42605_RawData *data);

void ICM42605_Convert(const ICM42605_RawData *raw, ICM42605_Data *data);


void ICM42605_ReadAll(ICM42605_Data *data);

uint8_t ICM42605_WhoAmI(void);

ICM_Status ICM42605_TriggerRead(void);

uint8_t ICM42605_IsDataReady(void);

void ICM42605_GetLatestData(ICM42605_Data *out);

#define CALIB_GYRO_MORON_THRESHOLD   1.0f
#define CALIB_ACC_MORON_THRESHOLD    (9.81f * 0.05f)
#define CALIB_MIN_VALID_SAMPLES      100U

typedef struct {
    /* Offset (zero calibration) */
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;

    /* Gain (scale calibration) - học từ INAV accGain */
    float ax_gain, ay_gain, az_gain;   /* mặc định 1.0 */

    /* Trạng thái */
    bool  gyro_done;
    bool  accel_done;
    uint16_t valid_samples;   /* số sample hợp lệ thực tế */
} ICM_Calibration_t;

typedef enum {
    CALIB_OK = 0,
    CALIB_ERR_NOT_STILL,    /* board rung quá, không đủ samples */
    CALIB_ERR_TIMEOUT,
    CALIB_ERR_PARAM
} ICM_CalibStatus_t;


ICM_CalibStatus_t ICM42605_CalibrateGyro(ICM_Calibration_t *cal, uint16_t samples);


ICM_CalibStatus_t ICM42605_CalibrateAccel(ICM_Calibration_t *cal, uint16_t samples);

void ICM42605_ApplyCalibration(ICM42605_Data *data, const ICM_Calibration_t *cal);

#endif 