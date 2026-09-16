#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdint.h>
#include <stdbool.h>

/* HMC5883L uses a 7-bit I2C address. */
#define HMC5883L_I2C_ADDR              0x1EU

/* Register map. */
#define HMC5883L_REG_CONFIG_A          0x00U
#define HMC5883L_REG_CONFIG_B          0x01U
#define HMC5883L_REG_MODE              0x02U
#define HMC5883L_REG_DATA_X_MSB        0x03U
#define HMC5883L_REG_DATA_X_LSB        0x04U
#define HMC5883L_REG_DATA_Z_MSB        0x05U
#define HMC5883L_REG_DATA_Z_LSB        0x06U
#define HMC5883L_REG_DATA_Y_MSB        0x07U
#define HMC5883L_REG_DATA_Y_LSB        0x08U
#define HMC5883L_REG_STATUS            0x09U
#define HMC5883L_REG_ID_A              0x0AU
#define HMC5883L_REG_ID_B              0x0BU
#define HMC5883L_REG_ID_C              0x0CU

/* Configuration A: averaging, output rate and bias mode. */
#define HMC5883L_AVG_1                 (0U << 5)
#define HMC5883L_AVG_2                 (1U << 5)
#define HMC5883L_AVG_4                 (2U << 5)
#define HMC5883L_AVG_8                 (3U << 5)
#define HMC5883L_RATE_0_75_HZ          (0U << 2)
#define HMC5883L_RATE_1_5_HZ           (1U << 2)
#define HMC5883L_RATE_3_HZ             (2U << 2)
#define HMC5883L_RATE_7_5_HZ           (3U << 2)
#define HMC5883L_RATE_15_HZ            (4U << 2)
#define HMC5883L_RATE_30_HZ            (5U << 2)
#define HMC5883L_RATE_75_HZ            (6U << 2)
#define HMC5883L_BIAS_NORMAL           0U
#define HMC5883L_BIAS_POSITIVE         1U
#define HMC5883L_BIAS_NEGATIVE         2U

/* Configuration B: gain and sensitivity in LSB/Gauss. */
#define HMC5883L_GAIN_0_88GA           (0U << 5)
#define HMC5883L_GAIN_1_3GA            (1U << 5)
#define HMC5883L_GAIN_1_9GA            (2U << 5)
#define HMC5883L_GAIN_2_5GA            (3U << 5)
#define HMC5883L_GAIN_4_0GA            (4U << 5)
#define HMC5883L_GAIN_4_7GA            (5U << 5)
#define HMC5883L_GAIN_5_6GA            (6U << 5)
#define HMC5883L_GAIN_8_1GA            (7U << 5)
#define HMC5883L_LSB_PER_GAUSS_0_88    1370.0f
#define HMC5883L_LSB_PER_GAUSS_1_3     1090.0f
#define HMC5883L_LSB_PER_GAUSS_1_9     820.0f
#define HMC5883L_LSB_PER_GAUSS_2_5     660.0f
#define HMC5883L_LSB_PER_GAUSS_4_0     440.0f
#define HMC5883L_LSB_PER_GAUSS_4_7     390.0f
#define HMC5883L_LSB_PER_GAUSS_5_6     330.0f
#define HMC5883L_LSB_PER_GAUSS_8_1     230.0f

#define HMC5883L_MODE_CONTINUOUS       0x00U
#define HMC5883L_MODE_SINGLE           0x01U
#define HMC5883L_MODE_IDLE             0x02U
#define HMC5883L_MODE_IDLE_2           0x03U

#define HMC5883L_STATUS_LOCK            (1U << 1)
#define HMC5883L_STATUS_READY          (1U << 0)
#define HMC5883L_RAW_SATURATION        (-4096)

#define HMC5883L_WARNING_X_SATURATED   (1U << 0)
#define HMC5883L_WARNING_Y_SATURATED   (1U << 1)
#define HMC5883L_WARNING_Z_SATURATED   (1U << 2)

typedef enum {
    HMC5883L_OK = 0,
    HMC5883L_ERR_I2C,
    HMC5883L_ERR_DEVICE_ID,
    HMC5883L_ERR_INVALID_ARGUMENT,
    HMC5883L_ERR_INVALID_CONFIG
} HMC5883L_Status;

typedef struct {
    uint8_t config_a;
    uint8_t config_b;
    uint8_t mode;
    float lsb_per_gauss;
} HMC5883L_Config;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883L_RawData;

typedef struct {
    float x;
    float y;
    float z;
} HMC5883L_Data;

HMC5883L_Status HMC5883L_Init(void);
HMC5883L_Status HMC5883L_InitWithConfig(const HMC5883L_Config *config);
HMC5883L_Status HMC5883L_Configure(const HMC5883L_Config *config);
HMC5883L_Config HMC5883L_DefaultConfig(void);
HMC5883L_Status HMC5883L_ReadRaw(HMC5883L_RawData *data);
HMC5883L_Status HMC5883L_Read(HMC5883L_Data *data);
HMC5883L_Status HMC5883L_ReadId(uint8_t id[3]);
HMC5883L_Status HMC5883L_ReadStatus(uint8_t *status);
HMC5883L_Status HMC5883L_StartSingleMeasurement(void);
bool HMC5883L_IsDataReady(void);
uint8_t HMC5883L_GetSaturationWarnings(const HMC5883L_RawData *data);
void HMC5883L_Convert(const HMC5883L_RawData *raw,
                      HMC5883L_Data *data,
                      float lsb_per_gauss);

#endif
