#include "hmc5883l.h"
#include "I2C.h"

static float s_lsb_per_gauss = HMC5883L_LSB_PER_GAUSS_1_3;

static int16_t hmc5883l_combine(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

HMC5883L_Status HMC5883L_ReadId(uint8_t id[3])
{
    if (!id) {
        return HMC5883L_ERR_INVALID_ARGUMENT;
    }

    if (!I2C1_ReadRegs(HMC5883L_I2C_ADDR, HMC5883L_REG_ID_A, id, 3U)) {
        return HMC5883L_ERR_I2C;
    }

    return (id[0] == 'H' && id[1] == '4' && id[2] == '3')
        ? HMC5883L_OK
        : HMC5883L_ERR_DEVICE_ID;
}

HMC5883L_Config HMC5883L_DefaultConfig(void)
{
    HMC5883L_Config config = {
        HMC5883L_AVG_8 | HMC5883L_RATE_15_HZ | HMC5883L_BIAS_NORMAL,
        HMC5883L_GAIN_1_3GA,
        HMC5883L_MODE_CONTINUOUS,
        HMC5883L_LSB_PER_GAUSS_1_3
    };
    return config;
}

HMC5883L_Status HMC5883L_Configure(const HMC5883L_Config *config)
{
    if (!config || config->lsb_per_gauss <= 0.0f ||
        (config->config_a & 0x80U) != 0U ||
        (config->config_a & 0x03U) == 3U ||
        (config->config_b & 0x1FU) != 0U ||
        (config->mode & 0xFCU) != 0U) {
        return HMC5883L_ERR_INVALID_CONFIG;
    }

    if (!I2C1_WriteReg(HMC5883L_I2C_ADDR, HMC5883L_REG_CONFIG_A,
                       config->config_a) ||
        !I2C1_WriteReg(HMC5883L_I2C_ADDR, HMC5883L_REG_CONFIG_B,
                       config->config_b) ||
        !I2C1_WriteReg(HMC5883L_I2C_ADDR, HMC5883L_REG_MODE,
                       config->mode)) {
        return HMC5883L_ERR_I2C;
    }

    s_lsb_per_gauss = config->lsb_per_gauss;
    return HMC5883L_OK;
}

HMC5883L_Status HMC5883L_InitWithConfig(const HMC5883L_Config *config)
{
    uint8_t id[3];
    HMC5883L_Status status;

    I2C1_Init();

    status = HMC5883L_ReadId(id);
    if (status != HMC5883L_OK) {
        return status;
    }

    return HMC5883L_Configure(config);
}

HMC5883L_Status HMC5883L_Init(void)
{
    HMC5883L_Config config = HMC5883L_DefaultConfig();
    return HMC5883L_InitWithConfig(&config);
}

HMC5883L_Status HMC5883L_ReadStatus(uint8_t *status)
{
    if (!status) {
        return HMC5883L_ERR_INVALID_ARGUMENT;
    }

    return I2C1_ReadReg(HMC5883L_I2C_ADDR, HMC5883L_REG_STATUS, status)
        ? HMC5883L_OK
        : HMC5883L_ERR_I2C;
}

HMC5883L_Status HMC5883L_StartSingleMeasurement(void)
{
    return I2C1_WriteReg(HMC5883L_I2C_ADDR, HMC5883L_REG_MODE,
                         HMC5883L_MODE_SINGLE)
        ? HMC5883L_OK
        : HMC5883L_ERR_I2C;
}

bool HMC5883L_IsDataReady(void)
{
    uint8_t status = 0U;
    return HMC5883L_ReadStatus(&status) == HMC5883L_OK &&
           (status & HMC5883L_STATUS_READY) != 0U;
}

HMC5883L_Status HMC5883L_ReadRaw(HMC5883L_RawData *data)
{
    uint8_t buffer[6];

    if (!data) {
        return HMC5883L_ERR_INVALID_ARGUMENT;
    }

    if (!I2C1_ReadRegs(HMC5883L_I2C_ADDR,
                       HMC5883L_REG_DATA_X_MSB,
                       buffer,
                       sizeof(buffer))) {
        return HMC5883L_ERR_I2C;
    }

    /* HMC5883L register order is X, Z, Y, not X, Y, Z. */
    data->x = hmc5883l_combine(buffer[0], buffer[1]);
    data->z = hmc5883l_combine(buffer[2], buffer[3]);
    data->y = hmc5883l_combine(buffer[4], buffer[5]);

    return HMC5883L_OK;
}

uint8_t HMC5883L_GetSaturationWarnings(const HMC5883L_RawData *data)
{
    uint8_t warnings = 0U;

    if (!data) {
        return 0U;
    }
    if (data->x == HMC5883L_RAW_SATURATION) {
        warnings |= HMC5883L_WARNING_X_SATURATED;
    }
    if (data->y == HMC5883L_RAW_SATURATION) {
        warnings |= HMC5883L_WARNING_Y_SATURATED;
    }
    if (data->z == HMC5883L_RAW_SATURATION) {
        warnings |= HMC5883L_WARNING_Z_SATURATED;
    }
    return warnings;
}

void HMC5883L_Convert(const HMC5883L_RawData *raw,
                      HMC5883L_Data *data,
                      float lsb_per_gauss)
{
    if (!raw || !data || lsb_per_gauss <= 0.0f) {
        return;
    }

    data->x = (float)raw->x / lsb_per_gauss;
    data->y = (float)raw->y / lsb_per_gauss;
    data->z = (float)raw->z / lsb_per_gauss;
}

HMC5883L_Status HMC5883L_Read(HMC5883L_Data *data)
{
    HMC5883L_RawData raw;
    HMC5883L_Status status;

    if (!data) {
        return HMC5883L_ERR_INVALID_ARGUMENT;
    }

    status = HMC5883L_ReadRaw(&raw);
    if (status != HMC5883L_OK) {
        return status;
    }

    HMC5883L_Convert(&raw, data, s_lsb_per_gauss);
    return HMC5883L_OK;
}
