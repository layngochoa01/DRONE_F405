#include "spl06.h"
#include "../drivers/I2C.h"

#define SPL06_ADDR  0X76

#define REG_PSR_B2  0x00
#define REG_TMP_B2  0x03
#define REG_PRS_CFG  0x06
#define REG_TMP_CFG  0x07
#define REG_MEAS_CFG    0x08
#define REG_ID  0x0D
#define REG_COEF  0x10
#define REG_RESET  0x0C

#define MEAS_CFG_MEAS_CTRL 0x07
#define MEAS_CFG_PRS_RDY (1U << 4)
#define MEAS_CFG_TMP_RDY (1U << 5)

#define SPL06_SCALE_FACTOR  7864320.0f 

static SPL06_Calib_t s_calib;

static int32_t signExtend(uint32_t raw, uint8_t bits)
{
    int32_t value = (int32_t)raw;
    if (raw & (1U << (bits - 1))) {
        value -= (1 << bits);
    }
    return value;
}

static bool readRaw24(uint8_t regAddr, int32_t *out)
{
    uint8_t buf[3];
    if (!I2C1_ReadRegs(SPL06_ADDR, regAddr, buf, 3)) return false;

    uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    *out = signExtend(raw, 24);
    return true;
}

static bool readCalibCoefficients(void)
{
    uint8_t c[18];
    if (!I2C1_ReadRegs(SPL06_ADDR, REG_COEF, c, 18)) return false;

    s_calib.c0  = signExtend(((uint32_t)c[0] << 4) | (c[1] >> 4), 12);
    s_calib.c1  = signExtend((((uint32_t)c[1] & 0x0F) << 8) | c[2], 12);
    s_calib.c00 = signExtend(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4) | (c[5] >> 4), 20);
    s_calib.c10 = signExtend((((uint32_t)c[5] & 0x0F) << 16) | ((uint32_t)c[6] << 8) | c[7], 20);
    s_calib.c01 = signExtend(((uint32_t)c[8] << 8) | c[9], 16);
    s_calib.c11 = signExtend(((uint32_t)c[10] << 8) | c[11], 16);
    s_calib.c20 = signExtend(((uint32_t)c[12] << 8) | c[13], 16);
    s_calib.c21 = signExtend(((uint32_t)c[14] << 8) | c[15], 16);
    s_calib.c30 = signExtend(((uint32_t)c[16] << 8) | c[17], 16);

    return true;
}

bool SPL06_Init(void)
{
    I2C1_Init();

    I2C1_WriteReg(SPL06_ADDR, REG_RESET, 0x09);

    for (volatile uint32_t i = 0; i < 100000U; i++) {}  

    uint8_t status = 0;
    uint32_t retries = 0;
    do {
        if (!I2C1_ReadReg(SPL06_ADDR, REG_MEAS_CFG, &status)) return false;
        if (status & 0x80) break;  
        for (volatile uint32_t i = 0; i < 10000U; i++) {}
        retries++;
    } while (retries < 50);  

    if (!readCalibCoefficients()) return false;

    if (!I2C1_WriteReg(SPL06_ADDR, REG_PRS_CFG, 0x03)) return false;
    if (!I2C1_WriteReg(SPL06_ADDR, REG_TMP_CFG, 0x83)) return false;
    if (!I2C1_WriteReg(SPL06_ADDR, REG_MEAS_CFG, MEAS_CFG_MEAS_CTRL)) return false;

    return true;
}

bool SPL06_Update(SPL06_Data_t *out)
{
    uint8_t status;
    if (!I2C1_ReadReg(SPL06_ADDR, REG_MEAS_CFG, &status)) return false;

    if (!(status & MEAS_CFG_PRS_RDY) || !(status & MEAS_CFG_TMP_RDY)) {
        return false; 
    }
    
    int32_t rawPressure, rawTemperature;
    if (!readRaw24(REG_PSR_B2, &rawPressure)) return false;
    if (!readRaw24(REG_TMP_B2, &rawTemperature)) return false;
    // UART5_WriteF("RAW: P=%ld T=%ld\r\n", (long)rawPressure, (long)rawTemperature);
    float pRaw = (float)rawPressure / SPL06_SCALE_FACTOR;
    float tRaw = (float)rawTemperature / SPL06_SCALE_FACTOR;

    out->tempC = (float)s_calib.c0 * 0.5f + (float)s_calib.c1 * tRaw;

    float pComp = (float)s_calib.c00
                + pRaw * ((float)s_calib.c10 + pRaw * ((float)s_calib.c20 + pRaw * (float)s_calib.c30))
                + tRaw * (float)s_calib.c01
                + tRaw * pRaw * ((float)s_calib.c11 + pRaw * (float)s_calib.c21);

    out->pressPa = pComp;

    return true;
}
