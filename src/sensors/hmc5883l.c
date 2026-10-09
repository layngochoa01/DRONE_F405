#include "hmc5883l.h"
#include "../drivers/I2C.h"
#include <math.h>

/* ---------------- HMC5883L ---------------- */
#define HMC5883L_ADDR            0x1E
#define HMC5883L_REG_CFG_A       0x00
#define HMC5883L_REG_CFG_B       0x01
#define HMC5883L_REG_MODE        0x02
#define HMC5883L_REG_DATA_X_MSB  0x03
#define HMC5883L_REG_STATUS      0x09
#define HMC5883L_REG_ID_A        0x0A
#define HMC5883L_STATUS_RDY      (1U << 0)
#define HMC5883L_GAUSS_PER_LSB   0.92e-3f      /* gain ±1.3 Ga */

/* ---------------- QMC5883L ---------------- */
#define QMC5883L_ADDR            0x0D
#define QMC5883L_REG_DATA_X_LSB  0x00
#define QMC5883L_REG_STATUS      0x06
#define QMC5883L_REG_CTRL1       0x09
#define QMC5883L_REG_CTRL2       0x0A
#define QMC5883L_REG_SET_RESET   0x0B
#define QMC5883L_REG_CHIP_ID     0x0D
#define QMC5883L_STATUS_DRDY     (1U << 0)
/* CTRL1: OSR=512 (00), RNG=8G (01), ODR=50Hz (01), MODE=continuous (01) */
#define QMC5883L_CTRL1_VALUE     0x15
#define QMC5883L_GAUSS_PER_LSB   (1.0f / 3000.0f)   /* RNG 8G */

static Compass_Type_e s_type = COMPASS_NONE;
static float s_offX = COMPASS_OFFSET_X;
static float s_offY = COMPASS_OFFSET_Y;
static float s_offZ = COMPASS_OFFSET_Z;

static bool  s_cal = false;
static float s_min[3], s_max[3];
static unsigned s_calSamples = 0;

static bool initHmc(void)
{
    uint8_t idA = 0;
    if (!I2C1_ReadReg(HMC5883L_ADDR, HMC5883L_REG_ID_A, &idA)) return false;
    if (idA != 0x48) return false;     /* 'H' */

    if (!I2C1_WriteReg(HMC5883L_ADDR, HMC5883L_REG_CFG_A, 0x70)) return false;  /* 8 avg, 15 Hz */
    if (!I2C1_WriteReg(HMC5883L_ADDR, HMC5883L_REG_CFG_B, 0x20)) return false;  /* ±1.3 Ga */
    if (!I2C1_WriteReg(HMC5883L_ADDR, HMC5883L_REG_MODE,  0x00)) return false;  /* continuous */
    return true;
}

static bool initQmc(void)
{
    uint8_t id = 0;
    if (!I2C1_ReadReg(QMC5883L_ADDR, QMC5883L_REG_CHIP_ID, &id)) return false;
    if (id != 0xFF) return false;

    if (!I2C1_WriteReg(QMC5883L_ADDR, QMC5883L_REG_CTRL2, 0x80)) return false;      /* soft reset */
    for (volatile uint32_t i = 0; i < 100000U; i++) {}
    if (!I2C1_WriteReg(QMC5883L_ADDR, QMC5883L_REG_SET_RESET, 0x01)) return false;  /* theo datasheet */
    if (!I2C1_WriteReg(QMC5883L_ADDR, QMC5883L_REG_CTRL1, QMC5883L_CTRL1_VALUE)) return false;
    return true;
}

bool Compass_Init(void)
{
    s_type = COMPASS_NONE;
    if (initHmc())      s_type = COMPASS_HMC5883L;
    else if (initQmc()) s_type = COMPASS_QMC5883L;
    return s_type != COMPASS_NONE;
}

/* Đọc trục chip, đơn vị gauss */
static bool readChip(float *mx, float *my, float *mz)
{
    uint8_t status = 0;
    uint8_t buf[6];

    if (s_type == COMPASS_HMC5883L) {
        if (!I2C1_ReadReg(HMC5883L_ADDR, HMC5883L_REG_STATUS, &status)) return false;
        if (!(status & HMC5883L_STATUS_RDY)) return false;
        if (!I2C1_ReadRegs(HMC5883L_ADDR, HMC5883L_REG_DATA_X_MSB, buf, 6)) return false;

        /* Thứ tự thanh ghi HMC: X, Z, Y — big endian */
        int16_t rx = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        int16_t rz = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        int16_t ry = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        *mx = (float)rx * HMC5883L_GAUSS_PER_LSB;
        *my = (float)ry * HMC5883L_GAUSS_PER_LSB;
        *mz = (float)rz * HMC5883L_GAUSS_PER_LSB;
        return true;
    }

    if (s_type == COMPASS_QMC5883L) {
        if (!I2C1_ReadReg(QMC5883L_ADDR, QMC5883L_REG_STATUS, &status)) return false;
        if (!(status & QMC5883L_STATUS_DRDY)) return false;
        if (!I2C1_ReadRegs(QMC5883L_ADDR, QMC5883L_REG_DATA_X_LSB, buf, 6)) return false;

        /* Thứ tự QMC: X, Y, Z — little endian */
        int16_t rx = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        int16_t ry = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        int16_t rz = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
        *mx = (float)rx * QMC5883L_GAUSS_PER_LSB;
        *my = (float)ry * QMC5883L_GAUSS_PER_LSB;
        *mz = (float)rz * QMC5883L_GAUSS_PER_LSB;
        return true;
    }

    return false;
}

bool Compass_Update(Compass_Data_t *out)
{
    float cx, cy, cz;
    if (!readChip(&cx, &cy, &cz)) return false;

    out->rawX = cx;
    out->rawY = cy;
    out->rawZ = cz;

    if (s_cal) {
        float v[3] = { cx, cy, cz };
        for (int i = 0; i < 3; i++) {
            if (v[i] < s_min[i]) s_min[i] = v[i];
            if (v[i] > s_max[i]) s_max[i] = v[i];
        }
        s_calSamples++;
    }

    float mx = cx - s_offX;
    float my = cy - s_offY;
    float mz = cz - s_offZ;

    /* Chip → body frame FLU. CHƯA XÁC ĐỊNH — sẽ sửa sau khi đo trục thực tế
     * (remap phải là phép quay, det = +1). */
    out->x = mx;
    out->y = my;
    out->z = mz;

    /* Heading khi nằm ngang: 0 = Bắc, 90 = Đông (chiều kim đồng hồ).
     * FLU: mũi hướng Đông → Bắc nằm bên TRÁI → y > 0 → atan2(y, x) = 90° */
    float heading = atan2f(out->y, out->x) * (180.0f / (float)M_PI);
    if (heading < 0.0f) heading += 360.0f;
    out->headingDeg = heading;

    return true;
}

Compass_Type_e Compass_GetType(void)
{
    return s_type;
}

const char *Compass_TypeName(void)
{
    switch (s_type) {
        case COMPASS_HMC5883L: return "HMC5883L";
        case COMPASS_QMC5883L: return "QMC5883L";
        default:               return "NONE";
    }
}

void Compass_SetOffsets(float ox, float oy, float oz)
{
    s_offX = ox;
    s_offY = oy;
    s_offZ = oz;
}

void Compass_GetOffsets(float *ox, float *oy, float *oz)
{
    *ox = s_offX;
    *oy = s_offY;
    *oz = s_offZ;
}

void Compass_CalStart(void)
{
    for (int i = 0; i < 3; i++) {
        s_min[i] =  1e9f;
        s_max[i] = -1e9f;
    }
    s_calSamples = 0;
    s_cal = true;
}

bool Compass_CalFinish(float *ox, float *oy, float *oz)
{
    s_cal = false;
    if (s_calSamples < 100U) return false;

    *ox = 0.5f * (s_max[0] + s_min[0]);
    *oy = 0.5f * (s_max[1] + s_min[1]);
    *oz = 0.5f * (s_max[2] + s_min[2]);
    Compass_SetOffsets(*ox, *oy, *oz);
    return true;
}

bool Compass_IsCalibrating(void)
{
    return s_cal;
}
