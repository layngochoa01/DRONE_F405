#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>

#define FLASH_MAGIC     0xBB420002U

#define FLASH_SECTOR11_ADDR     0x080E0000UL
#define FLASH_SECTOR11_SIZE     0x20000UL
#define FLASH_SLOT_SIZE         0x100UL
#define FLASH_SLOT_COUNT        (FLASH_SECTOR11_SIZE / FLASH_SLOT_SIZE)
#define FLASH_SECTOR11_NUM      11U

typedef struct {
    uint32_t magic;
    uint32_t sequence;

    float gx_offset;
    float gy_offset;
    float gz_offset;

    float ax_offset;
    float ay_offset;
    float az_offset;

    uint32_t gyro_done;
    uint32_t accel_done;

    uint32_t crc;
} CalibData_t;

typedef enum {
    FLASH_OK = 0,
    FLASH_ERR_NO_DATA,
    FLASH_ERR_CRC,
    FLASH_ERR_WRITE,
    FLASH_ERR_ERASE,
} FlashStatus_t;

FlashStatus_t FlashStorage_Init(CalibData_t *out);

FlashStatus_t FlashStorage_Save(const CalibData_t *data);

FlashStatus_t FlashStorage_Erase(void);

uint8_t FlashStorage_IsValid(const CalibData_t *data);
int32_t  FlashStorage_GetActiveSlot(uint32_t *sequence);

#endif
