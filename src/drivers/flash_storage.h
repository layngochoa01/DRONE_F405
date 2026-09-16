#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

/* =========================================================
 * flash_storage.h 
 * Layout Flash Sector 11 (0x080E0000, 128KB):
 *   Slot 0: 0x080E0000  (256 byte)
 *   Slot 1: 0x080E0100  (256 byte)
 *
 * Mỗi lần ghi: ghi vào slot có sequence nhỏ hơn
 * Mỗi lần đọc: dùng slot có sequence lớn hơn + magic hợp lệ + CRC đúng
 * ========================================================= */

#include <stdint.h>

#define FLASH_MAGIC     0xBB420001U  

#define FLASH_SECTOR11_ADDR     0x080E0000UL
#define FLASH_SLOT0_ADDR        (FLASH_SECTOR11_ADDR + 0x000U)
#define FLASH_SLOT1_ADDR        (FLASH_SECTOR11_ADDR + 0x100U)
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

/* =========================================================
 * PUBLIC API
 * ========================================================= */

/**
 * FlashStorage_Init - Đọc Flash khi boot, load calib nếu có
 * Gọi 1 lần trong main() trước Attitude_Init()
 * Return: FLASH_OK nếu có data hợp lệ, FLASH_ERR_NO_DATA nếu chưa calib
 */
FlashStatus_t FlashStorage_Init(CalibData_t *out);

FlashStatus_t FlashStorage_Save(const CalibData_t *data);

FlashStatus_t FlashStorage_Erase(void);

uint8_t FlashStorage_IsValid(const CalibData_t *data);

#endif 