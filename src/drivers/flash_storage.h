#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

/* =========================================================
 * flash_storage.h
 * Flash Sector 11 (0x080E0000, 128KB) chia thành 512 slot x 256 byte,
 * ghi nối tiếp kiểu log:
 *   - Lưu : ghi vào slot TRỐNG đầu tiên (toàn 0xFF), sequence = max + 1.
 *           Hết slot → xóa sector rồi ghi lại từ slot 0.
 *   - Đọc : chọn slot có magic đúng + CRC đúng + sequence lớn nhất.
 * Flash chỉ ghi được lên vùng đã xóa, nên KHÔNG bao giờ ghi đè slot cũ.
 * ========================================================= */
#include <stdint.h>

#define FLASH_MAGIC     0xBB420002U  /* đổi khi remap trục IMU thay đổi → calib cũ tự bị bỏ */

#define FLASH_SECTOR11_ADDR     0x080E0000UL
#define FLASH_SECTOR11_SIZE     0x20000UL
#define FLASH_SLOT_SIZE         0x100UL
#define FLASH_SLOT_COUNT        (FLASH_SECTOR11_SIZE / FLASH_SLOT_SIZE)   /* 512 */
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
/* Thông tin debug: slot đang dùng (-1 nếu không có) và sequence của nó */
int32_t  FlashStorage_GetActiveSlot(uint32_t *sequence);

#endif 