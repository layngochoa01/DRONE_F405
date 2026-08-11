#include "flash_storage.h"
#include <string.h>

/* =========================================================
 * FLASH REGISTERS - RM0090 Section 3.8
 * ========================================================= */
#define FLASH_BASE_ADDR     0x40023C00UL

typedef struct {
    volatile uint32_t ACR;      /* 0x00 - Access control    */
    volatile uint32_t KEYR;     /* 0x04 - Key register      */
    volatile uint32_t OPTKEYR;  /* 0x08 - Option key        */
    volatile uint32_t SR;       /* 0x0C - Status register   */
    volatile uint32_t CR;       /* 0x10 - Control register  */
    volatile uint32_t OPTCR;    /* 0x14 - Option control    */
} FLASH_TypeDef;

#define FLASH   ((FLASH_TypeDef *)FLASH_BASE_ADDR)

/* FLASH_KEYR -RM0090 3.9.3 */
#define FLASH_KEY1      0x45670123UL
#define FLASH_KEY2      0xCDEF89ABUL

/* FLASH_SR bits */
#define FLASH_SR_BSY    (1U << 16) 
#define FLASH_SR_PGSERR (1U << 7)   
#define FLASH_SR_PGPERR (1U << 6)   
#define FLASH_SR_PGAERR (1U << 5)   
#define FLASH_SR_WRPERR (1U << 4)   
#define FLASH_SR_OPERR  (1U << 1)   
#define FLASH_SR_EOP    (1U << 0)  
#define FLASH_SR_ERR_MASK (FLASH_SR_PGSERR | FLASH_SR_PGPERR | \
                           FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)

/* FLASH_CR bits */
#define FLASH_CR_LOCK   (1U << 31)  /* Lock                 */
#define FLASH_CR_ERRIE  (1U << 25)  /* Error interrupt      */
#define FLASH_CR_EOPIE  (1U << 24)  /* EOP interrupt        */
#define FLASH_CR_STRT   (1U << 16)  /* Start erase          */
#define FLASH_CR_PSIZE_x32 (2U << 8) /* 32-bit parallelism  */
#define FLASH_CR_SNB_SHIFT  3U      /* Sector number shift  */
#define FLASH_CR_MER    (1U << 2)   /* Mass erase           */
#define FLASH_CR_SER    (1U << 1)   /* Sector erase         */
#define FLASH_CR_PG     (1U << 0)   /* Programming          */

/* =========================================================
 * PRIVATE 
 * ========================================================= */
static uint32_t crc32_compute(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++) {
            if (crc & 1U)
                crc = (crc >> 1) ^ 0xEDB88320UL;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

static uint32_t calib_crc(const CalibData_t *d)
{
    return crc32_compute((const uint8_t *)d,
                         sizeof(CalibData_t) - sizeof(uint32_t));
}


static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_busy(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {}
}

static void flash_clear_errors(void)
{
    FLASH->SR = FLASH_SR_ERR_MASK | FLASH_SR_EOP;
}

static FlashStatus_t flash_erase_sector11(void)
{
    flash_wait_busy();
    flash_clear_errors();

    /* Sector erase: SER=1, SNB=11, PSIZE=32bit */
    FLASH->CR = FLASH_CR_SER
              | (FLASH_SECTOR11_NUM << FLASH_CR_SNB_SHIFT)
              | FLASH_CR_PSIZE_x32;

    FLASH->CR |= FLASH_CR_STRT;   
    flash_wait_busy();

    if (FLASH->SR & FLASH_SR_ERR_MASK) {
        flash_clear_errors();
        return FLASH_ERR_ERASE;
    }

    FLASH->CR &= ~FLASH_CR_SER;
    return FLASH_OK;
}

static FlashStatus_t flash_write_slot(uint32_t addr, const CalibData_t *data)
{
    flash_wait_busy();
    flash_clear_errors();

    /* Bật programming mode, 32-bit parallelism */
    FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_x32;

    const uint32_t *src = (const uint32_t *)data;
    volatile uint32_t *dst = (volatile uint32_t *)addr;
    uint32_t words = sizeof(CalibData_t) / 4U;

    for (uint32_t i = 0; i < words; i++) {
        *dst++ = *src++;
        flash_wait_busy();
        if (FLASH->SR & FLASH_SR_ERR_MASK) {
            flash_clear_errors();
            FLASH->CR &= ~FLASH_CR_PG;
            return FLASH_ERR_WRITE;
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;
    return FLASH_OK;
}

static const CalibData_t *flash_read_slot(uint32_t addr)
{
    return (const CalibData_t *)addr;
}

/* =========================================================
 * PUBLIC API
 * ========================================================= */

uint8_t FlashStorage_IsValid(const CalibData_t *data)
{
    if (data->magic != FLASH_MAGIC)     return 0;
    if (data->crc != calib_crc(data))   return 0;
    return 1;
}

FlashStatus_t FlashStorage_Init(CalibData_t *out)
{
    const CalibData_t *slot0 = flash_read_slot(FLASH_SLOT0_ADDR);
    const CalibData_t *slot1 = flash_read_slot(FLASH_SLOT1_ADDR);

    uint8_t v0 = FlashStorage_IsValid(slot0);
    uint8_t v1 = FlashStorage_IsValid(slot1);

    if (!v0 && !v1) {
        return FLASH_ERR_NO_DATA;   
    }

    const CalibData_t *best = NULL;

    if (v0 && v1) {
        best = (slot0->sequence > slot1->sequence) ? slot0 : slot1;
    } else if (v0) {
        best = slot0;
    } else {
        best = slot1;
    }

    memcpy(out, best, sizeof(CalibData_t));
    return FLASH_OK;
}

FlashStatus_t FlashStorage_Save(const CalibData_t *data)
{
    const CalibData_t *slot0 = flash_read_slot(FLASH_SLOT0_ADDR);
    const CalibData_t *slot1 = flash_read_slot(FLASH_SLOT1_ADDR);

    uint8_t v0 = FlashStorage_IsValid(slot0);
    uint8_t v1 = FlashStorage_IsValid(slot1);

    uint32_t write_addr;
    uint32_t new_seq;

    if (!v0 && !v1) {
        write_addr = FLASH_SLOT0_ADDR;
        new_seq    = 1U;
    } else if (v0 && !v1) {
        write_addr = FLASH_SLOT1_ADDR;
        new_seq    = slot0->sequence + 1U;
    } else if (!v0 && v1) {
        write_addr = FLASH_SLOT0_ADDR;
        new_seq    = slot1->sequence + 1U;
    } else {
        if (slot0->sequence <= slot1->sequence) {
            write_addr = FLASH_SLOT0_ADDR;
            new_seq    = slot1->sequence + 1U;
        } else {
            write_addr = FLASH_SLOT1_ADDR;
            new_seq    = slot0->sequence + 1U;
        }
    }

    CalibData_t new_data = *data;
    new_data.magic    = FLASH_MAGIC;
    new_data.sequence = new_seq;
    new_data.crc      = calib_crc(&new_data);

    FlashStatus_t status;
    flash_unlock();

    if (!v0 && !v1) {
        status = flash_erase_sector11();
        if (status != FLASH_OK) {
            flash_lock();
            return status;
        }
    }

    status = flash_write_slot(write_addr, &new_data);
    flash_lock();

    return status;
}

FlashStatus_t FlashStorage_Erase(void)
{
    flash_unlock();
    FlashStatus_t status = flash_erase_sector11();
    flash_lock();
    return status;
}