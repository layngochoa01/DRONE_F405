#include "flash_storage.h"
#include <string.h>

#define FLASH_BASE_ADDR     0x40023C00UL

typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
} FLASH_TypeDef;

#define FLASH   ((FLASH_TypeDef *)FLASH_BASE_ADDR)

#define FLASH_KEY1      0x45670123UL
#define FLASH_KEY2      0xCDEF89ABUL

#define FLASH_SR_BSY    (1U << 16)
#define FLASH_SR_PGSERR (1U << 7)
#define FLASH_SR_PGPERR (1U << 6)
#define FLASH_SR_PGAERR (1U << 5)
#define FLASH_SR_WRPERR (1U << 4)
#define FLASH_SR_OPERR  (1U << 1)
#define FLASH_SR_EOP    (1U << 0)
#define FLASH_SR_ERR_MASK (FLASH_SR_PGSERR | FLASH_SR_PGPERR | \
                           FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)

#define FLASH_CR_LOCK   (1U << 31)
#define FLASH_CR_ERRIE  (1U << 25)
#define FLASH_CR_EOPIE  (1U << 24)
#define FLASH_CR_STRT   (1U << 16)
#define FLASH_CR_PSIZE_x32 (2U << 8)
#define FLASH_CR_SNB_SHIFT  3U
#define FLASH_CR_MER    (1U << 2)
#define FLASH_CR_SER    (1U << 1)
#define FLASH_CR_PG     (1U << 0)

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

#define FLASH_ACR_ICEN   (1U << 9)
#define FLASH_ACR_DCEN   (1U << 10)
#define FLASH_ACR_ICRST  (1U << 11)
#define FLASH_ACR_DCRST  (1U << 12)

static void flash_flush_caches(void)
{
    uint32_t acr = FLASH->ACR;
    FLASH->ACR = acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = (acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN)) | FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR = acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = acr;
}

static FlashStatus_t flash_erase_sector11(void)
{
    flash_wait_busy();
    flash_clear_errors();

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

static const CalibData_t *flash_read_slot(uint32_t index)
{
    return (const CalibData_t *)(FLASH_SECTOR11_ADDR + index * FLASH_SLOT_SIZE);
}

static uint8_t flash_slot_is_blank(uint32_t index)
{
    const uint32_t *p = (const uint32_t *)flash_read_slot(index);
    for (uint32_t i = 0; i < sizeof(CalibData_t) / 4U; i++) {
        if (p[i] != 0xFFFFFFFFUL) return 0;
    }
    return 1;
}

static int32_t flash_scan(int32_t *firstBlank)
{
    int32_t  best = -1;
    uint32_t bestSeq = 0;

    for (uint32_t i = 0; i < FLASH_SLOT_COUNT; i++) {
        const CalibData_t *d = flash_read_slot(i);
        if (FlashStorage_IsValid(d) && (best < 0 || d->sequence > bestSeq)) {
            best = (int32_t)i;
            bestSeq = d->sequence;
        }
    }

    *firstBlank = -1;
    uint32_t start = (best < 0) ? 0U : (uint32_t)best + 1U;
    for (uint32_t i = start; i < FLASH_SLOT_COUNT; i++) {
        if (flash_slot_is_blank(i)) {
            *firstBlank = (int32_t)i;
            break;
        }
    }
    return best;
}

uint8_t FlashStorage_IsValid(const CalibData_t *data)
{
    if (data->magic != FLASH_MAGIC)     return 0;
    if (data->crc != calib_crc(data))   return 0;
    return 1;
}

FlashStatus_t FlashStorage_Init(CalibData_t *out)
{
    int32_t blank;
    int32_t best = flash_scan(&blank);
    if (best < 0) return FLASH_ERR_NO_DATA;

    memcpy(out, flash_read_slot((uint32_t)best), sizeof(CalibData_t));
    return FLASH_OK;
}

FlashStatus_t FlashStorage_Save(const CalibData_t *data)
{
    int32_t blank;
    int32_t best = flash_scan(&blank);

    CalibData_t new_data = *data;
    new_data.magic    = FLASH_MAGIC;
    new_data.sequence = (best < 0) ? 1U : flash_read_slot((uint32_t)best)->sequence + 1U;
    new_data.crc      = calib_crc(&new_data);

    FlashStatus_t status = FLASH_OK;
    flash_unlock();

    if (blank < 0) {
        status = flash_erase_sector11();
        flash_flush_caches();
        blank = 0;
    }

    if (status == FLASH_OK) {
        status = flash_write_slot(FLASH_SECTOR11_ADDR + (uint32_t)blank * FLASH_SLOT_SIZE, &new_data);
        flash_flush_caches();
    }
    flash_lock();

    if (status == FLASH_OK && !FlashStorage_IsValid(flash_read_slot((uint32_t)blank))) {
        status = FLASH_ERR_WRITE;
    }
    return status;
}

FlashStatus_t FlashStorage_Erase(void)
{
    flash_unlock();
    FlashStatus_t status = flash_erase_sector11();
    flash_flush_caches();
    flash_lock();
    return status;
}

int32_t FlashStorage_GetActiveSlot(uint32_t *sequence)
{
    int32_t blank;
    int32_t best = flash_scan(&blank);
    if (sequence) *sequence = (best < 0) ? 0U : flash_read_slot((uint32_t)best)->sequence;
    return best;
}
