#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 * GPS — giao thức UBX (u-blox), chỉ đọc gói NAV-PVT (class 0x01, id 0x07, 92 byte)
 *
 * Module: HGLRC M100 Pro, USART1 RX (PA10), 115200 baud, cấu hình sẵn xuất UBX.
 * Firmware KHÔNG gửi lệnh cấu hình cho module.
 *
 * Định nghĩa GPS_USE_NMEA để dùng parser NMEA cũ (GGA + RMC) thay cho UBX — chỉ để
 * debug; hai parser không chạy song song.
 * ========================================================= */

#define GPS_BAUD                 115200U

/* GPS_IsHealthy(): ngưỡng đánh giá mẫu dùng được cho điều hướng */
#define GPS_HEALTH_TIMEOUT_MS    500U     /* quá lâu không có NAV-PVT mới */
#define GPS_HEALTH_MIN_SATS      6U
#define GPS_HEALTH_MAX_HACC_M    3.0f

typedef struct {
    /* ---- Trường cũ (giữ tương thích API) ---- */
    bool     fixValid;      /* (fixType == 3 || fixType == 4) && gnssFixOK */
    uint8_t  satellites;    /* numSV */
    float    hdop;          /* NAV-PVT KHÔNG có HDOP: đây là PDOP (pDOP * 0.01), luôn ≥ HDOP */
    int32_t  latE7;         /* độ * 1e7 — dùng cho tính toán vị trí */
    int32_t  lonE7;
    float    latitude;      /* độ — CHỈ để log/hiển thị, không dùng tính toán */
    float    longitude;
    float    altitudeMsl;   /* m */
    float    speedKmh;      /* tốc độ mặt đất */
    float    courseDeg;     /* hướng chuyển động (headMot) */
    bool     velValid;      /* velN/velE dùng được (= fixValid) */
    float    velN;          /* m/s, lấy trực tiếp từ NAV-PVT */
    float    velE;          /* m/s */

    /* ---- Trường mới từ NAV-PVT ---- */
    uint32_t iTOW;          /* ms, thời gian trong tuần GPS */
    uint8_t  fixType;       /* 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 chỉ thời gian */
    uint8_t  flags;         /* bit0 = gnssFixOK */
    float    hAccM;         /* độ chính xác ngang chip tự ước lượng (m) */
    float    vAccM;         /* độ chính xác đứng (m) */
    float    sAccMs;        /* độ chính xác tốc độ (m/s) */

    /* ---- Giá trị nguyên gốc (cho log CSV toàn số nguyên) ---- */
    int32_t  hMSLmm;
    uint32_t hAccMm, vAccMm, sAccMms;
    int32_t  velNmms, velEmms, velDmms, gSpeedMms;
    uint16_t pDOP;          /* đơn vị 0.01 */
} GPS_Data_t;

typedef struct {
    uint32_t bytes;         /* số byte đọc từ UART1 */
    uint32_t packets;       /* gói UBX đúng checksum (mọi loại) */
    uint32_t pvt;           /* gói NAV-PVT hợp lệ (đúng checksum, payload 92 byte) */
    uint32_t badChecksum;   /* gói sai checksum */
    uint32_t resync;        /* số lần bỏ gói đang đọc dở để tìm lại B5 62 */
    uint32_t lastPvtTick;   /* get_tick() lúc nhận NAV-PVT gần nhất */
} GPS_Stats_t;

void GPS_Init(void);
void GPS_Process(void);                 /* gọi liên tục trong main loop, không chặn */
const GPS_Data_t  *GPS_GetData(void);
const GPS_Stats_t *GPS_GetStats(void);
bool GPS_HasFix(void);
/* true đúng 1 lần cho mỗi gói NAV-PVT hợp lệ mới (kể cả khi chưa fix) */
bool GPS_IsNewData(void);
/* false khi: quá GPS_HEALTH_TIMEOUT_MS không có NAV-PVT mới, chưa fix,
 * numSV < GPS_HEALTH_MIN_SATS, hoặc hAcc > GPS_HEALTH_MAX_HACC_M */
bool GPS_IsHealthy(uint32_t nowTick);

#endif
