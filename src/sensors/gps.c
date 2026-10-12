#include "gps.h"
#include "../drivers/usart1.h"
#include "system.h"
#include <string.h>

#define GPS_UART_CLK        84000000U   /* USART1 trên APB2 */

static GPS_Data_t  s_data;
static GPS_Stats_t s_stats;
static bool        s_newData = false;

static void markNewSample(void)
{
    s_newData = true;
    s_stats.lastPvtTick = get_tick();
}

#ifndef GPS_USE_NMEA
/* =========================================================
 * UBX
 *   B5 62 | class | id | len (2 byte LE) | payload | CK_A CK_B
 *   Checksum Fletcher-8 trên class, id, len, payload: ck_a += b; ck_b += ck_a
 *
 * Gói NAV-PVT 92 byte được lưu và giải mã. Gói khác: vẫn đếm qua đúng `len` byte
 * (không lưu) và kiểm checksum, để không lệch đồng bộ dù gói dài bao nhiêu.
 * ========================================================= */
#define UBX_SYNC1           0xB5U
#define UBX_SYNC2           0x62U
#define UBX_CLASS_NAV       0x01U
#define UBX_ID_NAV_PVT      0x07U
#define UBX_NAV_PVT_LEN     92U
#define UBX_MAX_LEN         1024U   /* length lớn hơn coi là rác → tìm lại đồng bộ */

typedef enum {
    UBX_SYNC_1, UBX_SYNC_2, UBX_CLASS, UBX_ID, UBX_LEN_1, UBX_LEN_2,
    UBX_PAYLOAD, UBX_CK_A, UBX_CK_B,
} UbxState_e;

static UbxState_e s_state = UBX_SYNC_1;
static uint8_t    s_class, s_id, s_ckA, s_ckB;
static uint16_t   s_len, s_idx;
static bool       s_store;                     /* gói đang đọc là NAV-PVT cần lưu */
static uint8_t    s_payload[UBX_NAV_PVT_LEN];  /* chỉ lưu NAV-PVT */

/* Đọc số little-endian bằng ghép byte (không ép kiểu con trỏ → không lỗi alignment) */
static uint16_t rdU2(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rdU4(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rdI4(const uint8_t *p)
{
    return (int32_t)rdU4(p);
}

/* Offset NAV-PVT theo tài liệu giao thức UBX (u-blox M8 trở lên) */
static void decodeNavPvt(const uint8_t *p)
{
    s_data.iTOW      = rdU4(p + 0);
    s_data.fixType   = p[20];
    s_data.flags     = p[21];
    s_data.satellites = p[23];
    s_data.lonE7     = rdI4(p + 24);
    s_data.latE7     = rdI4(p + 28);
    s_data.hMSLmm    = rdI4(p + 36);
    s_data.hAccMm    = rdU4(p + 40);
    s_data.vAccMm    = rdU4(p + 44);
    s_data.velNmms   = rdI4(p + 48);
    s_data.velEmms   = rdI4(p + 52);
    s_data.velDmms   = rdI4(p + 56);
    s_data.gSpeedMms = rdI4(p + 60);
    int32_t headMot  = rdI4(p + 64);           /* 1e-5 độ */
    s_data.sAccMms   = rdU4(p + 68);
    s_data.pDOP      = rdU2(p + 76);

    /* fixType 5 = chỉ có thời gian, không có vị trí → không nhận */
    bool fix3d = (s_data.fixType == 3U) || (s_data.fixType == 4U);
    s_data.fixValid = fix3d && ((s_data.flags & 0x01U) != 0U);
    s_data.velValid = s_data.fixValid;

    /* Đơn vị dẫn xuất (float chỉ cho hiển thị và các phép tính không phải vị trí) */
    s_data.latitude    = (float)s_data.latE7 * 1e-7f;
    s_data.longitude   = (float)s_data.lonE7 * 1e-7f;
    s_data.altitudeMsl = (float)s_data.hMSLmm * 0.001f;
    s_data.hAccM       = (float)s_data.hAccMm * 0.001f;
    s_data.vAccM       = (float)s_data.vAccMm * 0.001f;
    s_data.sAccMs      = (float)s_data.sAccMms * 0.001f;
    s_data.velN        = (float)s_data.velNmms * 0.001f;
    s_data.velE        = (float)s_data.velEmms * 0.001f;
    s_data.speedKmh    = (float)s_data.gSpeedMms * 0.0036f;
    s_data.courseDeg   = (float)headMot * 1e-5f;
    s_data.hdop        = (float)s_data.pDOP * 0.01f;   /* PDOP, không phải HDOP */
}

static void resync(void)
{
    s_stats.resync++;
    s_state = UBX_SYNC_1;
}

static void ubxByte(uint8_t c)
{
    switch (s_state) {
        case UBX_SYNC_1:
            if (c == UBX_SYNC1) s_state = UBX_SYNC_2;
            break;

        case UBX_SYNC_2:
            if (c == UBX_SYNC2)      s_state = UBX_CLASS;
            else if (c == UBX_SYNC1) s_state = UBX_SYNC_2;   /* B5 B5 62 */
            else                     resync();
            break;

        case UBX_CLASS:
            s_class = c;
            s_ckA = c;
            s_ckB = c;
            s_state = UBX_ID;
            break;

        case UBX_ID:
            s_id = c;
            s_ckA += c; s_ckB += s_ckA;
            s_state = UBX_LEN_1;
            break;

        case UBX_LEN_1:
            s_len = c;
            s_ckA += c; s_ckB += s_ckA;
            s_state = UBX_LEN_2;
            break;

        case UBX_LEN_2:
            s_len |= (uint16_t)((uint16_t)c << 8);
            s_ckA += c; s_ckB += s_ckA;
            if (s_len > UBX_MAX_LEN) { resync(); break; }
            s_store = (s_class == UBX_CLASS_NAV) && (s_id == UBX_ID_NAV_PVT) && (s_len == UBX_NAV_PVT_LEN);
            s_idx = 0;
            s_state = (s_len == 0U) ? UBX_CK_A : UBX_PAYLOAD;
            break;

        case UBX_PAYLOAD:
            if (s_store) s_payload[s_idx] = c;
            s_idx++;
            s_ckA += c; s_ckB += s_ckA;
            if (s_idx >= s_len) s_state = UBX_CK_A;
            break;

        case UBX_CK_A:
            if (c != s_ckA) { s_stats.badChecksum++; resync(); break; }
            s_state = UBX_CK_B;
            break;

        case UBX_CK_B:
            if (c != s_ckB) { s_stats.badChecksum++; resync(); break; }
            s_stats.packets++;
            if (s_store) {
                decodeNavPvt(s_payload);
                s_stats.pvt++;
                markNewSample();
            }
            s_state = UBX_SYNC_1;
            break;

        default:
            s_state = UBX_SYNC_1;
            break;
    }
}

#else  /* GPS_USE_NMEA — parser cũ, chỉ để debug */
/* =========================================================
 * NMEA: GGA (vị trí, fix, số vệ tinh, HDOP) + RMC (tốc độ, hướng)
 * Ánh xạ vào cùng GPS_Data_t; mỗi câu GGA đúng checksum tính là 1 "mẫu" (stats.pvt).
 * ========================================================= */
#include <math.h>

#define GPS_LINE_BUF_SIZE 100

static char    s_lineBuf[GPS_LINE_BUF_SIZE];
static uint8_t s_lineIndex = 0;

static float parseFloat(const char *s)
{
    float result = 0.0f, sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    while (*s >= '0' && *s <= '9') { result = result * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') { result += (float)(*s - '0') * frac; frac *= 0.1f; s++; }
    }
    return result * sign;
}

/* "ddmm.mmmmm" → độ * 1e7, tách phần độ nguyên để không mất chính xác */
static int32_t nmeaToE7(const char *s)
{
    int32_t intPart = 0;
    while (*s >= '0' && *s <= '9') { intPart = intPart * 10 + (*s - '0'); s++; }
    float minutes = (float)(intPart % 100);
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') { minutes += (float)(*s - '0') * frac; frac *= 0.1f; s++; }
    }
    return (intPart / 100) * 10000000 + (int32_t)(minutes * (10000000.0f / 60.0f) + 0.5f);
}

static int hexVal(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool nmeaChecksumOk(const char *line)
{
    if (line[0] != '$') return false;
    uint8_t cs = 0;
    const char *p = line + 1;
    while (*p && *p != '*') cs ^= (uint8_t)*p++;
    if (*p != '*') return false;
    int hi = hexVal(p[1]), lo = hexVal(p[2]);
    if (hi < 0 || lo < 0) return false;
    return cs == (uint8_t)((hi << 4) | lo);
}

static const char *nthField(const char *line, int n)
{
    int count = 0;
    for (const char *p = line; *p; p++) {
        if (count == n) return p;
        if (*p == ',') count++;
    }
    return NULL;
}

static void parseGGA(const char *line)
{
    const char *fLat = nthField(line, 2), *fNS = nthField(line, 3);
    const char *fLon = nthField(line, 4), *fEW = nthField(line, 5);
    const char *fFix = nthField(line, 6), *fSat = nthField(line, 7);
    const char *fHdop = nthField(line, 8), *fAlt = nthField(line, 9);
    if (!fLat || !fLon || !fFix) return;

    int q = (int)parseFloat(fFix);
    s_data.fixType    = (q > 0) ? 3U : 0U;
    s_data.flags      = (q > 0) ? 1U : 0U;
    s_data.fixValid   = (q > 0);
    s_data.satellites = fSat ? (uint8_t)parseFloat(fSat) : 0U;
    s_data.hdop       = (fHdop && *fHdop != ',') ? parseFloat(fHdop) : 99.0f;
    s_data.pDOP       = (uint16_t)(s_data.hdop * 100.0f);
    s_data.hAccM      = s_data.hdop * 2.5f;   /* NMEA không có hAcc: ước lượng thô từ HDOP */
    s_data.hAccMm     = (uint32_t)(s_data.hAccM * 1000.0f);

    if (s_data.fixValid) {
        int32_t lat = nmeaToE7(fLat), lon = nmeaToE7(fLon);
        if (fNS && fNS[0] == 'S') lat = -lat;
        if (fEW && fEW[0] == 'W') lon = -lon;
        s_data.latE7 = lat;
        s_data.lonE7 = lon;
        s_data.latitude  = (float)lat * 1e-7f;
        s_data.longitude = (float)lon * 1e-7f;
        if (fAlt) {
            s_data.altitudeMsl = parseFloat(fAlt);
            s_data.hMSLmm = (int32_t)(s_data.altitudeMsl * 1000.0f);
        }
    }
    s_stats.pvt++;
    markNewSample();
}

static void parseRMC(const char *line)
{
    const char *fStatus = nthField(line, 2), *fSpeed = nthField(line, 7), *fCourse = nthField(line, 8);
    if (!fStatus || fStatus[0] != 'A' || !fSpeed) { s_data.velValid = false; return; }

    float knots = parseFloat(fSpeed);
    s_data.speedKmh  = knots * 1.852f;
    if (fCourse && *fCourse != ',') s_data.courseDeg = parseFloat(fCourse);
    float ms = knots * 0.514444f, crs = s_data.courseDeg * (3.14159265f / 180.0f);
    s_data.velN = ms * cosf(crs);
    s_data.velE = ms * sinf(crs);
    s_data.velNmms = (int32_t)(s_data.velN * 1000.0f);
    s_data.velEmms = (int32_t)(s_data.velE * 1000.0f);
    s_data.gSpeedMms = (int32_t)(ms * 1000.0f);
    s_data.velValid = true;
}

static void nmeaByte(char c)
{
    if (c == '\r') return;
    if (c != '\n') {
        if (s_lineIndex < GPS_LINE_BUF_SIZE - 1) s_lineBuf[s_lineIndex++] = c;
        else { s_lineIndex = 0; s_stats.resync++; }
        return;
    }
    s_lineBuf[s_lineIndex] = '\0';
    s_lineIndex = 0;
    if (s_lineBuf[0] != '$') return;
    if (!nmeaChecksumOk(s_lineBuf)) { s_stats.badChecksum++; return; }
    s_stats.packets++;
    if (strncmp(s_lineBuf + 3, "GGA", 3) == 0)      parseGGA(s_lineBuf);
    else if (strncmp(s_lineBuf + 3, "RMC", 3) == 0) parseRMC(s_lineBuf);
}
#endif /* GPS_USE_NMEA */

/* =========================================================
 * API
 * ========================================================= */
void GPS_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_stats, 0, sizeof(s_stats));
    s_newData = false;
#ifndef GPS_USE_NMEA
    s_state = UBX_SYNC_1;
#endif
    UART1_Init(GPS_UART_CLK, GPS_BAUD);
}

void GPS_Process(void)
{
    while (UART1_RxAvailable()) {
        char c = UART1_RxRead();
        s_stats.bytes++;
#ifndef GPS_USE_NMEA
        ubxByte((uint8_t)c);
#else
        nmeaByte(c);
#endif
    }
}

const GPS_Data_t *GPS_GetData(void)
{
    return &s_data;
}

const GPS_Stats_t *GPS_GetStats(void)
{
    return &s_stats;
}

bool GPS_HasFix(void)
{
    return s_data.fixValid;
}

bool GPS_IsNewData(void)
{
    if (s_newData) {
        s_newData = false;
        return true;
    }
    return false;
}

bool GPS_IsHealthy(uint32_t nowTick)
{
    if (s_stats.pvt == 0U) return false;
    if ((nowTick - s_stats.lastPvtTick) > GPS_HEALTH_TIMEOUT_MS) return false;
    if (!s_data.fixValid) return false;
    if (s_data.satellites < GPS_HEALTH_MIN_SATS) return false;
    if (s_data.hAccM > GPS_HEALTH_MAX_HACC_M) return false;
    return true;
}
