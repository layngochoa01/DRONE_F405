#include "gps.h"
#include "../drivers/usart1.h"
#include "system.h"
#include <string.h>

#define GPS_UART_CLK        84000000U

static GPS_Data_t  s_data;
static GPS_Stats_t s_stats;
static bool        s_newData = false;

static void markNewSample(void)
{
    s_newData = true;
    s_stats.lastPvtTick = get_tick();
}

#define UBX_SYNC1           0xB5U
#define UBX_SYNC2           0x62U
#define UBX_CLASS_NAV       0x01U
#define UBX_ID_NAV_PVT      0x07U
#define UBX_NAV_PVT_LEN     92U
#define UBX_MAX_LEN         1024U

typedef enum {
    UBX_SYNC_1, UBX_SYNC_2, UBX_CLASS, UBX_ID, UBX_LEN_1, UBX_LEN_2,
    UBX_PAYLOAD, UBX_CK_A, UBX_CK_B,
} UbxState_e;

static UbxState_e s_state = UBX_SYNC_1;
static uint8_t    s_class, s_id, s_ckA, s_ckB;
static uint16_t   s_len, s_idx;
static bool       s_store;
static uint8_t    s_payload[UBX_NAV_PVT_LEN];

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
    int32_t headMot  = rdI4(p + 64);
    s_data.sAccMms   = rdU4(p + 68);
    s_data.pDOP      = rdU2(p + 76);

    bool fix3d = (s_data.fixType == 3U) || (s_data.fixType == 4U);
    s_data.fixValid = fix3d && ((s_data.flags & 0x01U) != 0U);
    s_data.velValid = s_data.fixValid;

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
    s_data.hdop        = (float)s_data.pDOP * 0.01f;
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
            else if (c == UBX_SYNC1) s_state = UBX_SYNC_2;
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

void GPS_Init(void)
{
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_stats, 0, sizeof(s_stats));
    s_newData = false;
    s_state = UBX_SYNC_1;
    UART1_Init(GPS_UART_CLK, GPS_BAUD);
}

void GPS_Process(void)
{
    while (UART1_RxAvailable()) {
        char c = UART1_RxRead();
        s_stats.bytes++;
        ubxByte((uint8_t)c);
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
