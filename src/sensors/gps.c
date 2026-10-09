#include "gps.h"
#include "../drivers/usart1.h"
#include <string.h>
#include <math.h>

#define GPS_LINE_BUF_SIZE 100

static char    s_lineBuf[GPS_LINE_BUF_SIZE];
static uint8_t s_lineIndex = 0;

static GPS_Data_t s_data = { .fixValid = false };
static bool s_newData = false;
static GPS_Stats_t s_stats = { 0 };

void GPS_Init(void)
{
    UART1_Init(84000000U, GPS_BAUD);
    s_lineIndex = 0;
    s_data.fixValid = false;
    s_data.velValid = false;
    s_newData = false;
}

static float parseFloat(const char *s)
{
    float result = 0.0f;
    float sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }

    while (*s >= '0' && *s <= '9') {
        result = result * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') {
            result += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }
    return result * sign;
}

/* "ddmm.mmmmm" / "dddmm.mmmmm" → độ * 1e7, tách phần độ nguyên để không mất chính xác */
static int32_t nmeaToE7(const char *s)
{
    int32_t intPart = 0;
    while (*s >= '0' && *s <= '9') {
        intPart = intPart * 10 + (*s - '0');
        s++;
    }
    float minutes = (float)(intPart % 100);
    if (*s == '.') {
        s++;
        float frac = 0.1f;
        while (*s >= '0' && *s <= '9') {
            minutes += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
        }
    }
    return (intPart / 100) * 10000000 + (int32_t)(minutes * (10000000.0f / 60.0f) + 0.5f);
}

static const char *nthField(const char *line, int n)
{
    int count = 0;
    const char *p = line;
    while (*p) {
        if (count == n) return p;
        if (*p == ',') count++;
        p++;
    }
    return NULL;
}

static void parseGGA(const char *line)
{
    const char *fLat = nthField(line, 2);
    const char *fNS  = nthField(line, 3);
    const char *fLon = nthField(line, 4);
    const char *fEW  = nthField(line, 5);
    const char *fFix = nthField(line, 6);
    const char *fSat = nthField(line, 7);
    const char *fHdop = nthField(line, 8);
    const char *fAlt = nthField(line, 9);

    if (!fLat || !fFix) return;

    int fixQuality = (int)parseFloat(fFix);
    s_stats.gga++;
    s_stats.fixQuality = (uint8_t)fixQuality;
    /* Số vệ tinh / HDOP có cả khi chưa fix — tiện theo dõi lúc chờ fix */
    s_data.satellites = fSat ? (uint8_t)parseFloat(fSat) : 0U;
    s_data.hdop = (fHdop && *fHdop != ',') ? parseFloat(fHdop) : 99.0f;

    s_data.fixValid = (fixQuality > 0);
    if (!s_data.fixValid) return;

    if (!fLon) { s_data.fixValid = false; return; }


    int32_t latE7 = nmeaToE7(fLat);
    if (fNS && fNS[0] == 'S') latE7 = -latE7;
    s_data.latE7 = latE7;
    s_data.latitude = (float)latE7 * 1e-7f;

    int32_t lonE7 = nmeaToE7(fLon);
    if (fEW && fEW[0] == 'W') lonE7 = -lonE7;
    s_data.lonE7 = lonE7;
    s_data.longitude = (float)lonE7 * 1e-7f;

    if (fAlt) s_data.altitudeMsl = parseFloat(fAlt);
    s_newData = true;
}

static void parseRMC(const char *line)
{
    const char *fStatus = nthField(line, 2);
    const char *fSpeed  = nthField(line, 7);
    const char *fCourse = nthField(line, 8);

    if (!fStatus || fStatus[0] != 'A' || !fSpeed) {
        s_data.velValid = false;
        return;
    }

    float speedKnots = parseFloat(fSpeed);
    s_data.speedKmh  = speedKnots * 1.852f;
    if (fCourse && *fCourse != ',') s_data.courseDeg = parseFloat(fCourse);

    float speedMs = speedKnots * 0.514444f;
    float course  = s_data.courseDeg * (3.14159265f / 180.0f);
    s_data.velN = speedMs * cosf(course);
    s_data.velE = speedMs * sinf(course);
    s_data.velValid = true;
}

void GPS_Process(void)
{
    while (UART1_RxAvailable()) {
        char c = UART1_RxRead();
        s_stats.bytes++;

        if (c == '\n') {
            s_lineBuf[s_lineIndex] = '\0';
            s_lineIndex = 0;
            if (s_lineBuf[0] == '$') s_stats.sentences++;

            if (strncmp(s_lineBuf, "$GPGGA", 6) == 0 || strncmp(s_lineBuf, "$GNGGA", 6) == 0) {
                parseGGA(s_lineBuf);
            } else if (strncmp(s_lineBuf, "$GPRMC", 6) == 0 || strncmp(s_lineBuf, "$GNRMC", 6) == 0) {
                parseRMC(s_lineBuf);
            }
            continue;
        }
        if (c == '\r') continue;

        if (s_lineIndex < GPS_LINE_BUF_SIZE - 1) {
            s_lineBuf[s_lineIndex++] = c;
        } else {
            s_lineIndex = 0;
        }
    }
}

const GPS_Data_t *GPS_GetData(void)
{
    return &s_data;
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

const GPS_Stats_t *GPS_GetStats(void)
{
    return &s_stats;
}
