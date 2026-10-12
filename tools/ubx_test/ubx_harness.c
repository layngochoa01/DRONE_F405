/* Harness PC cho parser UBX của firmware (src/sensors/gps.c) — biên dịch bằng gcc.
 *
 *   ./ubx_harness <file.ubx>          đưa từng byte của log thô vào parser, in NAV-PVT + thống kê
 *   ./ubx_harness --selftest          test đơn vị với gói dựng tay
 *   ./ubx_harness --make-sample out   ghi 1 file .ubx mẫu (PVT tốt, gói lạ, sai checksum, gói cụt)
 *
 * Log thô lấy trực tiếp từ GPS qua USB-UART (không qua board):
 *   stty -F /dev/ttyUSB0 115200 raw -echo && timeout 30 cat /dev/ttyUSB0 > raw.ubx
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "gps.h"

/* ---------- Stub phần cứng mà gps.c cần ---------- */
static const uint8_t *s_feed;
static size_t s_feedLen, s_feedPos;
static uint32_t s_tick;

uint32_t get_tick(void) { return s_tick; }
void delay_ms(uint32_t ms) { s_tick += ms; }
void UART1_Init(uint32_t clk, uint32_t baud) { (void)clk; (void)baud; }
bool UART1_RxAvailable(void) { return s_feedPos < s_feedLen; }
char UART1_RxRead(void) { return (char)s_feed[s_feedPos++]; }

static void feed(const uint8_t *buf, size_t len)
{
    s_feed = buf; s_feedLen = len; s_feedPos = 0;
    GPS_Process();
}

/* ---------- Dựng gói ---------- */
static void w4(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void w2(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

/* Ghi khung UBX vào out, trả về số byte */
static size_t ubxFrame(uint8_t *out, uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
    out[0] = 0xB5; out[1] = 0x62; out[2] = cls; out[3] = id; out[4] = len & 0xFF; out[5] = len >> 8;
    memcpy(out + 6, pl, len);
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < 6U + len; i++) { a += out[i]; b += a; }
    out[6 + len] = a; out[7 + len] = b;
    return 8U + len;
}

typedef struct {
    uint32_t iTOW; uint8_t fixType, flags, numSV;
    int32_t lat, lon, hMSL; uint32_t hAcc, vAcc;
    int32_t velN, velE, velD, gSpeed, headMot; uint32_t sAcc; uint16_t pDOP;
} Pvt;

static void pvtPayload(uint8_t p[92], const Pvt *v)
{
    memset(p, 0, 92);
    w4(p + 0, v->iTOW); p[20] = v->fixType; p[21] = v->flags; p[23] = v->numSV;
    w4(p + 24, (uint32_t)v->lon); w4(p + 28, (uint32_t)v->lat); w4(p + 36, (uint32_t)v->hMSL);
    w4(p + 40, v->hAcc); w4(p + 44, v->vAcc);
    w4(p + 48, (uint32_t)v->velN); w4(p + 52, (uint32_t)v->velE); w4(p + 56, (uint32_t)v->velD);
    w4(p + 60, (uint32_t)v->gSpeed); w4(p + 64, (uint32_t)v->headMot); w4(p + 68, v->sAcc);
    w2(p + 76, v->pDOP);
}

static const Pvt k_pvt = {
    .iTOW = 345600100, .fixType = 3, .flags = 0x01, .numSV = 12,
    .lat = 107769123, .lon = 1067001234, .hMSL = 12345, .hAcc = 850, .vAcc = 1400,
    .velN = 1250, .velE = -340, .velD = 20, .gSpeed = 1295, .headMot = 34500000,
    .sAcc = 180, .pDOP = 135,
};

/* Ghi chuỗi test: PVT tốt, gói NAV khác (bỏ qua), gói dài 300 byte (bỏ qua), rác,
 * PVT sai checksum, PVT bị cắt giữa chừng, PVT tốt. Trả về số byte. */
static size_t buildSequence(uint8_t *buf)
{
    size_t n = 0;
    uint8_t pl[300];
    Pvt v = k_pvt;

    pvtPayload(pl, &v);                      n += ubxFrame(buf + n, 0x01, 0x07, pl, 92);   /* 1. tốt */
    memset(pl, 0x55, 16);                    n += ubxFrame(buf + n, 0x01, 0x03, pl, 16);   /* NAV-STATUS */
    memset(pl, 0xB5, 300);                   n += ubxFrame(buf + n, 0x0A, 0x09, pl, 300);  /* gói dài, payload chứa B5 */
    const uint8_t junk[] = { 0x00, 0xB5, 0x13, 0x24, 0x0D, 0x0A };
    memcpy(buf + n, junk, sizeof junk);      n += sizeof junk;                              /* rác */
    v.iTOW += 100; pvtPayload(pl, &v);
    size_t f = ubxFrame(buf + n, 0x01, 0x07, pl, 92); buf[n + f - 1] ^= 0x01; n += f;       /* 2. sai checksum */
    v.iTOW += 100; pvtPayload(pl, &v);
    f = ubxFrame(buf + n, 0x01, 0x07, pl, 92); n += 40;                                     /* 3. bị cắt sau 40 byte */
    (void)f;
    v.iTOW += 100; v.fixType = 5; pvtPayload(pl, &v); n += ubxFrame(buf + n, 0x01, 0x07, pl, 92);  /* 4. bị nuốt bởi gói cụt */
    v.iTOW += 100; v.fixType = 3; v.lat += 10; pvtPayload(pl, &v); n += ubxFrame(buf + n, 0x01, 0x07, pl, 92);  /* 5. tốt */
    return n;
}

static void printPvt(void)
{
    const GPS_Data_t *g = GPS_GetData();
    printf("PVT iTOW=%u fix=%u flags=0x%02X sv=%u lat=%ld lon=%ld hMSL=%ldmm hAcc=%umm vAcc=%umm "
           "vN=%ld vE=%ld vD=%ld gs=%ld sAcc=%u pDOP=%u | fixValid=%d\n",
           g->iTOW, g->fixType, g->flags, g->satellites, (long)g->latE7, (long)g->lonE7, (long)g->hMSLmm,
           g->hAccMm, g->vAccMm, (long)g->velNmms, (long)g->velEmms, (long)g->velDmms,
           (long)g->gSpeedMms, g->sAccMms, g->pDOP, g->fixValid);
}

static void printStats(void)
{
    const GPS_Stats_t *s = GPS_GetStats();
    printf("STATS bytes=%u packets=%u pvt=%u badChecksum=%u resync=%u\n",
           s->bytes, s->packets, s->pvt, s->badChecksum, s->resync);
}

static int s_fail = 0;
#define CHECK(cond) do { if (cond) printf("  PASS  %s\n", #cond); \
                         else { printf("  FAIL  %s\n", #cond); s_fail++; } } while (0)

static int selftest(void)
{
    uint8_t buf[2048];
    size_t n;
    const GPS_Data_t *g;
    const GPS_Stats_t *s;

    printf("[1] Một gói NAV-PVT tốt\n");
    GPS_Init();
    uint8_t pl[92]; pvtPayload(pl, &k_pvt);
    n = ubxFrame(buf, 0x01, 0x07, pl, 92);
    s_tick = 1000; feed(buf, n);
    g = GPS_GetData(); s = GPS_GetStats();
    CHECK(s->pvt == 1 && s->packets == 1 && s->badChecksum == 0);
    CHECK(g->iTOW == 345600100u && g->fixType == 3 && g->satellites == 12);
    CHECK(g->latE7 == 107769123 && g->lonE7 == 1067001234 && g->hMSLmm == 12345);
    CHECK(g->hAccMm == 850 && g->vAccMm == 1400 && g->sAccMms == 180 && g->pDOP == 135);
    CHECK(g->velNmms == 1250 && g->velEmms == -340 && g->velDmms == 20 && g->gSpeedMms == 1295);
    CHECK(g->fixValid && g->velValid);
    CHECK(GPS_IsNewData() && !GPS_IsNewData());
    CHECK(GPS_IsHealthy(1200) && !GPS_IsHealthy(1600));

    printf("[2] Sai checksum → bỏ gói, đếm badChecksum + resync\n");
    GPS_Init();
    n = ubxFrame(buf, 0x01, 0x07, pl, 92); buf[n - 2] ^= 0xFF;
    feed(buf, n);
    s = GPS_GetStats();
    CHECK(s->pvt == 0 && s->badChecksum == 1 && s->resync == 1 && !GPS_IsNewData());

    printf("[3] Chuỗi hỗn hợp (gói lạ, gói 300 byte, rác, sai checksum, gói cụt)\n");
    GPS_Init();
    n = buildSequence(buf);
    feed(buf, n);
    g = GPS_GetData(); s = GPS_GetStats();
    printStats();
    CHECK(s->pvt == 2);                       /* gói 1 và 5 */
    CHECK(s->packets == 4);                   /* PVT1, NAV-STATUS, gói 300 byte, PVT5 */
    CHECK(s->badChecksum >= 2);               /* gói 2 + gói cụt nuốt gói 4 */
    CHECK(g->latE7 == 107769123 + 10 && g->fixType == 3);

    printf("[4] fixType 5 (chỉ thời gian) không được coi là fix\n");
    GPS_Init();
    Pvt v = k_pvt; v.fixType = 5; pvtPayload(pl, &v);
    n = ubxFrame(buf, 0x01, 0x07, pl, 92); feed(buf, n);
    CHECK(GPS_GetStats()->pvt == 1 && !GPS_GetData()->fixValid);

    printf("[5] gnssFixOK = 0 → không fix; hAcc > 3 m → không healthy\n");
    GPS_Init();
    v = k_pvt; v.flags = 0; pvtPayload(pl, &v);
    n = ubxFrame(buf, 0x01, 0x07, pl, 92); s_tick = 5000; feed(buf, n);
    CHECK(!GPS_GetData()->fixValid);
    v = k_pvt; v.hAcc = 3500; pvtPayload(pl, &v);
    n = ubxFrame(buf, 0x01, 0x07, pl, 92); feed(buf, n);
    CHECK(GPS_GetData()->fixValid && !GPS_IsHealthy(5000));

    printf("[6] NAV-PVT sai độ dài (84 byte) bị bỏ qua, không lệch đồng bộ\n");
    GPS_Init();
    n  = ubxFrame(buf, 0x01, 0x07, pl, 84);
    pvtPayload(pl, &k_pvt);
    n += ubxFrame(buf + n, 0x01, 0x07, pl, 92);
    feed(buf, n);
    s = GPS_GetStats();
    CHECK(s->packets == 2 && s->pvt == 1 && s->resync == 0);

    printf("\n%s (%d lỗi)\n", s_fail ? "CÓ TEST HỎNG" : "TẤT CẢ ĐẠT", s_fail);
    return s_fail ? 1 : 0;
}

static int runFile(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 1; }
    GPS_Init();
    uint8_t chunk[256];
    size_t n;
    uint32_t lastPvt = 0;
    while ((n = fread(chunk, 1, sizeof chunk, f)) > 0) {
        s_tick += 10;
        s_feed = chunk; s_feedLen = n; s_feedPos = 0;
        while (s_feedPos < s_feedLen) {
            size_t end = s_feedPos + 1;          /* đưa từng byte để in đúng từng gói */
            size_t save = s_feedLen;
            s_feedLen = end;
            GPS_Process();
            s_feedLen = save;
            if (GPS_GetStats()->pvt != lastPvt) {
                lastPvt = GPS_GetStats()->pvt;
                printPvt();
            }
        }
    }
    fclose(f);
    printStats();
    return 0;
}

static int makeSample(const char *path)
{
    uint8_t buf[2048];
    size_t n = buildSequence(buf);
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return 1; }
    fwrite(buf, 1, n, f);
    fclose(f);
    printf("Đã ghi %zu byte vào %s\n", n, path);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--selftest") == 0) return selftest();
    if (argc == 3 && strcmp(argv[1], "--make-sample") == 0) return makeSample(argv[2]);
    if (argc == 2) return runFile(argv[1]);
    fprintf(stderr, "Dùng: %s <file.ubx> | --selftest | --make-sample <out.ubx>\n", argv[0]);
    return 2;
}
