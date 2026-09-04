#ifndef ALTITUDE_H
#define ALTITUDE_H

typedef struct {
    float altitude;          /* mét, tương đối so với điểm khởi động */
    float verticalVelocity;   /* m/s, dương = đang lên */
} AltitudeState_t;

/* Gọi 1 lần lúc khởi động, sau khi đã có vài mẫu baro ổn định */
void Altitude_Init(void);

/* Đặt áp suất tham chiếu (mốc 0m) - gọi sau Altitude_Init(), dùng mẫu baro đầu tiên */
void Altitude_SetBaseline(float currentPressurePa);

/* Chuyển áp suất (Pa) sang độ cao tương đối (m), dựa trên baseline đã set */
float Altitude_PressureToMeters(float pressurePa);

/*
 * Cập nhật complementary filter.
 * baroAltitude    : độ cao tính từ Altitude_PressureToMeters(), đơn vị mét
 * verticalAccel   : gia tốc thẳng đứng THẬT (world-frame Z đã trừ gravity), đơn vị cm/s^2 (theo GRAVITY_CMSS trong imu.h)
 * dT              : chu kỳ gọi hàm này, giây
 */
void Altitude_Update(float baroAltitude, float verticalAccel, float dT);

const AltitudeState_t *Altitude_Get(void);

#endif