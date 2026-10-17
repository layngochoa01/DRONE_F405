# PROJECT_SUMMARY: BlueberryF405 (STM32F405, bare-metal)

Bản tóm tắt này được viết sau khi đọc toàn bộ mã nguồn tại commit `305a0a6` (nhánh `main`). Mọi khẳng định kèm `file:dòng` theo đúng nội dung file ở commit đó.

Nhãn dùng trong tài liệu:

- **(tính)**: con số tôi tính từ hằng số trong code, chưa đo trên phần cứng.
- **(chưa đo)**: cần số đo thật từ board mới biết.
- **(chưa chắc)**: chưa xác nhận được, xem mục 13.

Số dòng trong `register.h`, `main.c`... có thể lệch nếu file bị sửa sau commit trên.

## Mục lục

1. Cây thư mục và vai trò file
2. Luồng chạy của `main.c`
3. BẢNG TẦN SỐ
4. Danh sách dT và hằng số thời gian gõ cứng
5. Cấu hình bộ lọc
6. Danh sách ngắt
7. Từng module
8. Flash storage
9. Trạng thái từng phần
10. Chuỗi PWM và motor
11. TODO, warning, bug và rủi ro
12. Build và nạp
13. Điều chưa chắc

---

## 1. Cây thư mục và vai trò file

Tất cả 30 file `.c` của firmware đều có trong `Makefile`. File `.h` không liệt kê riêng, chỉ được include. Bảng gộp `.c` và `.h` cùng tên vào một dòng.

```
.
├── Makefile, stm32_flash.ld, startup.s, README.md, PROJECT_SUMMARY.md, .gitignore
├── include/        register.h, system.h, types.h
├── src/            main.c, syscalls.c
│   ├── drivers/    spi, I2C, uart5, usart1, sbus, timer, pwm_out, flash_storage
│   ├── sensors/    icm42605, spl06, hmc5883l, gps, hc05
│   ├── estimation/ imu, attitude, quaternion, filter, altitude, position
│   └── control/    flight_control, flight_state, altitude_control, position_control,
│                   yaw_control, attitude_control, rate_control, pid, motor_mixer
├── tools/          gps_logger.py, analyze_gps.py, ubx_test/
└── docs/gps_test/  dữ liệu và hình của một lần đo GPS đứng yên
```

| File | Vai trò (một dòng) | Trong Makefile |
|---|---|---|
| `Makefile` | Build, nạp (`flash`, `dfu`), `debug`, `dump`, `map` | n/a |
| `stm32_flash.ld` | Linker script: Flash 1 MB, RAM 128 KB, stack ở đỉnh RAM | `Makefile:74` |
| `startup.s` | Bảng vector, reset handler (bật FPU, copy `.data`, xoá `.bss`), handler mặc định | `Makefile:52` |
| `include/register.h` | Địa chỉ và struct thanh ghi (RCC, GPIO, SPI1, I2C1, USART, DMA2, SCB, SysTick, NVIC, TIM) cùng các cờ bit | header |
| `include/system.h` | `get_tick`, `delay_ms`, `CPU_HZ`, bộ đếm chu kỳ DWT, `CRITICAL_BEGIN/END` | header |
| `include/types.h` | `Vec3_t` | header |
| `src/main.c` | Clock, SysTick, boot, handler ngắt (SysTick, HardFault, TIM7, USART1/2), xử lý lệnh Bluetooth, log | `Makefile:13` |
| `src/syscalls.c` | Stub newlib: `_sbrk`, `_write` (ra UART5), các hàm còn lại rỗng | `Makefile:27` |
| `src/drivers/spi.c` | SPI1 master 10.5 MHz và DMA2 (stream 0 RX, stream 3 TX) cho IMU, ISR DMA2 | `Makefile:19` |
| `src/drivers/I2C.c` | I2C1 100 kHz kiểu polling, có timeout đếm vòng lặp | `Makefile:21` |
| `src/drivers/uart5.c` | UART5 115200 (HC-05): TX chờ từng ký tự, RX ring 256 byte qua ngắt, `UART5_WriteF` | `Makefile:20` |
| `src/drivers/usart1.c` | USART1 chỉ RX cho GPS: ring 512 byte qua ngắt | `Makefile:26` |
| `src/drivers/sbus.c` | USART2 100 kbaud 8E2, ngắt giải mã 16 kênh, failsafe, mất khung | `Makefile:24` |
| `src/drivers/timer.c` | TIM7 2 kHz (ngắt, kích đọc IMU) và TIM5 one-shot (đọc cờ, không ngắt) | `Makefile:22` |
| `src/drivers/pwm_out.c` | TIM1 và TIM2 xuất PWM 400 Hz cho 4 motor | `Makefile:25` |
| `src/drivers/flash_storage.c` | Lưu calib vào Flash sector 11 dạng log nối tiếp, CRC32 | `Makefile:23` |
| `src/sensors/icm42605.c` | Driver IMU: cấu hình, đọc DMA, callback, calib, remap trục | `Makefile:14` |
| `src/sensors/spl06.c` | Baro SPL06 qua I2C (8 Hz, oversample 8x) | `Makefile:16` |
| `src/sensors/hmc5883l.c` | La bàn, tự nhận HMC5883L hoặc QMC5883L (tên file giữ từ bản đầu) | `Makefile:17` |
| `src/sensors/gps.c` | Parser UBX, chỉ gói NAV-PVT | `Makefile:18` |
| `src/sensors/hc05.c` | Parser lệnh text từ Bluetooth HC-05 | `Makefile:15` |
| `src/estimation/imu.c` | AHRS Mahony trên quaternion (accel + mag) | `Makefile:28` |
| `src/estimation/attitude.c` | Bọc `imu.c`: đổi đơn vị gyro (rad/s) và accel (cm/s²) | `Makefile:30` |
| `src/estimation/quaternion.c` | Phép toán quaternion, tích phân tốc độ góc | `Makefile:31` |
| `src/estimation/filter.c` | Lọc thông thấp PT1 cho 6 trục IMU | `Makefile:29` |
| `src/estimation/altitude.c` | Kalman 3 trạng thái độ cao, đổi áp suất sang mét, trung bình trượt baro | `Makefile:32` |
| `src/estimation/position.c` | Bộ lọc bù vị trí/vận tốc ngang (GPS + gia tốc IMU) | `Makefile:33` |
| `src/control/flight_control.c` | Điều phối: `FlightControl_FastUpdate` (2 kHz, trong ngắt) và `FlightControl_SlowUpdate` (main loop) | `Makefile:42` |
| `src/control/flight_state.c` | Máy trạng thái chế độ bay, đọc SBUS | `Makefile:36` |
| `src/control/altitude_control.c` | Vòng độ cao (P vị trí rồi PID vận tốc rồi ga) | `Makefile:35` |
| `src/control/position_control.c` | Vòng vị trí XY (P rồi PID vận tốc rồi góc nghiêng) | `Makefile:41` |
| `src/control/yaw_control.c` | Tích phân cần yaw thành yaw mục tiêu | `Makefile:40` |
| `src/control/attitude_control.c` | Sai số quaternion nhân Kp thành tốc độ góc mong muốn | `Makefile:34` |
| `src/control/rate_control.c` | Rate PID 3 trục (deg/s vào, µs ra) | `Makefile:38` |
| `src/control/pid.c` | PID dùng chung: D theo phép đo, anti-windup back-calculation, lọc D tuỳ chọn | `Makefile:37` |
| `src/control/motor_mixer.c` | Quad X: ga và 3 mô-men thành 4 xung µs | `Makefile:39` |
| `tools/gps_logger.py` | Ghi log GPS qua HC-05 ra CSV | không (công cụ PC) |
| `tools/analyze_gps.py` | Phân tích CSV: std, CEP50, R95, trôi, nhiễu vận tốc | không |
| `tools/ubx_test/` | `Makefile` + `ubx_harness.c`: biên dịch `src/sensors/gps.c` bằng gcc trên PC và test | Makefile riêng |
| `docs/gps_test/` | Một bộ dữ liệu GPS (csv + 2 hình) | không |
| `README.md` | Giới thiệu dự án | không |

---

## 2. Luồng chạy của `main.c`

### 2.1 Khởi tạo (thứ tự thật, `main.c:271-355`)

1. `main.c:273` bật FPU (CPACR). `startup.s:122-125` cũng làm việc này trước đó.
2. `main.c:275` `clock_init`: HSI, PLL M=16 N=336 P=2, SYSCLK 168 MHz, APB1 /4 (42 MHz), APB2 /2 (84 MHz). Hằng ở `register.h:88-92, 84-86`.
3. `main.c:276` `systick_init`: `LOAD = 168000-1` (`main.c:80`), tick 1 ms.
4. `main.c:277` `cycle_counter_init` (DWT, `system.h:14-19`).
5. `main.c:278` `led_init`.
6. `main.c:280-281` `Motor_Init(168000000U, 84000000U)` rồi `Motor_StopAll`. Motor giữ 1000 µs từ đây.
7. `main.c:283` `ICM42605_Init`. Thành công thì LED đỏ nháy 3 lần (`285-291`). Thất bại thì LED xanh nháy mãi (`292-297`) và dừng.
8. `main.c:299-300` `HC05_Init(42000000U, 115200U)`, `SBUS_Init(42000000U)`.
9. `main.c:301-303` `GPS_Init()` nếu `GPS_ENABLE` (cờ ở `main.c:28`, hiện bằng 1).
10. `main.c:304` `delay_ms(15000)`.
11. `main.c:305` `SPL06_Init()` (có gọi `I2C1_Init`, `spl06.c:64`), `main.c:306` `AltitudeBaro_Init(101325.0f)`.
12. `main.c:308-311`: nếu baro OK thì `delay_ms(8000)` rồi `altitude_calibrate_baseline` (`main.c:240-269`): 50 mẫu, timeout 10 s, `delay_ms(20)` mỗi vòng.
13. `main.c:313-317` `Compass_Init()` nếu `COMPASS_ENABLE` (`main.c:27`, hiện bằng 1). `main.c:318-319` in `BARO:.. MAG:.. (loại)`.
14. `main.c:321-345` đọc calib từ Flash vào `s_cal`. Không có thì dùng offset 0 và in `No calib data`.
15. `main.c:347` `AltitudeKF_Init(NOISE_*)`, `main.c:348` `FlightControl_Init`.
16. `main.c:350` `IMU_FilterInit(&s_imuFilter, FC_DT)`, `main.c:351` `ICM42605_SetSampleCallback(imu_sample_isr)`.
17. `main.c:353` `Timer5_Init(84000000U)`, `main.c:354-355` `Timer7_InitHz(84000000U, FC_LOOP_HZ)` rồi `Timer7_Start()`. **Từ dòng 355, chuỗi ngắt 2 kHz bắt đầu chạy.**

Ngắt được bật dần theo từng driver: SysTick từ `main.c:82`; USART1, USART2, UART5 khi `GPS_Init`, `SBUS_Init`, `HC05_Init` chạy (`main.c:299-303`); DMA2 được cho phép trong NVIC lúc `ICM42605_Init` (`spi.c:97-100`) nhưng chưa có truyền nào; TIM7 từ `main.c:354`. Từ lúc `HC05_Init` (dòng 299) đến hết `delay_ms` (dòng 309), các ngắt USART1/USART2/UART5 đã hoạt động nhưng main loop chưa đọc buffer. Tổng thời gian chặn trước vòng lặp chính khoảng 27–36 s (tính: 3 s nháy LED, 15 s, 8 s, cộng 1–10 s lấy mốc baro).

### 2.2 Vòng lặp chính (`main.c:362-508`)

Mỗi vòng, theo thứ tự:

| Dòng | Việc |
|---|---|
| `365` | `FlightControl_SlowUpdate()` |
| `367-369` | `GPS_Process()` |
| `370` | `HC05_Poll()` |
| `372-435` | Xử lý lệnh Bluetooth: log, `MAGCAL`, `BENCH`, calib (chỉ khi DISARMED, `414-420`), motor test |
| `437-506` | Nếu đã `START`: in log theo `log_mode`. Log thường mỗi 200 ms (`459`); chế độ `GPSCSV` in mỗi gói PVT (`440`) và dòng `S` mỗi 1000 ms (`451`) |
| `507` | Chưa `START` thì bật LED xanh |

### 2.3 Khối nào chạy ở ngữ cảnh nào

| Ngữ cảnh | Việc |
|---|---|
| **Trong ngắt** | SysTick (`main.c:39`). TIM7 (`main.c:126`): kích đọc IMU. DMA2 stream 0 (`spi.c:124`): kết thúc SPI, gọi callback IMU rồi `imu_sample_isr` (`main.c:134-141`) rồi `FlightControl_FastUpdate`. USART1 (`main.c:143`), USART2/SBUS (`main.c:148`), UART5 (`uart5.c:95`): nhận byte |
| **Trong main loop** | `FlightControl_SlowUpdate` (đọc baro, compass, Kalman predict/update, máy trạng thái, vòng độ cao, yaw, XY), parse GPS, parse lệnh Bluetooth, calib, ghi Flash, mọi lệnh in UART5 |

Dữ liệu chia sẻ giữa hai bên đi qua `s_cmd` (main ghi, ngắt đọc) và `s_acc` (ngắt ghi, main lấy), xem mục 11.

---

## 3. BẢNG TẦN SỐ

Mỗi dòng: tần số thực tế (theo cấu hình trong code), nơi cấu hình, dòng code. Chỗ ghi "(chưa đo)" là chưa có số đo từ phần cứng.

| # | Hạng mục | Tần số | Nơi cấu hình (file:dòng) | Ghi chú |
|---|---|---|---|---|
| 1 | Gyro ODR (ICM-42605) | **4 kHz** | `icm42605.c:100` (macro `icm42605.h:76`) | Ghi `GYRO_CONFIG0` |
| 2 | Accel ODR (ICM-42605) | **4 kHz** | `icm42605.c:102` (macro `icm42605.h:84`) | Ghi `ACCEL_CONFIG0` |
| 3 | Bộ lọc UI gyro và accel | ghi `0xFF` (`ICM_FILT_BW_LOW_LATENCY`) | `icm42605.c:105` (`icm42605.h:91`) | Băng thông tương ứng: (chưa chắc), cần datasheet thanh ghi `GYRO_ACCEL_CONFIG0` |
| 4 | AAF gyro (bank 1) | `DELT=21`, `DELT_SQR=440`, `BITSHIFT=6` | `icm42605.c:10-12` (hằng), `109-111` (ghi thanh ghi) | Băng thông Hz: (chưa chắc) |
| 5 | AAF accel (bank 2) | cùng giá trị, `DELT<<1` ở thanh ghi STATIC2 | `icm42605.c:113-116` | Như trên |
| 6 | Xung SPI1 | 84 MHz / 8 = **10.5 MHz** | `spi.c:43` | Mode 3 (CPOL, CPHA). Burst 15 byte (`icm42605.h:103`), lý thuyết khoảng 11.4 µs (tính) |
| 7 | Timer kích đọc IMU | **2000 Hz** (PSC=83, ARR=499) | `main.c:354` rồi `timer.c:4-30`, `FC_LOOP_HZ` ở `flight_control.h:9` | TIM7 chạy ở 84 MHz |
| 8 | Chu kỳ lấy mẫu IMU thực tế | 2 kHz danh định | đo bằng `LOGPERF` (`main.c:463-468`, `flight_control.c:371-388`) | **(chưa đo)**: không có số liệu trong repo |
| 9 | Mahony (AHRS) | **2 kHz**, dt đo bằng DWT | `flight_control.c:180` rồi `attitude.c:25` rồi `imu.c:171` | dt đo ở `flight_control.c:169-176`, kẹp trong [0.25, 4] × `FC_DT` |
| 10 | Điều khiển tư thế | **2 kHz** | `flight_control.c:197-209` | Chỉ khi `cmd.runRate` |
| 11 | Rate PID | **2 kHz** | `flight_control.c:210` | dT truyền vào là `FC_DT` danh định |
| 12 | Mixer và ghi PWM (CCR) | **2 kHz** | `flight_control.c:215-229` | Xung PWM thực ra 400 Hz, xem dòng 29 |
| 13 | Kalman độ cao, Predict | **khoảng 100 Hz** | `flight_control.h:11` (`FC_ACCEL_BLOCK`=20), `flight_control.c:242-252` | Trung bình 20 mẫu gia tốc, dT = tổng thời gian đo thực (`a.sumDt`) |
| 14 | Kalman độ cao, Update (baro) | **khi SPL06 có mẫu mới, 8 Hz** | SPL06: `spl06.c:83, 85`. Gọi: `flight_control.c:265-266, 127-135` | Poll 25 Hz, `SPL06_Update` trả `false` nếu chưa có mẫu (`spl06.c:95-97`) |
| 15 | Vòng độ cao (ga) | **25 Hz** | `flight_control.c:265, 297-299` | `PERIOD_25HZ_MS`=40 (`flight_control.c:20`) |
| 16 | Máy trạng thái `FlightState_Update` | **25 Hz** | `flight_control.c:272` | Đọc SBUS bên trong |
| 17 | Vòng yaw | **25 Hz** | `flight_control.c:303-306` | |
| 18 | Vòng vị trí XY | **10 Hz** | `flight_control.c:316-329` | Chỉ ở POS_HOLD, FORWARD, LEFT. `PERIOD_10HZ_MS`=100 (`flight_control.c:21`) |
| 19 | Ước lượng vị trí, Predict | **khoảng 100 Hz** | `flight_control.c:254` | Cùng khối với dòng 13 |
| 20 | Ước lượng vị trí, Update (GPS) | **khi có gói PVT mới** | `flight_control.c:257` | Module phát 10 Hz theo số đo trong `docs/gps_test/` (không có lệnh cấu hình tần số trong firmware) |
| 21 | Đọc compass | **25 Hz** (lệch pha 20 ms với baro) | `flight_control.c:260`, khởi tạo `175` | Tần số của chip suy ra từ giá trị thanh ghi theo datasheet, chưa đối chiếu: HMC 15 Hz (`hmc5883l.c:41`, `0x70`), QMC 50 Hz (`hmc5883l.c:23`, `0x15`). Hiệu dụng khoảng 15 / 25 Hz (tính) |
| 22 | Đọc SPL06 | **25 Hz poll**, SPL06 ra mẫu 8 Hz | `flight_control.c:266`, `spl06.c:83-86` | 8 Hz suy ra từ giá trị `0x33` theo datasheet, chưa đối chiếu |
| 23 | Parse GPS | mỗi vòng main loop | `main.c:368` | Không có nhịp cố định. Tần số vòng main loop: (chưa đo) |
| 24 | Parse lệnh Bluetooth | mỗi vòng main loop | `main.c:370` | |
| 25 | Đọc kênh SBUS | **25 Hz** | `flight_state.c:190-193` và `flight_control.c:101-109` | Nhịp khung từ bộ thu: không có trong code, (chưa chắc) |
| 26 | Log Bluetooth thường | **5 Hz** (200 ms) | `main.c:459` | `GPSCSV`: mỗi gói PVT (`main.c:440`) và dòng `S` mỗi 1 s (`main.c:451`) |
| 27 | SysTick | **1 kHz** | `main.c:80` | `get_tick()` đơn vị ms |
| 28 | TIM5 (đếm 5 s cho FORWARD/LEFT) | tick 10 kHz, one-shot | `timer.c:45, 55`, `Timer5_Init` gọi ở `main.c:353` | PSC=8399. Không dùng ngắt |
| 29 | Xung PWM ra ESC | **400 Hz** (chu kỳ 2500 µs) | `pwm_out.h:9`, `pwm_out.c:44` | Motor chỉ cập nhật lệnh 400 lần mỗi giây |
| 30 | PT1 accel, PT1 gyro | cắt 20 Hz, 100 Hz | `filter.c:19-20` | Xem mục 5 |

### 3.1 Khối `ICM42605_IsDataReady`: điều kiện chạy và hàm trong khối

Trong code hiện tại **không còn khối `if (ICM42605_IsDataReady())` trong main loop**. Khối đó đã được thay bằng callback chạy trong ngắt DMA.

- `ICM42605_IsDataReady()` trả `s_data_ready` (`icm42605.c:198-201`). Cờ được bật ở `icm42605.c:191` (trong callback DMA) và xoá trong `ICM42605_GetLatestData` (`icm42605.c:207`).
- Hiện chỉ có `calibCollect` dùng nó (`icm42605.c:223`), để vòng lặp calib chờ mẫu mới.
- Luồng chạy mỗi mẫu IMU:
  1. `TIM7_IRQHandler` (`main.c:126-132`) gọi `ICM42605_TriggerRead` (`icm42605.c:153-166`). Nếu DMA lần trước chưa xong thì trả `ICM_ERR_BUSY` và tăng `s_imuMiss` (`main.c:130`).
  2. DMA2 stream 0 xong thì `DMA2_Stream0_IRQHandler` (`spi.c:124-140`) kéo CS lên, gọi `ICM42605_DMA_Callback` (`icm42605.c:175-196`): giải mã 15 byte, đổi đơn vị, đặt `s_data_ready`, rồi gọi `s_sampleCb` (`icm42605.c:195`).
  3. `imu_sample_isr` (`main.c:134-141`) chạy lần lượt: `ICM42605_RemapAxes` → `ICM42605_ApplyCalibration` → `IMU_FilterApply` → `FlightControl_FastUpdate`.
  4. `FlightControl_FastUpdate` (`flight_control.c:165-234`) chạy: `Attitude_Update` (Mahony), cộng dồn gia tốc hệ trái đất, `Quaternion_FromEuler`, `AttitudeControl_Update`, `RateControl_Update`, `MotorMixer_Update`, `Motor_SetUs` ×4.

IMU không dùng ngắt data-ready: các macro `ICM_UI_DRDY_INT1_EN` (`icm42605.h:98`) và thanh ghi `INT_CONFIG` có khai báo nhưng không được dùng trong `icm42605.c`. Timer đọc theo nhịp riêng, không đồng bộ với nhịp ra mẫu của cảm biến (4 kHz).

---

## 4. Danh sách dT và hằng số thời gian gõ cứng

Đã `grep` toàn bộ `src/` và `include/`. Không còn dạng `0.01f`, `0.002f` hay `1.0f / 100.0f` dùng làm dT. Mọi dT hiện đi theo hai nguồn: **đo thật** (DWT) hoặc **hằng danh định** dẫn xuất từ `FC_LOOP_HZ`, `PERIOD_*`.

### 4.1 Nơi truyền dT và dT lấy từ đâu

| File:dòng | Hàm nhận dT | dT lấy từ | Loại |
|---|---|---|---|
| `flight_control.c:169-176` | (tính dt của vòng nhanh) | chênh `cycle_now()` chia `CPU_HZ`, kẹp [0.25, 4] × `FC_DT` | đo thật |
| `flight_control.c:180` | `Attitude_Update(.., dt)` rồi `IMU_Update`, `Quaternion_IntegrateBodyRate` | dt đo thật | đo thật |
| `flight_control.c:191` | cộng dồn `s_acc.sumDt += dt` | dt đo thật | đo thật |
| `flight_control.c:252` | `AltitudeKF_Predict(azAvg, a.sumDt)` | tổng dt các mẫu trong khối | đo thật |
| `flight_control.c:254` | `Position_Predict(.., a.sumDt)` | như trên | đo thật |
| `flight_control.c:210` | `RateControl_Update(.., FC_DT)` | `FC_DT` = 1/2000 | **danh định** |
| `flight_control.c:272` | `FlightState_Update(.., DT_25HZ)` | `DT_25HZ` = 0.04 | **danh định** |
| `flight_control.c:299` | `AltitudeControl_Update(.., DT_25HZ)` | 0.04 | **danh định** |
| `flight_control.c:305` | `YawControl_Update(.., DT_25HZ)` | 0.04 | **danh định** |
| `flight_control.c:319, 327-328` | `PositionControl_Update*(.., DT_10HZ)` | `DT_10HZ` = 0.1 | **danh định** |
| `flight_control.c:143` | `Attitude_Init(FC_DT)` rồi `IMU_Init` | không dùng (`imu.c:160` có `(void)dT`) | n/a |
| `flight_control.c:148` | `RateControl_Init(FC_DT)` rồi `PID_SetDFilter` | tính hệ số lọc D (`pid.c:37-42`) | danh định |
| `main.c:350` | `IMU_FilterInit(.., FC_DT)` | tính α của PT1 (`filter.c:6-11`) | danh định |

Hệ quả: vòng chậm dùng dT cố định 0.04 s và 0.1 s trong khi nhịp thật dao động do main loop bị chặn bởi I2C và UART. Xem rủi ro R4 ở mục 11.

### 4.2 Hằng số thời gian, tần số khác

| File:dòng | Giá trị | Dùng làm gì |
|---|---|---|
| `flight_control.h:9-11` | `FC_LOOP_HZ`=2000, `FC_DT`=1/2000, `FC_ACCEL_BLOCK`=20 | Nhịp vòng nhanh; số mẫu gộp cho predict |
| `flight_control.c:20-23` | `PERIOD_25HZ_MS`=40, `PERIOD_10HZ_MS`=100, `DT_25HZ`=0.04, `DT_10HZ`=0.1 | Nhịp vòng chậm |
| `flight_control.c:25` | `MAG_TIMEOUT_MS`=200 | Compass coi là hỏng sau 200 ms không có mẫu |
| `flight_control.c:27` | `BARO_NUDGE_ALPHA`=0.05 | Kéo áp suất gốc khi ở dưới đất. Mỗi mẫu baro 8 Hz nên τ khoảng 2.5 s (tính) |
| `flight_control.c:29-30` | `BLIND_THROTTLE`=0.30, `BLIND_RAMP_MS`=10000 | Ga hạ cánh mù |
| `altitude.c:5` | `BIAS_FREEZE_VELOCITY_THRESHOLD`=0.15 | Chỉ học bias khi \|v\| nhỏ hơn 0.15 m/s |
| `altitude.c:6` | `VELOCITY_DAMPING`=0.998 | Nhân vào vận tốc **mỗi lần Predict** (khoảng 100 Hz, nên khoảng 0.82 mỗi giây, tính) |
| `altitude.c:7-8` | `BARO_INNOVATION_GATE`=3.0, `BARO_MA_WINDOW`=4 | Cổng 3σ, trung bình trượt 4 mẫu |
| `altitude.c:10` | `BARO_REJECT_RESET_COUNT`=8 | 8 lần bị loại liên tiếp (khoảng 1 s ở 8 Hz) thì reset theo baro |
| `altitude.c:11` | `BARO_HEALTH_TIMEOUT_MS`=500 | Quá 500 ms không nhận mẫu baro thì không khoẻ |
| `altitude.c:12-13` | `GROUND_BIAS_ALPHA`=0.02, `GROUND_P_ALT_VEL`=1e-3 | Neo mặt đất. Mỗi lần gọi (khoảng 100 Hz) nên τ khoảng 0.5 s (tính) |
| `flight_state.c:15-17` | `ARM_HOLD_TIME_MS`=1000, `DISARM_HOLD_TIME_MS`=500, `GESTURE_HOLD_TIME_MS`=500 | Cộng dồn `dT*1000` mỗi lần gọi (`flight_state.c:176, 199-200, 210-211, 253-254`) nên phụ thuộc dT danh định |
| `flight_state.h:20` | `MOVE_DURATION_MS`=5000 | Đếm bằng TIM5 phần cứng (`flight_state.c:161`), không cộng dồn dT |
| `flight_state.h:31` | `BT_BENCH_TIMEOUT_MS`=120000 | Cộng dồn dT (`flight_state.c:180-181`) |
| `flight_state.h:18-19` | `CLIMB_STEP_METERS`=1.0, `GROUND_ALTITUDE_EPS`=0.05 | Bước leo; ngưỡng chạm đất |
| `position.c:7-11` | `K_POS`=0.30, `K_VEL`=0.30, `K_VEL_FROM_POS`=0.10, `MAX_GPS_JUMP_M`=50 | Hệ số bù, tính theo từng mẫu GPS, không dùng dT |
| `position.h:8`, `gps.h:9` | `POS_GPS_TIMEOUT_MS`=1500, `GPS_HEALTH_TIMEOUT_MS`=500 | Timeout GPS |
| `sbus.c:12` | `SBUS_TIMEOUT_MS`=500 | Mất SBUS sau 500 ms |
| `filter.c:19-20` | 20 Hz, 100 Hz | Cắt PT1 |
| `rate_control.h:9` | `RATE_D_LPF_HZ`=80 | Lọc khâu D |
| `icm42605.h:154`, `main.c:30` | `CALIB_TIMEOUT_MS`=5000, `CALIB_SAMPLES`=2000 | Calib: 2000 mẫu ở 2 kHz khoảng 1 s |
| `I2C.c:4` | `I2C_TIMEOUT`=100000 | Timeout là số vòng lặp, không phải thời gian |
| `main.c:304, 309` | `delay_ms(15000)`, `delay_ms(8000)` | Chờ lúc boot |
| `main.c:242-257` | 50 mẫu, 10000 ms, `delay_ms(20)` | Lấy mốc áp suất |
| `timer.c:12-15, 45, 55` | tick 1 MHz, `/10000`, `ms * 10` | Cách tính PSC/ARR |

---

## 5. Cấu hình bộ lọc

| Bộ lọc | Có/không | Chi tiết (file:dòng) |
|---|---|---|
| PT1 gyro | Có | Cắt **100 Hz** (`filter.c:20`), áp ở 2 kHz trước AHRS (`main.c:139`). α khoảng 0.239 (tính từ `filter.c:6-11`) |
| PT1 accel | Có | Cắt **20 Hz** (`filter.c:19`). α khoảng 0.059 (tính) |
| Lọc khâu D | **Có**, PT1 80 Hz | Roll và pitch (`rate_control.c:11-12`, `RATE_D_LPF_HZ` ở `rate_control.h:9`). α khoảng 0.201 (tính). **Yaw không có**: `kd=0` và không gọi `PID_SetDFilter` (`rate_control.c:10`) |
| Median | **Không có** | Không có trong code |
| Notch, biquad | **Không có** | Không có trong code |
| Lọc nội bộ IMU | Có | UI filter `0xFF` (`icm42605.c:105`) và AAF (`icm42605.c:109-116`). Băng thông thực: (chưa chắc) |
| Baro | Có | Trung bình trượt 4 mẫu (`altitude.c:215-225`), rồi Kalman. Mẫu 8 Hz nên độ trễ trung bình khoảng 1.5 mẫu, tức khoảng 190 ms (tính) |
| Compass | Không | Chỉ trừ offset hard-iron (`hmc5883l.c:122-124`) |
| SBUS | Vùng chết 5% cho yaw | `flight_control.h:18`, `flight_control.c:101-109` |

Lọc D dùng đạo hàm của **phép đo** (gyro), không phải sai số (`pid.c:47`), nên setpoint đổi không gây giật khâu D.

---

## 6. Danh sách ngắt

Mức ưu tiên NVIC: số nhỏ ưu tiên cao hơn. STM32F405 có 4 bit ưu tiên (giá trị ghi vào thanh ghi dịch trái 4). Không có dòng nào đặt `AIRCR/PRIGROUP` nên dùng mặc định.

| Handler | Định nghĩa | IRQ | Ưu tiên đã đặt | Làm gì |
|---|---|---|---|---|
| `systick_handler` | `main.c:39` | SysTick | **không đặt (mặc định 0)** | `ms_tick++` |
| `hardfault_handler` | `main.c:116` | HardFault | cố định | Nháy LED đỏ vô hạn |
| `TIM7_IRQHandler` | `main.c:126` | 55 | **10** (`timer.c:28`) | Xoá cờ UIF, `ICM42605_TriggerRead` (đếm `s_imuMiss` nếu bận) |
| `DMA2_Stream0_IRQHandler` | `spi.c:124` | 56 | **8** (`spi.c:99`) | Kết thúc SPI RX: tắt DMA, CS lên, gọi callback, rồi **toàn bộ vòng nhanh** |
| `DMA2_Stream3_IRQHandler` | `spi.c:142` | 59 | **8** (`spi.c:100`) | Xoá cờ TX, không làm gì khác |
| `USART1_IRQHandler` | `main.c:143` rồi `usart1.c:43` | 37 | **không đặt (mặc định 0)** | Đẩy byte GPS vào ring 512 byte |
| `USART2_IRQHandler` | `main.c:148` rồi `sbus.c:76` | 38 | **không đặt (mặc định 0)** | Ghép khung SBUS, giải mã 16 kênh |
| `UART5_IRQHandler` | `uart5.c:95` | 53 | **13** (`uart5.c:54`) | Đẩy byte HC-05 vào ring 256 byte |
| TIM2 | không có handler | 28 | không có ngắt | `startup.s:60` đặt vector bằng 0. TIM2 chỉ tạo PWM (`pwm_out.c:54-60`, `DIER=0`) |
| TIM5 | không có handler | 50 | không có ngắt | `startup.s:82` bằng 0. Đọc cờ UIF bằng `Timer5_IsExpired` (`timer.c:67-70`) |
| NMI, MemManage, BusFault, UsageFault, SVC, DebugMon, PendSV | weak, vòng lặp vô hạn | n/a | n/a | `startup.s:157-203` |

**Ưu tiên DMA2 là 8. TIM2 không phải nguồn ngắt** (kích đọc IMU đã chuyển sang TIM7 ưu tiên 10). Thứ tự ưu tiên thực tế: SysTick và USART1/USART2 (0) trên DMA2 (8), trên TIM7 (10), trên UART5 (13), trên main loop. Hệ quả: ngắt USART có thể chen vào vòng nhanh (mỗi ngắt rất ngắn nhưng chưa đo); vòng nhanh không bị TIM7 hay UART5 chen vào được.

---

## 7. Từng module

### 7.1 `imu.c`: Mahony

- Hằng số: `IMU_KP_ACC`=2.0, `IMU_KI_ACC`=0.005, `IMU_KP_MAG`=1.0 (`imu.h:21-23`).
- Sai số accel = tích có hướng giữa hướng accel chuẩn hoá và hướng trọng lực ước lượng (`imu.c:113`). P: cộng `vErr * kp * accWeight` vào tốc độ góc (`imu.c:130-132`). I: `gyroDrift += vErr * ki * accWeight * dT` khi \|gyro\| nhỏ hơn 20 °/s (`imu.c:117-122`), kẹp khoảng ±2 °/s (`imu.c:124-127`, tính).
- `accWeight` (`imu.c:57-69`): bằng 1 khi \|a\| gần 1 g, về 0 khi lệch quá 20% (`imu.h:25`), nhân thêm hệ số giảm theo tốc độ quay (đủ 0 ở 80 °/s).
- Compass (`imu.c:71-89, 96-104`): chiếu vector mag xuống mặt phẳng ngang rồi tính lỗi heading, chỉ tác động lên yaw. I của mag cộng vào `gyroDrift.z` (`imu.c:98-100`) và **không bị kẹp** (phần kẹp chỉ nằm trong nhánh accel, `imu.c:124-127`).
- Tích phân quaternion theo phép quay chính xác (`quaternion.c:41-66`), chuẩn hoá bậc nhất không dùng sqrt (`quaternion.c:21-28`).
- Mỗi lần update còn tính góc Euler bằng `atan2f` và `asinf` (`imu.c:144-156`, gọi ở `imu.c:177`), dùng cho log. Xem rủi ro R12.

### 7.2 `attitude.c`
Đổi gyro sang rad/s và accel sang cm/s² (nhân 100) rồi gọi `IMU_Update` (`attitude.c:13-25`).

### 7.3 `altitude.c`: Kalman độ cao

- **Vector trạng thái** `x[3]` = [độ cao, vận tốc, bias accel] (`altitude.c:15`); `P` 3×3 (`:16`), `Q` đường chéo 3 phần tử (`:17`), `R` vô hướng (`:18`). `Q`, `R` nạp ở `AltitudeKF_Init` (`:98-102`).
- **`NOISE_*`** ở `main.c:22-25`: `NOISE_ALT`=0.001, `NOISE_VEL`=0.001, `NOISE_BIAS`=0.00001, `NOISE_BARO`=0.0025 (R).
- **Predict** (`:105-141`): tích phân với accel đã trừ bias; vận tốc nhân `VELOCITY_DAMPING`=0.998 (`:111`); `P = F P Fᵀ + Q`. Q cộng theo từng lần gọi, **không nhân dT** (`:137-138`). Ma trận F (`:118-122`) không chứa hệ số damping.
- **Gating** (`:143-156`): bỏ mẫu baro nếu \|innovation\| lớn hơn 3·√S. Bị loại 8 lần liên tiếp thì reset độ cao theo baro (`resetAltVel`, `:30-41`).
- **Update** (`:158-188`): cập nhật vô hướng với H=[1 0 0]; bias chỉ được cập nhật khi \|v\| nhỏ hơn 0.15 m/s (`:161`).
- **Sức khoẻ**: `AltitudeKF_IsHealthy` (`:190-193`) đúng khi từng nhận ít nhất một mẫu và mẫu gần nhất trong 500 ms.
- **Neo mặt đất** (`:195-199`): đặt h=0, v=0, học bias bằng EMA (α=0.02). Gọi khi DISARMED hoặc IDLE (`flight_control.c:251`).
- Đổi áp suất sang mét: `44330 * (1 - (P/P0)^(1/5.255))` (`:79`).

### 7.4 `position.c`: ước lượng vị trí ngang
Bộ lọc bù, hệ trái đất N/W. Home là fix khoẻ đầu tiên (`position.c:43-49`). Đổi sang mét bằng **hiệu số nguyên E7** rồi nhân hằng (`:51-53`). Predict bằng gia tốc (`:31-34`). Update kéo vị trí và vận tốc về GPS với hệ số 0.30 (`:67-72`). Reset theo GPS nếu mất hơn 1.5 s hoặc nhảy hơn 50 m (`:57-65`). Cổng chất lượng là `GPS_IsHealthy` (`:39`).

### 7.5 `pid.c`
- **Derivative on measurement**: `dRaw = -(measurement - prev) / dT` (`pid.c:47`); lọc PT1 tuỳ chọn (`:49-52`).
- **Anti-windup** back-calculation: `kt = 2/(Ti+Td)` với `Ti=kp/ki`, `Td=kd/kp` (`:23-27`); cộng `backCalc*kt*dT` vào tích phân, bỏ qua nếu cùng dấu với tích phân (`:58-63`); tích phân kẹp trong [outMin, outMax] (`:64`).
- Không có kiểm tra dT bằng 0 trong `PID_Update`. Hàm gọi chịu trách nhiệm (`altitude_control.c:40` có kiểm tra).

### 7.6 `rate_control.c`
| Trục | Kp | Ki | Kd | Giới hạn ra | Lọc D |
|---|---|---|---|---|---|
| Roll | 0.6 | 0.02 | 0.01 | ±400 µs | 80 Hz |
| Pitch | 0.6 | 0.02 | 0.01 | ±400 µs | 80 Hz |
| Yaw | 1.0 | 0.03 | 0.0 | ±200 µs | không |

Hằng ở `rate_control.c:8-10`, giới hạn ở `rate_control.h:6-7`. Vào là deg/s, ra là µs.

### 7.7 `attitude_control.c`
`q_err = conj(q_hiện_tại) ⊗ q_mục_tiêu`, đổi dấu nếu `q0 < 0` (đường ngắn nhất), vector lỗi = `2·q_err.xyz` (`attitude_control.c:4-20`). Tốc độ góc mong muốn = Kp × lỗi (`:22-30`) với Kp 5 / 5 / 3 (`flight_control.h:20-22`), đổi sang deg/s và kẹp ±250 / ±250 / ±120 (`flight_control.h:23-24`, `flight_control.c:205-209`).

### 7.8 `altitude_control.c` (hằng `ALT_*` ở `altitude_control.h`)
- Tầng ngoài P vị trí: Kp=1.0 (`altitude_control.h:8`), vận tốc đặt kẹp [-0.5, +1.0] m/s (`:9-10`).
- Tầng trong PID vận tốc: Kp=0.15, Ki=0.10, Kd=0 (`:11-13`), phần hiệu chỉnh kẹp ±0.25 (`:14`).
- Ga = (0.50 + hiệu chỉnh) / cos(nghiêng), cos kẹp từ 0.5 (`:15, :19`, `altitude_control.c:57-58`); kẹp [0.15, 0.85] (`:17-18`, `altitude_control.c:59`); giới hạn tốc độ đổi ga lên 0.5/s, xuống 1.0/s (`:21-22`, `altitude_control.c:61-63`).
- Ga hover **cố định 0.50**, không tự học (`altitude_control.h:15`).
- Dưới đất: ga bằng 0 (`altitude_control.c:40-45`, `ALT_IDLE_THROTTLE`=0.0).

### 7.9 `yaw_control.c`
Tích phân cần yaw thành yaw mục tiêu với tốc độ tối đa 90 °/s (`yaw_control.c:22`, `yaw_control.h:4`). Yaw mục tiêu không được lệch quá 45° so với yaw thực (`yaw_control.c:24-29`, `yaw_control.h:5`).

### 7.10 `position_control.c`
P vị trí (Kp=0.8) rồi vận tốc đặt kẹp 2 m/s (`position_control.c:43-52`, `position_control.h:6-7`); PID vận tốc Kp=1.2, Ki=0.15, Kd=0 và gia tốc kẹp 3 m/s² (`:28-33`, `position_control.h:8-11`); xoay từ hệ N/W sang hệ thân (`:34-36`); góc nghiêng = atan(gia tốc / g) kẹp 20° (`:38-40`, `position_control.h:12`).

### 7.11 `motor_mixer.c`
Bảng `s_mix` 4×3 (`motor_mixer.c:9-14`), mỗi hàng là [roll, pitch, yaw] của M1..M4. Ra là µs: `throttleUs = 1000 + throttle * 1000`, cộng mô-men; nếu một motor vượt khoảng 1000–2000 thì thu nhỏ phần mô-men, giữ nguyên ga (`:42-60`). Khi chưa armed trả 1000 µs (`:37-40`).

### 7.12 `flight_state.c`: cách đọc SBUS
- Đọc 4 kênh bằng `SBUS_GetChannel` (µs, `flight_state.c:190-193`) rồi phân mức: thấp ≤1200, giữa 1300–1700, cao ≥1800 (`flight_state.c:10-13, 48-54`).
- ARM: ga thấp + yaw cao giữ 1 s (`:208-222`). DISARM: ga thấp + yaw thấp giữ 0.5 s (`:198-206`). Mất SBUS hoặc failsafe thì DISARM ngay (`:185-188`).
- Ga xử lý theo **cạnh** (chỉ khi đổi mức, `:239-242`). Cử chỉ roll/pitch giữ 0.5 s, sau đó phải thả về giữa mới nhận lệnh tiếp (`:244-261`).
- Mất ước lượng độ cao khi đang bay thì chuyển LAND_BLIND (`:232-237`), thoát khi khoẻ lại (`:224-231`).
- ARM qua Bluetooth cho bench: bỏ qua SBUS, tự DISARM sau 120 s (`:179-183, 337-348`).

### 7.13 `sbus.c`
Ngắt USART2 ghép khung 25 byte: chỉ chấp nhận byte đầu `0x0F` (`sbus.c:83-87`), kiểm byte cuối `0x00` (`:95`), giải mã 16 kênh 11 bit (`:100-118`), cờ failsafe (`:120-122`). Đổi 172..1811 sang 1000..2000 µs (`:146`). **Không kiểm tra lỗi parity/framing** của USART. Khai báo nhưng chưa dùng: `s_frameLost`, `s_lastByteTick`, `SBUS_GAP_MS`, `SBUS_FLAG_FRAME_LOST` (`sbus.c:15, 9, 25, 28`).

### 7.14 `gps.c`
Máy trạng thái UBX từng byte (`gps.c:93-161`), chỉ lưu gói NAV-PVT dài 92 byte (`:129`). Gói khác vẫn đếm qua đúng độ dài và kiểm checksum. Giải mã các trường ở `:51-85`. `fixValid` đúng khi `fixType` là 3 hoặc 4 và `flags` bit 0 (`:70-71`). `hdop` thực chất là **pDOP** (`:84`). `GPS_IsHealthy` (`:200-208`). Không có lệnh cấu hình gửi xuống module: TX của USART1 không được cấu hình (`usart1.c:25-30` chỉ cấu hình PA10).

### 7.15 Driver
- **`uart5.c`**: BRR = `(clk + baud/2)/baud` (`:46`, tính ra khoảng 115 069 baud). Ghi từng ký tự chờ cờ TXE, **không timeout** (`:57-61`). `UART5_WriteF` dùng buffer 128 byte trên stack (`:68-76`).
- **`usart1.c`**: BRR tính từ mantissa/fraction (`:11-18`); ring 512 byte, đếm byte bị bỏ khi đầy (`:48-55`).
- **`I2C.c`**: 100 kHz (`CR2=42`, `CCR=210`, `TRISE=43`, `:38-40`); chờ cờ có timeout theo số vòng (`:6-13`); không có hàm khôi phục bus.
- **`spi.c`**: mode 3, NSS phần mềm, /8 (`:37-43`); DMA2 stream 0 (RX) và 3 (TX), kênh 3 (`:76, 88`); bật RX trước TX (`:120-121`).
- **`timer.c`**: xem mục 3 và mục 6.
- **`icm42605.c`**: trình tự khởi tạo `:72-136`; giá trị trả về của `ICM_WriteReg` không được kiểm (chỉ kiểm WHO_AM_I, `:92-95`). Đổi đơn vị: accel ×9.81/2048 (`:140-143`), gyro ÷16.4 (`:145-148`). Calib gyro/accel theo **độ lệch chuẩn cả lô** (`calibCollect`, `:211-245`), ngưỡng 1.5 °/s và 0.10 m/s², tối đa 3 lần (`icm42605.h:151-153`), dùng `double` trong main loop (`:214-215`). Remap trục: `X=-Yc, Y=-Xc, Z=-Zc` (`:342-350`).
- **`spl06.c`**: PRS_CFG và TMP_CFG ghi `0x33` (8 Hz, oversample 8x, `:83, 85`), đo liên tục (`:86`). Hệ số tỉ lệ 7864320 (`:21`).
- **`hmc5883l.c`**: HMC: `CFG_A=0x70`, `CFG_B=0x20`, liên tục (`:41-43`); QMC: `CTRL1=0x15` (`:56`, `:23`). **Chưa remap trục**: `x=mx, y=my, z=mz` theo trục chip (`:126-128`). Offset hard-iron mặc định 0 (`hmc5883l.h:7-9`), calib chỉ nằm trong RAM (`:165-185`).
- **`hc05.c`**: parse lệnh bằng `strcmp` (`:34-61`); `cmd_buf` 16 byte (`:10`), lệnh dài hơn 15 ký tự bị bỏ cả dòng (`:68-72`).

---

## 8. Flash storage (`flash_storage.c`)

**Bố cục** (`flash_storage.h:6-12`): Flash sector 11 tại `0x080E0000`, 128 KB, chia thành **512 slot × 256 byte** (`FLASH_SLOT_COUNT`). Magic `0xBB420002` (`:6`).

**Struct `CalibData_t`** (`flash_storage.h:14-30`): `magic`, `sequence`, `gx_offset`, `gy_offset`, `gz_offset`, `ax_offset`, `ay_offset`, `az_offset`, `gyro_done`, `accel_done`, `crc`. Kích thước **44 byte** (tính bằng `sizeof` khi biên dịch trên PC). Hệ số gain accel **không được lưu**; khi nạp lại, `main.c:331-333` đặt bằng 1.

**Hàm `FlashStorage_Save`** (`flash_storage.c:199-228`):

1. `flash_scan` (`:158-180`) duyệt 512 slot, chọn slot hợp lệ (đúng magic và CRC32, `:182-187`) có `sequence` lớn nhất, và tìm slot **trống** (toàn `0xFF`) đầu tiên nằm sau nó.
2. `sequence` mới = sequence lớn nhất + 1 (hoặc 1 nếu chưa có) (`:206`). Tính CRC32 trên 40 byte đầu (`:40-59`).
3. **Chỉ xoá sector khi không còn slot trống phía sau** (`blank < 0`, `:212-216`). Khi đó xoá cả sector rồi ghi vào slot 0.
4. Ghi từng từ 32 bit, chờ xong và kiểm cờ lỗi sau mỗi từ (`flash_write_slot`, `:119-142`). Làm mới cache Flash (`:89-96`).
5. Đọc lại slot vừa ghi và kiểm CRC; sai thì trả `FLASH_ERR_WRITE` (`:224-226`).

**Lần lưu thứ ba, khi hai slot đầu đã hợp lệ:** code hiện tại **không còn kiểu hai slot**. Slot 0 và 1 đã có (sequence 1 và 2), `flash_scan` trả `best = 1` và `firstBlank = 2`. Lần lưu thứ ba ghi vào **slot 2 với sequence 3, không xoá sector**. Sector chỉ bị xoá ở lần lưu thứ 513 hoặc khi gọi `FlashStorage_Erase` (`:230-237`, lệnh `CALIBERASE`).

**Lịch sử:** bản cũ (commit `b85c60a`, `flash_storage.c:184-223` ở commit đó) dùng 2 slot. Khi cả hai hợp lệ nó ghi vào slot có sequence nhỏ hơn mà không xoá, chỉ xoá khi cả hai không hợp lệ. Flash không ghi đè được các bit 0 thành 1, nên từ lần lưu thứ ba ghi sẽ hỏng. Bản hiện tại đã sửa điểm này.

**Lưu ý vận hành:** xoá và ghi Flash làm CPU bị đứng trong lúc thao tác (chưa đo thời gian). Calib chỉ cho phép khi DISARMED (`main.c:414-420`).

---

## 9. Trạng thái từng phần

Quy ước: **A** = đã chạy và có log trong repo; **B** = đã chạy theo báo cáo/log của người dùng trong phiên làm việc, nhưng không có log lưu trong repo; **C** = đã viết, chưa thấy bằng chứng chạy trên phần cứng; **D** = chưa có.

| Thành phần | Mức | Bằng chứng / ghi chú |
|---|---|---|
| Parser UBX NAV-PVT, log CSV, phân tích | **A** | `docs/gps_test/static_test_2000samples.csv` (+2 hình); `tools/ubx_test`: 17 kiểm tra PASS, 0 FAIL (chạy lại khi viết tài liệu này) |
| Đo GPS đứng yên (độ ổn định vị trí, vận tốc) | **A** (chỉ phần GPS thô) | Kết quả trong `docs/gps_test`: 10.00 Hz, std ngang 0.181 m. Chưa có số đo của bộ ước lượng vị trí (GPS + IMU) trên board |
| Đọc IMU, xác định trục, remap | **B** | Số liệu thô accel/gyro do người dùng gửi; remap hiện tại ở `icm42605.c:342-350` |
| Calib gyro/accel (batch std) | **B** | Người dùng gửi log lần chạy bị loại (std 0.4–0.8 °/s với ngưỡng cũ 0.5), sau đó nới ngưỡng lên 1.5 (`icm42605.h:151`); kết quả calib thành công do người dùng xác nhận bằng lời |
| Lưu và nạp calib từ Flash | **B** | Người dùng xác nhận "đúng hết"; không có log chi tiết |
| Xuất PWM, 4 motor quay | **B** | Người dùng xác nhận 4 motor đã quay (sau khi đổi chân sang S3–S6) |
| Lệnh Bluetooth HC-05 | **B** | Nhiều log lệnh/phản hồi trong phiên |
| BENCH, motor test | **B** | Người dùng chạy và xác nhận; log đối chiếu từng tư thế A–F không còn |
| Vòng nhanh 2 kHz (thời gian xử lý, số lần lỡ) | **C** | Có `LOGPERF` nhưng chưa có số đo nào |
| Compass: đọc, calib, tác động lên yaw | **C** | Chưa có số liệu; trục chưa remap (`hmc5883l.c:126-128`) |
| Baro SPL06 và Kalman độ cao | **C** | Logic đã chạy thử trên PC trong phiên (không lưu); chưa có log baro/KF trên board |
| Máy trạng thái SBUS (ARM, CLIMB, hold, LAND, FORWARD/LEFT) | **C** | Logic đã mô phỏng trên PC (không lưu); phiên này người dùng không có tay cầm |
| SBUS giải mã | **C** | Commit lịch sử `dc195e5`, `b85c60a` cho thấy từng gỡ lỗi; không có log |
| Vòng vị trí XY, ước lượng vị trí trong bay | **C** | Chưa chạy trên phần cứng |
| LAND_BLIND, neo mặt đất | **C** | Chưa chạy trên phần cứng |
| Bay thật, PID tinh chỉnh | **D** | Chưa bay thử; hệ số PID là giá trị khởi điểm |
| Watchdog | **D** | Không có trong code |
| Gửi lệnh cấu hình cho GPS (UBX-CFG) | **D** | TX USART1 không được cấu hình |
| Oneshot/DShot cho ESC | **D** | Chỉ có PWM 400 Hz |
| Ngắt data-ready của IMU | **D** | Macro có, không dùng |
| Test tự động cho estimation/control | **D** | Chỉ có test cho parser UBX |
| Remap trục compass, bù độ lệch từ, soft-iron | **D** | Chưa có |
| Lưu offset compass vào Flash | **D** | Chỉ RAM |
| Vùng Flash dành riêng cho calib trong linker script | **D** | Xem R7 |

---

## 10. Chuỗi PWM và motor

**Luồng gọi:** `motor_mixer` và `pwm_out` **đều đã được gọi**, nhưng không trực tiếp từ `main.c`:

1. `main.c:280-281`: `Motor_Init(168000000U, 84000000U)` rồi `Motor_StopAll()`. Xung 1000 µs có ngay từ đầu, ESC thấy tín hiệu ở boot.
2. `main.c:140` (`imu_sample_isr`) gọi `FlightControl_FastUpdate`. Bên trong: `MotorMixer_SetArmed` (`flight_control.c:215`), `MotorMixer_Update` (`:218`), rồi `Motor_SetUs` cho 4 motor (`:226-229`). Các lệnh chỉ chạy sau khi `Timer7_Start()` (`main.c:355`).
3. Trước khi vòng chậm chạy lần đầu, `s_cmd` bằng 0 nên `motorsEnabled=false` và mixer trả 1000 µs (`flight_control.c:155`, `motor_mixer.c:37-40`).

**Cấu hình timer** (`pwm_out.c`):

| Timer | Clock vào | PSC | ARR | Tick | Chu kỳ | Dòng |
|---|---|---|---|---|---|---|
| TIM1 | 168 MHz | **167** | **2499** | 1 µs | 2500 µs (400 Hz) | `pwm_out.c:47-48` |
| TIM2 | 84 MHz | **83** | **2499** | 1 µs | 2500 µs (400 Hz) | `pwm_out.c:56-57` |
| TIM8 | không dùng | n/a | n/a | n/a | n/a | không có trong `src/` (đã grep) |

PSC và ARR tính từ `MOTOR_PWM_HZ`=400 (`pwm_out.h:9`) và clock truyền vào (`pwm_out.c:44`). TIM1 và TIM2 đều ở chế độ PWM 1, có preload cho ARR và CCR (`pwm_out.c:49-50, 58-59, 65`). CCR bằng độ rộng xung tính theo µs.

**Ánh xạ motor** (`pwm_out.c:4-13, 39-42`): M1 = TIM1_CH3N (PB15), M2 = TIM1_CH1 (PA8), M3 = TIM2_CH4 (PB11), M4 = TIM2_CH3 (PB10). `Motor_SetUs` kẹp 1000–2000 µs (`pwm_out.c:70-77`).

**Ghi đè khi test:** `cmd.testActive` đặt xung 1200 µs cho motor được chọn và 1000 µs cho các motor còn lại (`flight_control.c:220-225`, `FC_MOTOR_TEST_US` ở `flight_control.h:49`).

**Khi HardFault:** `hardfault_handler` (`main.c:116-124`) chỉ nháy LED, **không đụng vào PWM**. Timer phần cứng tiếp tục xuất xung cuối cùng, nên motor giữ nguyên mức ga cuối (xem R2).

---

## 11. TODO, warning, bug và rủi ro

**TODO/FIXME/HACK/XXX:** không có (đã grep `src`, `include`, `startup.s`, `Makefile`, `tools`).

**Warning khi build:** build vào thư mục tạm bằng `-std=c11 -Wall -Wextra -O2` cho **0 warning**.

**Rủi ro**, độ ưu tiên (Cao/TB/Thấp) là đánh giá chủ quan:

| ID | Mức | Vị trí | Mô tả |
|---|---|---|---|
| R1 | Thấp | `register.h:220` | `#define USART_CR1_TXEIE (1U << 7) ontrol`: có chữ `ontrol` thừa trong macro. Hiện macro không được dùng nên không lỗi; nếu dùng sẽ lỗi biên dịch |
| R2 | **Cao** | `main.c:116-124`, `flight_control.c:341-355` | Không có watchdog. Nếu main loop treo hoặc vào HardFault khi đang bay, ngắt vòng nhanh (hoặc timer PWM) vẫn xuất lệnh cũ: nếu main loop treo thì `s_cmd` giữ ga và setpoint cũ; nếu HardFault thì PWM giữ xung cuối. Không có đường tự tắt motor |
| R3 | TB | `usart1.c:40`, `sbus.c:68`, `main.c:80` | USART1, USART2 và SysTick không đặt ưu tiên (mặc định 0, cao nhất) nên chen được vào ngắt DMA 2 kHz. Mỗi ngắt ngắn nhưng chưa đo jitter |
| R4 | TB | `flight_control.c:272, 299, 305, 319, 327` | Vòng chậm dùng dT cố định (0.04, 0.1 s) trong khi nhịp thật bị trễ do I2C (khoảng 0.4–1.5 ms mỗi lần đọc, tính) và `UART5_WriteF` (khoảng 87 µs mỗi ký tự, một dòng 100 ký tự khoảng 9 ms, tính). Tích phân/đạo hàm PID và các bộ đếm giữ cử chỉ có thể lệch so với thời gian thực |
| R5 | TB | `uart5.c:57-61` | `UART5_WriteChar` chờ TXE không có timeout. Không cho phép gọi trong ngắt |
| R6 | TB | `I2C.c:91-111` | Đọc nhiều byte: xoá ACK và đặt STOP chỉ ở lần đọc byte cuối, cửa sổ thời gian sát nhau. Nếu ngắt DMA chen vào đúng khoảng đó có thể sai (chưa xác nhận, chưa thấy trên phần cứng). Không có phục hồi bus khi SDA bị kẹt |
| R7 | TB | `stm32_flash.ld:3`, `flash_storage.h:8` | Linker script khai báo toàn bộ 1 MB Flash cho chương trình; sector 11 (calib) không được loại trừ. Hiện firmware khoảng 72 KB nên chưa chạm tới. Xoá/ghi Flash làm CPU đứng (một bank), kể cả ngắt |
| R8 | Thấp | `main.c:33, 138`, `icm42605.c:269-273, 308-314` | `s_cal` (không `volatile`) được ngắt đọc và main ghi khi calib hoặc nạp, không có khoá. Có thể thấy tạm thời bộ offset nửa cũ nửa mới |
| R9 | TB | `altitude.c:79, 148-149` | Không có kiểm tra NaN/inf ở bất kỳ đâu (đã grep). `powf` với áp suất không dương cho NaN; `fabsf(NaN) > limit` là false nên mẫu NaN lọt qua cổng, làm hỏng trạng thái Kalman mà `AltitudeKF_IsHealthy` vẫn báo khoẻ. `Quaternion_Normalize` không tự phục hồi từ NaN |
| R10 | Thấp | `altitude.c:111, 118-122, 137-138` | Q không nhân dT, F không chứa damping 0.998. Hợp lý chỉ khi bước predict gần 10 ms (đúng với `FC_ACCEL_BLOCK`=20) |
| R11 | TB | `sbus.c:76-129` | ISR không kiểm lỗi parity/framing, chỉ đồng bộ bằng byte đầu `0x0F` và byte cuối `0x00`; mảng kênh được cập nhật không nguyên tử nên khung có thể bị đọc nửa mới nửa cũ |
| R12 | TB | `imu.c:144-156, 177`, `flight_control.c:199` | Mỗi mẫu 2 kHz tính Euler (`atan2f` ×2, `asinf`) chỉ để log, và `Quaternion_FromEuler` (6 hàm lượng giác) dù setpoint ít đổi. Thời gian chạy thực của vòng nhanh: (chưa đo) |
| R13 | TB | `hmc5883l.c:126-128`, `imu.c:98-100`, `main.c:27` | Compass vào AHRS (khi `COMPASS_ENABLE`=1 và khoẻ) bằng trục chip chưa remap. Tích phân mag vào `gyroDrift.z` không bị kẹp. Offset hard-iron mặc định 0 và mất khi khởi động lại |
| R14 | Thấp | `main.c:126-132`, `icm42605.h:98` | IMU được đọc theo timer 2 kHz, không đồng bộ với data-ready 4 kHz của cảm biến, nên thời điểm lấy mẫu có jitter tối đa khoảng một chu kỳ cảm biến (tính) |
| R15 | Thấp | `syscalls.c:7-13`, `stm32_flash.ld:7, 58` | `_sbrk` không kiểm tra va chạm với stack; stack chỉ được ASSERT dự trữ 4 KB. Heap chỉ dùng bởi `printf` dạng float của newlib |
| R16 | Thấp | `hc05.c:8, 37-50, 57-61, 68-72` | Hộp lệnh `calib_cmd` chỉ một ô, lệnh đến trước khi main đọc sẽ bị ghi đè. `ARM`, `HOLD`, `CLIMB`, `LAND` được parse nhưng không dùng (chỉ `DISARM` được dùng, `main.c:409`). Lệnh dài hơn 15 ký tự bị bỏ cả dòng. Khai báo `HC05_GetFlightCmd` lặp hai lần (`hc05.h:53, 60`) |
| R17 | TB | `flight_state.c:268-272`, `altitude_control.h:17` | LAND chỉ về IDLE khi độ cao ước lượng ≤0.05 m, trong khi ga tối thiểu khi bay là 0.15. Nếu baro lệch, động cơ có thể quay ở ga tối thiểu cho đến khi người lái DISARM bằng cử chỉ |
| R18 | Thấp | `icm42605.c:89-123` | Giá trị trả về của `ICM_WriteReg` không được kiểm, chỉ kiểm WHO_AM_I. Cấu hình ghi lỗi âm thầm vẫn trả `ICM_OK` |
| R19 | Thấp | `usart1.c:48-55`, `main.c:302-304` | GPS được bật trước 23 s delay và không ai đọc buffer 512 byte, nên byte bị bỏ khi buffer đầy. Chỉ ảnh hưởng dữ liệu lúc boot (đếm bằng `UART1_GetDroppedCount`) |
| R20 | Thấp | `altitude.h:7`, `icm42605.c:140, 310`, `imu.h:26` | Ba hằng trọng lực khác nhau: 9.80665, 9.81, 981 cm/s². Sai khác khoảng 0.003 m/s², bị bias và neo mặt đất hấp thụ |
| R21 | Thấp | `main.c:354-355`, `imu.c:161` | Không có điều kiện "AHRS đã hội tụ" trước khi cho ARM/bay. Mahony bắt đầu từ quaternion đơn vị khi Timer7 khởi động |

Về **biến dùng chung với ISR**:

- `s_cmd`: main ghi trong `CRITICAL_BEGIN/END` (`flight_control.c:353-355`), ngắt chép nguyên khối (`:178`). Không có vấn đề.
- `s_acc`: ngắt ghi, main lấy trong vùng găng (`:243-246`). Không có vấn đề.
- `s_fastYaw`, `s_fastCosTilt`: `volatile float`, 32 bit nên ghi/đọc nguyên tử (`:54-55`).
- `s_status.motorUs`: ngắt ghi, main chỉ đọc để log (`flight_control.c:228`). Có thể lẫn giữa hai chu kỳ, chỉ ảnh hưởng log.
- `Attitude_Get()` được main đọc để log trong khi ngắt đang cập nhật. Chỉ ảnh hưởng log.
- `CRITICAL_END` bật ngắt vô điều kiện (`system.h:27`): không được lồng nhau. Hiện không có chỗ nào lồng.
- Không chia cho 0: `Baro_MovingAverage` có `n≥1` (`altitude.c:221`); `velMag` chỉ chia khi lớn hơn `POS_MAX_VEL` (`position_control.c:49-52`); `AltitudeControl_Update` chặn dT≤0 (`altitude_control.c:40`).

---

## 12. Build và nạp

- **Toolchain:** `arm-none-eabi-gcc` (`Makefile:1-6`).
- **Cờ biên dịch** (`Makefile:57-69`): `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -std=c11 -Wall -Wextra -O2 -ffunction-sections -fdata-sections -ffreestanding -MMD -MP -g`.
- **Cờ link** (`Makefile:73-78, 104`): `-T stm32_flash.ld -Wl,--gc-sections -Wl,-Map=... -Wl,--print-memory-usage -nostartfiles`, thư viện `-lm -lgcc -lc` (newlib đầy đủ, không dùng nano.specs).
- **Linker script** (`stm32_flash.ld`): FLASH `0x08000000`, 1024 KB (`:3`); RAM `0x20000000`, 128 KB (`:4`); `_stack_top = 0x20020000` (`:9`, xác nhận trong file map); `_stack_size = 0x1000` chỉ dùng cho ASSERT (`:7, 58-59`); `ENTRY(reset_handler)` (`:11`).
- **Startup** (`startup.s`): bảng vector `:14-113`; `reset_handler` `:121-152` (bật FPU, copy `.data`, xoá `.bss`, gọi `main`). Không đặt `VTOR` ở bất kỳ đâu (đã grep).
- **Kích thước** (build thử vào thư mục tạm): `.text` 69876 B, `.data` 2504 B, `.bss` 2168 B. Flash 72436 B (6.91%), RAM 4672 B (3.56%).
- **Lệnh** (`Makefile:80-141`): `make` tạo `.elf/.hex/.bin`; `make flash` (OpenOCD, ST-Link); `make dfu` (`dfu-util -a 0 -s 0x08000000:leave`); `make debug`; `make dump`; `make map`; `make clean`.
- **Nạp qua DFU:** phải rút GPS trước vì bootloader ROM STM32 cũng nghe USART1.
- **Test parser UBX:** `make -C tools/ubx_test test`.

---

## 13. Điều chưa chắc

Mục này liệt kê những điều tôi không xác nhận được từ mã nguồn, và cần xem thêm tài liệu hoặc số đo nào.

1. **Ý nghĩa thật của `GYRO_ACCEL_CONFIG0 = 0xFF` và băng thông AAF (21/440/6).** Mã chỉ ghi giá trị. Cần datasheet ICM-42605: các thanh ghi `GYRO_ACCEL_CONFIG0` (0x52), `GYRO_CONFIG_STATIC3..5` (bank 1), `ACCEL_CONFIG_STATIC2..4` (bank 2).
2. **Thời gian xử lý thực của vòng nhanh và jitter chu kỳ.** Cần log `LOGPERF` từ board (`exec`, `period`, `miss`) khi chạy đủ tải (kể cả lúc có log Bluetooth).
3. **Có cần đặt `VTOR` khi nạp qua DFU hay không.** Không có mã đặt `VTOR`. Firmware nạp bằng `dfu-util ... :leave` vẫn chạy theo người dùng, nhưng tôi không biết do bootloader tự đặt hay do reset. Cần RM0090 (mục SCB/VTOR) và hành vi bootloader ROM.
4. **Tần số khung SBUS của bộ thu và mức độ tin cậy giải mã.** Mã không nói; cần log từ board khi có tay cầm.
5. **Cấu hình tần số 10 Hz của GPS.** Firmware không cấu hình; con số 10 Hz đến từ số đo trong `docs/gps_test`, không biết module lưu cấu hình đó ở đâu.
6. **Compass thực tế nhận chip nào và trục.** Cần dòng boot `BARO:.. MAG:.. (loại)` và bộ số đo 5 tư thế (xem hướng dẫn đã trao đổi) để remap.
7. **Cơ chế xoá cờ PRS_RDY/TMP_RDY của SPL06** khi đọc thanh ghi dữ liệu (`spl06.c:95-101`). Cần datasheet SPL06.
8. **Va chạm khi đọc I2C nhiều byte bị ngắt chen vào** (R6). Cần RM0090 (mục I2C master receiver) và thử trên phần cứng.
9. **Thời gian chặn CPU của xoá/ghi Flash** (sector 128 KB) và hành vi khi mất điện giữa lúc ghi. Chưa thử.
10. **Chi tiết bench test A–F** (nghiêng phải, chúc mũi...): người dùng xác nhận bằng lời nhưng không còn log từng tư thế.
11. **Nguồn gốc chữ `ontrol` ở `register.h:220`**: không biết có sẵn trong bản gốc hay do chỉnh sửa; file cần xem lại lịch sử git.
12. **Giá trị ga hover thực của airframe.** `ALT_HOVER_THROTTLE`=0.50 là ước lượng, chưa đo.
13. **Hằng số `SPL06_SCALE_FACTOR` và cách bù nhiệt** lấy từ datasheet cho oversample 8x; tôi chưa đối chiếu lại từng hệ số.
14. **Các offset NAV-PVT** (`gps.c:51-85`) khớp bảng u-blox M8 theo hiểu biết của tôi, chưa đối chiếu với tài liệu giao thức. Chip trên module có thật là u-blox chính hãng hay không: chưa rõ, dù dữ liệu thật đã giải mã đúng.

File/số liệu cần xem thêm để chốt: datasheet ICM-42605, SPL06, HMC5883L/QMC5883L; RM0090 (NVIC, SCB, I2C); tài liệu giao thức u-blox; log `LOGPERF`, dòng boot, và log SBUS từ board.
