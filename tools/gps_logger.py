#!/usr/bin/env python3
"""Ghi log GPS (UBX NAV-PVT) từ drone qua HC-05 Bluetooth ra file .csv.

Firmware khi nhận START + GPSCSV in:
    G,<tick_ms>,<iTOW>,<fixType>,<flags>,<numSV>,<latE7>,<lonE7>,<hMSL_mm>,<hAcc_mm>,
      <vAcc_mm>,<velN_mmps>,<velE_mmps>,<velD_mmps>,<gSpeed_mmps>,<sAcc_mmps>,<pDOP>
                                    (mỗi gói NAV-PVT, kể cả chưa fix — toàn số nguyên)
    S,<bytes>,<packets>,<pvt>,<badChecksum>,<resync>,<fixType>,<numSV>   (mỗi giây)

Mẫu "hợp lệ" (fixType >= 3, gnssFixOK, numSV >= 6, hAcc <= 3000 mm — chỉnh bằng tham số)
được ghi vào file chính và tính vào N; mẫu khác ghi sang *_rejected.csv.
Chỉ dùng thư viện chuẩn (không cần pyserial).

Ví dụ:
    python3 tools/gps_logger.py                       # /dev/rfcomm0, 2000 mẫu hợp lệ
    python3 tools/gps_logger.py -n 500 --max-hacc 2000
"""

import argparse
import csv
import errno
import os
import select
import sys
import termios
import time
import tty

COLUMNS = [
    "t_ms", "iTOW", "fixType", "flags", "numSV", "lat_e7", "lon_e7", "hMSL_mm", "hAcc_mm",
    "vAcc_mm", "velN_mmps", "velE_mmps", "velD_mmps", "gSpeed_mmps", "sAcc_mmps", "pDOP",
]
COL = {name: i for i, name in enumerate(COLUMNS)}
G_FIELDS = 1 + len(COLUMNS)          # tiền tố "G" + 16 cột = 17 phần tử
S_FIELDS = 8                         # "S" + 7 cột
PROGRESS_IDLE_S = 2.0                # in tiến trình khi chưa có mẫu hợp lệ
NO_G_HINT_S = 30.0                   # quá lâu không có dòng G → gợi ý theo dòng S


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200   # rfcomm bỏ qua baud, cổng USB-UART thì cần
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def send(fd, cmd):
    os.write(fd, (cmd + "\n").encode("ascii"))
    time.sleep(0.15)


def main():
    ap = argparse.ArgumentParser(description="Ghi log GPS từ drone ra file CSV")
    ap.add_argument("-p", "--port", default="/dev/rfcomm0", help="cổng serial (mặc định /dev/rfcomm0)")
    ap.add_argument("-n", "--samples", type=int, default=2000, help="số mẫu HỢP LỆ cần ghi (mặc định 2000)")
    ap.add_argument("-o", "--output", default=None, help="tên file .csv (mặc định gps_log_<thời gian>.csv)")
    ap.add_argument("--min-fix", type=int, default=3, help="fixType tối thiểu (mặc định 3)")
    ap.add_argument("--min-sv", type=int, default=6, help="numSV tối thiểu (mặc định 6)")
    ap.add_argument("--max-hacc", type=int, default=3000, help="hAcc tối đa, mm (mặc định 3000)")
    args = ap.parse_args()

    log_dir = "/home/lnh/Documents/DO_AN_DRONE/BLUEBERRY/BlueberryProject/tools/LOG"
    os.makedirs(log_dir, exist_ok=True)
    output_name = os.path.basename(args.output) if args.output else time.strftime("gps_log_%Y%m%d_%H%M%S.csv")

    out_path = os.path.join(log_dir, output_name)
    rej_path = out_path[:-4] + "_rejected.csv" if out_path.endswith(".csv") else out_path + "_rejected.csv"

    def is_valid(v):
        return (v[COL["fixType"]] >= args.min_fix and (v[COL["flags"]] & 1) == 1
                and v[COL["numSV"]] >= args.min_sv and v[COL["hAcc_mm"]] <= args.max_hacc)

    try:
        fd = open_port(args.port)
    except OSError as e:
        if e.errno == errno.EBUSY:
            sys.exit(f"Cổng {args.port} đang bận (Device or resource busy).\n"
                     f"Có chương trình khác đang giữ cổng (picocom, app khác...). Kiểm tra bằng:\n"
                     f"    fuser -v {args.port}      hoặc    lsof {args.port}\n"
                     f"rồi thoát chương trình đó (picocom: Ctrl+A, Ctrl+X).")
        sys.exit(f"Không mở được {args.port}: {e}\n"
                 f"Kiểm tra đã ghép đôi HC-05 và chạy: sudo rfcomm bind 0 <MAC HC-05>")

    total_g = 0          # tổng dòng G hợp lệ về định dạng
    valid = 0            # mẫu đạt điều kiện → file chính
    rejected = 0         # mẫu không đạt → file phụ
    bad_lines = 0        # dòng G/S sai số cột hoặc không parse được
    last_g = None        # dòng G gần nhất (list int)
    last_s = None        # dòng S gần nhất (list int)
    prev_s = None        # dòng S trước đó (so sánh xem bộ đếm có tăng không)
    buf = b""
    t_start = time.time()
    last_g_time = t_start
    last_progress = t_start
    last_hint = t_start

    print(f"Ghi {args.samples} mẫu hợp lệ (fixType>={args.min_fix}, gnssFixOK, numSV>={args.min_sv}, "
          f"hAcc<={args.max_hacc}mm) vào {out_path}")
    print(f"Mẫu không đạt ghi vào {rej_path}. Nhấn Ctrl+C để dừng sớm (vẫn giữ các mẫu đã ghi).")

    send(fd, "START")
    send(fd, "GPSCSV")

    def status_line():
        if last_g is None:
            return f"dòng G: 0, hợp lệ: {valid}, dòng lỗi: {bad_lines}"
        return (f"dòng G: {total_g}, hợp lệ: {valid}/{args.samples}, fixType={last_g[COL['fixType']]} "
                f"numSV={last_g[COL['numSV']]} hAcc={last_g[COL['hAcc_mm']] / 1000:.2f}m, dòng lỗi: {bad_lines}")

    def hint():
        print("! 30 s không có dòng G nào. Chẩn đoán theo dòng S:")
        if last_s is None:
            print("  - Không có cả dòng S: board chưa nhận lệnh. Board khởi động xong chưa (~25 s)?"
                  " Bluetooth đã kết nối (đèn HC-05 nháy chậm)? Đang gửi lại START + GPSCSV...")
            send(fd, "START")
            send(fd, "GPSCSV")
            return
        b_, pkt, pvt = last_s[1], last_s[2], last_s[3]
        grow = prev_s is not None and last_s[1] > prev_s[1]
        if b_ == 0 or not grow:
            print("  - bytes không tăng: GPS không gửi gì vào PA10 → kiểm tra nguồn GPS, GND, TX GPS → PA10.")
        elif pkt == 0:
            print("  - Có byte nhưng không có gói UBX đúng checksum: module không xuất UBX ở 115200"
                  " (đang xuất NMEA hoặc baud khác).")
        elif pvt == 0:
            print("  - Có gói UBX nhưng không có NAV-PVT: module cần được cấu hình xuất NAV-PVT"
                  " (cần xác định chip M8 hay M10 trước khi gửi lệnh cấu hình).")
        else:
            print("  - Có NAV-PVT ở board nhưng không tới đây: dòng G bị mất trên Bluetooth?")

    with open(out_path, "w", newline="") as f, open(rej_path, "w", newline="") as fr:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        rej_writer = csv.writer(fr)
        rej_writer.writerow(COLUMNS)
        try:
            while valid < args.samples:
                now = time.time()
                if valid == 0 and now - last_progress >= PROGRESS_IDLE_S:
                    last_progress = now
                    print("  [chờ] " + status_line())
                if now - last_g_time >= NO_G_HINT_S and now - last_hint >= NO_G_HINT_S:
                    last_hint = now
                    hint()

                ready, _, _ = select.select([fd], [], [], 0.5)
                if not ready:
                    continue

                buf += os.read(fd, 1024)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("ascii", errors="replace").strip()
                    parts = line.split(",")

                    if parts[0] == "S":
                        try:
                            if len(parts) != S_FIELDS:
                                raise ValueError
                            prev_s, last_s = last_s, ["S"] + [int(x) for x in parts[1:]]
                        except ValueError:
                            bad_lines += 1
                        continue
                    if parts[0] != "G":
                        continue        # tin nhắn khác (OK_START, # GPSCSV ...)

                    try:
                        if len(parts) != G_FIELDS:
                            raise ValueError
                        values = [int(x) for x in parts[1:]]
                    except ValueError:
                        bad_lines += 1  # dòng bị cắt / nhiễu Bluetooth
                        continue

                    total_g += 1
                    last_g = values
                    last_g_time = time.time()

                    if is_valid(values):
                        writer.writerow(values)
                        valid += 1
                        if valid % 100 == 0 or valid == args.samples:
                            print("  " + status_line())
                            f.flush()
                        if valid >= args.samples:
                            break
                    else:
                        rej_writer.writerow(values)
                        rejected += 1
        except KeyboardInterrupt:
            print("\nDừng theo yêu cầu.")
        finally:
            try:
                send(fd, "LOGNORM")
                send(fd, "STOP")
            except OSError:
                pass
            os.close(fd)

    elapsed = time.time() - t_start
    print(f"\nĐã lưu {valid} mẫu hợp lệ vào {out_path} trong {elapsed:.0f} s")
    print(f"  Tổng dòng G: {total_g}, mẫu không đạt: {rejected} (→ {rej_path}), dòng lỗi: {bad_lines}")
    if last_s is not None:
        print(f"  Thống kê board: bytes={last_s[1]} packets={last_s[2]} pvt={last_s[3]} "
              f"badChecksum={last_s[4]} resync={last_s[5]}")


if __name__ == "__main__":
    main()
