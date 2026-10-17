#!/usr/bin/env python3
"""Ghi log la bàn (LOGMAGCSV) từ drone qua HC-05 Bluetooth ra file .csv, theo 5 tư thế.

Firmware khi nhận LOGMAGCSV + START in (mỗi mẫu la bàn mới, ~25 Hz):
    M,<tick_ms>,<rawX>,<rawY>,<rawZ>,<heading_deg>,<roll_deg>,<pitch_deg>,<yaw_deg>
rawX/Y/Z là giá trị thô, theo trục chip, chưa trừ offset.

Script hướng dẫn từng tư thế: đặt drone đúng tư thế, nhấn Enter, giữ YÊN trong --seconds giây.
Cả 5 tư thế được ghi vào một file CSV (cột pose), cuối cùng in bảng trung bình/độ lệch từng tư thế.

    P1  nằm phẳng, mũi hướng 0 (hướng bất kỳ, nhớ lại hướng này)
    P2  nằm phẳng, xoay 90° sang phải so với P1
    P3  nằm phẳng, xoay 180° so với P1
    P4  nằm phẳng, xoay 270° so với P1
    P5  chúc mũi xuống 90° (đuôi lên trời), từ hướng P1

Chỉ dùng thư viện chuẩn (không cần pyserial). File lưu trong tools/LOG (đã gitignore).

Ví dụ:
    python3 tools/mag_logger.py                  # /dev/rfcomm0, 4 s mỗi tư thế
    python3 tools/mag_logger.py -s 6 -o mag_hmc.csv
    python3 tools/mag_logger.py --poses P1,P5    # chỉ đo một số tư thế
"""

import argparse
import csv
import errno
import math
import os
import select
import sys
import termios
import time
import tty

COLUMNS = ["pose", "t_ms", "rawX", "rawY", "rawZ", "heading_deg", "roll_deg", "pitch_deg", "yaw_deg"]
M_FIELDS = 9
SETTLE_S = 0.7
NO_DATA_HINT_S = 10.0

POSES = {
    "P1": "Nằm PHẲNG, mũi hướng 0 (hướng bất kỳ, nhớ hướng này)",
    "P2": "Nằm PHẲNG, xoay 90° sang PHẢI so với P1",
    "P3": "Nằm PHẲNG, xoay 180° so với P1",
    "P4": "Nằm PHẲNG, xoay 270° so với P1",
    "P5": "CHÚC MŨI XUỐNG 90° (đuôi lên trời), từ hướng P1",
}


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def send(fd, cmd):
    os.write(fd, (cmd + "\n").encode("ascii"))
    time.sleep(0.15)


def parse_m(line):
    parts = line.split(",")
    if parts[0] != "M" or len(parts) != M_FIELDS:
        return None
    try:
        return [int(parts[1])] + [float(x) for x in parts[2:]]
    except ValueError:
        return None


def read_lines(fd, state, timeout):
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return []
    state["buf"] += os.read(fd, 1024)
    lines = []
    while b"\n" in state["buf"]:
        raw, state["buf"] = state["buf"].split(b"\n", 1)
        lines.append(raw.decode("ascii", errors="replace").strip())
    return lines


def stats(rows, idx):
    vals = [r[idx] for r in rows]
    mean = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))
    return mean, sd


def main():
    ap = argparse.ArgumentParser(description="Ghi log la bàn theo 5 tư thế ra file CSV")
    ap.add_argument("-p", "--port", default="/dev/rfcomm0", help="cổng serial (mặc định /dev/rfcomm0)")
    ap.add_argument("-s", "--seconds", type=float, default=4.0, help="thời gian ghi mỗi tư thế, giây (mặc định 4)")
    ap.add_argument("-o", "--output", default=None, help="tên file .csv (mặc định mag_log_<thời gian>.csv)")
    ap.add_argument("--poses", default="P1,P2,P3,P4,P5", help="danh sách tư thế, cách nhau dấu phẩy")
    args = ap.parse_args()

    pose_list = [p.strip().upper() for p in args.poses.split(",") if p.strip()]
    for p in pose_list:
        if p not in POSES:
            sys.exit(f"Tư thế không hợp lệ: {p} (chọn trong {', '.join(POSES)})")

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "LOG")
    os.makedirs(log_dir, exist_ok=True)
    output_name = os.path.basename(args.output) if args.output else time.strftime("mag_log_%Y%m%d_%H%M%S.csv")
    out_path = os.path.join(log_dir, output_name)

    try:
        fd = open_port(args.port)
    except OSError as e:
        if e.errno == errno.EBUSY:
            sys.exit(f"Cổng {args.port} đang bận (Device or resource busy).\n"
                     f"Có chương trình khác đang giữ cổng (picocom, app khác...). Kiểm tra bằng:\n"
                     f"    fuser -v {args.port}      hoặc    lsof {args.port}\n"
                     f"rồi thoát chương trình đó (picocom: Ctrl+A, Ctrl+X).")
        sys.exit(f"Không mở được {args.port}: {e}\n"
                 f"Kiểm tra đã ghép đôi HC-05 và chạy: sudo rfcomm bind /dev/rfcomm0 <MAC HC-05> 1")

    state = {"buf": b""}
    results = {}
    bad_lines = 0

    # Link rfcomm vừa mở hay nuốt ký tự đầu: gửi dòng trống (firmware bỏ qua) để làm nóng link.
    os.write(fd, b"\n")
    time.sleep(1.5)
    send(fd, "LOGMAGCSV")
    send(fd, "START")

    # Chờ mẫu M đầu tiên để biết la bàn đang chạy.
    t0 = time.time()
    last_hint = t0
    got = False
    print("Đang chờ dữ liệu la bàn từ board...")
    while not got:
        for line in read_lines(fd, state, 0.5):
            if parse_m(line) is not None:
                got = True
                break
        now = time.time()
        if not got and now - last_hint >= NO_DATA_HINT_S:
            last_hint = now
            print("! Chưa có dòng M nào. Board khởi động xong chưa (~25 s)? Bluetooth đã nối (HC-05 nháy chậm)?"
                  " La bàn có báo MAG:OK không? Đang gửi lại LOGMAGCSV + START...")
            send(fd, "LOGMAGCSV")
            send(fd, "START")

    try:
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(COLUMNS)
            for pose in pose_list:
                print(f"\n=== {pose}: {POSES[pose]}")
                try:
                    input("Đặt drone đúng tư thế, giữ YÊN rồi nhấn Enter... ")
                except EOFError:
                    break
                termios.tcflush(fd, termios.TCIFLUSH)
                state["buf"] = b""

                rows = []
                start = time.time()
                while time.time() - start < args.seconds + SETTLE_S:
                    for line in read_lines(fd, state, 0.2):
                        v = parse_m(line)
                        if v is None:
                            if line.startswith("M,"):
                                bad_lines += 1
                            continue
                        if time.time() - start >= SETTLE_S:
                            rows.append(v)
                            writer.writerow([pose] + v)
                f.flush()

                if len(rows) < 10:
                    print(f"  ! chỉ nhận {len(rows)} mẫu, có thể mất kết nối. Đo lại tư thế này bằng --poses {pose}")
                    continue
                results[pose] = rows
                mx, sx = stats(rows, 1)
                my, sy = stats(rows, 2)
                mz, sz = stats(rows, 3)
                print(f"  {len(rows)} mẫu | X={mx:+.4f}±{sx:.4f}  Y={my:+.4f}±{sy:.4f}  Z={mz:+.4f}±{sz:.4f}")
    except KeyboardInterrupt:
        print("\nDừng theo yêu cầu.")
    finally:
        try:
            send(fd, "LOGNORM")
            send(fd, "STOP")
        except OSError:
            pass
        os.close(fd)

    print(f"\nĐã lưu vào {out_path} (dòng M lỗi: {bad_lines})")
    if results:
        print("\nTrung bình theo tư thế (raw, trục chip, chưa trừ offset):")
        print("  pose |      X       Y       Z |  |B|   | heading  roll  pitch")
        for pose in pose_list:
            if pose not in results:
                continue
            rows = results[pose]
            mx, _ = stats(rows, 1)
            my, _ = stats(rows, 2)
            mz, _ = stats(rows, 3)
            hd, _ = stats(rows, 4)
            ro, _ = stats(rows, 5)
            pi, _ = stats(rows, 6)
            print(f"  {pose:4s} | {mx:+7.3f} {my:+7.3f} {mz:+7.3f} | {math.sqrt(mx*mx+my*my+mz*mz):5.3f} |"
                  f" {hd:6.1f} {ro:6.1f} {pi:6.1f}")
        print("Gửi bảng này cho Claude để xác định trục la bàn so với trục drone.")


if __name__ == "__main__":
    main()
