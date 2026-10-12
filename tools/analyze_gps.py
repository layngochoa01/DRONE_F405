#!/usr/bin/env python3
"""Phân tích log GPS đứng yên (CSV từ tools/gps_logger.py, cột UBX NAV-PVT).

    python3 tools/analyze_gps.py gps_log_20261009_153012.csv
    python3 tools/analyze_gps.py log.csv --max-gap-pct 1.0 --show

In thống kê độ ổn định vị trí/vận tốc, so sánh hAcc chip tự báo với sai số đo thực tế,
vẽ 2 hình PNG cạnh file CSV, và in kết luận ĐẠT / KHÔNG ĐẠT.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

M_PER_E7_DEG = 0.01113195          # 1e-7 độ vĩ ≈ 0.01113195 m
ITOW_WEEK_MS = 604800000           # iTOW quay vòng mỗi tuần GPS
GAP_MS = 150                       # dt iTOW lớn hơn → coi là hụt mẫu (ở 10 Hz)
JUMP_WARN_M = 2.0
JUMP_FAIL_M = 5.0


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        sys.exit(f"{path}: không có dữ liệu")
    cols = rows[0].keys()
    need = ["iTOW", "lat_e7", "lon_e7", "hAcc_mm", "velN_mmps", "velE_mmps", "sAcc_mmps"]
    miss = [c for c in need if c not in cols]
    if miss:
        sys.exit(f"{path}: thiếu cột {miss} — file có phải từ gps_logger.py bản UBX không?")
    return {c: np.array([int(r[c]) for r in rows], dtype=np.int64) for c in cols}


def int_mean(a):
    """Trung bình làm tròn, tính hoàn toàn bằng số nguyên Python (không mất chính xác)."""
    total = int(a.sum())
    n = len(a)
    return (total + n // 2) // n


def main():
    LOG_DIR = "/home/lnh/Documents/DO_AN_DRONE/BLUEBERRY/BlueberryProject/tools/LOG"
    ap = argparse.ArgumentParser(description="Phân tích log GPS đứng yên")
    ap.add_argument("csv", nargs="?", default=None, help="Tên hoặc đường dẫn file CSV (mặc định lấy file mới nhất trong LOG)")
    ap.add_argument("--max-std-m", type=float, default=1.0, help="ngưỡng std ngang (m), mặc định 1.0")
    ap.add_argument("--min-hz", type=float, default=9.0, help="ngưỡng tần số (Hz), mặc định 9")
    ap.add_argument("--max-gap-pct", type=float, default=1.0,
                    help="tỉ lệ khoảng hụt mẫu tối đa (%%), mặc định 1.0")
    ap.add_argument("--show", action="store_true", help="mở cửa sổ đồ thị (mặc định chỉ lưu PNG)")
    args = ap.parse_args()

    if args.csv is None:
        import glob

        csv_files = glob.glob(os.path.join(LOG_DIR, "gps_log_*.csv"))
        csv_files = [
            f for f in csv_files
            if not f.endswith("_rejected.csv")
        ]

        if not csv_files:
            sys.exit(f"Không tìm thấy file GPS CSV trong {LOG_DIR}")

        args.csv = max(csv_files, key=os.path.getmtime)

    elif not os.path.isabs(args.csv):
        # Nếu chỉ nhập tên file, tìm trong thư mục LOG
        candidate = os.path.join(LOG_DIR, args.csv)
        if os.path.isfile(candidate):
            args.csv = candidate

    print(f"Đang phân tích file: {args.csv}")

    d = load(args.csv)
    n = len(d["iTOW"])
    if n < 3:
        sys.exit("Cần ít nhất 3 mẫu")

    # ---- Thời gian theo iTOW (không dùng tick của board) ----
    dt_ms = np.diff(d["iTOW"])
    dt_ms = np.where(dt_ms < 0, dt_ms + ITOW_WEEK_MS, dt_ms)        # qua tuần GPS mới
    t_s = np.concatenate([[0], np.cumsum(dt_ms)]) / 1000.0
    span_s = t_s[-1]
    freq = (n - 1) / span_s if span_s > 0 else float("nan")
    gaps = int(np.sum(dt_ms > GAP_MS))
    gap_pct = 100.0 * gaps / len(dt_ms)

    # ---- Vị trí: hiệu số NGUYÊN so với vị trí trung bình rồi mới đổi sang mét ----
    lat0 = int_mean(d["lat_e7"])
    lon0 = int_mean(d["lon_e7"])
    coslat = math.cos(math.radians(lat0 * 1e-7))
    north = (d["lat_e7"] - lat0).astype(np.float64) * M_PER_E7_DEG
    east = (d["lon_e7"] - lon0).astype(np.float64) * M_PER_E7_DEG * coslat

    std_n, std_e = north.std(ddof=1), east.std(ddof=1)
    std_h = math.hypot(std_n, std_e)
    r = np.hypot(north - north.mean(), east - east.mean())
    cep50 = float(np.median(r))
    r95 = float(np.percentile(r, 95))
    step = np.hypot(np.diff(north), np.diff(east))
    jumps_warn = int(np.sum(step > JUMP_WARN_M))
    jumps_fail = int(np.sum(step > JUMP_FAIL_M))

    # ---- Trôi chậm: polyfit bậc 1 theo phút, std sau khi bỏ trôi ----
    t_min = t_s / 60.0
    pn = np.polyfit(t_min, north, 1)
    pe = np.polyfit(t_min, east, 1)
    drift = math.hypot(pn[0], pe[0])
    north_dt = north - np.polyval(pn, t_min)
    east_dt = east - np.polyval(pe, t_min)
    std_h_detr = math.hypot(north_dt.std(ddof=1), east_dt.std(ddof=1))

    # ---- Chỉ số riêng của NAV-PVT ----
    hacc = d["hAcc_mm"] / 1000.0
    vn = d["velN_mmps"] / 1000.0
    ve = d["velE_mmps"] / 1000.0
    sacc = d["sAcc_mmps"] / 1000.0

    print(f"File: {args.csv}")
    print(f"Số mẫu: {n}, thời lượng {span_s:.1f} s (theo iTOW)")
    print(f"Tần số thực: {freq:.2f} Hz (dt trung vị {np.median(dt_ms):.0f} ms)")
    print(f"Hụt mẫu (dt > {GAP_MS} ms): {gaps} khoảng ({gap_pct:.2f}%)")
    print(f"Vị trí trung bình: lat {lat0 * 1e-7:.7f}, lon {lon0 * 1e-7:.7f}")
    print()
    print(f"Std N: {std_n:.3f} m   Std E: {std_e:.3f} m   Std ngang: {std_h:.3f} m")
    print(f"Dao động đỉnh-đỉnh: N {np.ptp(north):.3f} m, E {np.ptp(east):.3f} m")
    print(f"CEP50: {cep50:.3f} m   R95: {r95:.3f} m   Lệch max: {r.max():.3f} m")
    print(f"Nhảy giữa 2 mẫu > {JUMP_WARN_M:.0f} m: {jumps_warn} lần (> {JUMP_FAIL_M:.0f} m: {jumps_fail})")
    print(f"Trôi chậm: {drift:.3f} m/phút (N {pn[0]:+.3f}, E {pe[0]:+.3f}); "
          f"std ngang sau khi bỏ trôi: {std_h_detr:.3f} m")
    print()
    print(f"hAcc chip báo: trung bình {hacc.mean():.3f} m (min {hacc.min():.3f}, max {hacc.max():.3f})")
    ratio = std_h / hacc.mean() if hacc.mean() > 0 else float("nan")
    verdict = "chip LẠC QUAN (báo tốt hơn thực tế)" if ratio > 1.0 else "chip thận trọng (báo xấu hơn thực tế)"
    print(f"  so với std ngang đo được {std_h:.3f} m → tỉ lệ đo/báo = {ratio:.2f}: {verdict}")
    print("  (hAcc của u-blox là ước lượng độ chính xác — chưa chắc tương ứng đúng 1 sigma)")
    print(f"Vận tốc khi đứng yên: velN {vn.mean():+.3f} ± {vn.std(ddof=1):.3f} m/s, "
          f"velE {ve.mean():+.3f} ± {ve.std(ddof=1):.3f} m/s")
    print(f"  → gợi ý R vận tốc cho Kalman ≈ {((vn.var(ddof=1) + ve.var(ddof=1)) / 2):.4f} (m/s)²")
    print(f"sAcc trung bình: {sacc.mean():.3f} m/s")

    # ---- Kết luận ----
    checks = [
        (f"std ngang < {args.max_std_m} m", std_h < args.max_std_m, f"{std_h:.3f} m"),
        (f"tần số >= {args.min_hz} Hz", freq >= args.min_hz, f"{freq:.2f} Hz"),
        (f"không nhảy > {JUMP_FAIL_M:.0f} m", jumps_fail == 0, f"{jumps_fail} lần"),
        (f"hụt mẫu <= {args.max_gap_pct}%", gap_pct <= args.max_gap_pct, f"{gap_pct:.2f}%"),
    ]
    print()
    for name, ok, val in checks:
        print(f"  [{'ĐẠT' if ok else 'KHÔNG ĐẠT'}] {name}: {val}")
    all_ok = all(ok for _, ok, _ in checks)
    print(f"KẾT LUẬN: {'ĐẠT' if all_ok else 'KHÔNG ĐẠT'}")

    # ---- Đồ thị ----
    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    base = os.path.splitext(args.csv)[0]
    c_n, c_e = "#2a78d6", "#eb6834"

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={"width_ratios": [1, 1.6]})
    ax1.scatter(east, north, s=4, color=c_n, alpha=0.5, linewidths=0)
    ax1.add_patch(plt.Circle((east.mean(), north.mean()), cep50, fill=False, color="0.3", lw=1, ls="--",
                             label=f"CEP50 {cep50:.2f} m"))
    ax1.add_patch(plt.Circle((east.mean(), north.mean()), r95, fill=False, color="0.3", lw=1,
                             label=f"R95 {r95:.2f} m"))
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.set_xlabel("Đông (m)")
    ax1.set_ylabel("Bắc (m)")
    ax1.set_title("Vị trí quanh điểm trung bình")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)
    ax2.plot(t_s, north, color=c_n, lw=1, label="Bắc")
    ax2.plot(t_s, east, color=c_e, lw=1, label="Đông")
    ax2.set_xlabel("Thời gian (s, theo iTOW)")
    ax2.set_ylabel("Lệch so với trung bình (m)")
    ax2.set_title(f"Vị trí theo thời gian — std ngang {std_h:.2f} m")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    pos_png = base + "_pos.png"
    fig.savefig(pos_png, dpi=120)

    fig2, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t_s, vn, color=c_n, lw=1, label=f"velN (std {vn.std(ddof=1):.3f})")
    ax.plot(t_s, ve, color=c_e, lw=1, label=f"velE (std {ve.std(ddof=1):.3f})")
    ax.axhline(0, color="0.5", lw=0.8)
    ax.set_xlabel("Thời gian (s, theo iTOW)")
    ax.set_ylabel("m/s")
    ax.set_title("Vận tốc NAV-PVT khi đứng yên")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    fig2.tight_layout()
    vel_png = base + "_vel.png"
    fig2.savefig(vel_png, dpi=120)

    print(f"\nĐã lưu hình: {pos_png}, {vel_png}")
    if args.show:
        plt.show()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
