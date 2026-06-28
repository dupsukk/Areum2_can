#!/usr/bin/env python3
"""
Usage:
    python3 plot_motors.py                          # 가장 최근 파일, 전체 구간
    python3 plot_motors.py <file.csv>               # 지정 파일, 전체 구간
    python3 plot_motors.py <file.csv> <t_start>     # t_start[s] ~ 끝
    python3 plot_motors.py <file.csv> <t_start> <t_end>
"""
import sys
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt

# ── 인자 파싱 ─────────────────────────────────────────────────────────────────
args = sys.argv[1:]

csv_path   = None
t_start    = None
t_end      = None

for a in args:
    try:
        val = float(a)
        if t_start is None:
            t_start = val
        else:
            t_end = val
    except ValueError:
        csv_path = a

if csv_path is None:
    files = sorted(glob.glob("motor_log_*.csv"))
    if not files:
        print("motor_log_*.csv 파일이 없습니다.")
        print(__doc__)
        sys.exit(1)
    csv_path = files[-1]
    print(f"Loading: {csv_path}")

# ── CSV 로드 ──────────────────────────────────────────────────────────────────
with open(csv_path, newline="") as f:
    reader = csv.reader(f)
    headers = next(reader)
    rows = [[float(v) for v in row] for row in reader]

data = np.array(rows).T  # shape: (n_cols, n_rows)
col  = {name: data[i] for i, name in enumerate(headers)}
t    = col["time_ms"] / 1000.0  # ms → s

# ── 구간 필터 ─────────────────────────────────────────────────────────────────
lo = t_start if t_start is not None else t[0]
hi = t_end   if t_end   is not None else t[-1]
mask = (t >= lo) & (t <= hi)
t    = t[mask]
col  = {k: v[mask] for k, v in col.items()}

duration = t[-1] - t[0] if len(t) > 1 else 0.0
actual_hz = 1.0 / np.mean(np.diff(t)) if len(t) > 1 else 0.0
print(f"구간: {t[0]:.3f}s ~ {t[-1]:.3f}s  ({duration:.2f}s, {actual_hz:.1f}Hz, {len(t)} rows)")

# ── 모터 배치 (4행 × 3열, 해부학적 그룹핑) ───────────────────────────────────
#   행0: 왼쪽 고관절   행1: 오른쪽 고관절
#   행2: 왼쪽 무릎+발목  행3: 오른쪽 무릎+발목
MOTORS = [
    ["L_HIP_PITCH", "L_HIP_ROLL",  "L_HIP_YAW" ],
    ["R_HIP_PITCH", "R_HIP_ROLL",  "R_HIP_YAW" ],
    ["L_KNEE_PITCH", "L_ANKLE_A",  "L_ANKLE_B" ],
    ["R_KNEE_PITCH", "R_ANKLE_A",  "R_ANKLE_B" ],
]
NROWS, NCOLS = 4, 3

# ── 플롯할 항목 정의 ──────────────────────────────────────────────────────────
PANELS = [
    ("Position [rad]",   "pos",  "rad"),
    ("Velocity [rad/s]", "vel",  "rad/s"),
    ("Torque [Nm]",      "torq", "Nm"),
]

for title, field, ylabel in PANELS:
    fig, axes = plt.subplots(NROWS, NCOLS, figsize=(16, 10), sharex=True)
    fig.suptitle(f"{title}  —  {csv_path}  [{t[0]:.2f}s ~ {t[-1]:.2f}s, {actual_hz:.0f}Hz]", fontsize=11)

    for r, row in enumerate(MOTORS):
        for c, motor in enumerate(row):
            ax = axes[r][c]
            ax.plot(t, col[f"{motor}_{field}"], linewidth=0.8, label="fb")
            if field == "pos":
                ax.plot(t, col[f"{motor}_ref"], "--", linewidth=0.8,
                        color="tab:orange", label="ref")
                ax.legend(fontsize=7, loc="upper right")
            ax.set_title(motor, fontsize=9)
            ax.set_ylabel(ylabel, fontsize=7)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4, alpha=0.6)

    for c in range(NCOLS):
        axes[-1][c].set_xlabel("time [s]", fontsize=8)

    fig.tight_layout()

plt.show()
