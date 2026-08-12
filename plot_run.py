#!/usr/bin/env python3
"""
candump 로그 하나(can0/can1/can4가 합쳐진 파일)를 읽어서
파워보드(ab.py), 다리 모터(can_motor_decode.py), PD 토크/버스전류 모델
(compute_motor_torque_PD.py) 그래프를 전부 한 번에 띄운다.

모델 버스전류는 실측 전류 그래프에 점선으로 겹쳐 그리고, 토크(측정 vs 계산) 비교는
별도 플롯으로 띄운다.

Usage:
    python3 plot_run.py                  # logs/ 안의 가장 최근 candump 로그
    python3 plot_run.py <candump.log>    # 지정 로그 파일
"""

import glob
import sys
from pathlib import Path

import matplotlib.pyplot as plt

from ab import parse_powerboard_log, plot_status, plot_branches
from can_motor_decode import parse_motor_log, plot_motor_grid
from compute_motor_torque_PD import compute_motor_torque, plot_torque_grid


def find_latest_log() -> Path:
    files = sorted(glob.glob("logs/candump-*.log"))
    if not files:
        print("logs/candump-*.log 파일이 없습니다. start.sh로 로깅을 시작했는지 확인하세요.")
        sys.exit(1)
    print(f"Loading: {files[-1]}")
    return Path(files[-1])


def main():
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else find_latest_log()

    if not log_path.exists():
        raise FileNotFoundError(f"로그 파일을 찾을 수 없습니다: {log_path}")

    status_cols, branch_cols = parse_powerboard_log(log_path, power_id=0xAA)
    motor_data = parse_motor_log(log_path)

    torque_csv_path = log_path.with_name(log_path.stem + "_torque.csv")
    torque_data, bus_current = compute_motor_torque(log_path, torque_csv_path)
    if torque_data:
        print(f"토크 CSV 기록 -> {torque_csv_path}")

    has_motor_data = any(
        d["fb"] is not None or d["ref"] is not None for d in motor_data.values()
    )

    if status_cols is None and branch_cols is None and not has_motor_data:
        raise RuntimeError("로그에서 파워보드/모터 프레임을 찾지 못했습니다.")

    if status_cols is not None:
        extra = {"Total bus current (PD model)": bus_current} if bus_current else None
        plot_status(status_cols, Path("run"), save=False, extra_current_series=extra)

    if branch_cols is not None:
        plot_branches(branch_cols, Path("run"), save=False)

    if has_motor_data:
        plot_motor_grid(motor_data, log_path.name)

    if torque_data:
        plot_torque_grid(torque_data, log_path.name)

    plt.show()


if __name__ == "__main__":
    main()
