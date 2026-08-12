#!/usr/bin/env python3
"""candump 로그에서 다리 모터(can0/can1) 프레임을 디코딩한다.

RobStride 프로토콜 중 두 프레임만 다룬다:
  - comm_type 1 (COMM_OPERATION_CONTROL): 호스트 -> 모터, 커맨드(ref) 프레임
    inc/RobstrideMotor.hpp의 write_updated_operation_frame()이 만드는 그 프레임
  - comm_type 2: 모터 -> 호스트, 피드백(fb) 프레임
    inc/Rx_handler.hpp의 parse_Rx_frame()과 동일한 레이아웃

CAN ID -> 관절 이름/스케일 매핑은 inc/Humanoid_config.h의 CAN_ID_* 정의와
src/main.cpp의 모터 등록 순서(RS04/RS03/RS06 배정)를 그대로 옮긴 것이다.
이 매핑이 바뀌면 이 테이블도 같이 갱신해야 한다.
"""

import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ab import parse_candump_line

POS_SCALE = 4 * math.pi  # 모든 모터 타입 공통 (inc/RobstrideMotor.hpp MotorConstants)


def _rs(name, vel_scale, kp_scale, kd_scale, tq_scale):
    return dict(name=name, vel_scale=vel_scale, kp_scale=kp_scale,
                kd_scale=kd_scale, tq_scale=tq_scale)


# RS04: HIP_PITCH / KNEE_PITCH,  RS03: HIP_ROLL / HIP_YAW,  RS06: ANKLE_A / ANKLE_B
MOTOR_MAP = {
    0x01: _rs("L_HIP_PITCH",  15.0, 5000.0, 100.0, 120.0),
    0x11: _rs("R_HIP_PITCH",  15.0, 5000.0, 100.0, 120.0),
    0x04: _rs("L_KNEE_PITCH", 15.0, 5000.0, 100.0, 120.0),
    0x14: _rs("R_KNEE_PITCH", 15.0, 5000.0, 100.0, 120.0),

    0x02: _rs("L_HIP_ROLL",   20.0, 5000.0, 100.0, 60.0),
    0x12: _rs("R_HIP_ROLL",   20.0, 5000.0, 100.0, 60.0),
    0x03: _rs("L_HIP_YAW",    20.0, 5000.0, 100.0, 60.0),
    0x13: _rs("R_HIP_YAW",    20.0, 5000.0, 100.0, 60.0),

    0x05: _rs("L_ANKLE_A",    50.0, 5000.0, 100.0, 36.0),
    0x15: _rs("R_ANKLE_A",    50.0, 5000.0, 100.0, 36.0),
    0x06: _rs("L_ANKLE_B",    50.0, 5000.0, 100.0, 36.0),
    0x16: _rs("R_ANKLE_B",    50.0, 5000.0, 100.0, 36.0),
}

# plot_motors.py와 동일한 해부학적 배치 (4행 x 3열)
MOTORS = [
    ["L_HIP_PITCH",  "L_HIP_ROLL", "L_HIP_YAW"],
    ["R_HIP_PITCH",  "R_HIP_ROLL", "R_HIP_YAW"],
    ["L_KNEE_PITCH", "L_ANKLE_A",  "L_ANKLE_B"],
    ["R_KNEE_PITCH", "R_ANKLE_A",  "R_ANKLE_B"],
]
NROWS, NCOLS = 4, 3

PANELS = [
    ("Position [rad]",   "pos",  "rad"),
    ("Velocity [rad/s]", "vel",  "rad/s"),
    ("Torque [Nm]",      "torq", "Nm"),
]


def _decode_feedback(can_id: int, data: bytes):
    """inc/Rx_handler.hpp parse_Rx_frame()과 동일한 레이아웃."""
    motor_id = (can_id >> 8) & 0xFF
    p_raw = (data[0] << 8) | data[1]
    v_raw = (data[2] << 8) | data[3]
    t_raw = (data[4] << 8) | data[5]
    temp = ((data[6] << 8) | data[7]) * 0.1
    return (
        motor_id,
        p_raw / 32767.0 - 1.0,
        v_raw / 32767.0 - 1.0,
        t_raw / 32767.0 - 1.0,
        temp,
    )


def _decode_command(can_id: int, data: bytes):
    """inc/RobstrideMotor.hpp write_updated_operation_frame()의 역변환."""
    target_id = can_id & 0xFF
    torque_u16 = (can_id >> 8) & 0xFFFF
    pos_u16 = (data[0] << 8) | data[1]
    vel_u16 = (data[2] << 8) | data[3]
    kp_u16 = (data[4] << 8) | data[5]
    kd_u16 = (data[6] << 8) | data[7]
    return (
        target_id,
        pos_u16 / 32767.0 - 1.0,
        vel_u16 / 32767.0 - 1.0,
        kp_u16 / 65535.0,
        kd_u16 / 65535.0,
        torque_u16 / 32767.0 - 1.0,
    )


def parse_motor_log(log_path: Path) -> dict:
    """candump 로그를 읽어 모터 이름별 {"fb": DataFrame, "ref": DataFrame} 을 반환한다."""

    fb_rows = defaultdict(list)
    ref_rows = defaultdict(list)

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            parsed = parse_candump_line(line)

            if parsed is None:
                continue

            timestamp, can_id, data = parsed

            if len(data) < 8:
                continue

            comm_type = (can_id >> 24) & 0x1F

            if comm_type == 2:
                motor_id, pos_n, vel_n, torq_n, temp = _decode_feedback(can_id, data)
                motor = MOTOR_MAP.get(motor_id)
                if motor is None:
                    continue
                fb_rows[motor["name"]].append({
                    "timestamp": timestamp,
                    "pos": pos_n * POS_SCALE,
                    "vel": vel_n * motor["vel_scale"],
                    "torq": torq_n * motor["tq_scale"],
                    "temp": temp,
                })

            elif comm_type == 1:
                target_id, pos_n, vel_n, kp_n, kd_n, torque_n = _decode_command(can_id, data)
                motor = MOTOR_MAP.get(target_id)
                if motor is None:
                    continue
                ref_rows[motor["name"]].append({
                    "timestamp": timestamp,
                    "pos": pos_n * POS_SCALE,
                    "vel": vel_n * motor["vel_scale"],
                    "kp": kp_n * motor["kp_scale"],
                    "kd": kd_n * motor["kd_scale"],
                    "torque_ff": torque_n * motor["tq_scale"],
                })

    first_timestamps = [
        rows[0]["timestamp"]
        for rows in (*fb_rows.values(), *ref_rows.values())
        if rows
    ]
    t0 = min(first_timestamps) if first_timestamps else 0.0

    motor_data = {}
    for motor in MOTOR_MAP.values():
        name = motor["name"]
        motor_data[name] = {
            "fb": _rows_to_columns(fb_rows.get(name), t0),
            "ref": _rows_to_columns(ref_rows.get(name), t0),
        }

    return motor_data


def _rows_to_columns(rows: list[dict] | None, t0: float) -> dict | None:
    """dict 리스트를 컬럼별 numpy 배열 dict로 변환한다. rows가 비어있으면 None."""

    if not rows:
        return None

    columns = {key: np.array([row[key] for row in rows]) for key in rows[0]}
    columns["time_s"] = columns["timestamp"] - t0
    return columns


def plot_motor_grid(motor_data: dict, title_suffix: str = ""):
    """plot_motors.py와 동일한 4x3 그리드를 그린다 (plt.show()는 호출부에서)."""

    for title, field, ylabel in PANELS:
        fig, axes = plt.subplots(NROWS, NCOLS, figsize=(16, 10), sharex=True)
        fig.suptitle(f"{title}  —  {title_suffix}", fontsize=11)

        for r, row in enumerate(MOTORS):
            for c, name in enumerate(row):
                ax = axes[r][c]
                fb = motor_data[name]["fb"]
                ref = motor_data[name]["ref"]

                if fb is not None:
                    ax.plot(fb["time_s"], fb[field], linewidth=0.8, label="fb")

                if field == "pos" and ref is not None:
                    ax.plot(ref["time_s"], ref["pos"], "--", linewidth=0.8,
                            color="tab:orange", label="ref")

                if fb is not None or (field == "pos" and ref is not None):
                    ax.legend(fontsize=7, loc="upper right")

                ax.set_title(name, fontsize=9)
                ax.set_ylabel(ylabel, fontsize=7)
                ax.tick_params(labelsize=7)
                ax.grid(True, linewidth=0.4, alpha=0.6)

        for c in range(NCOLS):
            axes[-1][c].set_xlabel("time [s]", fontsize=8)

        fig.tight_layout()
