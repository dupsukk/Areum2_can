#!/usr/bin/env python3
"""candump 로그에서 MIT 프로토콜 커맨드/피드백 프레임을 읽어
각 시점에 모터가 실제로 내야 하는 토크(PD+FF 제어식)를 계산해 CSV로 남기고,
바로 이어서 모터별 4x3 그리드로 (측정값 vs 계산값) 플롯을 띄운다.

제어식은 RobStride/MIT 모드 그대로:
    torque_computed = Kp * (pos_ref - pos_fb) + Kd * (vel_ref - vel_fb) + torque_ff

- pos_ref/vel_ref/Kp/Kd/torque_ff : comm_type 1 커맨드 프레임(호스트->모터)에서 디코딩
- pos_fb/vel_fb/torque_measured   : comm_type 2 피드백 프레임(모터->호스트)에서 디코딩
- 각 모터마다 "가장 최근에 수신한 커맨드"를 들고 있다가, 그 모터의 피드백 프레임이
  들어올 때마다 그 시점 기준으로 한 행을 계산해서 씁니다 (커맨드 프레임을 아직
  한 번도 못 받은 모터는 건너뜀 — 캘리브레이션 전 상태이므로).

Usage:
    python3 compute_motor_torque_PD.py                    # logs/candump-*.log 중 최신
    python3 compute_motor_torque_PD.py <candump.log>
    python3 compute_motor_torque_PD.py <candump.log> -o out.csv
"""

import argparse
import csv
import glob
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ab import parse_candump_line
from can_motor_decode import MOTOR_MAP, MOTORS, NCOLS, NROWS, POS_SCALE, _decode_command, _decode_feedback

CSV_FIELDS = [
    "time_s", "motor",
    "pos_fb", "vel_fb", "torque_measured",
    "pos_ref", "vel_ref", "kp", "kd", "torque_ff",
    "torque_computed",
]

# 파워보드 로그(ab.py)와 동일한 기본 POWER_ID
POWER_ID = 0xAA

# 로직/제어보드 등 토크와 무관하게 항상 흐르는 상시 소비전력[W] — 모터 스펙이 아니라 이 로봇의
# 시스템 구성값이라 motor_constants.json이 아니라 joint_motor_map.json의 SYSTEM.BASELINE_POWER_W에서
# 읽는다 (없으면 이 값을 기본값으로 사용). I_baseline(t) = BASELINE_POWER_W / V_bus(t)로 반영.
DEFAULT_BASELINE_POWER_W = 27.183

MOTOR_CONSTANTS_PATH = Path(__file__).resolve().parent / "motor_constants.json"
JOINT_MOTOR_MAP_PATH = Path(__file__).resolve().parent / "joint_motor_map.json"


def _load_baseline_power() -> float:
    if not JOINT_MOTOR_MAP_PATH.exists():
        return DEFAULT_BASELINE_POWER_W

    joint_map = json.loads(JOINT_MOTOR_MAP_PATH.read_text(encoding="utf-8"))
    value = joint_map.get("SYSTEM", {}).get("BASELINE_POWER_W")
    return value if value is not None else DEFAULT_BASELINE_POWER_W


def _load_motor_elec() -> dict:
    """motor_constants.json(타입별 K_T/PHASE_R/EFF) + joint_motor_map.json(관절->타입)을
    합쳐서 관절 이름 -> {k_t, phase_r, eff} 딕셔너리를 만든다. K_T/PHASE_R이 아직 안 채워진(null)
    관절/타입은 빠진다 (버스전류 계산에서 자동으로 제외됨). EFF가 없으면 1.0(무손실)로 취급."""

    if not MOTOR_CONSTANTS_PATH.exists() or not JOINT_MOTOR_MAP_PATH.exists():
        return {}

    constants = json.loads(MOTOR_CONSTANTS_PATH.read_text(encoding="utf-8"))
    joint_map = json.loads(JOINT_MOTOR_MAP_PATH.read_text(encoding="utf-8"))

    motor_elec = {}
    for joint_name, motor_type in joint_map.items():
        if joint_name == "SYSTEM" or motor_type is None:
            continue
        type_data = constants.get(motor_type)
        if not type_data:
            continue
        k_t = type_data.get("K_T")
        phase_r = type_data.get("PHASE_R")
        if k_t is None or phase_r is None:
            continue
        eff = type_data.get("EFF")
        motor_elec[joint_name] = {"k_t": k_t, "phase_r": phase_r, "eff": eff if eff else 1.0}

    return motor_elec


MOTOR_ELEC = _load_motor_elec()
BASELINE_POWER_W = _load_baseline_power()


def find_latest_log() -> Path:
    files = sorted(glob.glob("logs/candump-*.log"))
    if not files:
        print("logs/candump-*.log 파일이 없습니다. 로그 경로를 인자로 지정하세요.")
        sys.exit(1)
    print(f"Loading: {files[-1]}")
    return Path(files[-1])


def compute_motor_torque(log_path: Path, output_path: Path) -> tuple[dict, dict]:
    """CSV로 기록하면서 동시에 모터별 (time_s, torque_measured, torque_computed)와
    전체 파워버스 전류(time_s, total_bus_current_a)를 메모리에도 모아서 반환한다
    (바로 이어서 플롯할 수 있도록, CSV 재파싱 없이).

    버스전류 = BASELINE_POWER_W/V_bus + sum_motor( P_phase / V_bus ), 여기서
    I_phase = torque_measured/K_T, P_phase = I_phase²·PHASE_R + torque_measured·vel_fb/EFF.
    I²R(동손)은 EFF로 안 나눈다 — 실제 상전류로 이미 완결된 값이고 감속기를 거치지 않는 항이라서.
    T·vel/EFF만 나누는 이유: torque_measured/vel_fb는 감속기 출력(관절) 측 값인데, 전기-기계
    변환 자체는 로터에서 일어나므로 로터가 실제로 낸 기계출력은 감속기 손실만큼 T·vel보다 크다
    (EFF는 이 감속기+드라이버 결합 효율. 전기 효율은 보통 1에 가까워 감속기가 지배적).
    별도의 역기전력 상수는 안 쓴다 — torque_measured·vel_fb는 측정값 두 개를 그냥 곱한 기계출력이라
    K_T가 관여하지 않는다.
    BASELINE_POWER_W는 로직/제어보드 등 토크와 무관한 상시 소비전력(정지구간 실측으로 역산, 상수
    전력이라 V_bus로 나눠서 전류화).
    K_T/PHASE_R/EFF는 motor_constants.json + joint_motor_map.json에서, V_bus는 파워보드(Type 03,
    POWER_ID) 프레임에서 zero-order-hold로 가져온다. MOTOR_ELEC에 없는 관절은 기여분 0으로 취급."""

    last_cmd = {}  # motor name -> dict(pos_ref, vel_ref, kp, kd, torque_ff)
    plot_rows = defaultdict(lambda: defaultdict(list))
    t0 = None

    latest_vbus = None
    latest_motor_current = {}  # motor name -> 그 모터의 마지막 버스전류 기여분[A]
    bus_current_rows = defaultdict(list)  # {"time_s": [...], "total_bus_current_a": [...]}

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file, \
         output_path.open("w", newline="", encoding="utf-8") as out_file:

        writer = csv.writer(out_file)
        writer.writerow(CSV_FIELDS)

        for line in log_file:
            parsed = parse_candump_line(line)
            if parsed is None:
                continue

            timestamp, can_id, data = parsed
            if len(data) < 8:
                continue

            comm_type = (can_id >> 24) & 0x1F
            if comm_type not in (1, 2, 3):
                continue

            if t0 is None:
                t0 = timestamp

            if comm_type == 3:
                frame_power_id = (can_id >> 16) & 0xFF
                if frame_power_id == POWER_ID:
                    latest_vbus = int.from_bytes(data[0:2], "big") / 100.0
                continue

            if comm_type == 1:
                target_id, pos_n, vel_n, kp_n, kd_n, torque_n = _decode_command(can_id, data)
                motor = MOTOR_MAP.get(target_id)
                if motor is None:
                    continue

                last_cmd[motor["name"]] = {
                    "pos_ref": pos_n * POS_SCALE,
                    "vel_ref": vel_n * motor["vel_scale"],
                    "kp": kp_n * motor["kp_scale"],
                    "kd": kd_n * motor["kd_scale"],
                    "torque_ff": torque_n * motor["tq_scale"],
                }

            else:  # comm_type == 2, 피드백
                motor_id, pos_n, vel_n, torq_n, _temp = _decode_feedback(can_id, data)
                motor = MOTOR_MAP.get(motor_id)
                if motor is None:
                    continue

                cmd = last_cmd.get(motor["name"])
                if cmd is None:
                    continue  # 이 모터의 첫 커맨드가 아직 도착하지 않음

                pos_fb = pos_n * POS_SCALE
                vel_fb = vel_n * motor["vel_scale"]
                torque_measured = torq_n * motor["tq_scale"]

                torque_computed = (
                    cmd["kp"] * (cmd["pos_ref"] - pos_fb)
                    + cmd["kd"] * (cmd["vel_ref"] - vel_fb)
                    + cmd["torque_ff"]
                )

                time_s = timestamp - t0

                writer.writerow([
                    f"{time_s:.6f}", motor["name"],
                    f"{pos_fb:.6f}", f"{vel_fb:.6f}", f"{torque_measured:.6f}",
                    f"{cmd['pos_ref']:.6f}", f"{cmd['vel_ref']:.6f}",
                    f"{cmd['kp']:.4f}", f"{cmd['kd']:.4f}", f"{cmd['torque_ff']:.4f}",
                    f"{torque_computed:.6f}",
                ])

                fields = plot_rows[motor["name"]]
                fields["time_s"].append(time_s)
                fields["torque_measured"].append(torque_measured)
                fields["torque_computed"].append(torque_computed)

                elec = MOTOR_ELEC.get(motor["name"])
                if elec is not None and latest_vbus:
                    i_phase = torque_measured / elec["k_t"]
                    # P = I^2*R(동손, 실제 상전류로 이미 완결된 값) + T*vel/EFF(로터가 실제로 낸
                    # 기계출력 — T*vel은 감속기 출력측 값이라 감속기 손실만큼 로터측은 더 컸음).
                    # EFF를 I^2*R까지 나누면 안 됨: 그건 감속기를 거치지 않는 항이라.
                    power = i_phase**2 * elec["phase_r"] + (torque_measured * vel_fb) / elec["eff"]
                    latest_motor_current[motor["name"]] = power / latest_vbus

                    baseline_current = BASELINE_POWER_W / latest_vbus
                    bus_current_rows["time_s"].append(time_s)
                    bus_current_rows["total_bus_current_a"].append(
                        baseline_current + sum(latest_motor_current.values())
                    )

    motor_data = {
        name: {key: np.array(values) for key, values in fields.items()}
        for name, fields in plot_rows.items()
    }
    bus_current = {key: np.array(values) for key, values in bus_current_rows.items()}

    return motor_data, bus_current


def plot_torque_grid(motor_data: dict, title_suffix: str = ""):
    fig, axes = plt.subplots(NROWS, NCOLS, figsize=(16, 10), sharex=True)
    fig.suptitle(f"Torque [Nm] — measured vs. computed (PD+FF)  —  {title_suffix}", fontsize=11)

    for r, row in enumerate(MOTORS):
        for c, name in enumerate(row):
            ax = axes[r][c]
            data = motor_data.get(name)

            if data is not None:
                ax.plot(data["time_s"], data["torque_measured"], linewidth=0.8, label="measured")
                ax.plot(data["time_s"], data["torque_computed"], "--", linewidth=0.8,
                        color="tab:orange", label="computed")
                ax.legend(fontsize=7, loc="upper right")

            ax.set_title(name, fontsize=9)
            ax.set_ylabel("Nm", fontsize=7)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4, alpha=0.6)

    for c in range(NCOLS):
        axes[-1][c].set_xlabel("time [s]", fontsize=8)

    fig.tight_layout()


def plot_bus_current(bus_current: dict, title_suffix: str = ""):
    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle(f"Total power-bus current  —  {title_suffix}", fontsize=11)

    ax.plot(bus_current["time_s"], bus_current["total_bus_current_a"], linewidth=0.8)
    ax.axhline(0, linewidth=0.6, color="black", alpha=0.4)

    ax.set_xlabel("time [s]", fontsize=8)
    ax.set_ylabel("Computed Current [A]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, linewidth=0.4, alpha=0.6)

    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="candump 로그의 MIT 커맨드/피드백 프레임으로 모터별 계산 토크를 CSV로 남기고 바로 플롯한다."
    )
    parser.add_argument("log_file", type=Path, nargs="?", help="candump -l 로그 파일 (생략 시 logs/candump-*.log 중 최신)")
    parser.add_argument("-o", "--output", type=Path, default=None, help="출력 CSV 경로 (기본: <로그이름>_torque.csv)")
    args = parser.parse_args()

    log_path = args.log_file or find_latest_log()
    if not log_path.exists():
        raise FileNotFoundError(f"로그 파일을 찾을 수 없습니다: {log_path}")

    output_path = args.output or log_path.with_name(log_path.stem + "_torque.csv")

    if not MOTOR_ELEC:
        print("motor_constants.json / joint_motor_map.json에 K_T, PHASE_R이 채워진 관절이 없어서 "
              "버스전류 계산은 건너뜁니다 (토크 플롯은 그대로 진행).")

    motor_data, bus_current = compute_motor_torque(log_path, output_path)

    if not motor_data:
        raise RuntimeError("커맨드+피드백 프레임 쌍을 찾지 못했습니다 (comm_type 1/2 모터 프레임이 없는 로그인지 확인).")

    rows_written = sum(len(fields["time_s"]) for fields in motor_data.values())
    print(f"{rows_written}행 기록 -> {output_path}")

    plot_torque_grid(motor_data, log_path.name)

    if bus_current:
        plot_bus_current(bus_current, log_path.name)

    plt.show()


if __name__ == "__main__":
    main()
