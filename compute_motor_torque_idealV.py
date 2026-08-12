#!/usr/bin/env python3
"""candump 로그의 MIT 커맨드/피드백 프레임으로 토크를 계산하고, 전압이 안정적(고정)이라고
가정했을 때 필요한 파워버스 전류를 추정한다.

compute_motor_torque_PD.py와 계산식은 동일하되, 실측 V_bus(파워보드 로그, 방전에 따라 처짐)
대신 고정된 이상적 전압을 쓴다 — 전압 처짐에 의한 정궤환(V↓→I↑→V↓...) 없이, 순수하게
토크 수요만으로 얼마나 전류가 필요한지 보기 위함.

    I_phase = torque_measured / K_T
    P = I_phase²·PHASE_R + torque_measured·vel_fb/EFF   (동손은 그대로, 기계출력만 EFF로 나눔)
    I_bus_per_motor = P / VBUS_IDEAL (고정값, 기본 48V)
    total_bus_current = BASELINE_POWER_W/VBUS_IDEAL + 12개 관절 부호 있는 합

    I²R(동손)은 EFF로 안 나눈다 — 실제 상전류로 이미 완결된 값이고 감속기를 거치지 않는 항이라서.
    T·vel/EFF만 나누는 이유: torque_measured/vel_fb는 감속기 출력(관절) 측 값인데, 전기-기계
    변환 자체는 로터에서 일어나므로 로터가 실제로 낸 기계출력은 감속기 손실만큼 T·vel보다 크다
    (EFF는 이 감속기+드라이버 결합 효율. 전기 효율은 보통 1에 가까워 감속기가 지배적).
    별도의 역기전력 상수는 안 쓴다 — torque_measured·vel_fb는 측정값 두 개를 곱한 기계출력이라
    K_T가 관여하지 않는다.

Usage:
    python3 compute_motor_torque_idealV.py                    # logs/candump-*.log 중 최신, 48V
    python3 compute_motor_torque_idealV.py <candump.log>
    python3 compute_motor_torque_idealV.py <candump.log> --vbus 44.4
    python3 compute_motor_torque_idealV.py <candump.log> -o out.csv --vbus 50
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
from can_motor_decode import MOTOR_MAP, POS_SCALE, _decode_command, _decode_feedback

CSV_FIELDS = [
    "time_s", "motor",
    "pos_fb", "vel_fb", "torque_measured",
    "pos_ref", "vel_ref", "kp", "kd", "torque_ff",
    "torque_computed",
]

DEFAULT_VBUS = 48.0

# 로직/제어보드 등 토크와 무관하게 항상 흐르는 상시 소비전력[W] — 모터 스펙이 아니라 이 로봇의
# 시스템 구성값이라 motor_constants.json이 아니라 joint_motor_map.json의 SYSTEM.BASELINE_POWER_W에서
# 읽는다 (없으면 이 값을 기본값으로 사용).
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


def compute_motor_torque_ideal(log_path: Path, output_path: Path, vbus: float) -> tuple[dict, dict]:
    last_cmd = {}
    plot_rows = defaultdict(lambda: defaultdict(list))
    t0 = None

    latest_motor_current = {}
    bus_current_rows = defaultdict(list)

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
            if comm_type not in (1, 2):
                continue

            if t0 is None:
                t0 = timestamp

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
                    continue

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
                if elec is not None:
                    i_phase = torque_measured / elec["k_t"]
                    # P = I^2*R(동손, 실제 상전류로 이미 완결된 값) + T*vel/EFF(로터가 실제로 낸
                    # 기계출력 — T*vel은 감속기 출력측 값이라 감속기 손실만큼 로터측은 더 컸음).
                    # EFF를 I^2*R까지 나누면 안 됨: 그건 감속기를 거치지 않는 항이라.
                    power = i_phase**2 * elec["phase_r"] + (torque_measured * vel_fb) / elec["eff"]
                    latest_motor_current[motor["name"]] = power / vbus

                    baseline_current = BASELINE_POWER_W / vbus
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


def plot_bus_current(bus_current: dict, vbus: float, title_suffix: str = ""):
    fig, ax = plt.subplots(figsize=(14, 5))
    fig.suptitle(f"Total power-bus current — ideal stable V_bus={vbus:.1f}V  —  {title_suffix}", fontsize=11)

    ax.plot(bus_current["time_s"], bus_current["total_bus_current_a"], linewidth=0.8)
    ax.axhline(0, linewidth=0.6, color="black", alpha=0.4)

    ax.set_xlabel("time [s]", fontsize=8)
    ax.set_ylabel("Current [A]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, linewidth=0.4, alpha=0.6)

    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="candump 로그의 토크로부터, 전압이 안정적(고정값)이라고 가정한 이상적 버스전류를 CSV+플롯으로 남긴다."
    )
    parser.add_argument("log_file", type=Path, nargs="?", help="candump -l 로그 파일 (생략 시 logs/candump-*.log 중 최신)")
    parser.add_argument("-o", "--output", type=Path, default=None, help="출력 CSV 경로 (기본: <로그이름>_torque_idealV.csv)")
    parser.add_argument("--vbus", type=float, default=DEFAULT_VBUS, help=f"고정 버스전압[V] (기본: {DEFAULT_VBUS}V)")
    args = parser.parse_args()

    log_path = args.log_file or find_latest_log()
    if not log_path.exists():
        raise FileNotFoundError(f"로그 파일을 찾을 수 없습니다: {log_path}")

    output_path = args.output or log_path.with_name(log_path.stem + "_torque_idealV.csv")

    if not MOTOR_ELEC:
        print("motor_constants.json / joint_motor_map.json에 K_T, PHASE_R이 채워진 관절이 없어서 "
              "버스전류 계산은 건너뜁니다 (CSV의 토크 컬럼은 그대로 기록됩니다).")

    motor_data, bus_current = compute_motor_torque_ideal(log_path, output_path, args.vbus)

    if not motor_data:
        raise RuntimeError("커맨드+피드백 프레임 쌍을 찾지 못했습니다 (comm_type 1/2 모터 프레임이 없는 로그인지 확인).")

    rows_written = sum(len(fields["time_s"]) for fields in motor_data.values())
    print(f"{rows_written}행 기록 -> {output_path}  (V_bus={args.vbus:.1f}V 고정)")

    if bus_current:
        plot_bus_current(bus_current, args.vbus, log_path.name)
        plt.show()
    else:
        print("버스전류 데이터가 없어서 플롯을 생략합니다.")


if __name__ == "__main__":
    main()
