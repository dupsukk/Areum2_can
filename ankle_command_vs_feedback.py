#!/usr/bin/env python3
"""candump 로그의 앵클 모터(ANKLE_A/ANKLE_B) 커맨드·피드백 프레임에
ankle_transmission.py(= ankle_transmission_block.hpp 포팅)의 forward 변환을 걸어서,
- 커맨드(ref) 프레임 기준: 상위제어기가 실제로 보낸 roll/pitch 명령
- 피드백(fb) 프레임 기준: 상위제어기로 들어갈 roll/pitch (+ 각속도)
두 개를 좌우 다리별로 재구성해 CSV로 남기고 겹쳐서 플롯한다.

ANKLE_A -> qm5, ANKLE_B -> qm6 매핑은 tellus_sim2real/src/main.cpp의
fb_buf[8]=Ankle_Pitch(=SHM_MOTOR_INDEX_*_ANKLE_A), fb_buf[10]=Ankle_Roll(=ANKLE_B)
호출 순서 그대로다 (inc/Humanoid_config.h SHM_MOTOR_INDEX_* 참고).

두 모터는 서로 다른 CAN 프레임이라 타임스탬프가 정확히 일치하지 않는다 —
한쪽이 갱신될 때마다 다른 쪽의 최신값을 붙여서(zero-order hold) 그 시점의
roll/pitch를 계산한다 (compute_motor_torque_PD.py와 동일한 방식).

Usage:
    python3 ankle_command_vs_feedback.py                    # logs/candump-*.log 중 최신
    python3 ankle_command_vs_feedback.py <candump.log>
    python3 ankle_command_vs_feedback.py <candump.log> -o out.csv
"""

import argparse
import csv
import glob
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ankle_transmission import motor_to_roll_pitch, rad2deg
from can_motor_decode import parse_motor_log

LEGS = [
    ("LEFT", "L_ANKLE_A", "L_ANKLE_B"),
    ("RIGHT", "R_ANKLE_A", "R_ANKLE_B"),
]

CSV_FIELDS = ["time_s", "leg", "source", "roll_deg", "pitch_deg", "rollrate_deg_s", "pitchrate_deg_s"]

# 실제 발목 가동범위(inc/Humanoid_config.h 기준 약 ±65deg)보다 훨씬 넉넉한 상한.
# 클램프 경계(raw ±4pi)나 raw-0 커맨드처럼 다항식 피팅 범위를 벗어난 프레임만 걸러내는 용도.
PLAUSIBLE_DEG_LIMIT = 180.0


def find_latest_log() -> Path:
    files = sorted(glob.glob("logs/candump-*.log"))
    if not files:
        print("logs/candump-*.log 파일이 없습니다. 로그 경로를 인자로 지정하세요.")
        sys.exit(1)
    print(f"Loading: {files[-1]}")
    return Path(files[-1])


def _merge_and_transform(direction: str, a: dict | None, b: dict | None) -> dict | None:
    """ANKLE_A(qm5)/ANKLE_B(qm6) 시계열을 시간순으로 합치며, 갱신될 때마다
    그 시점 기준 최신 (qm5,qm6)로 roll/pitch(+rate)를 계산한다."""

    if a is None or b is None:
        return None

    events = sorted(
        [(t, "a", p, v) for t, p, v in zip(a["time_s"], a["pos"], a["vel"])]
        + [(t, "b", p, v) for t, p, v in zip(b["time_s"], b["pos"], b["vel"])],
        key=lambda e: e[0],
    )

    last_a = last_b = None
    time_s, roll_deg, pitch_deg, rollrate_deg_s, pitchrate_deg_s = [], [], [], [], []

    for t, which, pos, vel in events:
        if which == "a":
            last_a = (pos, vel)
        else:
            last_b = (pos, vel)

        if last_a is None or last_b is None:
            continue

        qm5_deg, qdotm5_deg_s = rad2deg(last_a[0]), rad2deg(last_a[1])
        qm6_deg, qdotm6_deg_s = rad2deg(last_b[0]), rad2deg(last_b[1])

        roll, pitch, rollrate, pitchrate = motor_to_roll_pitch(
            direction, qm5_deg, qm6_deg, qdotm5_deg_s, qdotm6_deg_s
        )

        # 다항식 피팅은 실제 발목 가동범위(약 ±65deg, inc/Humanoid_config.h) 근방에서만 유효.
        # 클램프 경계(±4pi raw, 에러 복구/셧다운 시 raw-0 커맨드 등)로 튄 프레임은 걸러낸다.
        if abs(roll) > PLAUSIBLE_DEG_LIMIT or abs(pitch) > PLAUSIBLE_DEG_LIMIT:
            continue

        time_s.append(t)
        roll_deg.append(roll)
        pitch_deg.append(pitch)
        rollrate_deg_s.append(rollrate)
        pitchrate_deg_s.append(pitchrate)

    if not time_s:
        return None

    return {
        "time_s": np.array(time_s),
        "roll_deg": np.array(roll_deg),
        "pitch_deg": np.array(pitch_deg),
        "rollrate_deg_s": np.array(rollrate_deg_s),
        "pitchrate_deg_s": np.array(pitchrate_deg_s),
    }


def _calibration_offset(pos0: float) -> float:
    """inc/RobstrideMotor.hpp RobstrideMotor::calibrate()와 동일: 절대엔코더 raw 값을
    2pi 단위로 감아서 원점 근방으로 가져오는 오프셋 (expected=0.0 기준)."""
    return -round(pos0 / (2 * math.pi)) * (2 * math.pi)


def _apply_offset(data: dict | None, offset: float) -> dict | None:
    if data is None:
        return None
    corrected = dict(data)
    corrected["pos"] = data["pos"] + offset
    return corrected


def _drop_before(data: dict | None, t_min: float) -> dict | None:
    """t_min 이전 행을 버린다. 캘리브레이션 전에 보내는 raw 0 목표(write_operation_frame(0,0,0))
    커맨드 프레임은 offset 좌표계가 아니라서 보정 후에도 값이 터무니없이 커진다 —
    calibrate()가 실행된 시점(=해당 모터의 첫 피드백 샘플) 이전 커맨드는 제외한다."""
    if data is None:
        return None
    mask = data["time_s"] >= t_min
    if not mask.any():
        return None
    return {key: values[mask] for key, values in data.items()}


def reconstruct_ankle_joints(log_path: Path) -> dict:
    """{leg: {"cmd": dict|None, "fb": dict|None}} 반환.

    CAN에 실리는 pos는 절대엔코더 raw 값(여러 바퀴 누적)이라, 실기에서 RobstrideMotor::
    calibrate()가 하는 것과 똑같이 각 모터의 첫 피드백 샘플 기준 2pi 오프셋을 보정한 뒤에
    앵클 변환식에 넣는다 — 안 그러면 다항식 피팅 범위(수십 deg)를 한참 벗어난 값이 나온다."""

    motor_data = parse_motor_log(log_path)

    result = {}
    for direction, name_a, name_b in LEGS:
        fb_a = motor_data[name_a]["fb"]
        fb_b = motor_data[name_b]["fb"]
        ref_a = motor_data[name_a]["ref"]
        ref_b = motor_data[name_b]["ref"]

        # calibrate()가 쓰는 기준과 동일하게 피드백의 첫 샘플로 오프셋을 정한다.
        # 피드백이 아예 없는 모터라면 커맨드의 첫 샘플로 대신한다.
        offset_a = _calibration_offset((fb_a or ref_a)["pos"][0]) if (fb_a or ref_a) else 0.0
        offset_b = _calibration_offset((fb_b or ref_b)["pos"][0]) if (fb_b or ref_b) else 0.0

        # calibrate()가 실행되기 전(=첫 피드백 이전)에 나간 raw 0 커맨드는 offset 좌표계가 아니므로 제외
        if fb_a is not None:
            ref_a = _drop_before(ref_a, fb_a["time_s"][0])
        if fb_b is not None:
            ref_b = _drop_before(ref_b, fb_b["time_s"][0])

        fb_a = _apply_offset(fb_a, offset_a)
        fb_b = _apply_offset(fb_b, offset_b)
        ref_a = _apply_offset(ref_a, offset_a)
        ref_b = _apply_offset(ref_b, offset_b)

        result[direction] = {
            "cmd": _merge_and_transform(direction, ref_a, ref_b),
            "fb": _merge_and_transform(direction, fb_a, fb_b),
        }

    return result


def write_csv(output_path: Path, ankle_data: dict) -> int:
    rows_written = 0

    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_FIELDS)

        for leg, sources in ankle_data.items():
            for source_label, key in (("cmd", "cmd"), ("fb", "fb")):
                data = sources[key]
                if data is None:
                    continue
                for i in range(len(data["time_s"])):
                    writer.writerow([
                        f"{data['time_s'][i]:.6f}", leg, source_label,
                        f"{data['roll_deg'][i]:.6f}", f"{data['pitch_deg'][i]:.6f}",
                        f"{data['rollrate_deg_s'][i]:.6f}", f"{data['pitchrate_deg_s'][i]:.6f}",
                    ])
                    rows_written += 1

    return rows_written


def plot_ankle_joints(ankle_data: dict, title_suffix: str = ""):
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig.suptitle(f"Ankle roll/pitch — command(ref) vs. feedback  —  {title_suffix}", fontsize=11)

    panels = [("roll_deg", "Roll [deg]"), ("pitch_deg", "Pitch [deg]")]
    legs = ["LEFT", "RIGHT"]

    for r, (field, ylabel) in enumerate(panels):
        for c, leg in enumerate(legs):
            ax = axes[r][c]
            sources = ankle_data[leg]

            fb = sources["fb"]
            cmd = sources["cmd"]

            if fb is not None:
                ax.plot(fb["time_s"], fb[field], linewidth=0.8, label="feedback")
            if cmd is not None:
                ax.plot(cmd["time_s"], cmd[field], "--", linewidth=0.8, color="tab:orange", label="command")
            if fb is not None or cmd is not None:
                ax.legend(fontsize=7, loc="upper right")

            ax.set_title(f"{leg} {ylabel}", fontsize=9)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.4, alpha=0.6)

    for c in range(2):
        axes[-1][c].set_xlabel("time [s]", fontsize=8)

    fig.tight_layout()


def main():
    parser = argparse.ArgumentParser(
        description="candump 로그의 앵클 모터 프레임으로 roll/pitch 커맨드 vs 피드백을 재구성한다."
    )
    parser.add_argument("log_file", type=Path, nargs="?", help="candump -l 로그 파일 (생략 시 logs/candump-*.log 중 최신)")
    parser.add_argument("-o", "--output", type=Path, default=None, help="출력 CSV 경로 (기본: <로그이름>_ankle.csv)")
    args = parser.parse_args()

    log_path = args.log_file or find_latest_log()
    if not log_path.exists():
        raise FileNotFoundError(f"로그 파일을 찾을 수 없습니다: {log_path}")

    output_path = args.output or log_path.with_name(log_path.stem + "_ankle.csv")

    ankle_data = reconstruct_ankle_joints(log_path)

    has_data = any(
        sources["cmd"] is not None or sources["fb"] is not None
        for sources in ankle_data.values()
    )
    if not has_data:
        raise RuntimeError("앵클 모터(ANKLE_A/ANKLE_B) 커맨드/피드백 프레임을 찾지 못했습니다.")

    rows_written = write_csv(output_path, ankle_data)
    print(f"{rows_written}행 기록 -> {output_path}")

    plot_ankle_joints(ankle_data, log_path.name)
    plt.show()


if __name__ == "__main__":
    main()
