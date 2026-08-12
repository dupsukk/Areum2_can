#!/usr/bin/env python3

import argparse
import csv
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# Type 03 상태 프레임의 실제 오류 비트
FAULT_BITS = {
    0: "Power-chip overcurrent",
    1: "Power-chip overtemperature",
    2: "Power-board overtemperature",
    3: "Sampled total overcurrent",
    4: "VBUS overvoltage",
    5: "VBUS undervoltage",
    6: "VMBUS overvoltage",
    7: "VMBUS undervoltage",
    8: "24V overvoltage",
    9: "24V undervoltage",
    10: "12V overvoltage",
    11: "12V undervoltage",
    12: "Output branch overcurrent",
    13: "INA238 fault",
    14: "Soft-start abnormal",
}

FAULT_MASK = sum(1 << bit for bit in FAULT_BITS)


# 다음 두 candump 형식을 모두 지원
#
# (1752380000.123456) can4 03AA0000#0ADB0AD519000000
#
# (000.000364) can4 03AA0000   [8]  0A DB 0A D5 19 00 00 00

HASH_PATTERN = re.compile(
    r"^\s*\((?P<timestamp>[\d.]+)\)\s+"
    r"(?P<interface>\S+)\s+"
    r"(?P<can_id>[0-9A-Fa-f]+)"
    r"(?:##?[0-9A-Fa-f])?#"
    r"(?P<data>[0-9A-Fa-f]*)"
)

BRACKET_PATTERN = re.compile(
    r"^\s*\((?P<timestamp>[\d.]+)\)\s+"
    r"(?P<interface>\S+)\s+"
    r"(?P<can_id>[0-9A-Fa-f]+)\s+"
    r"\[(?P<dlc>\d+)\]\s+"
    r"(?P<data>(?:[0-9A-Fa-f]{2}\s*)+)"
)


def parse_candump_line(line: str):
    """candump 로그 한 줄에서 timestamp, CAN ID, data bytes를 반환한다."""

    match = HASH_PATTERN.match(line)

    if match:
        timestamp = float(match.group("timestamp"))
        can_id = int(match.group("can_id"), 16)
        data_hex = match.group("data")

        if len(data_hex) % 2 != 0:
            return None

        try:
            data = bytes.fromhex(data_hex)
        except ValueError:
            return None

        return timestamp, can_id, data

    match = BRACKET_PATTERN.match(line)

    if match:
        timestamp = float(match.group("timestamp"))
        can_id = int(match.group("can_id"), 16)

        try:
            data = bytes.fromhex(match.group("data"))
        except ValueError:
            return None

        return timestamp, can_id, data

    return None


def decode_faults(status: int) -> list[str]:
    return [
        name
        for bit, name in FAULT_BITS.items()
        if status & (1 << bit)
    ]


def _columns_from_rows(rows: list[dict], dtypes: dict) -> dict | None:
    """dict 리스트를 컬럼별 numpy 배열 dict로 변환한다. rows가 비어있으면 None."""

    if not rows:
        return None

    return {
        key: np.array([row[key] for row in rows], dtype=dtypes.get(key))
        for key in rows[0]
    }


def parse_powerboard_log(
    log_path: Path,
    power_id: int,
) -> tuple[dict | None, dict | None]:

    status_rows = []
    branch_rows = []

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line_number, line in enumerate(log_file, start=1):
            parsed = parse_candump_line(line)

            if parsed is None:
                continue

            timestamp, can_id, data = parsed

            communication_type = (can_id >> 24) & 0x1F
            frame_power_id = (can_id >> 16) & 0xFF

            if frame_power_id != power_id:
                continue

            # Type 03: 전체 전류, 전압, 온도, Fault
            if communication_type == 0x03 and len(data) >= 8:
                total_current_a = (can_id & 0xFFFF) / 100.0

                battery_voltage_v = int.from_bytes(
                    data[0:2],
                    byteorder="big",
                    signed=False,
                ) / 100.0

                motor_voltage_v = int.from_bytes(
                    data[2:4],
                    byteorder="big",
                    signed=False,
                ) / 100.0

                temperature_c = data[4]

                status = int.from_bytes(
                    data[5:8],
                    byteorder="big",
                    signed=False,
                )

                fault_status = status & FAULT_MASK

                status_rows.append({
                    "line": line_number,
                    "timestamp": timestamp,
                    "total_current_a": total_current_a,
                    "battery_voltage_v": battery_voltage_v,
                    "motor_voltage_v": motor_voltage_v,
                    "temperature_c": temperature_c,
                    "status": status,
                    "fault_status": fault_status,
                    "faults": ", ".join(decode_faults(status)),
                    "12v_on": bool(status & (1 << 20)),
                    "soft_start_on": bool(status & (1 << 21)),
                    "vmbus_on": bool(status & (1 << 22)),
                    "24v_on": bool(status & (1 << 23)),
                })

            # Type 04: 네 개 모터 분기 전류
            elif communication_type == 0x04 and len(data) >= 8:
                branch_rows.append({
                    "line": line_number,
                    "timestamp": timestamp,
                    "AL_current_a": int.from_bytes(
                        data[0:2], "big"
                    ) / 100.0,
                    "AR_current_a": int.from_bytes(
                        data[2:4], "big"
                    ) / 100.0,
                    "LL_current_a": int.from_bytes(
                        data[4:6], "big"
                    ) / 100.0,
                    "LR_current_a": int.from_bytes(
                        data[6:8], "big"
                    ) / 100.0,
                })

    status_dtypes = {
        "line": np.int64,
        "timestamp": np.float64,
        "total_current_a": np.float64,
        "battery_voltage_v": np.float64,
        "motor_voltage_v": np.float64,
        "temperature_c": np.float64,
        "status": np.int64,
        "fault_status": np.int64,
        "faults": object,
        "12v_on": bool,
        "soft_start_on": bool,
        "vmbus_on": bool,
        "24v_on": bool,
    }
    branch_dtypes = {
        "line": np.int64,
        "timestamp": np.float64,
        "AL_current_a": np.float64,
        "AR_current_a": np.float64,
        "LL_current_a": np.float64,
        "LR_current_a": np.float64,
    }

    status_cols = _columns_from_rows(status_rows, status_dtypes)
    branch_cols = _columns_from_rows(branch_rows, branch_dtypes)

    first_timestamps = []

    if status_cols is not None:
        first_timestamps.append(status_cols["timestamp"][0])

    if branch_cols is not None:
        first_timestamps.append(branch_cols["timestamp"][0])

    if first_timestamps:
        first_timestamp = min(first_timestamps)

        if status_cols is not None:
            status_cols["time_s"] = status_cols["timestamp"] - first_timestamp

        if branch_cols is not None:
            branch_cols["time_s"] = branch_cols["timestamp"] - first_timestamp

    return status_cols, branch_cols


def find_fault_edges(status_cols: dict) -> dict:
    """Fault가 0에서 1로 바뀌는 행만 추출한다."""

    fault_status = status_cols["fault_status"]

    previous = np.zeros_like(fault_status)
    previous[1:] = fault_status[:-1]
    newly_asserted = fault_status & ~previous

    edge_idx = np.nonzero(newly_asserted != 0)[0]

    return {key: values[edge_idx] for key, values in status_cols.items()}


def print_summary(
    status_cols: dict | None,
    branch_cols: dict | None,
):
    if status_cols is None:
        print("Type 03 상태 프레임을 찾지 못했습니다.")
        return

    time_s = status_cols["time_s"]

    print("\n=== Type 03 요약 ===")
    print(f"샘플 수       : {len(time_s)}")
    print(
        f"측정 시간      : "
        f"{time_s[-1]:.6f} s"
    )
    print(
        f"VBUS 범위      : "
        f"{status_cols['battery_voltage_v'].min():.2f} ~ "
        f"{status_cols['battery_voltage_v'].max():.2f} V"
    )
    print(
        f"VMBUS 범위     : "
        f"{status_cols['motor_voltage_v'].min():.2f} ~ "
        f"{status_cols['motor_voltage_v'].max():.2f} V"
    )
    print(
        f"전체 전류 범위 : "
        f"{status_cols['total_current_a'].min():.2f} ~ "
        f"{status_cols['total_current_a'].max():.2f} A"
    )

    if len(time_s) >= 2:
        intervals = np.diff(time_s)

        print(
            f"중앙 샘플 간격 : "
            f"{np.median(intervals) * 1000:.3f} ms"
        )
        print(
            f"최대 샘플 간격 : "
            f"{np.max(intervals) * 1000:.3f} ms"
        )

    fault_edges = find_fault_edges(status_cols)
    n_edges = len(fault_edges["time_s"])

    print("\n=== Fault 발생 시점 ===")

    if n_edges == 0:
        print("Fault 비트가 새로 발생한 시점이 없습니다.")
    else:
        for i in range(n_edges):
            faults = fault_edges["faults"][i] or "Unknown fault"

            print(
                f"t={fault_edges['time_s'][i]:.6f} s | "
                f"I={fault_edges['total_current_a'][i]:.2f} A | "
                f"VBUS={fault_edges['battery_voltage_v'][i]:.2f} V | "
                f"VMBUS={fault_edges['motor_voltage_v'][i]:.2f} V | "
                f"{faults}"
            )

    if branch_cols is not None:
        print("\n=== Type 04 분기 전류 최대값 ===")

        for column in [
            "AL_current_a",
            "AR_current_a",
            "LL_current_a",
            "LR_current_a",
        ]:
            print(f"{column}: {branch_cols[column].max():.2f} A")


def add_fault_lines(ax, fault_edges: dict):
    n_edges = len(fault_edges["time_s"])

    if n_edges == 0:
        return

    for i in range(n_edges):
        label = "Fault 발생" if i == 0 else None

        ax.axvline(
            fault_edges["time_s"][i],
            linestyle="--",
            linewidth=1,
            alpha=0.7,
            label=label,
        )


def plot_status(
    status_cols: dict,
    output_prefix: Path,
    *,
    save: bool = True,
    extra_current_series: dict | None = None,
):
    """save=False면 파일로 저장하지 않고 figure를 열어둔 채로 반환한다
    (호출부에서 plt.show()로 한꺼번에 띄우는 용도, 예: plot_run.py).

    extra_current_series: {label: {"time_s": array, "total_bus_current_a": array}} 형태로
    주면, 전류 그래프에 실측치와 겹쳐서 점선으로 그려준다 (예: 모델 계산 버스전류 비교용)."""

    fault_edges = find_fault_edges(status_cols)

    # 전압 그래프
    fig, ax = plt.subplots(figsize=(14, 6))

    ax.plot(
        status_cols["time_s"],
        status_cols["battery_voltage_v"],
        label="VBUS",
        linewidth=1,
    )
    ax.plot(
        status_cols["time_s"],
        status_cols["motor_voltage_v"],
        label="VMBUS",
        linewidth=1,
    )

    add_fault_lines(ax, fault_edges)

    ax.set_title("Power-board voltage")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Voltage [V]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    voltage_path = None
    if save:
        voltage_path = output_prefix.with_name(
            output_prefix.name + "_voltage.png"
        )
        fig.savefig(voltage_path, dpi=180)
        plt.close(fig)

    # 전체 전류 그래프
    fig, ax = plt.subplots(figsize=(14, 6))

    ax.plot(
        status_cols["time_s"],
        status_cols["total_current_a"],
        label="Total motor current (measured)",
        linewidth=1,
    )

    for label, series in (extra_current_series or {}).items():
        ax.plot(
            series["time_s"],
            series["total_bus_current_a"],
            "--",
            label=label,
            linewidth=1,
        )

    add_fault_lines(ax, fault_edges)

    ax.set_title("Power-board total current")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Current [A]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    current_path = None
    if save:
        current_path = output_prefix.with_name(
            output_prefix.name + "_current.png"
        )
        fig.savefig(current_path, dpi=180)
        plt.close(fig)

    # 온도 그래프
    fig, ax = plt.subplots(figsize=(14, 5))

    ax.plot(
        status_cols["time_s"],
        status_cols["temperature_c"],
        label="Board temperature",
        linewidth=1,
    )

    add_fault_lines(ax, fault_edges)

    ax.set_title("Power-board temperature")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Temperature [°C]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    temperature_path = None
    if save:
        temperature_path = output_prefix.with_name(
            output_prefix.name + "_temperature.png"
        )
        fig.savefig(temperature_path, dpi=180)
        plt.close(fig)

    return voltage_path, current_path, temperature_path


def plot_branches(
    branch_cols: dict | None,
    output_prefix: Path,
    *,
    save: bool = True,
):
    if branch_cols is None:
        return None

    fig, ax = plt.subplots(figsize=(14, 6))

    for column in [
        "AL_current_a",
        "AR_current_a",
        "LL_current_a",
        "LR_current_a",
    ]:
        ax.plot(
            branch_cols["time_s"],
            branch_cols[column],
            label=column.replace("_current_a", ""),
            linewidth=1,
        )

    ax.set_title("Power-board branch currents")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Current [A]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    branch_path = None
    if save:
        branch_path = output_prefix.with_name(
            output_prefix.name + "_branches.png"
        )
        fig.savefig(branch_path, dpi=180)
        plt.close(fig)

    return branch_path


def _write_csv(path: Path, columns: dict):
    keys = list(columns.keys())

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(keys)
        writer.writerows(zip(*(columns[key] for key in keys)))


def main():
    parser = argparse.ArgumentParser(
        description="Parse and plot RobStride power-board CAN logs."
    )

    parser.add_argument(
        "log_file",
        type=Path,
        help="candump -l 로그 파일",
    )
    parser.add_argument(
        "--power-id",
        type=lambda value: int(value, 0),
        default=0xAA,
        help="POWER_ID. 기본값: 0xAA",
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=Path("powerboard"),
        help="출력 파일 이름 접두어",
    )

    args = parser.parse_args()

    if not args.log_file.exists():
        raise FileNotFoundError(
            f"로그 파일을 찾을 수 없습니다: {args.log_file}"
        )

    status_cols, branch_cols = parse_powerboard_log(
        args.log_file,
        args.power_id,
    )

    if status_cols is None and branch_cols is None:
        raise RuntimeError(
            "POWER_ID와 일치하는 Type 03/04 프레임을 찾지 못했습니다."
        )

    print_summary(status_cols, branch_cols)

    if status_cols is not None:
        status_csv = args.output_prefix.with_name(
            args.output_prefix.name + "_status.csv"
        )
        _write_csv(status_csv, status_cols)

        plot_paths = plot_status(
            status_cols,
            args.output_prefix,
        )

        print("\n생성된 파일:")
        print(f"- {status_csv}")

        for path in plot_paths:
            print(f"- {path}")

    if branch_cols is not None:
        branch_csv = args.output_prefix.with_name(
            args.output_prefix.name + "_branches.csv"
        )
        _write_csv(branch_csv, branch_cols)

        branch_plot = plot_branches(
            branch_cols,
            args.output_prefix,
        )

        print(f"- {branch_csv}")
        print(f"- {branch_plot}")


if __name__ == "__main__":
    main()
