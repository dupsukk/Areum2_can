#!/usr/bin/env python3
"""robot.yaml -> inc/{robot_name}_config.h + src/main.cpp

사용법: python3 gen_robot.py robot.yaml [--inc inc] [--src src]

생성 전에 YAML 을 검증한다. 출력에는 타임스탬프를 넣지 않는다(재생성해도 diff 가 없도록).
로봇에 무관한 고정 로직은 inc/can_runtime.hpp 에 있고, 여기서는 로봇별 표와 그것을 쓰는 부분만 만든다.
"""
import argparse
import hashlib
import math
import pathlib
import re
import sys

import yaml

MOTOR_TYPES = ["RS00", "RS01", "RS02", "RS03", "RS04", "RS05", "RS06", "EL05"]
MODES = ["MIT", "PP"]
ACT_KEYS = ["name", "type", "can_if", "can_id", "shm", "sign", "offset", "min_pos", "max_pos", "mode"]
MIT_KEYS = ["vel_limit", "torque_limit"]                    # MIT 모터는 필수, PP 모터는 쓰지 않음 (적어도 되지만 무시됨)
PP_KEYS = ["pp_vel_max", "pp_acc", "pp_current_limit"]      # PP 모터는 필수, MIT 모터는 쓰지 않음
ACT_OPT_KEYS = MIT_KEYS + PP_KEYS
TOP_KEYS = ["robot_name", "control_period_ns", "shm_sync_period_us", "shm_key", "can_interfaces",
            "init_pos_tolerance_deg", "actuators"]
IDENT = re.compile(r"^[A-Z][A-Z0-9_]*$")
ROBOT_IDENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def load_and_validate(raw: bytes):
    d = yaml.safe_load(raw)
    errs = []
    if not isinstance(d, dict):
        return None, ["최상위가 매핑이 아님"]

    for k in TOP_KEYS:
        if k not in d:
            errs.append(f"최상위 필드 누락: {k}")
    for k in d:
        if k not in TOP_KEYS:
            errs.append(f"알 수 없는 최상위 필드: {k}")
    if errs:
        return None, errs

    if not (isinstance(d["robot_name"], str) and ROBOT_IDENT.match(d["robot_name"])):
        errs.append(f"robot_name 이 C++ 식별자가 아님: {d['robot_name']!r}")
    if not (is_int(d["control_period_ns"]) and d["control_period_ns"] > 0):
        errs.append("control_period_ns 는 양의 정수여야 함")
    if not (is_int(d["shm_sync_period_us"]) and d["shm_sync_period_us"] >= 0):
        errs.append("shm_sync_period_us 는 0 이상의 정수여야 함")
    if not is_int(d["shm_key"]):
        errs.append("shm_key 는 정수여야 함")

    if not (is_num(d["init_pos_tolerance_deg"]) and d["init_pos_tolerance_deg"] > 0):
        errs.append("init_pos_tolerance_deg 는 양수여야 함")

    ifs = d["can_interfaces"]
    if not (isinstance(ifs, list) and ifs and all(isinstance(x, str) and x for x in ifs)):
        errs.append("can_interfaces 는 비어 있지 않은 문자열 리스트여야 함")
        ifs = []
    elif len(set(ifs)) != len(ifs):
        errs.append(f"can_interfaces 중복: {ifs}")

    acts = d["actuators"]
    if not (isinstance(acts, list) and acts):
        errs.append("actuators 는 비어 있지 않은 리스트여야 함")
        return None, errs

    names, ids, shms = [], [], []
    for i, a in enumerate(acts):
        tag = f"actuators[{i}]"
        if not isinstance(a, dict):
            errs.append(f"{tag}: 매핑이 아님")
            continue
        tag = f"actuators[{i}] ({a.get('name', '?')})"
        for k in ACT_KEYS:
            if k not in a:
                errs.append(f"{tag}: 필드 누락 {k}")
        for k in a:
            if k not in ACT_KEYS and k not in ACT_OPT_KEYS:
                errs.append(f"{tag}: 알 수 없는 필드 {k}")
        if any(k not in a for k in ACT_KEYS):
            continue

        if not (isinstance(a["name"], str) and IDENT.match(a["name"])):
            errs.append(f"{tag}: name 은 대문자/숫자/_ 로 된 식별자여야 함")
        if a["type"] not in MOTOR_TYPES:
            errs.append(f"{tag}: type={a['type']!r} 는 {MOTOR_TYPES} 중 하나여야 함")
        if a["can_if"] not in ifs:
            errs.append(f"{tag}: can_if={a['can_if']!r} 가 can_interfaces 에 없음")
        if not (is_int(a["can_id"]) and 1 <= a["can_id"] <= 0xFF):
            errs.append(f"{tag}: can_id 는 1..0xFF 정수여야 함 (0 은 '없음' 표식, 피드백 ID 는 8비트)")
        if not is_int(a["shm"]):
            errs.append(f"{tag}: shm 은 정수여야 함")
        if a["sign"] not in (1, -1) or isinstance(a["sign"], bool):
            errs.append(f"{tag}: sign 은 1 또는 -1 이어야 함")
        for k in ("offset", "min_pos", "max_pos"):
            if not is_num(a[k]):
                errs.append(f"{tag}: {k} 는 유한한 숫자여야 함")
        if a["mode"] not in MODES:
            errs.append(f"{tag}: mode={a['mode']!r} 는 {MODES} 중 하나여야 함")
        for k in ACT_OPT_KEYS:
            if k not in a:
                if (a["mode"] == "MIT" and k in MIT_KEYS) or (a["mode"] == "PP" and k in PP_KEYS):
                    errs.append(f"{tag}: {a['mode']} 모터는 {k} 필드가 필요함")
            elif not (is_num(a[k]) and a[k] > 0):
                errs.append(f"{tag}: {k} 는 양수여야 함")
        if is_num(a["min_pos"]) and is_num(a["max_pos"]) and not a["min_pos"] < a["max_pos"]:
            errs.append(f"{tag}: min_pos < max_pos 여야 함")

        names.append(a["name"])
        ids.append(a["can_id"])
        shms.append(a["shm"])

    if len(set(names)) != len(names):
        errs.append("name 중복")
    # Rx_handler 가 CAN ID 하나로 모터를 찾으므로 인터페이스와 무관하게 전역 유일해야 한다.
    if len(set(ids)) != len(ids):
        errs.append("can_id 중복 (Rx_handler 는 인터페이스 구분 없이 ID 로 조회함)")
    if sorted(shms) != list(range(len(acts))):
        errs.append(f"shm 이 0..{len(acts) - 1} 순열이 아님: {sorted(shms)}")
    return d, errs


def cpp_double(v) -> str:
    s = repr(float(v))
    return s


def build_config(d, digest, src_name):
    acts = d["actuators"]
    n = len(acts)
    guard = f"{d['robot_name'].upper()}_CONFIG_H"

    # 타입별 deque 안의 순번 = shm 순서 (emplace_motors 의 emplace_back 이 shm 인덱스 순서로 돌기 때문)
    counters = {}
    for a in sorted(acts, key=lambda x: x["shm"]):
        a["_vec"] = counters.get(a["type"], 0)
        counters[a["type"]] = a["_vec"] + 1
    for a in acts:
        a["_if_idx"] = d["can_interfaces"].index(a["can_if"])

    L = []
    L.append(f"// 자동 생성 -- 고치지 말 것.  출처: {src_name}  (생성기: gen_robot.py)")
    L.append(f"// YAML sha256[:16] = {digest}")
    L.append(f"#ifndef {guard}")
    L.append(f"#define {guard}")
    L.append("")
    L.append("#define ROBOT_CONFIG_INCLUDED  // can_runtime.hpp 가 include 순서를 검사하는 용도")
    L.append("")
    L.append("#include <cstdint>")
    L.append("#include <tuple>")
    L.append('#include "RobstrideMotor.hpp"  // Motor_con, *_Vec')
    L.append("")
    L.append("// MIT: write_updated_operation_frame (pos/vel/Kp/Kd),  PP: write_pp_position (목표 위치만)")
    L.append("enum class ControlMode { MIT, PP };")
    L.append("")
    L.append(f'inline constexpr const char* ROBOT_NAME = "{d["robot_name"]}";')
    L.append(f'inline constexpr const char* ROBOT_YAML_SHA = "{digest}";')
    L.append(f"inline constexpr long CONTROL_PERIOD_NS = {d['control_period_ns']};")
    L.append(f"inline constexpr int SHM_SYNC_PERIOD_US = {d['shm_sync_period_us']};")
    L.append(f"inline constexpr int ROBOT_SHM_KEY = {d['shm_key']};")
    L.append(f"inline constexpr int MOTOR_NUM = {n};")
    L.append(f"inline constexpr int CAN_MAX_ID = {max(a['can_id'] for a in acts) + 1};  // 최대 can_id + 1 (Rx_handler 테이블 크기)")
    L.append(f"inline constexpr double INIT_POS_TOLERANCE_RAD = {cpp_double(math.radians(d['init_pos_tolerance_deg']))};  // 시작 시 영점 허용 편차 ({d['init_pos_tolerance_deg']} deg)")
    L.append(f"inline constexpr int CAN_INTERFACE_NUM = {len(d['can_interfaces'])};")
    names = ", ".join(f'"{x}"' for x in d["can_interfaces"])
    L.append(f"inline constexpr const char* CAN_INTERFACE_NAMES[CAN_INTERFACE_NUM] = {{{names}}};")
    L.append("")

    def block(title, fmt):
        L.append(f"// {title}")
        for a in acts:
            L.append(fmt(a))
        L.append("")

    w = max(len(a["name"]) for a in acts)
    pad = lambda a: " " * (w - len(a["name"]))
    block("CAN ID", lambda a: f"inline constexpr uint32_t CAN_ID_{a['name']}{pad(a)} = 0x{a['can_id']:02X};")
    block("shared memory index", lambda a: f"inline constexpr int SHM_INDEX_{a['name']}{pad(a)} = {a['shm']};")
    block("vector index (타입별 deque 내 순번)", lambda a: f"inline constexpr int VEC_INDEX_{a['name']}{pad(a)} = {a['_vec']};")
    block("CAN interface index (CAN_INTERFACE_NAMES 의 인덱스)", lambda a: f"inline constexpr int CAN_IF_INDEX_{a['name']}{pad(a)} = {a['_if_idx']};")
    block("방향 부호 (모터 -> 관절 좌표계)", lambda a: f"inline constexpr double SIGN_{a['name']}{pad(a)} = {cpp_double(a['sign'])};")
    block("기계적 오프셋 [rad] (부호 적용된 관절 좌표계)", lambda a: f"inline constexpr double MECH_OFFSET_{a['name']}{pad(a)} = {cpp_double(a['offset'])};")
    block("제어 모드", lambda a: f"inline constexpr ControlMode MODE_{a['name']}{pad(a)} = ControlMode::{a['mode']};")
    block("init_motor_MIT 제한 (속도 [rad/s], 토크 [Nm]). PP 모터는 쓰지 않음(0)", lambda a:
          f"inline constexpr double VEL_LIMIT_{a['name']}{pad(a)} = {cpp_double(a.get('vel_limit', 0.0))};\n"
          f"inline constexpr double TORQUE_LIMIT_{a['name']}{pad(a)} = {cpp_double(a.get('torque_limit', 0.0))};")
    block("위치 한계 [rad] (shm 좌표계)", lambda a:
          f"inline constexpr double MIN_POS_{a['name']}{pad(a)} = {cpp_double(a['min_pos'])};\n"
          f"inline constexpr double MAX_POS_{a['name']}{pad(a)} = {cpp_double(a['max_pos'])};")

    block("PP 파라미터 (속도 [rad/s], 가속도 [rad/s^2], 전류 제한 [A]). MIT 모터는 쓰지 않음(0)", lambda a:
          f"inline constexpr double PP_VEL_MAX_{a['name']}{pad(a)} = {cpp_double(a.get('pp_vel_max', 0.0))};\n"
          f"inline constexpr double PP_ACC_{a['name']}{pad(a)} = {cpp_double(a.get('pp_acc', 0.0))};\n"
          f"inline constexpr double PP_CURRENT_LIMIT_{a['name']}{pad(a)} = {cpp_double(a.get('pp_current_limit', 0.0))};")

    by_shm = sorted(acts, key=lambda x: x["shm"])
    L.append("// shm 인덱스로 조회하는 표 (can_runtime.hpp 의 루프가 사용)")
    for arr, pre, ctype in (("SHM_MODE", "MODE", "ControlMode"), ("SHM_SIGN", "SIGN", "double"), ("SHM_MECH_OFFSET", "MECH_OFFSET", "double"),
                            ("SHM_MIN_POS", "MIN_POS", "double"), ("SHM_MAX_POS", "MAX_POS", "double"),
                            ("SHM_VEL_LIMIT", "VEL_LIMIT", "double"), ("SHM_TORQUE_LIMIT", "TORQUE_LIMIT", "double"),
                            ("SHM_PP_VEL_MAX", "PP_VEL_MAX", "double"), ("SHM_PP_ACC", "PP_ACC", "double"),
                            ("SHM_PP_CURRENT_LIMIT", "PP_CURRENT_LIMIT", "double"),
                            ("SHM_CAN_IF_INDEX", "CAN_IF_INDEX", "int")):
        items = ", ".join(f"{pre}_{a['name']}" for a in by_shm)
        L.append(f"inline constexpr {ctype} {arr}[MOTOR_NUM] = {{{items}}};")
    L.append("")
    L.append("// shm 인덱스 -> 해당 타입의 모터 객체. 타입이 달라 배열로 못 다루므로 switch 로 골라 f 에 넘긴다.")
    L.append("template <class F>")
    L.append("inline void with_motor(Motor_con& motors, int shm_index, F&& f) {")
    L.append("    switch (shm_index) {")
    for a in by_shm:
        L.append(f"        case SHM_INDEX_{a['name']}:{pad(a)} f(std::get<{a['type']}_Vec>(motors)[VEC_INDEX_{a['name']}]); return;")
    L.append("        default: return;")
    L.append("    }")
    L.append("}")
    L.append("")
    L.append("// 초기화용: shm 인덱스에 해당하는 타입의 deque 에 모터를 추가한다. (호출 순서 == VEC_INDEX 순서여야 하므로 shm 순서로 호출할 것)")
    L.append("inline void emplace_motor(Motor_con& motors, int shm_index, int can_socket, int can_id, double pos_offset_mech) {")
    L.append("    switch (shm_index) {")
    for a in by_shm:
        L.append(f"        case SHM_INDEX_{a['name']}:{pad(a)} std::get<{a['type']}_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;")
    L.append("        default: return;")
    L.append("    }")
    L.append("}")
    L.append("")
    L.append("// SHM_INDEX -> CAN ID. 표에 없는 인덱스는 0 (유효하지 않은 ID).")
    L.append("constexpr uint32_t shm_index_to_can_id(int shm_index) {")
    L.append("    switch (shm_index) {")
    for a in sorted(acts, key=lambda x: x["shm"]):
        L.append(f"        case SHM_INDEX_{a['name']}:{pad(a)} return CAN_ID_{a['name']};")
    L.append("        default: return 0;")
    L.append("    }")
    L.append("}")
    L.append("")
    L.append("constexpr bool all_shm_index_mapped() {")
    L.append("    for (int i = 0; i < MOTOR_NUM; ++i) if (shm_index_to_can_id(i) == 0) return false;")
    L.append("    return true;")
    L.append("}")
    L.append('static_assert(all_shm_index_mapped(), "shm_index_to_can_id 가 0..MOTOR_NUM-1 을 모두 덮지 못함");')
    L.append("")
    L.append(f"#endif  // {guard}")
    L.append("")
    return "\n".join(L)


MAIN_TEMPLATE = r'''// 자동 생성 -- 고치지 말 것.  출처: @@SRC@@  (생성기: gen_robot.py)
// YAML sha256[:16] = @@SHA@@
#include "@@CONFIG@@"
#include "can_runtime.hpp"

int main() {
    signal(SIGINT, signal_handler);

    std::vector<int> can_interface;
    for (const char* name : CAN_INTERFACE_NAMES) {
        int s = init_can(name);
        if (s < 0) return -1;
        can_interface.push_back(s);
    }

    emplace_motors(can_interface);
    init_motors();
    usleep(10000);

    // 여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,    &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임.
    pthread_create(&shm_t,   NULL, update_Control_params, nullptr);

    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t,   nullptr);

    if (g_startup_fault) return -1;  // 시작 시 영점 편차 초과 (CAN_Comm_thread 가 설정)
    return 0;
}
'''


def build_main(d, digest, src_name, config_name):
    return (MAIN_TEMPLATE
            .replace("@@SRC@@", src_name)
            .replace("@@SHA@@", digest)
            .replace("@@CONFIG@@", config_name))


def write_if_changed(path: pathlib.Path, text: str):
    if path.exists() and path.read_text() == text:
        return False
    path.write_text(text)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml")
    ap.add_argument("--inc", default="inc")
    ap.add_argument("--src", default="src")
    args = ap.parse_args()

    src = pathlib.Path(args.yaml)
    raw = src.read_bytes()
    d, errs = load_and_validate(raw)
    if errs:
        for e in errs:
            print(f"[gen_robot] ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    digest = hashlib.sha256(raw).hexdigest()[:16]
    config_name = f"{d['robot_name']}_config.h"

    cfg_text = build_config(d, digest, src.name)  # _vec / _if_idx 채움
    main_text = build_main(d, digest, src.name, config_name)

    inc_dir, src_dir = pathlib.Path(args.inc), pathlib.Path(args.src)
    inc_dir.mkdir(parents=True, exist_ok=True)
    src_dir.mkdir(parents=True, exist_ok=True)
    c1 = write_if_changed(inc_dir / config_name, cfg_text)
    c2 = write_if_changed(src_dir / "main.cpp", main_text)
    print(f"[gen_robot] {config_name}{'' if c1 else ' (변경 없음)'}, main.cpp{'' if c2 else ' (변경 없음)'} "
          f"({len(d['actuators'])}개 액추에이터, sha={digest})")


if __name__ == "__main__":
    main()
