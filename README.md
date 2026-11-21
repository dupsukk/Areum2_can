# Areum2_can

로봇 제어용 CAN 통신 프로세스 (Robstride 모터, 2채널 CAN). 상위 제어기와는 공유메모리(SysV)로 통신한다.

모터 구성(타입, CAN ID, shm 인덱스, 부호, 오프셋, 한계)은 **`robot.yaml` 하나가 유일한 진실 원천**이고,
`gen_robot.py` 가 이 파일에서 `src/main.cpp` 와 `inc/{robot_name}_config.h` 를 생성한다.

```
robot.yaml ──gen_robot.py──> inc/AReuMii_config.h   (표, 상수, with_motor, emplace_motor, shm_index_to_can_id)
                             src/main.cpp           (main: 소켓 열기, 모터 생성/초기화, 스레드 시작)
inc/can_runtime.hpp  ← 로봇에 무관한 고정 로직 (직접 수정하는 곳)
```

## 빌드 / 실행

```bash
make              # robot.yaml 또는 gen_robot.py 가 바뀌었으면 자동 재생성 후 빌드
./start.sh        # can0/can1 up, 빌드, candump 로깅, 별도 터미널에서 실행
```

- 생성기 단독 실행: `python3 gen_robot.py robot.yaml [--inc inc] [--src src]`
- 필요한 것: `python3`, `pyyaml`. 생성 파일은 git 으로 추적하지만, 새로 클론하면 스탬프(`.gen_stamp`)가 없어
  첫 `make` 에서 생성기가 한 번 돈다. 출력에 타임스탬프가 없어서 결과는 항상 같다(YAML 해시만 박힘).
- 생성 결과가 바뀌지 않았으면 파일을 다시 쓰지 않는다 (불필요한 재빌드 방지).

## robot.yaml

최상위:

| 필드 | 설명 |
|---|---|
| `robot_name` | 생성되는 헤더 이름 (`{robot_name}_config.h`) |
| `control_period_ns` | CAN 통신 주기 [ns]. 현재 4000000 (250Hz) |
| `shm_sync_period_us` | shm 동기화 스레드의 `usleep` 값 [us] |
| `shm_key` | SysV `shmget` key. POSIX 전환 시 `shm_name` 으로 교체 예정 |
| `can_interfaces` | CAN 인터페이스 목록 (`[can0, can1]`) |
| `actuators` | 액추에이터 목록 (아래) |

액추에이터 필드 (전부 필수):

| 필드 | 설명 |
|---|---|
| `name` | 대문자/숫자/`_`. C++ 상수 이름에 쓰인다 (`CAN_ID_<name>` 등) |
| `type` | `RS00` … `RS06`, `EL05` |
| `can_if` | `can_interfaces` 중 하나 |
| `can_id` | 1..0xFF, **전역 유일** (`Rx_handler` 가 인터페이스 구분 없이 ID 로 조회) |
| `shm` | 공유메모리 인덱스. 0..N-1 의 순열 |
| `sign` | `1` 또는 `-1`. 모터 → 관절(shm) 좌표계 방향 |
| `offset` | 기계적 오프셋 [rad]. **부호가 적용된 관절 좌표계** 값 |
| `min_pos`, `max_pos` | 위치 한계 [rad]. **shm 좌표계** (부호·오프셋 적용 후) |
| `vel_limit`, `torque_limit` | `init_motor_MIT` 에 전달되는 속도 [rad/s] / 토크 [Nm] 제한 |

생성기가 검증하는 것: 필드 누락/미지원 필드, 타입, `can_if`, `can_id` 범위/중복, `shm` 순열, `sign`, `min_pos < max_pos`,
제한값 양수. 하나라도 걸리면 아무것도 생성하지 않고 종료 코드 1.

타입별 `deque` 안의 순번(`VEC_INDEX_*`)은 YAML 에 적지 않는다. 생성기가 **shm 순서**로 계산한다
(`emplace_motors` 가 shm 인덱스 순서로 모터를 만들기 때문).

## 좌표계 규약 (방향 우선)

- `sign` 은 거울상으로 배치된 모터의 회전축을 통일하는 값이다. pos / vel / torque 에 모두 같은 부호가 적용된다.
- `offset` 은 pos 에만 적용된다. 거울상 모터에 **같은 offset 을 적으면 같은 관절 각도가 원점**이 된다.
  생성기가 모터 생성자에 `sign * offset` 을 넘기고(`pos_offset_mech`), 모터는 부호를 모른다.
  `calibrate()` 는 SPARQ 와 같이 `pos_offset -= pos_offset_mech` 로 반영한다.
- 한계(`min_pos`/`max_pos`)는 상위(shm) 좌표계 기준이다. 모터에 보내는 값은 `sign * (한계 처리된 pos)`.

```
shm 피드백 = sign * (raw 피드백 + pos_offset)          (vel, torque: sign * raw)
모터 명령  = sign * 한계처리(shm 명령)
```

한계 처리(`limit_pos_cmd`): 피드백이 `[min, max]` 안이면 명령을 **그대로 통과**시키고, 피드백이 한계 밖일 때만
바깥쪽 명령을 막는다 (`fb < min` 이면 `max(cmd, min)`, `fb > max` 이면 `min(cmd, max)`).

## 코드 구성

| 파일 | 역할 |
|---|---|
| `robot.yaml` | 모터 구성 (유일한 진실 원천) |
| `gen_robot.py` | YAML 검증 + `main.cpp` / `{robot_name}_config.h` 생성 |
| `inc/can_runtime.hpp` | 고정 로직: 스레드(`CAN_Comm_thread`, `update_Control_params`, `print_thread_func`), 신호 핸들러, `emplace_motors`, `init_motors_MIT`, shm↔모터 변환 |
| `inc/RobstrideMotor.hpp`, `src/RobstrideMotor.cpp` | 모터 클래스 / CAN 입출력 |
| `inc/Rx_handler.hpp` | 수신 프레임 파싱 및 모터별 피드백 기록 |
| `inc/Sharemem.hpp` | 공유메모리 (seqlock) |
| `inc/utils.hpp` | `RealTimeClock` |
| `inc/AReuMii_config.h`, `src/main.cpp` | **생성물 — 직접 수정 금지** (다음 생성 때 덮어써짐) |

생성되는 config 헤더의 주요 내용: 액추에이터별 `CAN_ID_*`, `SHM_INDEX_*`, `VEC_INDEX_*`, `CAN_IF_INDEX_*`, `SIGN_*`,
`MECH_OFFSET_*`, `MIN_POS_*`, `MAX_POS_*`, `VEL_LIMIT_*`, `TORQUE_LIMIT_*`, shm 인덱스로 조회하는 표
(`SHM_SIGN[]` 등), `with_motor()`(shm 인덱스 → 타입에 맞는 모터 객체), `emplace_motor()`,
`shm_index_to_can_id()`, 그리고 모든 shm 인덱스가 매핑되는지 보는 `static_assert`.
`can_runtime.hpp` 는 config 헤더를 먼저 include 하지 않으면 `#error` 로 막는다.

## 실행 흐름

1. `main()`: `SIGINT` 핸들러 등록, `can_interfaces` 의 소켓을 연다.
2. `emplace_motors()`: shm 인덱스 순서로 모터 객체를 만든다 (CAN 전송 없음).
3. `init_motors_MIT()`: 모터 문제 때문에 **두 번 초기화**한다 — 모터마다 `init → stop → 10ms`, 100ms 대기,
   모터마다 `init` 다시. 그 뒤 `main()` 이 10ms 더 쉰다.
4. 스레드 시작
   - `CAN_Comm_thread` (SCHED_FIFO 99): 수신 → 첫 사이클에 `calibrate()` → 이후 매 주기 모든 모터에 동일하게
     `write_updated_operation_frame()` (그리퍼 특수 처리 없음).
   - `update_Control_params`: shm 을 읽고, **피드백(모터→shm)을 먼저, 명령(shm→모터)을 나중에** 처리
     (한계 처리가 이번 주기의 피드백을 쓰기 때문), `shm_sync_period_us` 만큼 sleep.
   - `print_thread_func`: 0.5초마다 모터 상태 출력 (초기 확인용).


## 수정하는 방법

- 모터 추가/변경, 부호·오프셋·한계·제한값·주기·키 변경 → **`robot.yaml`** 만 수정하고 `make`.
- 런타임 로직(스레드, 초기화 순서, 변환 등) 수정 → **`inc/can_runtime.hpp`** 직접 수정.
- `main()` 수정 → **`gen_robot.py` 의 `MAIN_TEMPLATE`** 수정 후 재생성 (`@@SRC@@`, `@@SHA@@`, `@@CONFIG@@` 는 유지).
- 생성 결과의 형태(표, `with_motor` 등) 수정 → `gen_robot.py` 의 `build_config()`.
