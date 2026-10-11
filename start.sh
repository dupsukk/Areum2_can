CAN_INT_0="can0"
CAN_INT_1="can1"
BITRATE="1000000"
SAMPLE_POINT="0.875"
LOG_DIR="log"

echo "--- $CAN_INT 인터페이스 초기화 시작 ---"

sudo ip link set ${CAN_INT_0} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT}
sudo ip link set ${CAN_INT_1} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT}


echo "프로젝트 빌드 상태를 확인합니다..."
make

# 나중에 디버깅용. log 디렉토리가 없으면 만들고, 그 안에 candump-<타임스탬프>.log 로 남는다.
mkdir -p "${LOG_DIR}"
( cd "${LOG_DIR}" && exec candump -l ${CAN_INT_0} ${CAN_INT_1} ) &
CANDUMP_PID=$!

# 이 셸을 Ctrl+C 로 끄면 로깅도 같이 멈춘다.
trap 'kill ${CANDUMP_PID} 2>/dev/null' INT TERM EXIT

# 이전 실행에서 남은 공유메모리(key 13563267)를 제거한다. 남아 있으면 이전 목표 위치가 그대로 읽혀 PP 모터가 시작과 동시에 움직일 수 있다.
# 없으면 에러가 나므로 무시. 새 세그먼트는 Areum2_can 이 0으로 초기화해서 만든다.
SHM_KEY="13563267"
ipcrm -M ${SHM_KEY} 2>/dev/null && echo "공유메모리(${SHM_KEY}) 초기화" || echo "공유메모리(${SHM_KEY}) 없음 - 새로 생성"

echo "CAN 통신 프로그램을 별도 터미널에서 실행합니다."
#./Areum2_can > /dev/null 2>&1 &
gnome-terminal -- ./Areum2_can

echo "로깅 중... 종료하려면 Ctrl+C"
wait ${CANDUMP_PID}
