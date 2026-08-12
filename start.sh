CAN_INT_0="can0"
CAN_INT_1="can1"
BITRATE="1000000"
SAMPLE_POINT="0.875"
LOG_DIR="logs"

echo "--- $CAN_INT 인터페이스 초기화 시작 ---"

sudo ip link set ${CAN_INT_0} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT}
sudo ip link set ${CAN_INT_1} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT}
#sudo ip link set can4 up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT}
cansend can4 060000AA#0100000000000000

mkdir -p "${LOG_DIR}"
echo "--- candump 로깅 시작 (${CAN_INT_0}, ${CAN_INT_1}/) ---"
( cd "${LOG_DIR}" && exec candump -l ${CAN_INT_0} ${CAN_INT_1}  ) &
CANDUMP_PID=$!
trap 'echo "--- candump 종료 중 ---"; kill ${CANDUMP_PID} 2>/dev/null; wait ${CANDUMP_PID} 2>/dev/null' EXIT

echo "CAN 통신 프로그램을 실행합니다."
#./Areum2_can > /dev/null 2>&1 &
./tellus_can
