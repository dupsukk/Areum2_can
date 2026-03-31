CAN_INT_0="can0"
#CAN_INT_1="can1"
BITRATE="1000000"
SAMPLE_POINT="0.875"

echo "--- $CAN_INT 인터페이스 초기화 시작 ---"

sudo ip link set ${CAN_INT_0} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT}


echo "프로젝트 빌드 상태를 확인합니다..."
make

echo "CAN 통신 프로그램을 실행합니다."
#./Areum2_can > /dev/null 2>&1 &
./Areum2_can > output.log 2>&1 &  # TODO : 이거 잘못되면 디스크 가득차는거 한순간임. 뭔가 조치를 취할 것 