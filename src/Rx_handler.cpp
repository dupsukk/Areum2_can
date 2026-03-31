#include "Rx_handler.hpp"

static constexpr float CAN_DATA_ZERO_SCALE = 1.0f / 32767.0f;
static constexpr float POS_GAIN = M_PI*4;

Rx_handler::Rx_handler(){

}

std::tuple<uint32_t, uint32_t, float,float,float,float> Rx_handler::parse_Rx_frame(struct can_frame* frame){  // 피드백 프레임을 적절히 맞춰주는. 29th bit of can id is error flag. i think we can use it 
    //frame->can_id
    uint32_t motor_id = (frame->can_id>>8)&0xff;       // 데이터는 각각 1바이트, 2바이트지만 메모리 개미눈꼽만큼 아끼기보단 속도 선택 
    uint32_t err_st = (frame->can_id>>16)&0b00111111;     // 에러가 났다면 비트 내부에 에러가 들어 있을건데 밖에서 읽으라지. [[unlikely]]로 에러 처리 부분을 떼 둘 것

    uint16_t p_raw = (frame->data[0] << 8) | frame->data[1];
    uint16_t v_raw = (frame->data[2] << 8) | frame->data[3];
    uint16_t t_raw = (frame->data[4] << 8) | frame->data[5];

    // (Raw / 32767.0 - 1.0) * GAIN
    float pos = (static_cast<float>(p_raw) * CAN_DATA_ZERO_SCALE - 1.0f) * POS_GAIN;
    float vel = (static_cast<float>(v_raw) * CAN_DATA_ZERO_SCALE - 1.0f) ; // 게인 추가
    float torq = (static_cast<float>(t_raw) * CAN_DATA_ZERO_SCALE - 1.0f);  // 나눗셈 대신 곱셈을 사용할 것이 권장됨 
    float temp = static_cast<float>((frame->data[6] << 8) | frame->data[7]) * 0.1f;
    // 근데 여기서 다 처리하려면 밖에서 뭐 탬플릿 받아오고 해야하는데 귀찮으니까 로우 데이터 던지고 밖에서 처리? <<솔직히 킹쁘지 않음 
    return {motor_id , err_st,pos,vel,torq,temp};
}

int Rx_handler::write_Fb(){ // 각 모터의 아이디에 맞는 피드백 구조체에 데이터를 쓰는 함수임 << 이거? 내가봤을때 모터 함수로 편입시켜야 함 

}