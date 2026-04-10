#ifndef RX_HANDLER_HPP
#define RX_HANDLER_HPP

#include "RobstrideMotor.hpp"


using Fb_f = void (*) (void* ,float,float,float,float) ;


class Rx_handler{
    private:
    Fb_f jump_table[32] = {nullptr};
    void* Motor_object[32] = {nullptr} ;

    template <typename Tuple>
    void init_helper(Tuple& t);

    template <typename V>
    void register_vector(V& vec);

    public:
    //const Motor_con MC;


    Rx_handler(Motor_con& MC);
    std::tuple<uint32_t, uint32_t, float,float,float,float> parse_Rx_frame(struct can_frame* frame);

    void Write_Fb(int id , float pos, float vel , float torq , float temp){
        if (id >= 0 && id < 32 && jump_table[id] && Motor_object[id]) {
            jump_table[id](Motor_object[id] , pos, vel, torq , temp);
        }
    }
};

#endif