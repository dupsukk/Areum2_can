#include "RobstrideMotor.hpp"

#ifndef RX_HANDLER_HPP
#define RX_HANDLER_HPP

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
    
    Rx_handler(const Motor_con MC);
    std::tuple<uint32_t, uint32_t, float,float,float,float> parse_Rx_frame(struct can_frame* frame);
};

#endif