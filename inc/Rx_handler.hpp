#include "RobstrideMotor.hpp"

#ifndef RX_HANDLER_HPP
#define RX_HANDLER_HPP

class Rx_handler{
    public:
    
    Rx_handler();
    std::tuple<uint32_t, uint32_t, float,float,float,float> parse_Rx_frame(struct can_frame* frame);
    int write_Fb();
};

#endif