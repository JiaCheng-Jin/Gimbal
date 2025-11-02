//
// Created by a2005 on 2025/10/25.
//

#ifndef INC_1012_SPI_CONTROLLER_HPP
#define INC_1012_SPI_CONTROLLER_HPP

#include <main.h>
#include <usart.h>

class Controller {
public:
    enum class Position { UP = 1, DOWN = 2, MID = 3 };
    struct RcData {
        float left_joystick_x_, left_joystick_y_;
        float right_joystick_x_, right_joystick_y_;
        Position S1_, S2_;
    };
    uint8_t rx_data[32];
    RcData data;

private:
    uint32_t last_tick_ = 0;

public:
    explicit Controller();
    bool test_connection() const;
    void handle();
};

#endif //INC_1012_SPI_CONTROLLER_HPP
