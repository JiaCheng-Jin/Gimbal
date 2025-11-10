
#include "bmi088.h"
#include "can.h"
#include "Controller.hpp"
#include "Motor.hpp"
#include "imu.h"
#include <cstring>

uint8_t controller_rx_buffer[32];
uint8_t can1_rx_buffer[8];

Controller rc{};
CAN_RxHeaderTypeDef can1_rx_header;

extern Motor gimbal_yaw_motor, gimbal_pitch_motor;


// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
//     if (htim == &htim6) {
//         imu.acc_calculate();
//         imu.gyro_calculate();
//         imu.update_orientation(1);
//     }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart3 && huart->RxEventType != HAL_UART_RXEVENT_HT) {
        if (Size == 18) {
            memcpy(rc.rx_data, controller_rx_buffer, 18 * sizeof(uint8_t));
            rc.handle();
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, controller_rx_buffer, 32);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1_rx_header, can1_rx_buffer);
        uint8_t id = can1_rx_header.StdId - 0x204;
        if (id == gimbal_yaw_motor.can_id_) {
            gimbal_yaw_motor.parse_can_msg_callback(can1_rx_buffer);
        }
        else if (id == gimbal_pitch_motor.can_id_) {
            gimbal_pitch_motor.parse_can_msg_callback(can1_rx_buffer);
        }
    }
}

