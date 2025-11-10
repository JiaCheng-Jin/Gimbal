
#include "BMI088.hpp"
#include "can.h"
#include "Controller.hpp"
#include "Motor.hpp"
#include <cstring>

uint8_t rx_buffer[32];

IMU imu;
Controller rc{};


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
            memcpy(rc.rx_data, rx_buffer, 18 * sizeof(uint8_t));
            rc.handle();
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, 32);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan == &hcan1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x204) {
            motor.parse_can_msg_callback(rx_data);
        }
    }
}

