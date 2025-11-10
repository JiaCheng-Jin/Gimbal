
#include "task.hpp"
#include "Motor.hpp"
#include "Controller.hpp"
#include "can.h"
#include "cmsis_os2.h"

extern Controller rc;
extern bool stop_flag;

uint32_t can_tx_mailbox;
uint8_t can_rx_data[8];
uint8_t m6020_1_4_tx_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t m6020_5_8_tx_data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
CAN_TxHeaderTypeDef m6020_1_4_tx_header = {   .StdId = 0x1FE,
                                              .ExtId = 0x000,
                                              .IDE = CAN_ID_STD,
                                              .RTR = CAN_RTR_DATA,
                                              .DLC = 8,
                                              .TransmitGlobalTime = DISABLE };
CAN_TxHeaderTypeDef m6020_5_8_tx_header = {   .StdId = 0x2FE,
                                              .ExtId = 0x000,
                                              .IDE = CAN_ID_STD,
                                              .RTR = CAN_RTR_DATA,
                                              .DLC = 8,
                                              .TransmitGlobalTime = DISABLE };

Motor gimbal_yaw_motor(1, Motor::MotorType::GM6020, 16.8f, m6020_1_4_tx_data,
    PID(198, 0.4, 57, 30, 3000, 0.05),
    PID(0.022f, 0, 0.011f, 0, 3.f, 0.03));
Motor gimbal_pitch_motor(2, Motor::MotorType::GM6020, 16.8f, m6020_1_4_tx_data,
    PID(198, 0.4, 57, 30, 3000, 0.05),
    PID(0.022f, 0, 0.011f, 0, 3.f, 0.03));


osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
    .name = "mainTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
[[noreturn]] void main_task(void* params) {
    while (true) {
        // 根据遥控器控制电机
        const auto ctrl_frame = rc.data;
        // 急停 + 速度控制
        stop_flag = ctrl_frame.S2_ == Controller::Position::DOWN;
        float yaw_spd = ctrl_frame.left_joystick_x_ * 1000;
        float pitch_spd = ctrl_frame.left_joystick_y_ * 1000;
        // 死区设置
        if (yaw_spd > -100 && yaw_spd < 100) {
            yaw_spd = 0;
        }
        if ( pitch_spd > -100 && pitch_spd < 100 ) {
            pitch_spd = 0;
        }
        gimbal_yaw_motor.set_speed(yaw_spd);
        gimbal_pitch_motor.set_speed(pitch_spd);

        // 发送Can包
        gimbal_yaw_motor.handle();
        gimbal_pitch_motor.handle();
        HAL_CAN_AddTxMessage(&hcan1, &m6020_1_4_tx_header, m6020_1_4_tx_data, &can_tx_mailbox);
        osDelay(1);
    }
}


void register_tasks() {
    mainTaskHandle = osThreadNew(main_task, nullptr, &mainTask_attributes);
}



