/**
 * @file    app.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "app.hpp"

#include "can.h"
#include "chassis.hpp"
#include "cmsis_os2.h"
#include "device.hpp"
#include "tim.h"
#include "drivers/dji.hpp"
#include <bsp/can_driver.h>

using namespace motor_if;
#define init(_)

#include "init-macros.hpp"

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    for (auto& ctrl : motor_wheel_ctrl)
        ctrl->update();

    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
extern "C" void Init(void* argument)
{
    /* 初始化代码 */
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    Device_Init();

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);

    motor_wheel[0] = new_(motors::DJIMotor,
                          { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 1 });

    /* 初始化完成后退出线程 */
    osThreadExit();
}