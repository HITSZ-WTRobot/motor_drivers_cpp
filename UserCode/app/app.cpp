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
#include "dji.hpp"
#include "can_driver.h"

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

    // enable controllers
    for (auto& ctrl : motor_wheel_ctrl)
        if (!ctrl->enable())
            Error_Handler();

    float a = 2.0f;
    for (;;)
    {
        a += 1;
        motor_wheel_ctrl[0]->setRef(100.0f + a);
        osDelay(1000);
        motor_wheel_ctrl[0]->setRef(0.0f);
        osDelay(1000);
    }

    /* 初始化完成后退出线程 */
    osThreadExit();
}