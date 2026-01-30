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
#include "macros.hpp"
#include "motor_trajectory.hpp"

controllers::MotorVelController* ctrl;

template class trajectory::MotorTrajectory<1>;
trajectory::MotorTrajectory<1>* traj;

size_t prescaler = 0;

extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    ++prescaler;
    if (prescaler == 2)
    {
        traj->errorUpdate();
        prescaler = 0;
    }
    traj->controllerUpdate();

    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);

    service::Watchdog::EatAll();
}

extern "C" void TIM_Callback_200Hz(TIM_HandleTypeDef* htim)
{
    traj->profileUpdate(0.005);
}

float target;

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

    ctrl = static_new(controllers::MotorVelController(
            motor_test,
            { .pid = { .Kp = 60.0f, .Ki = 0.2f, .Kd = 0.0f, .abs_output_max = 4000.0f } }));

    traj = static_new(trajectory::MotorTrajectory<1>(&ctrl,
                                                     { .vm = 360, .am = 180, .jm = 360 },
                                                     { .Kp = 5, .Kd = 3, .abs_output_max = 35 }));

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_RegisterCallback(&htim13, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_200Hz);
    HAL_TIM_Base_Start_IT(&htim13);

    // enable controllers
    // for (auto& ctrl : motor_wheel_ctrl)
    //     if (!ctrl->enable())
    //         Error_Handler();

    while (!motor_test->isConnected())
        osDelay(1);

    osDelay(1000);

    if (!traj->enable())
        Error_Handler();

    float target_ = target;

    for (;;)
    {
        if (target_ != target)
        {
            traj->setTarget(target);
            target_ = target;
        }
        osDelay(1);
    }

    /* 初始化完成后退出线程 */
    osThreadExit();
}