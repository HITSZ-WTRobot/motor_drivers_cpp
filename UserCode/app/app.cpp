/**
 * @file    app.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "app.hpp"

#include "can.h"
#include "cmsis_os2.h"
#include "device.hpp"
#include "drivers/dji.hpp"
#include <bsp/can_driver.h>

using namespace motor_if;
#define init(_)

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

    /* 初始化完成后退出线程 */
    osThreadExit();
}