/**
 * @file    device.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "device.hpp"
#include "can.h"
#include "init-macros.hpp"

motors::DJIMotor* motor_wheel[4];

void Device_Init()
{
    motor_wheel[0] = new_(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 1 }));
    motor_wheel[1] = new_(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 2 }));
    motor_wheel[2] = new_(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 3 }));
    motor_wheel[3] = new_(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 4 }));
}