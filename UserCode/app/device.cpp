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
#include "macros.hpp"

motors::DJIMotor* motor_wheel[4];

motors::DJIMotor* motor_elev[2];

void Device_Init()
{
    motor_wheel[0] = static_new(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 1 }));
    motor_wheel[1] = static_new(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 2 }));
    motor_wheel[2] = static_new(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 3 }));
    motor_wheel[3] = static_new(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 4 }));

    motor_elev[0] = static_new(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 5 }));
    motor_elev[1] = static_new(motors::DJIMotor(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 6 }));
}