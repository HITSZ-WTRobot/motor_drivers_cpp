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

#include <array>

std::array<motors::DJIMotor, 4>& motor_wheel()
{
    static std::array<motors::DJIMotor, 4> _ = {
        motors::DJIMotor({ .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 1 }),
        motors::DJIMotor({ .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 2 }),
        motors::DJIMotor({ .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 3 }),
        motors::DJIMotor({ .hcan = &hcan1, .type = motors::DJIMotor::Type::M3508_C620, .id1 = 4 })
    };
    return _;
}

motors::DJIMotor& motor_turnable()
{
    static motors::DJIMotor _(
            { .hcan = &hcan1, .type = motors::DJIMotor::Type::M2006_C610, .id1 = 5 });
    return _;
}

#define init(f) f()
void Device_Init()
{
    init(motor_wheel);
    init(motor_turnable);
}