/**
 * @file    device.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#ifndef DEVICE_HPP
#define DEVICE_HPP
#include "drivers/dji.hpp"
#include <array>

using namespace motor_if;

std::array<motors::DJIMotor, 4>& motor_wheel();

motors::DJIMotor& motor_turnable();

void Device_Init();

#endif // DEVICE_HPP
