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
#include "dji.hpp"
#include <array>

extern motors::DJIMotor* motor_wheel[4];

motors::DJIMotor& motor_turnable();

void Device_Init();

#endif // DEVICE_HPP
