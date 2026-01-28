/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */

#include "device.hpp"
#include "controllers/motor_vel_controller.hpp"
using namespace motor_if;

constexpr controllers::MotorVelController::Config cfg = {
    .pid = { .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, .abs_output_max = 8000.0f }
};

auto& motor_wheel_ctrl()
{
    using namespace controllers;
    static std::array<MotorVelController, 4> _ = { MotorVelController(&motor_wheel()[0], cfg),
                                                   MotorVelController(&motor_wheel()[1], cfg),
                                                   MotorVelController(&motor_wheel()[2], cfg),
                                                   MotorVelController(&motor_wheel()[3], cfg) };
    return _;
}