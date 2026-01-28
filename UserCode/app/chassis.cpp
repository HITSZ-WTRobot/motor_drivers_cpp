/**
 * @file    chassis.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */

#include "device.hpp"
#include "chassis.hpp"
#include "init-macros.hpp"
#include "controllers/motor_vel_controller.hpp"
using namespace motor_if;

constexpr controllers::MotorVelController::Config cfg = {
    .pid = { .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, .abs_output_max = 8000.0f }
};

controllers::MotorVelController* motor_wheel_ctrl[4];

void Chassis_Init()
{
    using namespace controllers;
    motor_wheel_ctrl[0] = new_(MotorVelController, motor_wheel[0], cfg);
    motor_wheel_ctrl[1] = new_(MotorVelController, motor_wheel[1], cfg);
    motor_wheel_ctrl[2] = new_(MotorVelController, motor_wheel[2], cfg);
    motor_wheel_ctrl[3] = new_(MotorVelController, motor_wheel[3], cfg);
}