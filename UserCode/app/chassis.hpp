/**
 * @file    chassis.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#ifndef CHASSIS_HPP
#define CHASSIS_HPP
#include "controllers/motor_vel_controller.hpp"

using namespace motor_if;

extern controllers::MotorVelController* motor_wheel_ctrl[4];

#endif // CHASSIS_HPP
