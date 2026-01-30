/**
 * @file    elevator.cpp
 * @author  syhanjin
 * @date    2026-01-30
 */
#include "elevator.hpp"

#include "device.hpp"
#include "macros.hpp"

static PIDMotor::Config motor_elev_vel_pid = {
    .Kp             = 45.0f,
    .Ki             = 0.07f,
    .Kd             = 5.00f,
    .abs_output_max = 8000,
};

controllers::MotorVelController* motor_elev_vel_ctrl[2];
trajectory::MotorTrajectory<2>*  elev;

void APP_Elevator_BeforeUpdate(void)
{
    motor_elev_vel_ctrl[0] = static_new(
            controllers::MotorVelController(motor_elev[0], { .pid = motor_elev_vel_pid }));
    motor_elev_vel_ctrl[1] = static_new(
            controllers::MotorVelController(motor_elev[1], { .pid = motor_elev_vel_pid }));

    elev = static_new(
            trajectory::MotorTrajectory<2>(motor_elev_vel_ctrl,
                                           { .vm = 360, .am = 60, .jm = 100 },
                                           { .Kp = 20.0f, .Kd = 20.0f, .abs_output_max = 600.0f }));
}

void APP_Elevator_Init(void)
{
    // try to enable trajectory controller
    assert_param(elev->enable());
}

void APP_Elevator_Update_1kHz(void)
{
    // update trajectory controller
    elev->controllerUpdate();
}

void APP_Elevator_Update_500Hz(void)
{
    elev->errorUpdate();
}

void APP_Elevator_Update_200Hz(void)
{
    elev->profileUpdate(0.005);
}