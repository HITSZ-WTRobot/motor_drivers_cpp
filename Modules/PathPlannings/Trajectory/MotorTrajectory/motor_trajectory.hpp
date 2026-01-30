/**
 * @file    motor_trajectory.hpp
 * @author  syhanjin
 * @date    2026-01-29
 */
#ifndef MOTOR_TRAJECTORY_HPP
#define MOTOR_TRAJECTORY_HPP
#include "motor_vel_controller.hpp"
#include "pid_pd.hpp"
#include "s_curve.hpp"
#include "cstddef"

namespace trajectory
{

template <size_t MotorNum> class MotorTrajectory
{
public:
    MotorTrajectory(controllers::MotorVelController*        motor_controllers[MotorNum],
                    velocity_profile::SCurveProfile::Config profile_cfg,
                    const PD::Config&                       error_pd_cfg);

    void profileUpdate(float dt);
    void errorUpdate();
    void controllerUpdate();

    void stop();
    bool setTarget(const float& target);
    bool setRelativeTarget(const float& target);

    float getCurrentAvePosition() const;
    float getCurrentAveVelocity() const;

    bool enable()
    {
        bool enabled = true;
        for (auto& ctrl : ctrl_)
            enabled &= ctrl->enable();
        if (!enabled)
            for (auto& ctrl : ctrl_)
                ctrl->disable();
        enabled_ = enabled;
        return enabled;
    }
    void disable()
    {
        stop();
        for (auto& ctrl : ctrl_)
            ctrl->disable();
    }
    bool enabled() const { return enabled_; }

    void lock() { lock_ = true; }
    void unlock() { lock_ = false; }
    bool locked() const { return lock_; }

private:
    bool enabled_{ false };
    bool lock_{ false };
    bool stopped_{ true };

    controllers::MotorVelController* ctrl_[MotorNum];

    PD pd_[MotorNum]{};

    velocity_profile::SCurveProfile::Config profile_cfg_;
    velocity_profile::SCurveProfile         profile_;

    float p_ref_curr_{ 0 };
    float v_ref_curr_{ 0 };

    float now_{ 0 };
};

} // namespace trajectory

#endif // MOTOR_TRAJECTORY_HPP
