/**
 * @file    motor_trajectory.cpp
 * @author  syhanjin
 * @date    2026-01-29
 */
#include "motor_trajectory.hpp"

namespace trajectory
{

template <size_t MotorNum>
MotorTrajectory<MotorNum>::MotorTrajectory(
        controllers::MotorVelController*        motor_controllers[MotorNum],
        velocity_profile::SCurveProfile::Config profile_cfg,
        const PD::Config&                       error_pd_cfg) :
    ctrl_(motor_controllers), profile_cfg_(profile_cfg), profile_(profile_cfg, 0, 0, 0, 0)
{
    for (auto& pd : pd_)
        pd.setConfig(error_pd_cfg);
}

template <size_t MotorNum> void MotorTrajectory<MotorNum>::profileUpdate(const float dt)
{
    if (!enabled() || locked() || stopped_)
        return;
    now_ += dt;
    p_ref_curr_ = profile_.CalcX(now_);
    v_ref_curr_ = profile_.CalcV(now_);
}
template <size_t MotorNum> void MotorTrajectory<MotorNum>::errorUpdate()
{
    if (!enabled() || locked())
        return;
    for (size_t i = 0; i < MotorNum; ++i)
        ctrl_[i]->setRef(pd_[i].calc(p_ref_curr_, ctrl_[i]->getMotor()->getAngle()));
}
template <size_t MotorNum> void MotorTrajectory<MotorNum>::controllerUpdate()
{
    if (!enabled() || locked())
        return;
    for (auto& ctrl : ctrl_)
        ctrl->update();
}
template <size_t MotorNum> void MotorTrajectory<MotorNum>::stop()
{
    stopped_    = true;
    p_ref_curr_ = getCurrentAvePosition();
    v_ref_curr_ = 0;
}
template <size_t MotorNum> bool MotorTrajectory<MotorNum>::setTarget(const float& target)
{
    if (!enabled() || locked())
        return false;

    // try to construct profile
    const velocity_profile::SCurveProfile p(profile_cfg_,
                                            getCurrentAvePosition(),
                                            getCurrentAveVelocity(),
                                            profile_.CalcA(now_),
                                            target);
    // if failed
    if (!p.success())
        // if failed, return false;
        return false;
    // if success, set profile and reset now
    lock(); // lock before writing
    profile_ = p;
    now_     = 0;
    stopped_ = false;
    unlock();
    return true;
}
template <size_t MotorNum> bool MotorTrajectory<MotorNum>::setRelativeTarget(const float& target)
{
    if (!enabled() || locked())
        return false;

    // try to construct profile
    const velocity_profile::SCurveProfile p(profile_cfg_,
                                            getCurrentAvePosition(),
                                            getCurrentAveVelocity(),
                                            profile_.CalcA(now_),
                                            getCurrentAvePosition() + target);
    // if failed
    if (!p.success())
        // if failed, return false;
        return false;
    // if success, set profile and reset now
    lock(); // lock before writing
    profile_ = p;
    now_     = 0;
    stopped_ = false;
    unlock();
    return true;
}
template <size_t MotorNum> float MotorTrajectory<MotorNum>::getCurrentAvePosition() const
{
    float sum = 0;
    for (auto& ctrl : ctrl_)
        sum += ctrl->getMotor()->getAngle();
    return sum / MotorNum;
}
template <size_t MotorNum> float MotorTrajectory<MotorNum>::getCurrentAveVelocity() const
{
    float sum = 0;
    for (auto& ctrl : ctrl_)
        sum += ctrl->getMotor()->getVelocity();
    return sum / MotorNum;
}

} // namespace trajectory