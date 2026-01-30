add_library(TrajectoryMotorTrajectory STATIC
    "./motor_trajectory.cpp"
)

target_include_directories(TrajectoryMotorTrajectory
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(TrajectoryMotorTrajectory PUBLIC VelocityProfile::SCurve)
target_link_libraries(TrajectoryMotorTrajectory PUBLIC MotorDrivers::Controller)
target_link_libraries(TrajectoryMotorTrajectory PUBLIC libs::PID_PD)

# alias for external use
add_library(Trajectory::MotorTrajectory ALIAS TrajectoryMotorTrajectory)
