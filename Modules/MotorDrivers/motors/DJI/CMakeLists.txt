file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
add_library(__MotorDrivers_DJI STATIC ${SRC_FILES})
add_library(MotorDrivers::DJI ALIAS __MotorDrivers_DJI)
target_include_directories(__MotorDrivers_DJI
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
add_dependencies(__MotorDrivers_DJI MotorDrivers::Core)
target_link_libraries(__MotorDrivers_DJI PUBLIC MotorDrivers::Core)
add_dependencies(__MotorDrivers_DJI bsp::CANDriver)
target_link_libraries(__MotorDrivers_DJI PUBLIC bsp::CANDriver)