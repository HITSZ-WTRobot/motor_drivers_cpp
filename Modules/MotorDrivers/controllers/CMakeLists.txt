file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
add_library(__MotorDrivers_Controller STATIC ${SRC_FILES})
add_library(MotorDrivers::Controller ALIAS __MotorDrivers_Controller)
target_include_directories(__MotorDrivers_Controller
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_dependencies(__MotorDrivers_Controller MotorDrivers::Core)
target_link_libraries(__MotorDrivers_Controller PUBLIC MotorDrivers::Core)

add_dependencies(__MotorDrivers_Controller libs::PIDMotor)
target_link_libraries(__MotorDrivers_Controller PUBLIC libs::PIDMotor)