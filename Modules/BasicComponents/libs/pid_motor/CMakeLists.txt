file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
add_library(__PIDMotor STATIC ${SRC_FILES})
add_library(libs::PIDMotor ALIAS __PIDMotor)
target_include_directories(__PIDMotor
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)