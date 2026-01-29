file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

add_library(__CANDriver STATIC ${SRC_FILES})

add_library(bsp::CANDriver ALIAS __CANDriver)

target_include_directories(__CANDriver
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(__CANDriver PUBLIC stm32cubemx)