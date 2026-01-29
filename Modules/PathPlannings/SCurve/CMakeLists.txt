add_library(PlanningSCurve STATIC
    "./src/s_curve.cpp"
)

target_include_directories(PlanningSCurve
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# link dependencies if any


# alias for external use
add_library(Planning::SCurve ALIAS PlanningSCurve)
