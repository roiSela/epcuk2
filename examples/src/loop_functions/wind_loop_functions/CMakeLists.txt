add_library(wind_loop_functions MODULE
        wind_loop_functions.cpp
        wind_loop_functions.h)

target_link_libraries(wind_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_dynamics2d)

set_target_properties(wind_loop_functions PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_BINARY_DIR}/lib/loop_functions/wind_loop_functions)
