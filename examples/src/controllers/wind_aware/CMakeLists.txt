# Build the derived plugin as its own MODULE
add_library(wind_aware_air_resistance MODULE
    wind_aware_air_resistance.cpp)

# Point directly at the folder that has air_resistance.h
target_include_directories(wind_aware_air_resistance PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../air_resistance)

# Link to ARGoS (same as your base). No linking to the base MODULE.
target_link_libraries(wind_aware_air_resistance PRIVATE
    argos3core_simulator
    argos3plugin_simulator_genericrobot)

