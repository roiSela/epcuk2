add_library(epuck2_airResistance MODULE
        epuck2_airResistance.cpp
        epuck2_airResistance.h)

target_link_libraries(epuck2_airResistance
        argos3core_simulator
        argos3plugin_simulator_epuck2
        argos3plugin_simulator_genericrobot)
