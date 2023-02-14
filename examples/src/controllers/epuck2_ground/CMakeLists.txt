add_library(epuck2_ground MODULE epuck2_ground.h epuck2_ground.cpp)
target_link_libraries(epuck2_ground
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)


