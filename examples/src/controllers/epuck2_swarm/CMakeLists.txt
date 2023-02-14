add_library(epuck2_swarm MODULE epuck2_swarm.h epuck2_swarm.cpp)
target_link_libraries(epuck2_swarm
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)

