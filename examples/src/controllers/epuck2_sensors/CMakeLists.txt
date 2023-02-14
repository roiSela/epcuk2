add_library(epuck2_sensors MODULE epuck2_sensors.h epuck2_sensors.cpp)
target_link_libraries(epuck2_sensors
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)

