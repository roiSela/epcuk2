add_library(epuck2_battery MODULE epuck2_battery.h epuck2_battery.cpp)
target_link_libraries(epuck2_battery
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)

