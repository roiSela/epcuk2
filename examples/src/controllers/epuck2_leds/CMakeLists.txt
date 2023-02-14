add_library(epuck2_leds MODULE epuck2_leds.h epuck2_leds.cpp)
target_link_libraries(epuck2_leds
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)


