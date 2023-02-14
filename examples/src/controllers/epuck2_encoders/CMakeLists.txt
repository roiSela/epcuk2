add_library(epuck2_encoders MODULE epuck2_encoders.h epuck2_encoders.cpp)
target_link_libraries(epuck2_encoders
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)


