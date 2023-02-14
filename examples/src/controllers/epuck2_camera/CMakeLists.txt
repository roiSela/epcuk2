add_library(epuck2_camera MODULE epuck2_camera.h epuck2_camera.cpp)
target_link_libraries(epuck2_camera
  argos3core_simulator
  argos3plugin_simulator_epuck2
  argos3plugin_simulator_genericrobot)


