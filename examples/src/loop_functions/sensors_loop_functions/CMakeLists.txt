add_library(sensors_loop_functions MODULE
  sensors_loop_functions.h sensors_loop_functions.cpp)

target_link_libraries(sensors_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
