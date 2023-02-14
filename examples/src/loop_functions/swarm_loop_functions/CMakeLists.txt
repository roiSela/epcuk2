add_library(swarm_loop_functions MODULE
  swarm_loop_functions.h swarm_loop_functions.cpp)

target_link_libraries(swarm_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
