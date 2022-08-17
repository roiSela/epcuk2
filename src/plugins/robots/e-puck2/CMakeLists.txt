#
# E-puck2 headers
#
# argos3/plugins/robots/e-puck2/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_CONTROLINTERFACE
  control_interface/ci_epuck2_proximity_sensor.h
  control_interface/ci_epuck2_leds_actuator.h
  control_interface/ci_epuck2_tof_sensor.h
  control_interface/ci_epuck2_encoder_sensor.h)
# argos3/plugins/robots/e-puck2/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_SIMULATOR
    simulator/dynamics2d_epuck2_model.h
    # simulator/physx_epuck_model.h
    simulator/epuck2_entity.h
    simulator/epuck2_led_equipped_entity.h
    simulator/epuck2_tof_equipped_entity.h
    simulator/epuck2_encoder_equipped_entity.h
    simulator/epuck2_proximity_default_sensor.h
    simulator/epuck2_light_default_sensor.h
    simulator/epuck2_ground_rotzonly_sensor.h
    simulator/epuck2_led_default_actuator.h
    simulator/epuck2_tof_default_sensor.h
    simulator/epuck2_colored_blob_perspective_camera_default_sensor.h
    simulator/epuck2_battery_equipped_entity.h
    simulator/epuck2_camera_equipped_entity.h
    simulator/epuck2_battery_default_sensor.h
    simulator/epuck2_encoder_default_sensor.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# E-puck sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK2
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_CONTROLINTERFACE}
  control_interface/ci_epuck2_proximity_sensor.cpp
  control_interface/ci_epuck2_leds_actuator.cpp
  control_interface/ci_epuck2_tof_sensor.cpp
  control_interface/ci_epuck2_encoder_sensor.cpp)
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK2
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK2}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_SIMULATOR}
    simulator/dynamics2d_epuck2_model.cpp
    # simulator/physx_epuck_model.cpp
    simulator/epuck2_entity.cpp
    simulator/epuck2_led_equipped_entity.cpp    
    simulator/epuck2_tof_equipped_entity.cpp
    simulator/epuck2_encoder_equipped_entity.cpp
    simulator/epuck2_light_default_sensor.cpp
    simulator/epuck2_ground_rotzonly_sensor.cpp
    simulator/epuck2_tof_default_sensor.cpp
    simulator/epuck2_encoder_default_sensor.cpp
    simulator/epuck2_colored_blob_perspective_camera_default_sensor.cpp
    simulator/epuck2_battery_equipped_entity.cpp
    simulator/epuck2_camera_equipped_entity.cpp
    simulator/epuck2_battery_default_sensor.cpp
    simulator/epuck2_led_default_actuator.cpp
    simulator/epuck2_proximity_default_sensor.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  if(ARGOS_QTOPENGL_FOUND)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_SIMULATOR}
      simulator/qtopengl_epuck2.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK2
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK2}
      simulator/qtopengl_epuck2.h
      simulator/qtopengl_epuck2.cpp)
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create e-puck plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_epuck2 SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_EPUCK2})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_epuck2
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
  # argos3plugin_${ARGOS_BUILD_FOR}_physx)
if(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_QTOPENGL_FOUND)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_epuck2 argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_BUILD_FOR_SIMULATOR AND ARGOS_QTOPENGL_FOUND)

install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/e-puck2/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_EPUCK2_SIMULATOR}      DESTINATION include/argos3/plugins/robots/e-puck2/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_epuck2
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
