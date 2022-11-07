/**
 * @file <epuck2_sensors.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_SENSORS_H
#define EPUCK2_SENSORS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_light_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_encoder_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>

using namespace argos;

class CEPuck2Sensors : public CCI_Controller {

public:

   CEPuck2Sensors();

   virtual ~CEPuck2Sensors() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_EPuck2ProximitySensor* m_pcProximity;
   CCI_RangeAndBearingActuator* m_pcRABAct;
   CCI_RangeAndBearingSensor* m_pcRABSens;
   CCI_EPuck2LEDsActuator* m_pcLedAct;
   CCI_EPuck2LightSensor* m_pcLightSens;
   CCI_EPuck2TOFSensor* m_pcTOFSensor;
   CCI_EPuck2EncoderSensor* m_pcEncoderSensor;
   CCI_Epuck2GroundSensor* m_pcGroundSensor;
   CCI_ColoredBlobPerspectiveCameraSensor* m_pcCamera;
   CCI_BatterySensor* m_pcBattery;
   Real m_fWheelVelocity;
   unsigned m_uLED;
   bool m_bGreen;

};

#endif
