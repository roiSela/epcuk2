/**
 * @file <epuck2_camera.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_CAMERA_H
#define EPUCK2_CAMERA_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>

using namespace argos;

class CEPuck2Camera : public CCI_Controller {

public:

   CEPuck2Camera();

   virtual ~CEPuck2Camera() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_ColoredBlobPerspectiveCameraSensor* m_pcCamera;
   Real m_fWheelVelocity;

};

#endif
