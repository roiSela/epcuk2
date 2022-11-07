/**
 * @file <epuck2_ground.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_GROUND_H
#define EPUCK2_GROUND_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_ground_sensor.h>
#include <argos3/core/utility/math/vector3.h>
#include <fstream>

using namespace argos;

class CCI_EPuck2Ground: public CCI_Controller {

public:

   CCI_EPuck2Ground();

   virtual ~CCI_EPuck2Ground() {
   }

   virtual void Init(TConfigurationNode &t_node);

   virtual void ControlStep();

   virtual void Reset() {
   }

   virtual void Destroy();

private:

   CCI_DifferentialSteeringActuator *m_pcWheels;
   CCI_Epuck2GroundSensor *m_pcGroundSensor;
   Real m_fWheelVelocityLeft, m_fWheelVelocityRight;
   bool m_bVerb;
   std::ofstream m_cLogFile;
};

#endif
