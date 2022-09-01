/**
 * @file <epuck2_encoders.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_ENCODERS_H
#define EPUCK2_ENCODERS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_encoder_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
#include <fstream>

using namespace argos;

class CCI_EPuck2Tof : public CCI_Controller {

public:

   CCI_EPuck2Tof();

   virtual ~CCI_EPuck2Tof() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

private:

   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_EPuck2TOFSensor* m_pcTOFSensor;
   CCI_EPuck2EncoderSensor* m_pcEncoderSensor;
   int m_iPreviousEncoder;
   Real m_fDistance, m_fToFbegin, m_fWheelVelocityLeft, m_fWheelVelocityRight;
};

#endif
