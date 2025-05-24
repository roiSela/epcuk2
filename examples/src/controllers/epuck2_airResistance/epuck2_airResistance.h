#ifndef EPUCK2_AIR_RESISTANCE_H
#define EPUCK2_AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CEPuck2AirResistance : public CCI_Controller {

public:
   CEPuck2AirResistance();
   virtual ~CEPuck2AirResistance() {}
   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset() {}
   virtual void Destroy() {}

private:
   CCI_DifferentialSteeringActuator* m_pcWheels;
   CVector2 m_cAirResistance;
   Real m_fBaseVelocity;
};

#endif
