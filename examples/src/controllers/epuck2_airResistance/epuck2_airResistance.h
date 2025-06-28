#ifndef EPUCK2_AIR_RESISTANCE_H
#define EPUCK2_AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

struct cpBody;                                 // forward decl for Chipmunk

namespace argos {

class CEPuck2AirResistance final : public CCI_Controller {

public:
   void Init(TConfigurationNode& t_node) override;
   void ControlStep()                   override;
   void Reset()                         override {}
   void Destroy()                       override {}

private:
   void LazyInitBody();                            // Chipmunk hookup

   /* devices */
   CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
   CCI_PositioningSensor*            m_pcPos    = nullptr;

   /* params */
   Real     m_fBaseCms  = 5.0f;                    // wheel cmd (cm s⁻¹)
   CVector2 m_cWindCms;                            // wind   (cm s⁻¹)

   /* cached physics */
   bool     m_bBodyReady = false;
   cpBody*  m_ptBody     = nullptr;
};

} // namespace argos
#endif
