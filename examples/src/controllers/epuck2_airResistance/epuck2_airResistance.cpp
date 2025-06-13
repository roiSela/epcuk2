#include "epuck2_airResistance.h"
#include <argos3/core/utility/configuration/argos_configuration.h>

void CEPuck2AirResistance::Init(TConfigurationNode& node) {
    m_pcWheels =
      GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    GetNodeAttributeOrDefault(node,"velocity",m_fBaseCms,m_fBaseCms);
}

void CEPuck2AirResistance::ControlStep() {
    m_pcWheels->SetLinearVelocity(m_fBaseCms, m_fBaseCms);   /* straight */
}

REGISTER_CONTROLLER(CEPuck2AirResistance,
                    "epuck2_air_resistance_controller")
