#ifndef EPUCK2_AIR_RESISTANCE_H
#define EPUCK2_AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CEPuck2AirResistance : public CCI_Controller {

public:
    CEPuck2AirResistance() = default;
    void Init(TConfigurationNode& t_node) override;
    void ControlStep()                   override;

private:
    CCI_DifferentialSteeringActuator* m_pcWheels{nullptr};
    CCI_PositioningSensor*            m_pcPos{nullptr};
    CVector2 m_cWindCms;   /* global wind (cm/s, world) */
    Real     m_fBaseCms{5.0f};
};

#endif
