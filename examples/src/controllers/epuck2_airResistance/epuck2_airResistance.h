#ifndef EPUCK2_AIR_RESISTANCE_H
#define EPUCK2_AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CEPuck2AirResistance : public CCI_Controller {

public:
    CEPuck2AirResistance();
    ~CEPuck2AirResistance() override = default;

    void Init(TConfigurationNode& t_node) override;
    void ControlStep()                   override;
    void Reset()                         override {}
    void Destroy()                       override {}

private:
    CCI_DifferentialSteeringActuator* m_pcWheels;
    CCI_PositioningSensor*            m_pcPos;

    CVector2 m_cAirResistance;   /* wind vector (world frame)      */
    Real     m_fBaseVelocity;    /* speed when there is no wind    */
};

#endif /* EPUCK2_AIR_RESISTANCE_H */
