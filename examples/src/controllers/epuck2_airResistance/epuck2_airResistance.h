#ifndef EPUCK2_AIR_RESISTANCE_H
#define EPUCK2_AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

using namespace argos;

class CEPuck2AirResistance : public CCI_Controller {
public:
    void Init(TConfigurationNode& node) override;
    void ControlStep()                 override;
    void Reset() override {}  void Destroy() override {}

private:
    CCI_DifferentialSteeringActuator* m_pcWheels{nullptr};
    Real m_fBaseCms{5.0f};                /* cm s⁻¹ forward */
};

#endif
