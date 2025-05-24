#include "epuck2_airResistance.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

CEPuck2AirResistance::CEPuck2AirResistance() :
   m_pcWheels(nullptr),
   m_fBaseVelocity(5.0f) {}

void CEPuck2AirResistance::Init(TConfigurationNode& t_node)
{
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");

    // Read base velocity from <params velocity="..."/>
    GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseVelocity, m_fBaseVelocity);

    // Read air resistance from <configuration><air_resistance .../>
    TConfigurationNode tRoot = CSimulator::GetInstance().GetConfigurationRoot();
    TConfigurationNode tConfig = GetNode(tRoot, "configuration");
    TConfigurationNode tAirResist = GetNode(tConfig, "air_resistance");

    CVector2 cDir;
    Real fMag;
    GetNodeAttribute(tAirResist, "direction", cDir);
    GetNodeAttribute(tAirResist, "magnitude", fMag);

    m_cAirResistance = cDir.Normalize() * fMag;

    LOG << "Air Resistance: " << m_cAirResistance << std::endl;
}

void CEPuck2AirResistance::ControlStep()
{
    // Original base velocity before applying resistance
    LOG << "Base velocity: " << m_fBaseVelocity << std::endl;

    CVector2 cMove(m_fBaseVelocity, 0);
    cMove += m_cAirResistance;

    Real fVel = cMove.Length();
    if (fVel < 0.01f) fVel = 0;

    // Check direction: if X < 0, we are going backward
    Real fSignedVel = (cMove.GetX() < 0 ? -fVel : fVel);

    LOG << "Velocity after air resistance (Signed velocity): " << fSignedVel << std::endl;
    m_pcWheels->SetLinearVelocity(fSignedVel, fSignedVel);
}

REGISTER_CONTROLLER(CEPuck2AirResistance, "epuck2_air_resistance_controller")
