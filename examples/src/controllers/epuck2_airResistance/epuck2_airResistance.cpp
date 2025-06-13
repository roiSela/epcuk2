#include "epuck2_airResistance.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>

/* -------- Init ----------------------------------------------------- */
void CEPuck2AirResistance::Init(TConfigurationNode& node) {

   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");

   GetNodeAttributeOrDefault(node,"velocity",m_fBaseCms,m_fBaseCms);

   /* read <configuration><air_resistance …/> */
   TConfigurationNode air =
     GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),"configuration"),
             "air_resistance");
   Real deg=0, mag=0;
   GetNodeAttribute(air,"angle_deg",deg);
   GetNodeAttribute(air,"magnitude",mag);

   const Real PI = 3.14159265358979323846;
   Real rad = deg*PI/180.0;
   m_cWindCms.Set(std::cos(rad)*mag, std::sin(rad)*mag);
}

/* -------- ControlStep ---------------------------------------------- */
void CEPuck2AirResistance::ControlStep() {

   CRadians yaw,tmp1,tmp2;
   m_pcPos->GetReading().Orientation.ToEulerAngles(yaw,tmp1,tmp2);

   CVector2 fwd( Cos(yaw), Sin(yaw) );         /* body X axis */
   CVector2 v   = fwd*m_fBaseCms + m_cWindCms; /* cm/s world  */
   Real      s  = v.DotProduct(fwd);           /* forward component */

   m_pcWheels->SetLinearVelocity(s, s);        /* keep heading */
}

REGISTER_CONTROLLER(CEPuck2AirResistance,
                    "epuck2_air_resistance_controller")
