#include "epuck2_airResistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

#include <cmath>   // std::cos, std::sin

/******** Init ***************************************************************/
void CEPuck2AirResistance::Init(TConfigurationNode& t_node) {

   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");

   /* base speed */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseCms, m_fBaseCms);

   /* Δt from experiment */
   UInt32 unTPS = 10;
   GetNodeAttribute(
       GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),"framework"),
               "experiment"),
       "ticks_per_second", unTPS);
   m_fDt = 1.0f / static_cast<Real>(unTPS);

   /* wind (polar) */
   TConfigurationNode tAir =
       GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),"configuration"),
               "air_resistance");
   Real deg = 0.0f, mag = 0.0f;
   GetNodeAttribute(tAir,"angle_deg", deg);
   GetNodeAttribute(tAir,"magnitude", mag);

   const Real PI = 3.14159265358979323846;
   Real rad = deg * PI / 180.0;
   m_cWindCms.Set(std::cos(rad)*mag, std::sin(rad)*mag);
}

/******** ControlStep *******************************************************/
void CEPuck2AirResistance::ControlStep() {

   /* ----------------------------------
    * 1. Resolve body pointer once
    * ---------------------------------- */
   if(!m_pcBody) {
      const std::string& strId = GetId();
      CEntity& cEnt = CSimulator::GetInstance().GetSpace().GetEntity(strId);
      auto* pcRobot = dynamic_cast<CEPuck2Entity*>(&cEnt);
      if(pcRobot) m_pcBody = &pcRobot->GetEmbodiedEntity();
      else        return;                     /* wait until next tick */
   }

   /* ----------------------------------
    * 2. Wheel command: straight forward
    * ---------------------------------- */
   m_pcWheels->SetLinearVelocity(m_fBaseCms, m_fBaseCms);

   /* ----------------------------------
    * 3. Pose & heading
    * ---------------------------------- */
   const auto& pose = m_pcPos->GetReading();
   CRadians yaw,tmp1,tmp2;
   pose.Orientation.ToEulerAngles(yaw,tmp1,tmp2);
   CVector2 fwd(Cos(yaw), Sin(yaw));

   /* ----------------------------------
    * 4. Galilean velocity addition
    * ---------------------------------- */
   CVector2 vTot = fwd * m_fBaseCms + m_cWindCms;   /* cm/s */

   /* ----------------------------------
    * 5. Displacement this tick (m)
    * ---------------------------------- */
   CVector2 d = vTot * (m_fDt / 100.0f);           /* cm→m */
   CVector3 newPos(pose.Position.GetX()+d.GetX(),
                   pose.Position.GetY()+d.GetY(),
                   pose.Position.GetZ());

   m_pcBody->MoveTo(newPos, pose.Orientation, false, true);
}

/******** Register ***********************************************************/
REGISTER_CONTROLLER(CEPuck2AirResistance,
                    "epuck2_air_resistance_controller")
