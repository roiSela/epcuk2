#include "epuck2_airResistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <algorithm>   // std::clamp
#include <cmath>       // std::fabs

/****************************************/
/* CONSTRUCTOR */
/****************************************/

CEPuck2AirResistance::CEPuck2AirResistance() :
   m_pcWheels(nullptr),
   m_pcPos(nullptr),
   m_fBaseVelocity(5.0f) {}

/****************************************/
/* INIT */
/****************************************/

void CEPuck2AirResistance::Init(TConfigurationNode& t_node) {

   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");

   /* <params velocity="…">  (default 5 cm/s) */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseVelocity, m_fBaseVelocity);

   /***** wind vector *****/
   TConfigurationNode tRoot      = CSimulator::GetInstance().GetConfigurationRoot();
   TConfigurationNode tCfg       = GetNode(tRoot, "configuration");
   TConfigurationNode tAir       = GetNode(tCfg,  "air_resistance");

   CVector2 cDir;
   Real     fMag;
   GetNodeAttribute(tAir, "direction",  cDir);
   GetNodeAttribute(tAir, "magnitude",  fMag);

   m_cAirResistance = cDir.Normalize() * fMag;

   LOG << "[AirRes] dir=" << cDir
       << "  mag=" << fMag
       << "  → vec=" << m_cAirResistance
       << std::endl;
}

/****************************************/
/* CONTROL STEP */
/****************************************/

void CEPuck2AirResistance::ControlStep() {

   /* Current yaw (Z-rotation) */
   CRadians cYaw, cTmp1, cTmp2;
   m_pcPos->GetReading().Orientation.ToEulerAngles(cYaw, cTmp1, cTmp2);

   /* Forward unit vector */
   CVector2 cFwd(Cos(cYaw), Sin(cYaw));

   /***** decide which axis we are “mostly” on *****/
   Real fEffect = 0.0f;      /* what we will add/subtract */

   if(std::fabs(cFwd.GetX()) >= std::fabs(cFwd.GetY())) {
      /* heading is mainly east/west → use wind_X */
      fEffect = (cFwd.GetX() >= 0.0f ?  m_cAirResistance.GetX()
                                     : -m_cAirResistance.GetX());
   }
   else {
      /* heading is mainly north/south → use wind_Y */
      fEffect = (cFwd.GetY() >= 0.0f ?  m_cAirResistance.GetY()
                                     : -m_cAirResistance.GetY());
   }

   /* final forward speed */
   Real fVel = m_fBaseVelocity + fEffect;

   /* dead-zone + clamp */
   if(std::fabs(fVel) < 0.01f) fVel = 0.0f;
   constexpr Real MAX_W = 12.0f;            // wheel limit
   fVel = std::clamp(fVel, -MAX_W, MAX_W);

   LOG << "[Ctrl] base=" << m_fBaseVelocity
       << "  effect=" << fEffect
       << "  → vel=" << fVel
       << std::endl;

   /* straight line */
   m_pcWheels->SetLinearVelocity(fVel, fVel);
}

/****************************************/
/* REGISTER */
/****************************************/

REGISTER_CONTROLLER(CEPuck2AirResistance,
                     "epuck2_air_resistance_controller")
