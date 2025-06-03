#include "epuck2_airResistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>     // Cos(), Sin()
#include <algorithm>                             // std::clamp
#include <cmath>                                 // std::cos, std::sin, std::fabs

/****************************************/
/* CONSTRUCTOR */
/****************************************/

CEPuck2AirResistance::CEPuck2AirResistance() :
   m_pcWheels(nullptr),
   m_pcPos(nullptr),
   m_fBaseVel(5.0f) {}

/****************************************/
/* INIT */
/****************************************/

void CEPuck2AirResistance::Init(TConfigurationNode& t_node) {

   /* Devices */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");

   /* <params velocity="…">  (default 5 cm/s) */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseVel, m_fBaseVel);

   /*****  GLOBAL WIND (polar form only)  *****/
   TConfigurationNode tRoot = CSimulator::GetInstance().GetConfigurationRoot();
   TConfigurationNode tCfg  = GetNode(tRoot, "configuration");
   TConfigurationNode tAir  = GetNode(tCfg,  "air_resistance");

   /* Mandatory attributes */
   Real fAngleDeg = 0.0f, fMag = 0.0f;
   GetNodeAttribute(tAir, "angle_deg", fAngleDeg);
   GetNodeAttribute(tAir, "magnitude", fMag);

   /* Convert polar → Cartesian */
   const Real PI = 3.14159265358979323846;
   Real fAngRad  = fAngleDeg * PI / 180.0;
   CVector2 cDir(std::cos(fAngRad), std::sin(fAngRad));   // unit vector
   m_cWind = cDir * fMag;                                 // wind vector

   LOG << "[AirRes] wind = " << m_cWind << std::endl;
}

/****************************************/
/* CONTROL STEP : vector-addition drag  */
/****************************************/

void CEPuck2AirResistance::ControlStep() {

   /* Current yaw (rotation about Z) */
   CRadians cYaw, cTmp1, cTmp2;
   m_pcPos->GetReading().Orientation.ToEulerAngles(cYaw, cTmp1, cTmp2);

   /* Forward unit vector in WORLD frame */
   CVector2 cFwd(Cos(cYaw), Sin(cYaw));

   /* Base velocity vector (world) */
   CVector2 cBase = cFwd * m_fBaseVel;

   /* Vector addition: base + wind */
   CVector2 cNet  = cBase + m_cWind;

   /* Speed is the length of the resultant vector                       */
   Real fSpeed = cNet.Length();

   /* Preserve sign: if resultant points backwards relative to body axis
      the robot should drive in reverse                                */
   if(cNet.DotProduct(cFwd) < 0.0f)
      fSpeed = -fSpeed;

   /* Dead-zone & clamp to wheel capability                             */
   if(std::fabs(fSpeed) < 0.01f) fSpeed = 0.0f;
   constexpr Real MAX_W = 12.0f;                  // e-puck2 wheel limit (cm/s)
   fSpeed = std::clamp(fSpeed, -MAX_W, MAX_W);

   LOG << "[Ctrl] base=" << m_fBaseVel
       << "  wind=" << m_cWind
       << "  net="  << cNet
       << "  → speed=" << fSpeed
       << std::endl;

   /* Straight-line command (no heading change) */
   m_pcWheels->SetLinearVelocity(fSpeed, fSpeed);
}

/****************************************/
/* REGISTER CONTROLLER */
/****************************************/

REGISTER_CONTROLLER(CEPuck2AirResistance,
                     "epuck2_air_resistance_controller")
