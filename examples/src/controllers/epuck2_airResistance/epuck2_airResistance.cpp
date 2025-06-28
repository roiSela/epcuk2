#include "epuck2_airResistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/chipmunk-physics/include/chipmunk.h>

#include <cmath>

using namespace argos;

/* -------------------------------------------------- Init ---------- */
void CEPuck2AirResistance::Init(TConfigurationNode& t_node) {

   /* devices -------------------------------------------------------- */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");

   /* robot-specific params ----------------------------------------- */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseCms, m_fBaseCms);

   /* read global wind vector from <configuration><air_resistance> -- */
   TConfigurationNode tAir =
     GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),
                     "configuration"),
             "air_resistance");

   Real fDeg = 0.0, fMag = 0.0;
   GetNodeAttribute(tAir, "angle_deg", fDeg);
   GetNodeAttribute(tAir, "magnitude", fMag);

   /* convert degrees → radians and build wind vector in cm s⁻¹ */
   const Real rad = fDeg * ARGOS_PI / 180.0;
   m_cWindCms.Set(fMag * std::cos(rad),
                  fMag * std::sin(rad));
}

/* -------------------------------------------------- chipmunk body-- */
void CEPuck2AirResistance::LazyInitBody() {
   if(m_bBodyReady) return;

   auto& cEntity   = CSimulator::GetInstance().GetSpace().GetEntity(GetId());
   auto& cEmbodied = *dynamic_cast<CEmbodiedEntity*>(
                        &static_cast<CComposableEntity&>(cEntity).GetComponent("body"));

   auto* pcModel = dynamic_cast<CDynamics2DSingleBodyObjectModel*>(
                     &cEmbodied.GetPhysicsModel("dyn2d"));
   if(!pcModel)
      THROW_ARGOSEXCEPTION("No dyn2d model for " << GetId());

   m_ptBody     = pcModel->GetBody();
   m_bBodyReady = true;
}

/* -------------------------------------------------- step ---------- */
void CEPuck2AirResistance::ControlStep() {

   LazyInitBody();

   /* REMOVE the wheel command – Chipmunk velocity alone controls motion */
   /* m_pcWheels->SetLinearVelocity(m_fBaseCms, m_fBaseCms); */

   /* heading in world frame --------------------------------------- */
   CRadians yaw, pitch, roll;
   m_pcPos->GetReading().Orientation.ToEulerAngles(yaw, pitch, roll);
   CVector2 fwd(Cos(yaw), Sin(yaw));                // robot forward unit-vec

   /* vectors in m s⁻¹ ---------------------------------------------- */
   CVector2 vWheel = fwd        * (m_fBaseCms / 100.0);   // cm → m
   CVector2 vWind  = m_cWindCms / 100.0;
   CVector2 vTot   = vWheel + vWind;

   /* override Chipmunk body velocity ------------------------------ */
   cpBodySetVel(m_ptBody, cpv(2*vTot.GetX(), 2*vTot.GetY()));
}

/* register with ARGoS */
REGISTER_CONTROLLER(CEPuck2AirResistance,
                    "epuck2_air_resistance_controller")
