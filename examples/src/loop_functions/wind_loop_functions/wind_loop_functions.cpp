#include "wind_loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
/* Chipmunk headers come transitively from the include above */

#include <cmath>

/* ---------- Init : read wind + Δt ---------------------------------- */
void CWindLoopFunctions::Init(TConfigurationNode&) {

   TConfigurationNode air =
     GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),
                     "configuration"),
             "air_resistance");

   Real deg = 0.0, mag = 0.0;
   GetNodeAttribute(air,"angle_deg",deg);
   GetNodeAttribute(air,"magnitude",mag);

   const Real PI = 3.14159265358979323846;
   Real rad = deg*PI/180.0;
   m_cWindCms.Set(std::cos(rad)*mag, std::sin(rad)*mag);

   UInt32 tps = 10;
   GetNodeAttribute(
     GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),
                     "framework"),
             "experiment"),
     "ticks_per_second", tps);
   m_fDt = 1.0/static_cast<Real>(tps);
}

/* ---------- PreStep : apply F = m·a to every e-puck2 --------------- */
void CWindLoopFunctions::PreStep() {

   const CVector2 a = (m_cWindCms/100.0)*(1.0/m_fDt);          /* m s⁻² */

   /* all e-puck2 entities in the space */
   CSpace::TMapPerType& robots =
     CSimulator::GetInstance().GetSpace().GetEntitiesByType("e-puck2");

   for(const auto& it : robots) {
      auto& epuck = *any_cast<CEPuck2Entity*>(it.second);

      /* fetch the physics model through the body component            */
      CPhysicsModel& phys =
        epuck.GetComponent<CEmbodiedEntity>("body").GetPhysicsModel("dyn2d");

      /* every e-puck2 is single-body */
      auto* pSingle =
        dynamic_cast<CDynamics2DSingleBodyObjectModel*>(&phys);
      if(!pSingle) continue;                     /* safety */

      cpBody* body  = pSingle->GetBody();
      cpFloat mass  = cpBodyGetMass(body);

      /* F = m·a (Newtons) applied at CoM                              */
      cpBodyApplyForce(body,
                       cpv(a.GetX()*mass, a.GetY()*mass),
                       cpvzero);
   }
}

/* ---------- register ------------------------------------------------ */
REGISTER_LOOP_FUNCTIONS(CWindLoopFunctions,"wind_loop_functions")
