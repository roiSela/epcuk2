#include "wind_loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/core/utility/math/angles.h>

/* ------------ read global wind once ----------------------- */
void CWindLoopFunctions::Init(TConfigurationNode&) {
   TConfigurationNode air =
     GetNode(GetNode(CSimulator::GetInstance().GetConfigurationRoot(),
                     "configuration"),
             "air_resistance");

   Real deg = 0.0, mag = 0.0;
   GetNodeAttribute(air,"angle_deg",deg);
   GetNodeAttribute(air,"magnitude",mag);

   const Real PI = 3.14159265358979323846;
   m_cWindCms.Set(std::cos(deg*PI/180.0)*mag,
                  std::sin(deg*PI/180.0)*mag);
}

/* ------------ overwrite body velocity before physics ------ */
void CWindLoopFunctions::PreStep() {

   const CVector2 wind_ms = m_cWindCms / 100.0;          /* m s⁻¹ */

   auto& bots =
     CSimulator::GetInstance().GetSpace().GetEntitiesByType("e-puck2");

   for(const auto& it : bots) {
      auto& epk = *any_cast<CEPuck2Entity*>(it.second);

      /* heading */
      CRadians yaw,p,r;
      epk.GetEmbodiedEntity().GetOriginAnchor().Orientation
          .ToEulerAngles(yaw,p,r);
      CVector2 fwd( Cos(yaw), Sin(yaw) );

      /* wheel command: 5 cm s⁻¹ straight */
      const Real BASE_CMS = 5.0;
      CVector2 vWheel_ms = fwd * (BASE_CMS/100.0);

      /* physics model */
      auto& phys =
        epk.GetComponent<CEmbodiedEntity>("body").GetPhysicsModel("dyn2d");
      auto* pSingle =
        dynamic_cast<CDynamics2DSingleBodyObjectModel*>(&phys);
      if(!pSingle) continue;

      cpBodySetVel(pSingle->GetBody(),
                   cpv(vWheel_ms.GetX()+wind_ms.GetX(),
                       vWheel_ms.GetY()+wind_ms.GetY()));
   }
}

REGISTER_LOOP_FUNCTIONS(CWindLoopFunctions,"wind_loop_functions")
