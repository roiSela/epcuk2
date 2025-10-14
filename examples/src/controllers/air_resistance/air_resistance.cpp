#include "air_resistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/chipmunk-physics/include/chipmunk.h>

#include <algorithm>
#include <cmath>

using namespace argos;

/* ----- Post-step: apply accumulated impulse (after collisions) ----- */
struct SWindPostData {
   cpBody* body;
   cpVect  J;
};

static void ApplyAccumPostStep(cpSpace* /*space*/, void* /*key*/, void* data) {
   SWindPostData* p = static_cast<SWindPostData*>(data);
   if(p && p->body) {
      cpBodyActivate(p->body);
      cpBodyApplyImpulse(p->body, p->J, cpvzero);  // at COM
   }
   delete p;
}

/* --------------------------------------------------------------- */
void CAirResistance::Init(TConfigurationNode& t_node)
{
   /* devices */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");
   m_pcRABSens= GetSensor  <CCI_RangeAndBearingSensor>       ("range_and_bearing");
   m_pcRABAct = GetActuator<CCI_RangeAndBearingActuator>     ("range_and_bearing");

   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseCms, m_fBaseCms);

   /* read <configuration><air_resistance .../> */
   TConfigurationNode& tRoot = CSimulator::GetInstance().GetConfigurationRoot();
   TConfigurationNode  tConf = GetNode(tRoot, "configuration");
   TConfigurationNode  tAir  = GetNode(tConf, "air_resistance");

   Real deg = 0.0, mag = 0.0;
   GetNodeAttribute(tAir, "angle_deg", deg);
   GetNodeAttribute(tAir, "magnitude", mag);

   const Real rad = deg * ARGOS_PI / 180.0;
   m_cWindCms.Set(mag * std::cos(rad), mag * std::sin(rad));
}

/* --------------------------------------------------------------- */
void CAirResistance::EnsurePhysicsHandle()
{
   if(m_bBodyReady) return;

   CEntity& cEntity = CSimulator::GetInstance().GetSpace().GetEntity(GetId());
   auto& cComposable = dynamic_cast<CComposableEntity&>(cEntity);
   auto& cEmbodied   = cComposable.GetComponent<CEmbodiedEntity>("body");

   auto* pcModel = dynamic_cast<CDynamics2DSingleBodyObjectModel*>(
                     &cEmbodied.GetPhysicsModel("dyn2d"));
   if(!pcModel)
      THROW_ARGOSEXCEPTION("No dyn2d model for " << GetId());

   m_ptBody     = pcModel->GetBody();
   m_bBodyReady = true;
}

/* --------------------------------------------------------------- */
Real CAirResistance::GetYawRadians() const
{
   CRadians cZ, cY, cX;
   m_pcPos->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
   return cZ.GetValue();
}

/* --------------------------------------------------------------- */
bool CAirResistance::IsBlockedByRAB(Real& fOutReduction) const
{
   fOutReduction = 0.0;
   if(!m_pcPos || !m_pcRABSens) return false;
   if(m_cWindCms.Length() < 1e-9) return false;

   /* unit wind */
   CVector2 wdir = m_cWindCms;
   wdir.Normalize();

   const Real yaw = GetYawRadians();

   const auto& rs = m_pcRABSens->GetReadings();
   bool bAny = false;

   for(const auto& rcv : rs) {
      const Real range = rcv.Range;
      if(range <= 1e-6) continue;

      /* neighbor-announced radius (meters) in byte0; fallback to ours */
      Real r_occ = m_fSelfRadiusM;
      if(rcv.Data.Size() >= 1) {
         r_occ = static_cast<Real>(rcv.Data[0]) * 0.001; /* mm -> m */
         if(r_occ <= 0.0) r_occ = m_fSelfRadiusM;
      }

      /* world vector me->other */
      const Real ang_world = yaw + rcv.HorizontalBearing.GetValue();
      const CVector2 v_me_other(range * std::cos(ang_world),
                                range * std::sin(ang_world));

      /* other->me */
      const CVector2 r = -v_me_other;

      /* along-wind distance and lateral offset */
      const Real s = r.DotProduct(wdir);     /* meters */
      if(s <= 0.0) continue;                 /* only downwind neighbors */

      const CVector2 lateral = r - (wdir * s);
      const Real     lat     = lateral.Length();

      /* cross-section coverage: 1 at centerline, 0 at edge */
      const Real coverage = std::max(0.0, 1.0 - (lat / std::max(r_occ, 1e-6)));
      if(coverage <= 0.0) continue;

      /* slower distance decay to keep effect visible at ~30cm */
      const Real ratio = std::min(1.0, r_occ / std::max(s, 1e-6));
      const Real distance_factor = std::sqrt(ratio);

      const Real reduction = std::clamp(coverage * distance_factor, 0.0, 1.0);
      fOutReduction = std::max(fOutReduction, reduction);
      bAny = true;
   }

   return bAny && fOutReduction > 1e-6;
}

/* --------------------------------------------------------------- */
CVector2 CAirResistance::ComputeEffectiveWind() const
{
   if(m_cWindCms.Length() < 1e-9) return m_cWindCms;

   Real red = 0.0;
   if(IsBlockedByRAB(red)) {
      const Real mult = std::max(0.0, 1.0 - red);
      return m_cWindCms * mult;
   }
   return m_cWindCms;
}

/* --------------------------------------------------------------- */
/* Adds wind impulse to the per-tick accumulator */
void CAirResistance::ApplyWindImpulse()
{
   const CVector2 eff = ComputeEffectiveWind();
   if(eff.Length() < 1e-9) return;

   const Real mass = cpBodyGetMass(m_ptBody);
   const CVector2 Jv = (eff / 100.0) * mass * WIND_IMPULSE_SCALE;

   m_cAccumImpulse += Jv;
}

/* Adds drive impulse (forward along yaw) to the per-tick accumulator */
void CAirResistance::DriveImpulse(Real velocity_cm_s)
{
   const Real yaw = GetYawRadians();
   const CVector2 fwd(std::cos(yaw), std::sin(yaw));

   const CVector2 v_cm_s = fwd * velocity_cm_s;
   const Real      mass  = cpBodyGetMass(m_ptBody);
   const CVector2  Jv    = (v_cm_s / 100.0) * mass * WIND_IMPULSE_SCALE;

   m_cAccumImpulse += Jv;
}

/* --------------------------------------------------------------- */
/* PRE: reset + wind + broadcast; no scheduling here */
void CAirResistance::HandleAerodynamicsPreStep()
{
   EnsurePhysicsHandle();

   /* reset per-tick accumulator */
   m_cAccumImpulse.Set(0.0f, 0.0f);

   /* wind contribution */
   ApplyWindImpulse();

   /* RAB broadcast: radius in mm */
   if(m_pcRABAct) {
      UInt8 r_mm = static_cast<UInt8>(
         std::min<Real>(255.0, std::round(m_fSelfRadiusM * 1000.0)));
      CByteArray data(1, r_mm);
      m_pcRABAct->SetData(data);
   }
}

/* POST: schedule one post-step to apply the summed impulse */
void CAirResistance::HandleAerodynamicsPostStep()
{
   auto& dyn2d = dynamic_cast<CDynamics2DEngine&>(
      CSimulator::GetInstance().GetPhysicsEngine("dyn2d"));
   cpSpace* space = dyn2d.GetPhysicsSpace();

   auto* payload = new SWindPostData{
      m_ptBody,
      cpv(m_cAccumImpulse.GetX(), m_cAccumImpulse.GetY())
   };
   /* key = body → called once per body after step */
   cpSpaceAddPostStepCallback(space, ApplyAccumPostStep, m_ptBody, payload);
}

/* --------------------------------------------------------------- */
void CAirResistance::ControlStep()
{
   /* 1) before drive: reset, add wind, broadcast RAB */
   HandleAerodynamicsPreStep();

   /* 2) drive adds to accumulator */
   DriveImpulse(m_fBaseCms);

   /* 3) schedule a single post-step to apply (wind + drive) */
   HandleAerodynamicsPostStep();
}

/* --------------------------------------------------------------- */
REGISTER_CONTROLLER(CAirResistance, "air_resistance_controller")

