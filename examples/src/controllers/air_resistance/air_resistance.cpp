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
/**
 * Computes whether any upwind neighbor (from RAB) aerodynamically “shields” this robot
 * and, if so, how much the global wind should be reduced.
 *
 * Output:
 *   fOutReduction ∈ [0, 1] — 0=no shielding (full wind), 1=full shielding (no wind)
 *
 * Conventions & Units:
 *   - World positions, radii, and wind magnitudes are in METERS (and cm/s for wind speed vector magnitude).
 *   - RAB range (rcv.Range) arrives in CENTIMETERS → converted to meters here.
 *   - We keep a linear lateral coverage and a tuned distance falloff to match desired motion feel.
 *
 * Shadow model (one neighbor):
 *   1) Lateral coverage in [0,1]:
 *        coverage = max(0, 1 - lateral_offset_m / blocker_radius_m)
 *      → full on the wind centerline; fades linearly to 0 at one radius aside.
 *
 *   2) Distance falloff in [0,1]:
 *        distance_factor = sqrt( blocker_radius_m / (along_wind_m * S_ALONG_SCALE) ), capped at 1
 *      → weaker farther behind; S_ALONG_SCALE (100) preserves the original “feel”.
 *
 *   3) Combined reduction per neighbor:
 *        reduction = coverage * distance_factor
 *
 *   4) We take the maximum reduction over all valid upwind neighbors.
 */
bool CAirResistance::IsBlockedByRAB(Real& fOutReduction) const
{
   fOutReduction = 0.0;

   /* Basic guards */
   if(!m_pcPos || !m_pcRABSens) return false;
   if(m_cWindCms.Length() < 1e-9) return false;

   /* --- Tunable constants --- */
   /* Scale for along-wind distance inside the falloff; 100 ≈ use centimeters for s. */
   constexpr Real S_ALONG_SCALE = 100.0;

   /* Acceptable advertised blocker radius window [m]; reject absurd values. */
   constexpr Real ADV_RADIUS_MIN_M = 0.005;  /* 5 mm  */
   constexpr Real ADV_RADIUS_MAX_M = 0.20;   /* 20 cm */

   /* Wind direction (unit vector in world frame) */
   CVector2 wind_dir = m_cWindCms;
   wind_dir.Normalize();

   const Real yaw_world = GetYawRadians();
   const auto& readings = m_pcRABSens->GetReadings();

   bool any_blocker = false;

   for(const auto& rcv : readings) {

      /* --- Range and bearing to neighbor --- */

      /* RAB range is in centimeters → convert to meters */
      const Real range_cm = rcv.Range;
      if(range_cm <= 1e-6) continue;
      const Real range_m  = range_cm * 0.01;

      /* Neighbor-advertised radius (byte0 = mm → m), with sanity window.
         If missing or absurd, fall back to our default self radius. */
      Real blocker_radius_m = m_fSelfRadiusM;
      if(rcv.Data.Size() >= 1) {
         const Real advertised_radius_m = static_cast<Real>(rcv.Data[0]) * 0.001; /* mm → m */
         if(advertised_radius_m > ADV_RADIUS_MIN_M && advertised_radius_m < ADV_RADIUS_MAX_M) {
            blocker_radius_m = advertised_radius_m;
         }
      }

      /* Transform sensor-relative bearing into world frame */
      const Real bearing_world = yaw_world + rcv.HorizontalBearing.GetValue();

      /* Vector from ME → OTHER (world frame, meters) */
      const CVector2 me_to_other(range_m * std::cos(bearing_world),
                                 range_m * std::sin(bearing_world));

      /* Vector from OTHER → ME (reverse) */
      const CVector2 other_to_me = -me_to_other;

      /* --- Decompose OTHER→ME into along-wind and lateral components --- */

      /* Along-wind distance (meters). We only consider UPWIND neighbors (s > 0). */
      const Real along_wind_m = other_to_me.DotProduct(wind_dir);
      if(along_wind_m <= 0.0) continue;  /* neighbor is downwind of me → no block */

      /* Lateral offset from the wind centerline (meters) */
      const CVector2 lateral_vec = other_to_me - (wind_dir * along_wind_m);
      const Real     lateral_offset_m = lateral_vec.Length();

      /* --- Lateral coverage [0,1]: full on centerline, linear fade to 0 at one radius --- */
      const Real coverage = std::max(
         0.0,
         1.0 - (lateral_offset_m / std::max(blocker_radius_m, 1e-6))
      );
      if(coverage <= 0.0) continue;  /* too far to the side → no shielding */

      /* --- Distance falloff [0,1]: tuned with S_ALONG_SCALE to match desired feel --- */
      const Real denom_m = std::max(along_wind_m * S_ALONG_SCALE, 1e-6);
      const Real ratio   = std::min(1.0, blocker_radius_m / denom_m);
      const Real distance_factor = std::sqrt(ratio);  /* gentle 1/s^0.5 fade */

      /* Combined reduction from this neighbor */
      const Real reduction = std::clamp(coverage * distance_factor, 0.0, 1.0);

      fOutReduction = std::max(fOutReduction, reduction);
      any_blocker = true;

      /* Optional debug:
      // LOG << "[blk] range_m=" << range_m
      //     << " s=" << along_wind_m
      //     << " lat=" << lateral_offset_m
      //     << " r_blocker=" << blocker_radius_m
      //     << " cov=" << coverage
      //     << " dist=" << distance_factor
      //     << " red=" << reduction
      //     << std::endl;
      */
   }

   return any_blocker && fOutReduction > 1e-6;
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

