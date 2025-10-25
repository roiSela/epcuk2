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
 * Determine whether any upwind neighbor aerodynamically shields this robot,
 * and by how much to reduce the global wind as a result.
 *
 * Output:
 *   fOutReduction ∈ [0, 1]
 *     0.0 => no shielding (full wind)
 *     1.0 => full shielding (no wind)
 *
 * Model overview (per neighbor):
 *   - We treat the upwind neighbor as casting a "shadow" downwind.
 *   - The shadow has:
 *       (1) a lateral extent (how far to the side you can be and still feel it),
 *       (2) a longitudinal fade (how the effect weakens as you get farther behind).
 *
 * Lateral "width" (sideways tolerance):
 *   coverage = 1 - (lateral_offset / effective_lateral_radius)^WIDTH_POWER, clamped to [0,1]
 *   where effective_lateral_radius = WIDTH_SCALE * blocker_radius_m
 *   Defaults: WIDTH_SCALE=1.0 (one blocker radius), WIDTH_POWER=1.0 (linear)
 *
 * Longitudinal "length" (downwind reach/fade):
 *   distance_factor = ( blocker_radius_m / (along_wind_m * LENGTH_SCALE) )^(LENGTH_ALPHA), clamped to [0,1]
 *   Defaults: LENGTH_SCALE=100.0 (short reach; matches previous feel), LENGTH_ALPHA=0.5 (sqrt)
 *
 * Notes:
 *   - All distances in meters.
 *   - RAB range (rcv.Range) arrives in centimeters and is converted to meters here.
 *   - The neighbor may advertise an "effective blocking radius" in Data[0] (millimeters).
 *     We accept only sane values in [ADV_RADIUS_MIN_M, ADV_RADIUS_MAX_M]; otherwise we
 *     fall back to m_fSelfRadiusM.
 *   - We take the maximum reduction across all valid upwind neighbors.
 */
bool CAirResistance::IsBlockedByRAB(Real& fOutReduction) const
{
   fOutReduction = 0.0;

   /* Guard conditions */
   if(!m_pcPos || !m_pcRABSens) return false;
   if(m_cWindCms.Length() < 1e-9) return false;

   /* ---------------- Tunable constants (internal, no XML) ---------------- */

   /* Width controls (sideways tolerance) */
   constexpr Real WIDTH_SCALE = 1.0;  /* multiply blocker radius for lateral reach (1.0 = one radius) */
   constexpr Real WIDTH_POWER = 1.0;  /* lateral falloff exponent (1.0=linear, 2.0=quadratic, etc.) */

   /* Length controls (downwind reach) */
   constexpr Real LENGTH_SCALE = 100.0; /* multiplies along-wind distance inside falloff; higher = shorter reach */
   constexpr Real LENGTH_ALPHA = 0.5;   /* distance exponent; 0.5 = sqrt (gentle), 1.0 ≈ 1/s (steeper) */

   /* Sanity window for advertised radius (meters) */
   constexpr Real ADV_RADIUS_MIN_M = 0.005; /* 5 mm  */
   constexpr Real ADV_RADIUS_MAX_M = 0.20;  /* 20 cm */

   /* --------------------------------------------------------------------- */

   /* Unit wind direction in world frame */
   CVector2 wind_dir = m_cWindCms;
   wind_dir.Normalize();

   const Real yaw_world = GetYawRadians();
   const auto& readings = m_pcRABSens->GetReadings();

   bool any_blocker = false;

   for(const auto& rcv : readings) {
      /* ---- Range & bearing to neighbor ----
         RAB range is provided in centimeters → convert to meters. */
      const Real range_cm = rcv.Range;
      if(range_cm <= 1e-6) continue;
      const Real range_m = range_cm * 0.01;

      /* Neighbor-advertised effective radius (byte0: millimeters → meters), with sanity window.
         If absent or absurd, use our local default m_fSelfRadiusM. */
      Real blocker_radius_m = m_fSelfRadiusM; /* default ~0.04 m for e-puck2 */
      if(rcv.Data.Size() >= 1) {
         const Real advertised_radius_m = static_cast<Real>(rcv.Data[0]) * 0.001; /* mm → m */
         if(advertised_radius_m > ADV_RADIUS_MIN_M && advertised_radius_m < ADV_RADIUS_MAX_M) {
            blocker_radius_m = advertised_radius_m;
         }
      }

      /* Transform sensor-relative bearing to world frame and build vectors */
      const Real bearing_world = yaw_world + rcv.HorizontalBearing.GetValue();

      /* Vector from ME → OTHER (meters, world frame) */
      const CVector2 me_to_other(range_m * std::cos(bearing_world),
                                 range_m * std::sin(bearing_world));

      /* Vector from OTHER → ME */
      const CVector2 other_to_me = -me_to_other;

      /* Along-wind component (meters): only consider UPWIND neighbors (positive projection) */
      const Real along_wind_m = other_to_me.DotProduct(wind_dir);
      if(along_wind_m <= 0.0) continue; /* neighbor is downwind → no shielding */

      /* Lateral (perpendicular) offset from wind centerline (meters) */
      const CVector2 lateral_vec      = other_to_me - (wind_dir * along_wind_m);
      const Real     lateral_offset_m = lateral_vec.Length();

      /* ---------------- Lateral coverage (WIDTH) ----------------
         Any shielding if lateral_offset < effective_lateral_radius.
         Falloff shape is controlled by WIDTH_POWER. */
      const Real effective_lateral_radius_m =
         std::max<Real>(1e-6, WIDTH_SCALE * blocker_radius_m);

      const Real lat_norm = std::min<Real>(1.0, lateral_offset_m / effective_lateral_radius_m);

      Real lateral_coverage = 1.0 - std::pow(lat_norm, std::max<Real>(1.0, WIDTH_POWER));
      if(lateral_coverage <= 0.0) continue; /* too far to the side → no shielding */

      /* ---------------- Distance factor (LENGTH) ----------------
         Tuned with LENGTH_SCALE and LENGTH_ALPHA for longitudinal reach/fade. */
      const Real denom_m = std::max<Real>(along_wind_m * std::max<Real>(1e-6, LENGTH_SCALE), 1e-6);
      const Real ratio   = std::min< Real >(1.0, blocker_radius_m / denom_m);
      const Real distance_factor = std::pow(ratio, std::max< Real >(0.1, LENGTH_ALPHA));

      /* Combined reduction from this neighbor (clamped) */
      const Real reduction = std::clamp(lateral_coverage * distance_factor, 0.0, 1.0);

      fOutReduction = std::max(fOutReduction, reduction);
      any_blocker   = true;

      /* Optional debug:
      // LOG << "[blk] s=" << along_wind_m
      //     << " lat=" << lateral_offset_m
      //     << " r_blocker=" << blocker_radius_m
      //     << " eff_lat_r=" << effective_lateral_radius_m
      //     << " cov=" << lateral_coverage
      //     << " dist=" << distance_factor
      //     << " red=" << reduction << std::endl;
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
   const CVector2 Jv = (eff / 100.0) * mass;

   m_cAccumImpulse += Jv;
}

/* Adds drive impulse (forward along yaw) to the per-tick accumulator */
void CAirResistance::DriveImpulse(Real velocity_cm_s)
{
   const Real yaw = GetYawRadians();
   const CVector2 fwd(std::cos(yaw), std::sin(yaw));

   const CVector2 v_cm_s = fwd * velocity_cm_s;
   const Real      mass  = cpBodyGetMass(m_ptBody);
   const CVector2  Jv    = (v_cm_s / 100.0) * mass;

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

