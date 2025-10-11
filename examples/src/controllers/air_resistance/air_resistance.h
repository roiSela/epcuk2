#ifndef AIR_RESISTANCE_H
#define AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/utility/math/vector2.h>

/* Forward decl for Chipmunk body (dyn2d) */
struct cpBody;

namespace argos {

   /**
    * Air resistance controller with wind blocking using RAB-only geometry.
    * - Robots broadcast a 1-byte radius (mm) each tick via RAB actuator.
    * - Neighbors reconstruct relative vectors from RAB range+bearing and own yaw.
    * - Blocking reduction is computed from lateral coverage and distance (sqrt decay).
    * - No robot-specific strings or hand-tuned wake constants.
    */
   class CAirResistance final : public CCI_Controller {

   public:
      void Init(TConfigurationNode& t_node) override;
      void ControlStep()                   override;
      void Reset()                         override {}
      void Destroy()                       override {}

   private:
      /* physics + wind handling */
      void EnsurePhysicsHandle();
      void ApplyWindImpulse();
      void HandleAerodynamicsStep();

      /* blocking (returns true if any neighbor provides reduction; sets [0,1]) */
      bool     IsBlockedByRAB(Real& fOutReduction) const;
      CVector2 ComputeEffectiveWind() const;
      Real     GetYawRadians() const;

      /* devices */
      CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
      CCI_PositioningSensor*            m_pcPos    = nullptr;
      CCI_RangeAndBearingSensor*        m_pcRABSens= nullptr;
      CCI_RangeAndBearingActuator*      m_pcRABAct = nullptr;

      /* params */
      Real     m_fBaseCms = 5.0f;  /* wheel command (cm/s) */
      CVector2 m_cWindCms;         /* global wind vector (cm/s) */

      /* impulse scaling for Chipmunk */
      static constexpr Real WIND_IMPULSE_SCALE = 4.25f;

      /* simple fallback radius broadcast (meters) */
      Real m_fSelfRadiusM = 0.04f;

      /* cached physics */
      bool     m_bBodyReady = false;
      cpBody*  m_ptBody     = nullptr;
   };

} /* namespace argos */

#endif /* AIR_RESISTANCE_H */