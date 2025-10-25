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

   class CAirResistance final : public CCI_Controller {

   public:
      void Init(TConfigurationNode& t_node) override;
      void ControlStep()                   override;
      void Reset()                         override {}
      void Destroy()                       override {}

   private:
      /* physics + wind handling */
      void EnsurePhysicsHandle();
      void ApplyWindImpulse();                 // adds to accumulator
      void DriveImpulse(Real velocity_cm_s);   // adds to accumulator
      void HandleAerodynamicsPreStep();        // reset + wind + broadcast
      void HandleAerodynamicsPostStep();       // schedule single post-step to apply sum

      /* blocking (returns true if any neighbor provides reduction; sets [0,1]) */
      bool     IsBlockedByRAB(Real& fOutReduction) const;
      CVector2 ComputeEffectiveWind() const;
      Real     GetYawRadians() const;

      /* devices */
      CCI_DifferentialSteeringActuator* m_pcWheels = nullptr; // unused when driving by impulse
      CCI_PositioningSensor*            m_pcPos    = nullptr;
      CCI_RangeAndBearingSensor*        m_pcRABSens= nullptr;
      CCI_RangeAndBearingActuator*      m_pcRABAct = nullptr;

      /* params */
      Real     m_fBaseCms = 5.0f;  /* desired forward speed, cm/s */
      CVector2 m_cWindCms;         /* global wind vector, cm/s */

      /* simple fallback radius broadcast (meters) */
      Real m_fSelfRadiusM = 0.04f;

      /* cached physics */
      bool     m_bBodyReady = false;
      cpBody*  m_ptBody     = nullptr;

      /* per-tick accumulated impulse (world frame) */
      CVector2 m_cAccumImpulse;    // J = (cm/s)/100 * mass * WIND_IMPULSE_SCALE
   };

} /* namespace argos */

#endif /* AIR_RESISTANCE_H */

