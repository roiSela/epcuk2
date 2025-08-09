#ifndef AIR_RESISTANCE_H
#define AIR_RESISTANCE_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/vector2.h>

struct cpBody;  // Chipmunk forward declaration

namespace argos {

   class CAirResistance final : public CCI_Controller {

   public:
      void Init(TConfigurationNode& t_node) override;
      void ControlStep()                   override;
      void Reset()                         override {}
      void Destroy()                       override {}

   private:
      /* One-shot body hookup + per-tick wind handling --------------- */
      /** Ensures we have a valid Chipmunk body pointer (lazy init). */
      void EnsurePhysicsHandle();

      /**
       * Applies the wind impulse for the current tick.
       * The impulse J = (wind_cm_s / 100) * mass * WIND_IMPULSE_SCALE.
       */
      void ApplyWindImpulse();

      /**
       * Convenience wrapper called as the **first line** of ControlStep().
       * It performs EnsurePhysicsHandle() and then ApplyWindImpulse().
       */
      void HandleAerodynamicsStep();

      /* Devices (generic) ------------------------------------------- */
      CCI_DifferentialSteeringActuator* m_pcWheels = nullptr;
      CCI_PositioningSensor*            m_pcPos    = nullptr;

      /* Parameters --------------------------------------------------- */
      Real     m_fBaseCms = 5.0f;   ///< forward command to wheels (cm/s)
      CVector2 m_cWindCms;          ///< global wind vector (cm/s)

      /**
       * WIND_IMPULSE_SCALE:
       * -------------------
       * The wind in the .argos file is given as a speed in cm/s. Each simulation tick
       * we convert that into an impulse J applied to the robot’s physics body:
       *
       *      J = (wind_cm_s / 100) * mass * WIND_IMPULSE_SCALE
       *
       * The constant 3.5 was found by trial and error so that, with ARGoS dyn2d at
       * 10 ticks/sec, a “X cm/s” wind in the config produces about X cm/s of steady
       * sideways drift for a small differential-drive robot moving straight ahead.
       * In plain terms: type 5 cm/s in the config, and you see ~5 cm/s of drift in
       * the sim — without the wind completely overpowering the wheels.
       *
       * If you change the physics engine, robot mass, or tick rate, you may need
       * to re-tune this number.
       */

      static constexpr Real WIND_IMPULSE_SCALE = 3.5f;

      /* Cached physics ---------------------------------------------- */
      bool     m_bBodyReady = false;
      cpBody*  m_ptBody     = nullptr;
   };

} /* namespace argos */

#endif /* AIR_RESISTANCE_H */
