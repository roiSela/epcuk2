/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_encoder_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_ENCODER_SENSOR_H
#define EPUCK2_ENCODER_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CEPuck2EncoderSensor;
   class CEPuck2EncoderEquippedEntity;
}

#include "../control_interface/ci_epuck2_encoder_sensor.h"
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CEPuck2EncoderDefaultSensor : public CSimulatedSensor,
                                       public CCI_EPuck2EncoderSensor {

   public:

      CEPuck2EncoderDefaultSensor();

      virtual ~CEPuck2EncoderDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to encoder sensor equipped entity associated to this sensor */
      CEPuck2EncoderEquippedEntity* m_pcEncoderEntity;

      /** Reference to controllable entity associated to this sensor */
      //CControllableEntity* m_pcControllableEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      Real m_fLeft;
      Real m_fRight;

      /** Reference to the space */
      //CSpace& m_cSpace;
   };

}

#endif
