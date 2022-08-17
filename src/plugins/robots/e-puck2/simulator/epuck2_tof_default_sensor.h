/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_tof_equipped_entity.h>
 *
 * @author Daniel H. Stolfi based on the Danesh Tarapore's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK_TOF_DEFAULT_SENSOR_H
#define EPUCK_TOF_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CEPuck2TOFDefaultSensor;
   class CProximitySensorEquippedEntity;
}

#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CEPuck2TOFDefaultSensor : public CSimulatedSensor,
                                   public CCI_EPuck2TOFSensor {

   public:

      CEPuck2TOFDefaultSensor();

      virtual ~CEPuck2TOFDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to tof sensor equipped entity associated to this sensor */
      CEPuck2TOFEquippedEntity* m_pcTOFEntity;

      /** Reference to controllable entity associated to this sensor */
      CControllableEntity* m_pcControllableEntity;

      /** Flag to show rays in the simulator */
      bool m_bShowRays;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      CSpace& m_cSpace;
   };

}

#endif
