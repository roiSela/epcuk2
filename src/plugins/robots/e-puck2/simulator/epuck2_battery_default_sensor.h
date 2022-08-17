/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_battery_default_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Adhavan Jayabalan's work.
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_BATTERY_DEFAULT_SENSOR_H
#define EPUCK2_BATTERY_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CEPuck2BatteryDefaultSensor;
   class CEPuck2BatteryEquippedEntity;
   class CPhysicsEngine;
}

#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
//#include "../control_interface/ci_epuck2_battery_sensor.h"
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>

namespace argos {

   class CEPuck2BatteryDefaultSensor : public CSimulatedSensor,
                                       public CCI_BatterySensor {

   public:

      CEPuck2BatteryDefaultSensor();

      virtual ~CEPuck2BatteryDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to battery sensor equipped entity associated to this sensor */
      CEPuck2BatteryEquippedEntity* m_pcBatteryEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range on battery level */
      CRange<Real> m_cNoiseRange;
   };

}

#endif
