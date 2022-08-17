/**
 * @file <argos3/plugins/robots/e-puck2/simulator/leds_default_actuator.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_LED_DEFAULT_ACTUATOR_H
#define EPUCK2_LED_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CEPuck2LEDsDefaultActuator;
   class CLEDMedium;
}

#include <argos3/core/simulator/actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include "epuck2_led_equipped_entity.h"

namespace argos {

   class CEPuck2LEDsDefaultActuator : public CSimulatedActuator,
                                      public CCI_EPuck2LEDsActuator {

   public:

      CEPuck2LEDsDefaultActuator();

      virtual ~CEPuck2LEDsDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

   private:

      CEPuck2LEDEquippedEntity* m_pcLEDEquippedEntity;
      CLEDMedium*         m_pcLEDMedium;

   };

}

#endif
