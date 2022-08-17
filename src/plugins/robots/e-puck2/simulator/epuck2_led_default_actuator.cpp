/**
 * @file <argos3/plugins/robots/e-puck2/simulator/leds_default_actuator.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include "epuck2_led_default_actuator.h"

namespace argos {

   /****************************************/
   /****************************************/

    CEPuck2LEDsDefaultActuator::CEPuck2LEDsDefaultActuator() :
      m_pcLEDEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CEPuck2LEDsDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcLEDEquippedEntity = &(c_entity.GetComponent<CEPuck2LEDEquippedEntity>("epuck2_leds"));
      m_tSettings.resize(m_pcLEDEquippedEntity->GetLEDs().size());
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDsDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         CCI_EPuck2LEDsActuator::Init(t_tree);
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcLEDMedium = &CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium);
            m_pcLEDEquippedEntity->SetMedium(*m_pcLEDMedium);
            m_pcLEDEquippedEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initialising the EPuck2 LEDs default actuator", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDsDefaultActuator::Update() {
      m_pcLEDEquippedEntity->SetAllLEDsColors(m_tSettings);
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDsDefaultActuator::Reset() {
      SetAllBlack();
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDsDefaultActuator::Destroy() {
      m_pcLEDEquippedEntity->Disable();
   }

   /****************************************/
   /****************************************/

}

REGISTER_ACTUATOR(CEPuck2LEDsDefaultActuator,
                  "epuck2_leds", "default",
                  "Daniel H. Stolfi based on Carlo Pinciroli's work",
                  "1.0",
                  "The EPuck2 LEDs actuator.",

                  "This actuator controls a group of LEDs. For a complete description of its\n"
                  "usage, refer to the ci_leds_actuator.h file.\n\n"

                  "REQUIRED XML CONFIGURATION\n\n"

                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <leds implementation=\"default\"\n"
                  "              medium=\"leds\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"

                  "The 'medium' attribute sets the id of the LED medium declared in the <media>\n"
                  "XML section.\n\n"

                  "OPTIONAL XML CONFIGURATION\n\n"

                  "None.\n",

                  "Usable"
   );

