/**
 * @file <epuck2_leds.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_leds.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

/****************************************/
/****************************************/

CEPuck2Leds::CEPuck2Leds() : m_pcLedAct(NULL) {}

/****************************************/
/****************************************/

void CEPuck2Leds::Init(TConfigurationNode& t_node) {
   m_pcLedAct = GetActuator<CCI_EPuck2LEDsActuator>("epuck2_leds");
   m_pcLedAct->SetAllBlack();
}

/****************************************/
/****************************************/

void CEPuck2Leds::ControlStep() {
   unsigned uTick = (&CSimulator::GetInstance())->GetSpace().GetSimulationClock();
   switch (uTick % 100) {
      case 10:
         m_pcLedAct->SetAllBlack();
         m_pcLedAct->SetAllRedLeds(true);
         break;
      case 20:
         m_pcLedAct->SetAllBlack();
         m_pcLedAct->SetRGBLed2Color(CColor::PURPLE);
         m_pcLedAct->SetRGBLed4Color(CColor::BLUE);
         m_pcLedAct->SetRGBLed6Color(CColor::YELLOW);
         m_pcLedAct->SetRGBLed8Color(CColor::CYAN);
         break;
      case 30:
         m_pcLedAct->SetAllBlack();
         m_pcLedAct->SetBodyLed(true);
         break;
      case 40:
         m_pcLedAct->SetAllBlack();
         m_pcLedAct->SetFrontLed(true);
         break;
      case 50:
         m_pcLedAct->SetRedLed1(true);
         break;
      case 55:
         m_pcLedAct->SetRedLed3(true);
         break;
      case 60:
         m_pcLedAct->SetRedLed5(true);
         break;
      case 65:
         m_pcLedAct->SetRedLed7(true);
         break;
      case 70:
         m_pcLedAct->SetRGBLed2Color(CColor::WHITE);
         break;
      case 75:
         m_pcLedAct->SetRGBLed4Color(CColor::WHITE);
         break;
      case 80:
         m_pcLedAct->SetRGBLed6Color(CColor::WHITE);
         break;
      case 85:
         m_pcLedAct->SetRGBLed8Color(CColor::WHITE);
         break;
      case 90:
         m_pcLedAct->SetBodyLed(true);
         break;
      case 0:
         m_pcLedAct->SetAllBlack();
         break;
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEPuck2Leds, "epuck2_leds_controller")
